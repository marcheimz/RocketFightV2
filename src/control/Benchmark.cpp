#include "control/Benchmark.hpp"

#include <algorithm>
#include <cmath>

namespace rf {

bool intentModeFromString(std::string_view text, Intent::Mode& out) {
    if (text == "accel" || text == "acceleration") {
        out = Intent::Mode::Acceleration;
        return true;
    }
    if (text == "velocity" || text == "vel") {
        out = Intent::Mode::Velocity;
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// BenchmarkIntentSource
// ---------------------------------------------------------------------------

BenchmarkIntentSource::BenchmarkIntentSource(BenchmarkConfig cfg, std::uint64_t seed)
    : cfg_(cfg), seed_(seed), rng_(seed) {
    sample();
}

void BenchmarkIntentSource::restart(std::uint64_t seed) {
    seed_         = seed;
    rng_          = Rng(seed);
    tick_         = 0;
    nextChange_   = 0;
    commandIndex_ = 0;
    held_         = Intent{};
    sample();
}

void BenchmarkIntentSource::reset(std::uint64_t seed, const WorldConfig&) {
    // The WorldConfig is ignored on purpose: reseeding from the vehicle would
    // hand two rockets two different tasks, which is the one thing this class
    // exists to prevent.
    restart(seed);
}

void BenchmarkIntentSource::setMode(Intent::Mode m) {
    if (m == cfg_.mode) return;
    cfg_.mode = m;
    restart(seed_);
}

void BenchmarkIntentSource::sample() {
    // Direction first, magnitude second, duration third -- and exactly three
    // draws in either mode, so acceleration and velocity runs from the same seed
    // share their directions and their step timing. That makes the two modes
    // comparable without making them interleaved.
    const Vec2 unit = fromAngle(rng_.nextRange(-kPi, kPi));

    held_      = Intent{};
    held_.mode = cfg_.mode;
    switch (cfg_.mode) {
        case Intent::Mode::Acceleration:
            held_.vector = unit * rng_.nextRange(cfg_.accelMin, cfg_.accelMax);
            break;
        case Intent::Mode::Velocity:
            held_.vector = unit * rng_.nextRange(Real(0), cfg_.speedMax);
            break;
    }

    // facing stays empty throughout. Where to point is the fly-by-wire's own
    // judgement -- on a vehicle that must lead with its nose it is most of the
    // job, and on a lander it is barely any of it -- and this benchmark scores
    // that judgement rather than the ability to obey an attitude order.

    // Quantised to whole control periods, which is the finest granularity at
    // which a command can actually change hands. A boundary at a fractional tick
    // would make the sequence depend on rounding rather than on the seed.
    const Real seconds  = rng_.nextRange(cfg_.holdMinSeconds, cfg_.holdMaxSeconds);
    const auto periods  = static_cast<Tick>(std::max(1LL, std::llround(seconds / kControlDt)));
    nextChange_         = tick_ + periods * Tick{kControlEvery};
    ++commandIndex_;
}

Intent BenchmarkIntentSource::intent(const Observation&) {
    if (tick_ >= nextChange_) sample();

    // The source keeps its own clock, advanced by one control period per call,
    // because that is the only rate anything evaluates a controller at. Using
    // obs.tick instead would tie the schedule to when the benchmark was armed.
    tick_ += Tick{kControlEvery};
    return held_;
}

// ---------------------------------------------------------------------------
// Scoring
// ---------------------------------------------------------------------------

Real compositeScore(const BenchmarkMetrics& m, const ScoreWeights& w) {
    return w.tracking * m.trackingErrorMean + w.settling * m.settlingMean +
           w.impulse * m.meanThrust() + w.wander * m.meanAngRate();
}

// ---------------------------------------------------------------------------
// BenchmarkMonitor
// ---------------------------------------------------------------------------

void BenchmarkMonitor::reset(const RocketSpec& spec, const BenchmarkConfig& cfg) {
    spec_ = spec;
    cfg_  = cfg;
    m_    = BenchmarkMetrics{};

    commandIndex_    = 0;
    started_         = false;
    settlingSum_     = 0;
    stepElapsed_     = 0;
    lastExcursion_   = 0;
    withinTolerance_ = false;
    error_           = 0;
}

Vec2 BenchmarkMonitor::actualAcceleration(const Observation& obs) const {
    Vec2 bodyForce{};
    const std::size_t n = std::min(obs.thrusterCount, spec_.count());
    for (std::size_t i = 0; i < n; ++i) {
        const ThrusterState& a = obs.actuators[i];
        if (a.thrust <= Real(0)) continue;
        bodyForce += spec_[i].forceLocal(a.thrust, a.gimbalAngle);
    }
    // Same wrench sum ThrustAllocator::achieved builds, and the same force
    // Rocket::applyControls applies -- rotated into the world once, because the
    // rotation is linear and doing it per thruster would only cost accuracy.
    return spec_.mass > Real(0) ? rotate(bodyForce, obs.angle) / spec_.mass : Vec2{};
}

void BenchmarkMonitor::closeStep() {
    // A step that is inside tolerance now settled at the last moment it was
    // outside; one that is still outside is charged the whole interval, which is
    // a floor on the truth. Dropping it instead would let a controller that
    // never settles outscore one that settles slowly, by contributing nothing.
    const Real settling = withinTolerance_ ? lastExcursion_ : stepElapsed_;

    settlingSum_ += settling;
    ++m_.steps;
    if (withinTolerance_) ++m_.stepsSettled;
    m_.settlingWorst = std::max(m_.settlingWorst, settling);
    m_.settlingMean  = settlingSum_ / static_cast<Real>(m_.steps);
}

void BenchmarkMonitor::sample(const Intent& commanded, std::uint64_t commandIndex,
                              const Observation& obs, Real dt) {
    if (!started_ || commandIndex != commandIndex_) {
        if (started_) closeStep();
        commandIndex_    = commandIndex;
        started_         = true;
        stepElapsed_     = 0;
        lastExcursion_   = 0;
        withinTolerance_ = false;
    }

    Vec2 want{};
    Vec2 got{};
    Real tolerance = 0;

    switch (commanded.mode) {
        case Intent::Mode::Acceleration:
            // Exactly the quantity RocketFlyByWire was asked to produce, so the
            // error is the controller's and not a units mismatch.
            want      = clampLength(commanded.vector, Real(1)) * obs.maxAccel;
            got       = actualAcceleration(obs);
            tolerance = obs.maxAccel * cfg_.accelSettleFraction;
            break;
        case Intent::Mode::Velocity:
            want      = commanded.vector;
            got       = obs.vel;
            tolerance = cfg_.velocitySettleTolerance;
            break;
    }

    error_ = length(want - got);

    Real thrust = 0;
    for (std::size_t i = 0; i < obs.thrusterCount; ++i) thrust += obs.actuators[i].thrust;

    m_.trackingErrorIntegral += error_ * dt;
    m_.impulse               += thrust * dt;
    m_.attitudeWander        += std::abs(obs.angVel) * dt;
    m_.seconds               += dt;
    m_.trackingErrorMean = m_.seconds > Real(0) ? m_.trackingErrorIntegral / m_.seconds : Real(0);

    stepElapsed_ += dt;
    withinTolerance_ = error_ <= tolerance;
    if (!withinTolerance_) lastExcursion_ = stepElapsed_;
}

void BenchmarkMonitor::finish() {
    if (!started_) return;
    closeStep();
    started_ = false;
}

// ---------------------------------------------------------------------------
// BenchmarkHarness
// ---------------------------------------------------------------------------

BenchmarkHarness::BenchmarkHarness(BenchmarkConfig cfg, std::uint64_t seed)
    : cfg_(cfg), seed_(seed) {
    rebuild();
}

void BenchmarkHarness::rebuild() {
    auto source = std::make_unique<BenchmarkIntentSource>(cfg_, seed_);
    source_     = source.get();
    // The auto-selecting constructor: the fly-by-wire is built at reset from
    // whatever vehicle the world actually holds, so swapping airframes in the
    // game refits it without this class ever naming a rocket.
    controller_ = std::make_unique<LayeredController>(std::move(source));
}

void BenchmarkHarness::reset(const WorldConfig& env) {
    if (!env.rockets.empty()) spec_ = env.rockets.front().spec;
    controller_->reset(seed_, env);
    monitor_.reset(spec_, cfg_);
}

void BenchmarkHarness::setMode(Intent::Mode m) {
    if (m == cfg_.mode) return;
    cfg_.mode = m;
    source_->setMode(m);
    monitor_.reset(spec_, cfg_);
}

void BenchmarkHarness::setSeed(std::uint64_t seed) {
    seed_ = seed;
    source_->restart(seed);
    monitor_.reset(spec_, cfg_);
}

void BenchmarkHarness::sample(const Observation& obs, Real dt) {
    const Intent* commanded = controller_->lastIntent();
    if (!commanded) return;
    monitor_.sample(*commanded, source_->commandIndex(), obs, dt);
}

}  // namespace rf
