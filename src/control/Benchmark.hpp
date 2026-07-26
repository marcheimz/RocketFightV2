#pragma once

#include <cstdint>
#include <memory>
#include <string_view>

#include "control/Controller.hpp"
#include "core/Rng.hpp"

namespace rf {

// ---------------------------------------------------------------------------
// A scripted flying test for a fly-by-wire, and the meter that scores it.
//
// Placement: this is control/, not eval/, because both the interactive game
// (rf_sim) and the headless harness (rf_eval) must drive the *same* sequence,
// and rf_sim links rf_control but not rf_eval. Putting it here reaches both
// without adding an edge to the target graph -- and without rf_sim acquiring,
// via rf_eval -> rf_data, a dependency on file I/O it has no business having.
//
// What stays in eval/ is the runner: worlds, sweeps, JSON, aggregation. This
// file only answers "what is commanded" and "what happened", both of which are
// pure functions of simulation state. Nothing here opens a file or reads a
// clock, which is also what lets the tests drive it with no world at all.
// ---------------------------------------------------------------------------

// The velocity band the benchmark works in -- and, deliberately, the band the
// left stick commands when the pilot is flying in velocity mode, so hand-flying
// and the benchmark feel like the same task.
//
// Velocity intents are *absolute*, not a fraction of anything: in zero-g there
// is no maximum speed to normalise against, so this is simply the range being
// tested for now. One constant, so widening the band widens both at once.
inline constexpr Real kBenchmarkMaxSpeed = Real(100.0);

// The whole definition of a run. Everything a submission is judged against is
// in here, so two controllers face an identical task by construction.
struct BenchmarkConfig {
    // The two modes are separate experiments and are never interleaved inside
    // one run: their commanded quantities have different units, so a tracking
    // error integrated across both would be an addition of metres to m/s.
    Intent::Mode mode{Intent::Mode::Acceleration};

    // Acceleration mode. Intent::Mode::Acceleration's vector is already a
    // fraction of the vehicle's maximum, so this band is vehicle-agnostic.
    Real accelMin{0.3};
    Real accelMax{1.0};

    // Velocity mode, m/s.
    Real speedMax{kBenchmarkMaxSpeed};

    // How long each commanded value is held before it is resampled.
    Real holdMinSeconds{1.0};
    Real holdMaxSeconds{5.0};

    // "Settled" thresholds. Acceleration is relative to the vehicle's own
    // authority, because a 4 m/s^2 error means something different on a lander
    // than on an interceptor; velocity is absolute for the same reason the
    // command is.
    Real accelSettleFraction{0.10};     // of maxAccel
    Real velocitySettleTolerance{5.0};  // m/s
};

// Lookup tables rather than a switch returning a constexpr std::string_view:
// GCC 15.2.1 miscompiles that shape at -O2 when it is called from inside a
// comparison loop, and a bug that only exists in optimised builds is the worst
// kind to own.
inline constexpr std::string_view kIntentModeNames[] = {"acceleration", "velocity"};

inline std::string_view intentModeName(Intent::Mode m) {
    return kIntentModeNames[static_cast<std::size_t>(m)];
}

// Accepts the long name and the flag spelling. Returns false on anything else,
// so a typo in --mode= is an error rather than a silent acceleration run.
bool intentModeFromString(std::string_view text, Intent::Mode& out);

// ---------------------------------------------------------------------------
// Layer 1: the benchmark as a pilot.
//
// It plugs in exactly where GamepadIntentSource does, which is the point --
// what is being measured is the fly-by-wire, so the layer above it must be
// interchangeable with a human.
//
// Determinism is the whole value of this class. The sequence depends on the
// seed *alone*: not on the vehicle, not on where the rocket is, not on what the
// controller did with the last command, and not on which tick the pilot pressed
// the button. Two different rockets, or two different submitted controllers,
// therefore face a byte-identical task.
// ---------------------------------------------------------------------------
class BenchmarkIntentSource final : public IntentSource {
public:
    static constexpr std::uint64_t kDefaultSeed = 0x5EEDB00C00000001ull;

    explicit BenchmarkIntentSource(BenchmarkConfig cfg = {}, std::uint64_t seed = kDefaultSeed);

    // The Observation is deliberately unused. Reading it -- even to normalise
    // against maxAccel -- would make the sequence a function of the vehicle,
    // and the whole comparison rests on it not being one.
    Intent intent(const Observation& obs) override;

    void reset(std::uint64_t seed, const WorldConfig& env) override;

    // Restart from a seed with no world in hand. The interactive path arms the
    // benchmark mid-flight and has nothing else to hand it.
    void restart(std::uint64_t seed);

    // Increments exactly when the held command is resampled. The monitor
    // watches this to know when its settling clock restarts, rather than trying
    // to infer a step change by comparing floating-point vectors -- which would
    // miss a resample that happened to land near the previous value.
    std::uint64_t commandIndex() const { return commandIndex_; }

    // Simulated ticks this source has issued, counted by itself rather than
    // read from the observation, for the same seed-alone reason as above.
    Tick elapsedTicks() const { return tick_; }

    Intent::Mode           mode() const { return cfg_.mode; }
    const BenchmarkConfig& config() const { return cfg_; }
    std::uint64_t          seed() const { return seed_; }

    // Changing mode restarts the sequence, because the two modes are separate
    // experiments and half a run of each is neither.
    void setMode(Intent::Mode m);

private:
    void sample();

    BenchmarkConfig cfg_;
    std::uint64_t   seed_;
    Rng             rng_;

    Intent        held_{};
    Tick          tick_{0};
    Tick          nextChange_{0};
    std::uint64_t commandIndex_{0};
};

// ---------------------------------------------------------------------------
// What a run is worth, reported as components rather than as one number.
//
// Weights for any composite will change the moment there are real submissions
// to look at, so the components are the output and the score is a convenience
// derived from them. Anyone can re-rank an archive of results without re-running
// a single episode.
// ---------------------------------------------------------------------------
struct BenchmarkMetrics {
    // Time-integrated |commanded - actual|. In acceleration mode both sides are
    // m/s^2, so the integral is m/s; in velocity mode both are m/s and it is
    // metres. Reported as an integral *and* a mean, because the integral grows
    // with run length and comparisons across run lengths need the mean.
    Real trackingErrorIntegral{};
    Real trackingErrorMean{};

    // How long, after a step change, the error takes to fall below the
    // threshold and stay there. An unsettled step is charged the full hold
    // interval, which is a floor rather than the true figure -- otherwise a
    // controller that never settles would score better than one that settles
    // slowly, by contributing no samples at all.
    Real          settlingMean{};   // s
    Real          settlingWorst{};  // s
    std::uint32_t steps{};
    std::uint32_t stepsSettled{};

    // Efficiency. Two controllers that track equally well are not equally good
    // if one burns twice the propellant getting there.
    Real impulse{};  // N*s, time-integrated total thrust

    // Rotation nobody asked for is wasted authority, and in a fight it is a
    // liability. Integrated |angular velocity|, so it is the total angle swept
    // rather than the net rotation -- a ship that spins one way and back has
    // done the work twice, and this says so.
    Real attitudeWander{};  // rad

    Real seconds{};

    Real settledFraction() const {
        return steps > 0 ? static_cast<Real>(stepsSettled) / static_cast<Real>(steps) : Real(0);
    }
    Real meanThrust() const { return seconds > Real(0) ? impulse / seconds : Real(0); }
    Real meanAngRate() const { return seconds > Real(0) ? attitudeWander / seconds : Real(0); }
};

// A placeholder aggregate; lower is better. Every term is per-second or a mean,
// so the number does not drift with run length. These weights are a guess and
// are meant to be overruled -- which is why they travel in the JSON next to the
// components they were applied to.
struct ScoreWeights {
    Real tracking{1.0};     // per unit of mean tracking error
    Real settling{1.0};     // per second of mean settling time
    Real impulse{1.0e-5};   // per newton of mean thrust
    Real wander{1.0};       // per rad/s of mean angular rate
};

Real compositeScore(const BenchmarkMetrics& m, const ScoreWeights& w = {});

// ---------------------------------------------------------------------------
// The meter. Folded over the run one sample at a time; holds no history, so it
// costs the same at tick 1 and tick 60000 and allocates nothing.
// ---------------------------------------------------------------------------
class BenchmarkMonitor {
public:
    void reset(const RocketSpec& spec, const BenchmarkConfig& cfg);

    // One sample covering `dt` of simulated time. Call it once per physics tick
    // with kTickDt for the exact integral. Must be called *after* World::step,
    // so the actuator state is the one that pushed the hull this tick and the
    // velocity is the result of it.
    void sample(const Intent& commanded, std::uint64_t commandIndex, const Observation& obs,
                Real dt);

    // Closes the step in progress, so the last command counts towards settling
    // like every other one.
    void finish();

    const BenchmarkMetrics& metrics() const { return m_; }

    // Instantaneous |commanded - actual|, for a live readout. Not a metric.
    Real currentError() const { return error_; }

private:
    // The vehicle's real net acceleration, taken from the actuators rather than
    // by differencing positions. ThrusterState carries the thrust actually being
    // produced and the angle the nozzle has actually reached, which is exactly
    // what Rocket::applyControls turns into force -- so this measures the
    // vehicle, not the integrator, and cannot pick up anything else that was
    // pushing.
    Vec2 actualAcceleration(const Observation& obs) const;

    void closeStep();

    RocketSpec       spec_{};
    BenchmarkConfig  cfg_{};
    BenchmarkMetrics m_{};

    std::uint64_t commandIndex_{0};
    bool          started_{false};

    Real settlingSum_{};
    Real stepElapsed_{};   // seconds since the current command was issued
    Real lastExcursion_{}; // ...when the error was last outside tolerance
    bool withinTolerance_{false};
    Real error_{};
};

// ---------------------------------------------------------------------------
// The two halves wired together: the scripted IntentSource on top of whatever
// fly-by-wire the vehicle wants, which is precisely the composition a submitted
// FlyByWire gets dropped into. Both the game loop and the headless runner use
// this, so there is exactly one benchmark and not two that have to agree.
// ---------------------------------------------------------------------------
class BenchmarkHarness {
public:
    explicit BenchmarkHarness(BenchmarkConfig cfg          = {},
                              std::uint64_t   seed         = BenchmarkIntentSource::kDefaultSeed);

    // Fit to a vehicle and restart everything: a fresh sequence from the seed,
    // a fly-by-wire built from this world's rocket, and cleared metrics.
    void reset(const WorldConfig& env);

    Controller&             controller() { return *controller_; }
    BenchmarkIntentSource&  source() { return *source_; }
    const BenchmarkMonitor& monitor() const { return monitor_; }

    void setMode(Intent::Mode m);
    void setSeed(std::uint64_t seed);

    // Feed the meter. Pulls the command straight back out of the controller, so
    // there is no second copy of "what was asked for" to fall out of step.
    void sample(const Observation& obs, Real dt);
    void finish() { monitor_.finish(); }

    Intent::Mode           mode() const { return cfg_.mode; }
    std::uint64_t          seed() const { return seed_; }
    const BenchmarkConfig& config() const { return cfg_; }

private:
    void rebuild();

    BenchmarkConfig cfg_;
    std::uint64_t   seed_;
    RocketSpec      spec_{};

    BenchmarkIntentSource*             source_{nullptr};  // owned by controller_
    std::unique_ptr<LayeredController> controller_;
    BenchmarkMonitor                   monitor_;
};

}  // namespace rf
