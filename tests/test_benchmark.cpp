#include <doctest/doctest.h>

#include <cmath>
#include <vector>

#include "TestRockets.hpp"
#include "control/Benchmark.hpp"
#include "core/World.hpp"
#include "eval/BenchmarkRunner.hpp"

using namespace rf;
using rf::testing::classic;
using rf::testing::rocket;

// Two kinds of assertion live in this file, and the difference is deliberate.
//
// The *sequence* checks are tight: reproducibility is the entire value of the
// benchmark, so "same seed, same commands, same hash" is asserted exactly, and
// so are the bands the generator promises. If any of that drifts, every score
// ever produced becomes incomparable, which is not a thing to be relaxed about.
//
// The *quality* checks are loose, for the same reason the rest of the closed-loop
// tests are: the fly-by-wire is about to be brute-force optimised, and a test
// pinning today's tracking error would only ever be a false alarm.

namespace {

struct Command {
    std::uint64_t index;
    Intent::Mode  mode;
    Vec2          vector;
    bool          hasFacing;
    Tick          tick;
};

// Drive the source the way a control loop would, recording every distinct
// command and the tick it changed on.
std::vector<Command> record(BenchmarkIntentSource& source, Real seconds) {
    std::vector<Command> out;
    const auto steps = static_cast<int>(seconds / kControlDt);

    Observation obs;
    obs.maxAccel = Real(40);

    std::uint64_t last = 0;
    for (int i = 0; i < steps; ++i) {
        const Tick   at = source.elapsedTicks();
        const Intent in = source.intent(obs);
        if (source.commandIndex() != last) {
            last = source.commandIndex();
            out.push_back({last, in.mode, in.vector, in.facing.has_value(), at});
        }
    }
    return out;
}

BenchmarkRun run(const RocketSpec& spec, Intent::Mode mode, std::uint64_t seed, Real seconds) {
    BenchmarkSpec s;
    s.rocket      = spec;
    s.config.mode = mode;
    s.seed        = seed;
    s.maxTicks    = static_cast<std::uint32_t>(seconds * kTickRate);
    return runBenchmark(s);
}

}  // namespace

// ---------------------------------------------------------------------------
// The sequence: tight.
// ---------------------------------------------------------------------------

TEST_CASE("benchmark sequence depends on the seed alone") {
    BenchmarkIntentSource a({}, 12345);
    BenchmarkIntentSource b({}, 12345);

    const std::vector<Command> ra = record(a, Real(120));
    const std::vector<Command> rb = record(b, Real(120));

    REQUIRE(ra.size() > 20);
    REQUIRE(ra.size() == rb.size());
    for (std::size_t i = 0; i < ra.size(); ++i) {
        CHECK(ra[i].vector.x == rb[i].vector.x);   // bit-exact, not approximate
        CHECK(ra[i].vector.y == rb[i].vector.y);
        CHECK(ra[i].tick == rb[i].tick);
    }
}

TEST_CASE("different seeds give different sequences") {
    const std::vector<Command> a = [] {
        BenchmarkIntentSource s({}, 1);
        return record(s, Real(120));
    }();
    const std::vector<Command> b = [] {
        BenchmarkIntentSource s({}, 2);
        return record(s, Real(120));
    }();

    bool differs = a.size() != b.size();
    for (std::size_t i = 0; !differs && i < a.size(); ++i) {
        differs = a[i].vector.x != b[i].vector.x || a[i].vector.y != b[i].vector.y ||
                  a[i].tick != b[i].tick;
    }
    CHECK(differs);
}

TEST_CASE("the sequence does not depend on the vehicle") {
    // The source is handed an Observation, so this is the check that it ignores
    // it. Two vehicles with wildly different authority must face the same task,
    // or nothing measured on one can be compared with anything measured on the
    // other.
    BenchmarkIntentSource a({}, 777);
    BenchmarkIntentSource b({}, 777);

    Observation light;
    light.maxAccel = Real(1);
    light.pos      = {Real(1000), Real(-500)};
    light.vel      = {Real(37), Real(9)};
    light.angle    = Real(2.1);

    Observation heavy;
    heavy.maxAccel = Real(400);

    const int steps = 4000;
    for (int i = 0; i < steps; ++i) {
        const Intent ia = a.intent(light);
        const Intent ib = b.intent(heavy);
        REQUIRE(ia.vector.x == ib.vector.x);
        REQUIRE(ia.vector.y == ib.vector.y);
        REQUIRE(a.commandIndex() == b.commandIndex());
    }
}

TEST_CASE("acceleration commands stay inside their band") {
    BenchmarkConfig cfg;
    cfg.mode = Intent::Mode::Acceleration;
    BenchmarkIntentSource source(cfg, 4242);

    const std::vector<Command> cmds = record(source, Real(600));
    REQUIRE(cmds.size() > 100);

    for (const Command& c : cmds) {
        CHECK(c.mode == Intent::Mode::Acceleration);
        const Real mag = length(c.vector);
        CHECK(mag >= cfg.accelMin);
        CHECK(mag <= cfg.accelMax);
        // facing stays empty throughout: where to point is the fly-by-wire's
        // problem, and that is exactly what is being measured.
        CHECK_FALSE(c.hasFacing);
    }
}

TEST_CASE("velocity commands stay inside their band") {
    BenchmarkConfig cfg;
    cfg.mode = Intent::Mode::Velocity;
    BenchmarkIntentSource source(cfg, 4242);

    const std::vector<Command> cmds = record(source, Real(600));
    REQUIRE(cmds.size() > 100);

    for (const Command& c : cmds) {
        CHECK(c.mode == Intent::Mode::Velocity);
        CHECK(length(c.vector) <= cfg.speedMax);
        CHECK_FALSE(c.hasFacing);
    }
}

TEST_CASE("hold intervals are whole ticks in [1, 5] seconds") {
    BenchmarkIntentSource source({}, 909);
    const std::vector<Command> cmds = record(source, Real(600));
    REQUIRE(cmds.size() > 100);

    // The last recorded command is still being held when the recording stops,
    // so its interval is unknown; every earlier one has a measured length.
    for (std::size_t i = 1; i + 1 < cmds.size(); ++i) {
        const Tick held = cmds[i + 1].tick - cmds[i].tick;
        CHECK(held >= static_cast<Tick>(Real(1) * kTickRate));
        CHECK(held <= static_cast<Tick>(Real(5) * kTickRate));
        // Quantised: a boundary can only land where a controller actually runs.
        CHECK(held % kControlEvery == 0);
    }
}

TEST_CASE("directions cover the circle") {
    // Loose on purpose -- this is a smoke test for "uniform on the circle", not
    // a test of splitmix64. All four quadrants and both axes get visited.
    BenchmarkIntentSource source({}, 31337);
    const std::vector<Command> cmds = record(source, Real(1200));

    int quadrant[4] = {0, 0, 0, 0};
    for (const Command& c : cmds) {
        const int q = (c.vector.x >= 0 ? 0 : 1) + (c.vector.y >= 0 ? 0 : 2);
        ++quadrant[q];
    }
    for (int q = 0; q < 4; ++q) CHECK(quadrant[q] > 0);
}

TEST_CASE("both modes share their directions and their timing") {
    // Same three draws per command in either mode, so an acceleration run and a
    // velocity run from one seed step at the same instants in the same
    // directions. That makes the two modes comparable without ever interleaving
    // them inside a run.
    BenchmarkConfig accel;
    accel.mode = Intent::Mode::Acceleration;
    BenchmarkConfig vel;
    vel.mode = Intent::Mode::Velocity;

    BenchmarkIntentSource a(accel, 55);
    BenchmarkIntentSource v(vel, 55);

    const std::vector<Command> ra = record(a, Real(300));
    const std::vector<Command> rv = record(v, Real(300));

    REQUIRE(ra.size() == rv.size());
    for (std::size_t i = 0; i < ra.size(); ++i) {
        CHECK(ra[i].tick == rv[i].tick);
        CHECK(angleOf(ra[i].vector) == doctest::Approx(angleOf(rv[i].vector)));
    }
}

TEST_CASE("switching mode restarts the sequence rather than interleaving") {
    BenchmarkIntentSource source({}, 606);
    Observation obs;
    obs.maxAccel = Real(40);

    for (int i = 0; i < 500; ++i) source.intent(obs);
    REQUIRE(source.elapsedTicks() > 0);

    source.setMode(Intent::Mode::Velocity);
    CHECK(source.elapsedTicks() == 0);
    CHECK(source.commandIndex() == 1);
    CHECK(source.intent(obs).mode == Intent::Mode::Velocity);
}

// ---------------------------------------------------------------------------
// The run: reproducibility tight, quality loose.
// ---------------------------------------------------------------------------

TEST_CASE("a benchmark run is reproducible") {
    for (Intent::Mode mode : {Intent::Mode::Acceleration, Intent::Mode::Velocity}) {
        const BenchmarkRun a = run(classic(), mode, 2024, Real(20));
        const BenchmarkRun b = run(classic(), mode, 2024, Real(20));

        CHECK(a.stateHash == b.stateHash);
        CHECK(a.ticks == b.ticks);
        CHECK(a.metrics.trackingErrorIntegral == b.metrics.trackingErrorIntegral);
        CHECK(a.metrics.impulse == b.metrics.impulse);
        CHECK(a.metrics.attitudeWander == b.metrics.attitudeWander);
        CHECK(a.metrics.settlingMean == b.metrics.settlingMean);
        CHECK(a.metrics.steps == b.metrics.steps);
    }
}

TEST_CASE("a different seed is a different run") {
    const BenchmarkRun a = run(classic(), Intent::Mode::Acceleration, 1, Real(20));
    const BenchmarkRun b = run(classic(), Intent::Mode::Acceleration, 2, Real(20));

    CHECK(a.stateHash != b.stateHash);
    CHECK(a.metrics.trackingErrorIntegral != b.metrics.trackingErrorIntegral);
}

TEST_CASE("a different vehicle flies the same task to a different result") {
    const BenchmarkRun classicRun = run(classic(), Intent::Mode::Velocity, 99, Real(20));
    const BenchmarkRun landerRun  = run(rocket("lander"), Intent::Mode::Velocity, 99, Real(20));

    // Same script, different airframe: the states must differ, and both must
    // have been scored over the same amount of simulated time.
    CHECK(classicRun.stateHash != landerRun.stateHash);
    CHECK(classicRun.metrics.seconds == doctest::Approx(landerRun.metrics.seconds));
    CHECK(classicRun.metrics.steps == landerRun.metrics.steps);
}

TEST_CASE("the metrics are plausible") {
    // Loose. The point is that each component is measuring something real, not
    // that any of them has reached a particular value.
    const BenchmarkRun r = run(classic(), Intent::Mode::Acceleration, 7, Real(60));
    const BenchmarkMetrics& m = r.metrics;

    CHECK(m.seconds == doctest::Approx(Real(60)).epsilon(0.01));
    CHECK(m.steps > 10);

    // Flying at all costs propellant and produces some tracking error.
    CHECK(m.impulse > Real(0));
    CHECK(m.trackingErrorIntegral > Real(0));
    CHECK(m.trackingErrorMean > Real(0));

    // ...but not an unbounded amount of either. A controller that simply never
    // fired would sit at roughly maxAccel of error the whole time; anything at
    // or above that is not tracking.
    CHECK(m.trackingErrorMean < classic().maxForwardAccel());
    CHECK(m.attitudeWander >= Real(0));
    CHECK(m.settlingMean >= Real(0));
    CHECK(m.settledFraction() >= Real(0));
    CHECK(m.settledFraction() <= Real(1));
    CHECK(std::isfinite(compositeScore(m)));
}

TEST_CASE("idle costs nothing and tracks nothing") {
    // The floor case, as a sanity check on the meter itself: a vehicle whose
    // fly-by-wire is never asked for anything burns no propellant. Built by
    // hand rather than through runBenchmark, because there is no controller.
    BenchmarkMonitor monitor;
    BenchmarkConfig  cfg;
    monitor.reset(classic(), cfg);

    World world(defaultWorld(classic()));

    Intent commanded;
    commanded.mode   = Intent::Mode::Acceleration;
    commanded.vector = {Real(1), Real(0)};

    for (int i = 0; i < 1000; ++i) {
        world.step(kTickDt);
        monitor.sample(commanded, 1, world.observe(0), kTickDt);
    }
    monitor.finish();

    CHECK(monitor.metrics().impulse == Real(0));
    CHECK(monitor.metrics().attitudeWander == Real(0));
    // Commanded full forward acceleration, achieved none.
    CHECK(monitor.metrics().trackingErrorMean ==
          doctest::Approx(classic().maxForwardAccel()).epsilon(0.001));
    // Never settled, so the step is charged its whole length.
    CHECK(monitor.metrics().steps == 1);
    CHECK(monitor.metrics().stepsSettled == 0);
    CHECK(monitor.metrics().settlingMean == doctest::Approx(Real(1.0)));
}

TEST_CASE("actual acceleration is read from the actuators, not from differencing") {
    // Fire the main engine open-loop and let the actuator ramp settle, then
    // check the meter's idea of the achieved acceleration against the closed
    // form. If it were differencing positions this would be off by the
    // integrator's error rather than exact to rounding.
    const RocketSpec& spec = classic();

    World world(defaultWorld(spec));

    ControlInput full;
    for (std::size_t i = 0; i < spec.count(); ++i) {
        if (dot(spec[i].unitDirection(), Vec2{Real(1), Real(0)}) > Real(0.7)) full.level[i] = Real(1);
    }

    for (int i = 0; i < 3000; ++i) {   // well past ignition and ramp-up
        world.setControl(0, full);
        world.step(kTickDt);
    }

    BenchmarkMonitor monitor;
    monitor.reset(spec, BenchmarkConfig{});

    // Command exactly what the vehicle is doing: the error should be at the
    // noise floor, not merely small.
    Intent commanded;
    commanded.mode   = Intent::Mode::Acceleration;
    commanded.vector = fromAngle(world.observe(0).angle);

    monitor.sample(commanded, 1, world.observe(0), kTickDt);
    CHECK(monitor.currentError() < Real(1e-9));
}
