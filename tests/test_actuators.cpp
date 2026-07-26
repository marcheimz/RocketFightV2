#include <doctest/doctest.h>

#include <cmath>

#include "TestRockets.hpp"
#include "core/World.hpp"

using namespace rf;
using rf::testing::classic;

namespace {

constexpr std::size_t kMain      = 0;
constexpr std::size_t kNosePlus  = 1;
constexpr std::size_t kTailMinus = 4;

ControlInput fire(std::initializer_list<std::size_t> thrusters, Real level = Real(1)) {
    ControlInput in;
    for (std::size_t i : thrusters) in.level[i] = level;
    return in;
}

// Run a bare rocket, no world, so these test the actuator model and nothing else.
Rocket bench() { return Rocket(classic(), {}, {}, Real(0), Real(0)); }

void advance(Rocket& r, Real seconds) {
    const auto ticks = static_cast<int>(std::lround(seconds / kTickDt));
    for (int i = 0; i < ticks; ++i) r.stepActuators(kTickDt);
}

}  // namespace

TEST_CASE("a thruster produces nothing until it has lit") {
    Rocket r = bench();
    r.setControls(fire({kMain}));

    const Real ignition = classic()[kMain].ignitionTime;
    REQUIRE(ignition > Real(0));

    advance(r, ignition * Real(0.8));
    CHECK(r.actuators()[kMain].thrust == doctest::Approx(0.0));
    CHECK_FALSE(r.actuators()[kMain].lit);

    advance(r, ignition * Real(0.4));
    CHECK(r.actuators()[kMain].lit);
    CHECK(r.actuators()[kMain].thrust > Real(0));
}

TEST_CASE("lighting delivers the floor, not a trickle") {
    Rocket r = bench();
    r.setControls(fire({kMain}));
    advance(r, classic()[kMain].ignitionTime + kTickDt);

    // A thruster that cannot throttle below its minimum cannot ease in below it
    // either: the moment it lights, it is already producing the floor.
    CHECK(r.actuators()[kMain].thrust >= classic()[kMain].minThrust);
}

TEST_CASE("thrust ramps at the configured rate rather than jumping") {
    Rocket r = bench();
    r.setControls(fire({kMain}));

    const Thruster& t = classic()[kMain];
    advance(r, t.ignitionTime);

    // From the floor to full at rampUpRate newtons per second.
    const Real expected = (t.maxThrust - t.minThrust) / t.rampUpRate;
    REQUIRE(expected > Real(0.05));  // the test would be vacuous otherwise

    advance(r, expected * Real(0.5));
    CHECK(r.actuators()[kMain].thrust < t.maxThrust * Real(0.95));

    advance(r, expected * Real(0.7));
    CHECK(r.actuators()[kMain].thrust == doctest::Approx(t.maxThrust).epsilon(0.001));
}

TEST_CASE("shutting down decays rather than vanishing, with no relight delay") {
    Rocket r = bench();
    r.setControls(fire({kMain}));
    advance(r, Real(2));
    REQUIRE(r.actuators()[kMain].thrust == doctest::Approx(classic()[kMain].maxThrust));

    r.setControls(ControlInput{});
    r.stepActuators(kTickDt);
    CHECK(r.actuators()[kMain].thrust > Real(0));       // still burning down
    CHECK_FALSE(r.actuators()[kMain].lit);

    advance(r, classic()[kMain].maxThrust / classic()[kMain].rampDownRate + Real(0.01));
    CHECK(r.actuators()[kMain].thrust == doctest::Approx(0.0));
}

TEST_CASE("a demand below the floor shuts the thruster down instead of easing it") {
    Rocket          r = bench();
    const Thruster& t = classic()[kMain];
    const Real      floor = t.minThrust / t.maxThrust;

    // Just under half the floor: not worth lighting for.
    r.setControls(fire({kMain}, floor * Real(0.4)));
    advance(r, Real(2));
    CHECK(r.actuators()[kMain].thrust == doctest::Approx(0.0));

    // Just over: the thruster lights, and delivers the floor -- more than was
    // asked for. This is the discreteness that makes allocation hard.
    r.setControls(fire({kMain}, floor * Real(0.8)));
    advance(r, Real(2));
    CHECK(r.actuators()[kMain].thrust == doctest::Approx(t.minThrust).epsilon(0.001));
}

TEST_CASE("the nozzle slews at a finite rate") {
    Rocket r = bench();

    ControlInput in = fire({kMain});
    in.gimbal[kMain] = Real(1);
    r.setControls(in);

    const Thruster& t = classic()[kMain];
    REQUIRE(t.gimbalRate > Real(0));

    const Real full = t.maxGimbal / t.gimbalRate;  // seconds to full deflection

    advance(r, full * Real(0.4));
    CHECK(r.actuators()[kMain].gimbalAngle < t.maxGimbal * Real(0.6));

    advance(r, full);
    CHECK(r.actuators()[kMain].gimbalAngle == doctest::Approx(t.maxGimbal).epsilon(0.01));
}

TEST_CASE("actuator lag is visible in the trajectory, not just the actuator") {
    // The point of all of the above: the vehicle really does accelerate later
    // than it was told to. Compare against the ideal, instantaneous answer.
    World world(defaultWorld(classic()));
    world.setControl(0, fire({kMain}));

    for (int i = 0; i < 1000; ++i) world.step(kTickDt);  // one second

    const Real ideal  = classic().maxForwardAccel();  // m/s after 1 s if instant
    const Real actual = length(world.rockets()[0].body().vel);

    CHECK(actual < ideal);          // lag is real
    CHECK(actual > ideal * Real(0.5));  // ...but the engine did light and ramp
}

TEST_CASE("opposed attitude thrusters still rotate without translating") {
    // Actuator dynamics must not break the property the geometry guarantees:
    // both thrusters in the pair have identical specs, so they ramp together and
    // their forces cancel throughout, not merely at the end.
    World world(defaultWorld(classic()));
    world.setControl(0, fire({kNosePlus, kTailMinus}));

    for (int i = 0; i < 2000; ++i) {
        world.step(kTickDt);
        CHECK(length(world.rockets()[0].body().vel) == doctest::Approx(0.0));
    }
    CHECK(world.rockets()[0].body().angVel > Real(1.0));
}

TEST_CASE("actuator state is part of determinism") {
    // Two worlds stepped the same number of times must agree; one stepped mid-
    // ignition must not be mistaken for the other.
    World a(defaultWorld(classic()));
    World b(defaultWorld(classic()));

    a.setControl(0, fire({kMain}));
    b.setControl(0, fire({kMain}));

    for (int i = 0; i < 100; ++i) {
        a.step(kTickDt);
        b.step(kTickDt);
    }
    CHECK(a.hash() == b.hash());

    // Same position and velocity are reachable while the engines differ, so the
    // hash has to cover the actuators to catch it.
    b.step(kTickDt);
    CHECK(a.hash() != b.hash());
}
