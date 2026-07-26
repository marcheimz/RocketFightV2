#include <doctest/doctest.h>

#include <cmath>

#include "TestRockets.hpp"
#include "core/World.hpp"

using namespace rf;
using rf::testing::classic;

namespace {

// Thruster indices for the classic layout: main engine, then the attitude quad
// as nose-left, nose-right, tail-left, tail-right.
constexpr std::size_t kMain     = 0;
constexpr std::size_t kNosePlus = 1;
constexpr std::size_t kTailMinus = 4;

ControlInput fire(std::initializer_list<std::size_t> thrusters, Real level = Real(1)) {
    ControlInput in;
    for (std::size_t i : thrusters) in.level[i] = level;
    return in;
}

void run(World& world, const ControlInput& in, int ticks) {
    world.setControl(0, in);
    for (int i = 0; i < ticks; ++i) world.step(kTickDt);
}

}  // namespace

TEST_CASE("a force through the centre of mass produces no torque") {
    Body b;
    b.invMass    = Real(1) / Real(1000);
    b.invInertia = Real(1) / Real(12750);
    b.angle      = Real(0.7);

    // The force is in world space, so an *axial* push on a body rotated to 0.7
    // rad has to be rotated too. Getting this wrong is the classic way to make
    // a rocket mysteriously spin up under pure forward thrust.
    b.applyForceAt(rotate({100, 0}, b.angle), {Real(-6), Real(0)});
    CHECK(b.torque == doctest::Approx(0.0));

    b.clearForces();
    b.applyForceAt(rotate({0, 100}, b.angle), {Real(-6), Real(0)});  // sideways at the tail
    CHECK(b.torque == doctest::Approx(-600.0));

    // Torque must not depend on the body's orientation at all.
    for (Real angle : {Real(0), Real(1.3), Real(-2.9)}) {
        Body probe;
        probe.angle = angle;
        probe.applyForceAt(rotate({0, 100}, angle), {Real(-6), Real(0)});
        CHECK(probe.torque == doctest::Approx(-600.0));
    }
}

TEST_CASE("the main engine pushes without turning") {
    World world(defaultWorld(classic()));
    run(world, fire({kMain}), 3000);

    CHECK(world.rockets()[0].body().angVel == doctest::Approx(0.0));

    // Measure the steady acceleration once the engine has lit and ramped, rather
    // than assuming thrust appeared the instant it was commanded.
    const Real before = world.rockets()[0].body().vel.y;
    for (int i = 0; i < 500; ++i) world.step(kTickDt);
    const Real accel = (world.rockets()[0].body().vel.y - before) / Real(0.5);

    CHECK(accel == doctest::Approx(classic().maxForwardAccel()).epsilon(0.001));
}

TEST_CASE("a deflected nozzle turns the rocket") {
    World world(defaultWorld(classic()));
    ControlInput in = fire({kMain});
    in.gimbal[kMain] = Real(1);
    run(world, in, 3000);

    // Positive deflection swings the tail, so the body rotates clockwise.
    CHECK(world.rockets()[0].body().angVel < Real(-0.1));
}

TEST_CASE("opposed attitude thrusters rotate without translating") {
    // This is the property the old hardcoded 'rcs is a pure torque' field was
    // asserting. Here nothing asserts it: the forces cancel because of where the
    // thrusters are, and the torques add for the same reason.
    World world(defaultWorld(classic()));
    run(world, fire({kNosePlus, kTailMinus}), 2000);

    const Body& b = world.rockets()[0].body();
    CHECK(b.angVel > Real(1.0));
    CHECK(length(b.vel) == doctest::Approx(0.0));
    CHECK(length(b.pos) == doctest::Approx(0.0));
}

TEST_CASE("a single attitude thruster both turns and shoves") {
    // The flip side: fire only one and the vehicle translates too. Nothing in
    // the model prevents it, which is what makes asymmetric layouts meaningful.
    World world(defaultWorld(classic()));
    run(world, fire({kNosePlus}), 2000);

    const Body& b = world.rockets()[0].body();
    CHECK(b.angVel > Real(0.5));
    CHECK(length(b.vel) > Real(1.0));
}

TEST_CASE("zero-g means exactly zero drift") {
    World world(defaultWorld(classic()));
    for (int i = 0; i < 10'000; ++i) world.step(kTickDt);

    const Body& b = world.rockets()[0].body();
    CHECK(b.pos.x == Real(0));
    CHECK(b.pos.y == Real(0));
    CHECK(b.vel.x == Real(0));
    CHECK(b.vel.y == Real(0));
}

TEST_CASE("a circular orbit stays circular") {
    // The integrator's real test. Semi-implicit Euler is symplectic, so orbital
    // radius should wobble by a bounded amount and come back -- unlike explicit
    // Euler, which spirals outwards without limit.
    const WorldConfig cfg = orbitWorld(classic());
    World world(cfg);

    const Real r0     = length(world.rockets()[0].body().pos);
    const Real mu     = cfg.attractors.front().mu;
    const Real period = Real(2) * kPi * std::sqrt(r0 * r0 * r0 / mu);

    const auto ticks = static_cast<int>(period / kTickDt);

    Real minR = r0;
    Real maxR = r0;
    for (int i = 0; i < ticks; ++i) {
        world.step(kTickDt);
        const Real r = length(world.rockets()[0].body().pos);
        minR = std::min(minR, r);
        maxR = std::max(maxR, r);
    }

    // Radius held to a tenth of a percent over a full revolution.
    CHECK((maxR - minR) / r0 < Real(0.001));

    // And it comes back to where it started, rather than merely staying at the
    // right distance.
    const Real closingError = length(world.rockets()[0].body().pos - cfg.rockets.front().pos);
    CHECK(closingError / r0 < Real(0.01));
}

TEST_CASE("out of bounds is detected at the configured radius") {
    WorldConfig cfg = defaultWorld(classic());
    cfg.boundsRadius = Real(1000);
    cfg.rockets.front().vel = {Real(500), Real(0)};

    World world(cfg);
    CHECK_FALSE(world.anyOutOfBounds());

    for (int i = 0; i < 3000; ++i) world.step(kTickDt);  // 3 s at 500 m/s
    CHECK(world.anyOutOfBounds());
}
