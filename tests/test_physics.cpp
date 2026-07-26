#include <doctest/doctest.h>

#include <cmath>

#include "core/World.hpp"

using namespace rf;

namespace {

World makeWorld(WorldConfig cfg) { return World(std::move(cfg)); }

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

TEST_CASE("gimballed thrust rotates the rocket, axial thrust does not") {
    World world = makeWorld(defaultWorld());

    ControlInput straight;
    straight.throttle = 1.0;
    world.setControl(0, straight);
    for (int i = 0; i < 1000; ++i) world.step(kTickDt);

    CHECK(world.rockets()[0].body.angVel == doctest::Approx(0.0));
    // One second at 40 m/s^2, nose up.
    CHECK(world.rockets()[0].body.vel.y == doctest::Approx(40.0).epsilon(0.001));

    World deflected = makeWorld(defaultWorld());
    ControlInput gimballed;
    gimballed.throttle = 1.0;
    gimballed.gimbal   = 1.0;
    deflected.setControl(0, gimballed);
    for (int i = 0; i < 1000; ++i) deflected.step(kTickDt);

    // Positive deflection swings the tail, so the body rotates clockwise.
    CHECK(deflected.rockets()[0].body.angVel < Real(-0.1));
}

TEST_CASE("attitude thrusters apply torque without net force") {
    World world = makeWorld(defaultWorld());

    ControlInput spin;
    spin.rcs = 1.0;
    world.setControl(0, spin);
    for (int i = 0; i < 1000; ++i) world.step(kTickDt);

    const Body& b = world.rockets()[0].body;
    CHECK(b.angVel > Real(1.0));
    CHECK(length(b.vel) == doctest::Approx(0.0));
    CHECK(length(b.pos) == doctest::Approx(0.0));
}

TEST_CASE("zero-g means exactly zero drift") {
    World world = makeWorld(defaultWorld());
    for (int i = 0; i < 10'000; ++i) world.step(kTickDt);

    const Body& b = world.rockets()[0].body;
    CHECK(b.pos.x == Real(0));
    CHECK(b.pos.y == Real(0));
    CHECK(b.vel.x == Real(0));
    CHECK(b.vel.y == Real(0));
}

TEST_CASE("a circular orbit stays circular") {
    // The integrator's real test. Semi-implicit Euler is symplectic, so orbital
    // radius should wobble by a bounded amount and come back -- unlike explicit
    // Euler, which spirals outwards without limit.
    const WorldConfig cfg = orbitWorld();
    World world = makeWorld(cfg);

    const Real r0 = length(world.rockets()[0].body.pos);
    const Real mu = cfg.attractors.front().mu;
    const Real period = Real(2) * kPi * std::sqrt(r0 * r0 * r0 / mu);

    const auto ticks = static_cast<int>(period / kTickDt);

    Real minR = r0;
    Real maxR = r0;
    for (int i = 0; i < ticks; ++i) {
        world.step(kTickDt);
        const Real r = length(world.rockets()[0].body.pos);
        minR = std::min(minR, r);
        maxR = std::max(maxR, r);
    }

    // Radius held to a tenth of a percent over a full revolution.
    CHECK((maxR - minR) / r0 < Real(0.001));

    // And it comes back to where it started, rather than merely staying at the
    // right distance.
    const Real closingError = length(world.rockets()[0].body.pos - cfg.rockets.front().pos);
    CHECK(closingError / r0 < Real(0.01));
}

TEST_CASE("out of bounds is detected at the configured radius") {
    WorldConfig cfg = defaultWorld();
    cfg.boundsRadius = Real(1000);
    cfg.rockets.front().vel = {Real(500), Real(0)};

    World world = makeWorld(cfg);
    CHECK_FALSE(world.anyOutOfBounds());

    for (int i = 0; i < 3000; ++i) world.step(kTickDt);  // 3 s at 500 m/s
    CHECK(world.anyOutOfBounds());
}
