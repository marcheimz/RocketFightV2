#include <doctest/doctest.h>

#include <cmath>
#include <memory>
#include <string>

#include "TestRockets.hpp"
#include "control/RocketFlyByWire.hpp"
#include "control/ThrustAllocator.hpp"
#include "core/World.hpp"

using namespace rf;
using rf::testing::classic;
using rf::testing::rocket;

// Layout and allocator facts are asserted tightly -- they are arithmetic. The
// closed-loop flying checks below are deliberately loose: they should catch
// "this vehicle cannot be flown at all", not pin a particular set of gains that
// is expected to be optimised later.

namespace {

void fly(World& world, Controller& controller, Real seconds) {
    const auto   ticks = static_cast<int>(seconds / kTickDt);
    ControlInput held{};
    for (int i = 0; i < ticks; ++i) {
        if (world.tick() % kControlEvery == 0) held = controller.evaluate(world.observe(0));
        world.setControl(0, held);
        world.step(kTickDt);
    }
}

std::unique_ptr<Controller> pilot(const RocketSpec& spec, std::unique_ptr<IntentSource> source) {
    return std::make_unique<LayeredController>(std::move(source), makeFlyByWire(spec));
}

class FixedIntent final : public IntentSource {
public:
    explicit FixedIntent(Intent i) : intent_(i) {}
    Intent intent(const Observation&) override { return intent_; }

private:
    Intent intent_;
};

}  // namespace

TEST_CASE("the classic layout matches the hand-written rocket it replaced") {
    const RocketSpec& s = classic();

    CHECK(s.mass == doctest::Approx(1000.0));
    CHECK(s.inertia() == doctest::Approx(12750.0));
    CHECK(s.maxForwardAccel() == doctest::Approx(40.0));

    // The original model postulated 25 kN.m of RCS torque. Here it emerges from
    // two opposed pairs at a 5 m arm, and must still come out the same.
    CHECK(s.maxAngAccel() == doctest::Approx(25000.0 / 12750.0));
    CHECK(s.mustPointToThrust());
}

TEST_CASE("each rocket has the character its layout implies") {
    CHECK(rocket("norcs").maxAngAccel() == doctest::Approx(0.0));
    CHECK(rocket("norcs").maxGimbalAngAccel() > Real(0));
    CHECK(rocket("norcs").mustPointToThrust());

    // The lander can shove itself sideways hard enough that it need not turn.
    CHECK(rocket("lander").maxLateralAccel() > Real(5));
    CHECK_FALSE(rocket("lander").mustPointToThrust());

    // Twin mains either side of the centreline: differential throttle is torque.
    CHECK(rocket("interceptor").maxForwardAccel() > Real(70));
    CHECK(rocket("interceptor").maxAngAccel() > classic().maxAngAccel());
}

TEST_CASE("the allocator produces pure torque from opposed pairs") {
    const ThrustAllocator alloc(classic());

    ControlInput out;
    alloc.solve({Real(0), Real(0)}, Real(20000), out);

    Vec2 force{};
    Real torque = 0;
    alloc.achieved(out, force, torque);

    CHECK(torque == doctest::Approx(20000.0).epsilon(0.05));
    // Minimum-norm: asking for torque alone gets the balanced pair, so there is
    // no residual shove. Nothing told it to balance -- one-sided firing simply
    // costs more and leaves a force the solution is penalised for.
    CHECK(length(force) < Real(1.0));
}

TEST_CASE("the allocator respects the thrust floor") {
    const ThrustAllocator alloc(classic());

    // Ask for a torque far below what one attitude thruster produces at its
    // minimum. The vehicle cannot comply, so the honest outcomes are "off" or
    // "the floor" -- never something in between, which the simulation would
    // otherwise round behind the controller's back.
    ControlInput out;
    alloc.solve({Real(0), Real(0)}, Real(50), out);

    for (std::size_t i = 0; i < classic().count(); ++i) {
        const Thruster& t = classic()[i];
        if (t.minThrust <= Real(0)) continue;
        const Real floor = t.minThrust / t.maxThrust;
        INFO("thruster " << i << " level " << out.level[i] << " floor " << floor);
        CHECK((out.level[i] == Real(0) || out.level[i] >= floor - Real(1e-9)));
    }
}

TEST_CASE("the allocator refuses what a vehicle cannot do, rather than faking it") {
    // The classic rocket has no way to push sideways while staying pointed
    // forward. Damped least squares returns the closest achievable answer,
    // which here is almost nothing -- not a confident wrong one.
    const ThrustAllocator alloc(classic());

    ControlInput out;
    alloc.solve({Real(0), Real(30000)}, Real(0), out);

    Vec2 force{};
    Real torque = 0;
    alloc.achieved(out, force, torque);
    CHECK(std::abs(force.y) < Real(6000));
}

TEST_CASE("a gimbal-only rocket still turns, by burning to do it") {
    // norcs has zero attitude authority with the engine cold. The fly-by-wire
    // has to notice that and light the engine in order to steer at all -- then
    // wait out an ignition delay and a slow nozzle before anything happens.
    Intent want;
    want.facing = Real(0);

    World world(defaultWorld(rocket("norcs")));  // starts nose-up
    auto  controller = pilot(rocket("norcs"), std::make_unique<FixedIntent>(want));

    fly(world, *controller, Real(20));

    CHECK(std::abs(wrapPi(world.rockets()[0].body().angle)) < Real(0.4));
}

TEST_CASE("a lander translates without turning to face where it is going") {
    // The whole point of reading mustPointToThrust() from the layout: this
    // vehicle should slide sideways and keep its attitude, where the classic
    // rocket would have rotated first.
    Intent want;
    want.mode   = Intent::Mode::Acceleration;
    want.vector = {Real(1), Real(0)};  // push along world +x

    WorldConfig cfg = defaultWorld(rocket("lander"));
    cfg.rockets.front().angle = kPi / Real(2);  // nose up, thrust wanted to the right
    World world(cfg);

    auto controller = pilot(rocket("lander"), std::make_unique<FixedIntent>(want));
    fly(world, *controller, Real(8));

    const Body& b = world.rockets()[0].body();
    CHECK(b.vel.x > Real(5));                                     // it went where asked
    CHECK(std::abs(wrapPi(b.angle - kPi / Real(2))) < Real(0.5)); // without turning to do it
}

TEST_CASE("every rocket can hold an attitude despite its actuator lag") {
    for (const RocketSpec& spec : RocketCatalogue::instance().all()) {
        const std::string vehicle(spec.name.view());
        INFO("rocket: " << vehicle);

        Intent want;
        want.facing = Real(0);

        World world(defaultWorld(spec));
        auto  controller = pilot(spec, std::make_unique<FixedIntent>(want));
        fly(world, *controller, Real(30));

        CHECK(std::abs(wrapPi(world.rockets()[0].body().angle)) < Real(0.5));
        CHECK(std::abs(world.rockets()[0].body().angVel) < Real(0.5));
    }
}

TEST_CASE("the fly-by-wire admits how tightly it can actually hold") {
    // A vehicle whose thrusters cannot be throttled arbitrarily low cannot hold
    // an arbitrarily tight attitude. The controller names that limit rather than
    // pretending to chase zero, which is what stops it limit-cycling.
    const RocketFlyByWire fbw(classic());
    CHECK(fbw.attitudeDeadband() > Real(0));
    CHECK(fbw.attitudeDeadband() < Real(0.1));   // small, not a shrug
}
