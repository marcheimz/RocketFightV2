#include <doctest/doctest.h>

#include <cmath>

#include "control/HumanInput.hpp"
#include "control/RocketFlyByWire.hpp"
#include "control/ScriptedControllers.hpp"
#include "core/World.hpp"

using namespace rf;

namespace {

// Fly a world with a controller for a number of seconds, at the same control
// decimation the real loop uses.
void fly(World& world, Controller& controller, Real seconds) {
    const auto ticks = static_cast<int>(seconds / kTickDt);
    ControlInput held{};
    for (int i = 0; i < ticks; ++i) {
        if (world.tick() % kControlEvery == 0) held = controller.evaluate(world.observe(0));
        world.setControl(0, held);
        world.step(kTickDt);
    }
}

std::unique_ptr<Controller> layered(std::unique_ptr<IntentSource> source) {
    return std::make_unique<LayeredController>(std::move(source),
                                               std::make_unique<RocketFlyByWire>());
}

// An IntentSource that just returns whatever it was given. Lets a test command
// an Intent directly without going through a gamepad.
class FixedIntent final : public IntentSource {
public:
    explicit FixedIntent(Intent i) : intent_(i) {}
    Intent intent(const Observation&) override { return intent_; }

private:
    Intent intent_;
};

}  // namespace

TEST_CASE("the fly-by-wire points the nose at the requested acceleration") {
    Intent want;
    want.mode   = Intent::Mode::Acceleration;
    want.vector = {Real(1), Real(0)};  // full thrust, world +x

    World world(defaultWorld());  // starts nose-up, so this needs a 90 degree slew
    auto controller = layered(std::make_unique<FixedIntent>(want));

    fly(world, *controller, Real(6));

    const Body& b = world.rockets()[0].body;
    CHECK(std::abs(wrapPi(b.angle)) < Real(0.05));      // pointing along +x
    CHECK(std::abs(b.angVel) < Real(0.05));             // and settled, not spinning
    CHECK(b.vel.x > Real(50));                          // actually accelerated
    CHECK(std::abs(b.vel.y) < std::abs(b.vel.x) * Real(0.35));
}

TEST_CASE("attitude settles without oscillating") {
    Intent want;
    want.facing = Real(0);  // hold zero heading, no acceleration requested

    World world(defaultWorld());
    auto controller = layered(std::make_unique<FixedIntent>(want));

    fly(world, *controller, Real(5));

    // Sample the second half of the manoeuvre: if the loop were ringing, the
    // heading error would keep changing sign with meaningful amplitude.
    Real worstError = 0;
    ControlInput held{};
    for (int i = 0; i < 3000; ++i) {
        if (world.tick() % kControlEvery == 0) held = controller->evaluate(world.observe(0));
        world.setControl(0, held);
        world.step(kTickDt);
        worstError = std::max(worstError, std::abs(wrapPi(world.rockets()[0].body.angle)));
    }
    CHECK(worstError < Real(0.02));
}

TEST_CASE("velocity mode brings the rocket to a stop") {
    WorldConfig cfg = defaultWorld();
    cfg.rockets.front().vel = {Real(80), Real(-30)};
    World world(cfg);

    Intent kill;
    kill.mode   = Intent::Mode::Velocity;
    kill.vector = {};  // target: at rest

    auto controller = layered(std::make_unique<FixedIntent>(kill));
    fly(world, *controller, Real(15));

    CHECK(length(world.rockets()[0].body.vel) < Real(1.0));
}

TEST_CASE("the fly-by-wire refuses to thrust while pointing the wrong way") {
    // Commanding a facing that fights the requested acceleration is physically
    // impossible for a single-engine vehicle. It must not quietly accelerate in
    // the wrong direction.
    Intent conflicting;
    conflicting.mode   = Intent::Mode::Acceleration;
    conflicting.vector = {Real(1), Real(0)};   // push along +x
    conflicting.facing = kPi;                  // but point along -x

    World world(defaultWorld());
    auto controller = layered(std::make_unique<FixedIntent>(conflicting));
    fly(world, *controller, Real(8));

    const Body& b = world.rockets()[0].body;
    CHECK(std::abs(wrapPi(b.angle - kPi)) < Real(0.05));  // obeyed the facing
    CHECK(b.vel.x <= Real(0.001));                        // and did not go +x
}

TEST_CASE("an intent-level policy reaches its target") {
    // The point of layer 1: this policy never mentions a thruster, and still
    // flies the vehicle.
    World world(defaultWorld());
    auto controller = layered(std::make_unique<SeekPointIntentSource>(Vec2{Real(500), Real(300)}));

    fly(world, *controller, Real(40));

    const Vec2 pos = world.rockets()[0].body.pos;
    CHECK(length(pos - Vec2{Real(500), Real(300)}) < Real(20));
    CHECK(length(world.rockets()[0].body.vel) < Real(5));
}

TEST_CASE("stick deadzone removes drift but still reaches full deflection") {
    CHECK(length(applyDeadzone({Real(0.1), Real(0)}, Real(0.15))) == doctest::Approx(0.0));
    CHECK(length(applyDeadzone({Real(1.0), Real(0)}, Real(0.15))) == doctest::Approx(1.0));

    // Direction must survive the rescaling, or the ship would drift off the
    // commanded heading near the deadzone edge.
    const Vec2 in{Real(0.6), Real(0.8)};  // length 1
    const Vec2 out = applyDeadzone(in, Real(0.15));
    CHECK(cross(in, out) == doctest::Approx(0.0));
}

TEST_CASE("gamepad sticks map onto intent as the pilot expects") {
    InputState state;
    state.apply(Command::makeAxis(InputAxis::MoveX, Real(0)));
    state.apply(Command::makeAxis(InputAxis::MoveY, Real(1)));

    GamepadIntentSource source(state);
    Observation obs;
    obs.maxAccel = Real(40);

    const Intent up = source.intent(obs);
    CHECK(up.mode == Intent::Mode::Acceleration);
    CHECK(up.vector.y > Real(0.8));   // stick up means accelerate up
    CHECK_FALSE(up.facing.has_value());  // right stick centred: no facing opinion

    state.apply(Command::makeButton(InputButton::KillVelocity, true));
    const Intent killing = source.intent(obs);
    CHECK(killing.mode == Intent::Mode::Velocity);
    CHECK(length(killing.vector) == doctest::Approx(0.0));
}

TEST_CASE("control decimation holds its output between evaluations") {
    // Proves the zero-order hold is real: a controller evaluated every tenth
    // tick must still be actuating the rocket on the other nine.
    World world(defaultWorld());
    Intent want;
    want.mode   = Intent::Mode::Acceleration;
    want.vector = {Real(0), Real(1)};  // straight up, already the starting heading

    auto controller = layered(std::make_unique<FixedIntent>(want));

    ControlInput held{};
    int evaluations = 0;
    for (int i = 0; i < 1000; ++i) {
        if (world.tick() % kControlEvery == 0) {
            held = controller->evaluate(world.observe(0));
            ++evaluations;
        }
        world.setControl(0, held);
        world.step(kTickDt);
    }

    CHECK(evaluations == 100);                                  // 100 Hz, not 1000
    CHECK(world.rockets()[0].body.vel.y > Real(35));            // thrusting throughout
}
