#include "control/HumanInput.hpp"

#include <algorithm>
#include <cmath>

#include "control/Benchmark.hpp"   // kBenchmarkMaxSpeed: one band, both pilots

namespace rf {

namespace {
// Below this the right stick is treated as "no opinion about facing" rather
// than as a heading command, so letting go hands attitude back to the
// fly-by-wire instead of pinning the nose to wherever the stick last drifted.
constexpr Real kAimDeadzone  = Real(0.5);
constexpr Real kMoveDeadzone = Real(0.15);
}  // namespace

void InputState::apply(const Command& c) {
    switch (c.type) {
        case Command::Type::Axis:
            axes[static_cast<std::size_t>(c.axis)] = c.value;
            break;
        case Command::Type::ButtonDown:
            buttons[static_cast<std::size_t>(c.button)] = true;
            break;
        case Command::Type::ButtonUp:
            buttons[static_cast<std::size_t>(c.button)] = false;
            break;
    }
}

Vec2 applyDeadzone(Vec2 stick, Real deadzone) {
    const Real len = length(stick);
    if (len <= deadzone) return {};
    const Real scaled = (len - deadzone) / (Real(1) - deadzone);
    return stick * (clamp(scaled, Real(0), Real(1)) / len);
}

Intent GamepadIntentSource::intent(const Observation&) {
    Intent out;

    const Vec2 stick = applyDeadzone(
        {state_->axis(InputAxis::MoveX), state_->axis(InputAxis::MoveY)}, kMoveDeadzone);

    if (state_->button(InputButton::KillVelocity)) {
        // Match velocity with the origin frame. One button, and the fly-by-wire
        // works out which way to point and how hard to burn. Momentary, and it
        // wins in either mode -- it is the one command a pilot reaches for
        // without wanting to think about what mode they left the ship in.
        out.mode   = Intent::Mode::Velocity;
        out.vector = {};
    } else if (mode_ == Intent::Mode::Velocity) {
        // Full deflection is the top of the same band the benchmark samples, so
        // hand-flying and the scripted run are the same task at the same scale.
        out.mode   = Intent::Mode::Velocity;
        out.vector = stick * kBenchmarkMaxSpeed;
    } else {
        out.mode   = Intent::Mode::Acceleration;
        out.vector = stick;
    }

    const Vec2 aim{state_->axis(InputAxis::AimX), state_->axis(InputAxis::AimY)};
    if (length(aim) > kAimDeadzone) {
        out.facing = angleOf(aim);
    }

    out.fire = state_->button(InputButton::Fire);
    return out;
}

void DirectHumanController::reset(std::uint64_t, const WorldConfig& env) {
    if (!env.rockets.empty()) spec_ = env.rockets.front().spec;
}

ControlInput DirectHumanController::evaluate(const Observation&) {
    ControlInput out;

    const Real throttle = clamp(state_->axis(InputAxis::TriggerR), Real(0), Real(1));
    // Stick right should swing the nose right. Right is clockwise, and clockwise
    // is a negative angular rate in a y-up world.
    const Real turn   = -clamp(state_->axis(InputAxis::MoveX), Real(-1), Real(1));
    const Real gimbal = clamp(state_->axis(InputAxis::AimX), Real(-1), Real(1));

    for (std::size_t i = 0; i < spec_.count(); ++i) {
        const Thruster& t       = spec_[i];
        const Real      forward = dot(t.unitDirection(), Vec2{Real(1), Real(0)});

        // Main engines answer the trigger.
        if (forward > Real(0.7)) {
            out.level[i] = throttle;
        } else if (turn != Real(0) && sign(t.leverArm()) == sign(turn)) {
            // Everything else is an attitude thruster: open the ones pushing the
            // way the pilot asked. On a symmetric layout that is the opposed
            // pair, so the ship rotates without translating -- not because
            // anything balanced it, but because the geometry does.
            out.level[i] = std::abs(turn);
        }

        if (t.maxGimbal > Real(0)) out.gimbal[i] = gimbal;
    }

    out.fire = state_->button(InputButton::Fire);
    return out;
}

}  // namespace rf
