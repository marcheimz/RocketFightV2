#include "control/HumanInput.hpp"

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

    if (state_->button(InputButton::KillVelocity)) {
        // Match velocity with the origin frame. One button, and the fly-by-wire
        // works out which way to point and how hard to burn.
        out.mode   = Intent::Mode::Velocity;
        out.vector = {};
    } else {
        out.mode   = Intent::Mode::Acceleration;
        out.vector = applyDeadzone({state_->axis(InputAxis::MoveX), state_->axis(InputAxis::MoveY)},
                                   kMoveDeadzone);
    }

    const Vec2 aim{state_->axis(InputAxis::AimX), state_->axis(InputAxis::AimY)};
    if (length(aim) > kAimDeadzone) {
        out.facing = angleOf(aim);
    }

    out.fire = state_->button(InputButton::Fire);
    return out;
}

ControlInput DirectHumanController::evaluate(const Observation&) {
    ControlInput out;
    out.throttle = clamp(state_->axis(InputAxis::TriggerR), Real(0), Real(1));

    // Stick right should swing the nose right. Right is clockwise, and clockwise
    // is a negative angular rate in a y-up world.
    out.rcs    = -clamp(state_->axis(InputAxis::MoveX), Real(-1), Real(1));
    out.gimbal = clamp(state_->axis(InputAxis::AimX), Real(-1), Real(1));
    out.fire   = state_->button(InputButton::Fire);
    return out;
}

}  // namespace rf
