#pragma once

#include <cstdint>

#include "core/Types.hpp"

namespace rf {

// Commands are deliberately *semantically empty*. The render thread translates
// SFML events into these and nothing more; deciding what "left stick pushed
// forward" means belongs to the control layer, which lives on the sim side.
//
// That split is what keeps the input mapping testable headlessly and keeps the
// renderer from quietly becoming a second place where flight logic lives.
enum class InputAxis : std::uint8_t {
    MoveX,      // left stick, world-frame right
    MoveY,      // left stick, world-frame up
    AimX,       // right stick
    AimY,
    TriggerL,   // [0, 1]
    TriggerR,   // [0, 1]
    Count,
};

enum class InputButton : std::uint8_t {
    Fire,
    KillVelocity,   // held: Intent::Mode::Velocity with a target of zero
    ToggleDirect,   // fly-by-wire <-> raw manual control
    Reset,
    // Step through the vehicles the app handed the simulation. Which vehicles
    // those are, and what "next" means, is decided on the simulation side; the
    // translator only knows a button went down.
    NextVehicle,
    PrevVehicle,
    Count,
};

struct Command {
    enum class Type : std::uint8_t { Axis, ButtonDown, ButtonUp };

    Type        type{Type::Axis};
    InputAxis   axis{InputAxis::MoveX};
    InputButton button{InputButton::Fire};
    Real        value{};

    static Command makeAxis(InputAxis a, Real v) {
        Command c;
        c.type  = Type::Axis;
        c.axis  = a;
        c.value = v;
        return c;
    }

    static Command makeButton(InputButton b, bool down) {
        Command c;
        c.type   = down ? Type::ButtonDown : Type::ButtonUp;
        c.button = b;
        return c;
    }
};

}  // namespace rf
