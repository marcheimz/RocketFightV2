#pragma once

#include <array>
#include <cstddef>

#include "control/Controller.hpp"
#include "core/Command.hpp"

namespace rf {

// The current position of every stick and button, rebuilt from the command
// stream. Lives on the simulation side: the render thread only forwards raw
// axis values, and what they *mean* is decided here.
struct InputState {
    std::array<Real, static_cast<std::size_t>(InputAxis::Count)>   axes{};
    std::array<bool, static_cast<std::size_t>(InputButton::Count)> buttons{};

    void apply(const Command& c);

    Real axis(InputAxis a) const { return axes[static_cast<std::size_t>(a)]; }
    bool button(InputButton b) const { return buttons[static_cast<std::size_t>(b)]; }
};

// Radial deadzone that rescales the remainder, so the stick still reaches full
// deflection and there is no step at the edge of the dead region.
Vec2 applyDeadzone(Vec2 stick, Real deadzone);

// ---------------------------------------------------------------------------
// Layer 1, human. An analogue stick maps onto Intent almost exactly: its
// direction is where the pilot wants to accelerate and its magnitude is how
// hard, which is precisely what Intent::Mode::Acceleration means.
// ---------------------------------------------------------------------------
class GamepadIntentSource final : public IntentSource {
public:
    explicit GamepadIntentSource(const InputState& state) : state_(&state) {}

    Intent intent(const Observation& obs) override;

    // Which of Intent's two modes the stick means. Latched on the simulation
    // side, because a toggle is an edge and InputState only carries levels --
    // and because the benchmark has to be able to fly in the same mode the pilot
    // is in, which is a decision neither source can make alone.
    void         setMode(Intent::Mode m) { mode_ = m; }
    Intent::Mode mode() const { return mode_; }

private:
    const InputState* state_;
    Intent::Mode      mode_{Intent::Mode::Acceleration};
};

// ---------------------------------------------------------------------------
// Layer 3, human. Raw actuator control with the fly-by-wire bypassed: the
// debugging path, and the "fly it yourself" path. Not the default.
//
// It still has to know the layout, because "turn left" means firing whichever
// thrusters happen to produce a positive torque on *this* vehicle. What makes it
// direct is that there is no closed loop and no allocation: the stick opens
// thruster groups, and whatever the ship then does is the pilot's problem.
// ---------------------------------------------------------------------------
class DirectHumanController final : public Controller {
public:
    explicit DirectHumanController(const InputState& state) : state_(&state) {}

    ControlInput evaluate(const Observation& obs) override;
    void         reset(std::uint64_t, const WorldConfig& env) override;

private:
    const InputState* state_;
    RocketSpec        spec_{};
};

}  // namespace rf
