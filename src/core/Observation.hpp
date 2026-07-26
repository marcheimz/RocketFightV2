#pragma once

#include <array>

#include "core/RocketSpec.hpp"
#include "core/Types.hpp"
#include "core/Vec2.hpp"

namespace rf {

// What a controller is allowed to see. A value type, never a pointer into the
// world: controllers cannot mutate the simulation, cannot see more than the
// contract allows, and the contract stays serialisable -- which is what will
// later permit out-of-process or non-C++ controllers.
//
// Static environment data (attractors) arrives once via Controller::reset, so
// this stays small and allocation-free on the per-step path.
struct Observation {
    Tick tick{};
    Real time{};

    Vec2 pos{};
    Vec2 vel{};
    Real angle{};
    Real angVel{};

    // Vehicle limits, so a policy can normalise its output without a separate
    // handshake and without hardcoding the rocket it was written for.
    Real maxAccel{};
    Real maxAngAccel{};

    // What the actuators are actually doing, which is not what they were last
    // told to do. A controller that ignores this is flying blind through its own
    // ignition delays and ramp rates -- it would keep commanding a correction
    // that is already on its way, and overshoot every time.
    std::size_t                              thrusterCount{};
    std::array<ThrusterState, kMaxThrusters> actuators{};
};

}  // namespace rf
