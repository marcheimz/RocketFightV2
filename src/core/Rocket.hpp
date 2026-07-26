#pragma once

#include <array>

#include "core/Body.hpp"
#include "core/ControlInput.hpp"
#include "core/RocketSpec.hpp"

namespace rf {

// A vehicle: rigid-body state, the layout that defines what it can do, the
// actuator commands currently applied to it, and -- crucially -- what the
// actuators are actually doing about it.
//
// The gap between those last two is the point. A ControlInput is a *demand*.
// Thrust lags it: the thruster has to light, then ramp, and it cannot sit below
// its minimum. Nozzles slew rather than snap. Everything downstream of here sees
// the real force, not the requested one.
class Rocket {
public:
    Rocket() = default;
    Rocket(const RocketSpec& spec, Vec2 pos, Vec2 vel, Real angle, Real angVel);

    // Advance the actuators by dt, honouring ignition delay, thrust floor, ramp
    // rates and gimbal slew. Must run before applyControls each tick.
    void stepActuators(Real dt);

    // Turn the *actual* thruster state into forces and torques on the body.
    // Accumulates; does not integrate.
    void applyControls();

    // Where a thruster's nozzle is in world space. The renderer needs it, and
    // deriving it anywhere else would be a second place that has to agree with
    // the physics.
    Vec2 nozzleWorld(std::size_t i) const;

    Body&             body() { return body_; }
    const Body&       body() const { return body_; }
    const RocketSpec& spec() const { return spec_; }

    ControlInput&       controls() { return ctrl_; }
    const ControlInput& controls() const { return ctrl_; }
    void                setControls(const ControlInput& in) { ctrl_ = in; }

    const std::array<ThrusterState, kMaxThrusters>& actuators() const { return actuators_; }

    // Total thrust actually being produced, in newtons.
    Real actualThrust() const;

private:
    // What thrust this thruster is being asked for, having applied the on/off
    // decision and the floor. Zero means "shut down".
    Real demandFor(std::size_t i) const;

    Body                                     body_{};
    RocketSpec                               spec_{};
    ControlInput                             ctrl_{};
    std::array<ThrusterState, kMaxThrusters> actuators_{};
};

}  // namespace rf
