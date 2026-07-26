#pragma once

#include "control/Controller.hpp"

namespace rf {

// Layer 2 for a single-engine rocket with a gimballed nozzle and attitude
// thrusters.
//
// The job: the pilot asks for a world-frame acceleration, but this vehicle can
// only push along its own nose. So the fly-by-wire has to rotate the ship to
// point at the requested direction, decide how hard to push once it is pointed
// there, and allocate the required torque between two actuators with different
// authority. None of that is visible to the pilot, and none of it survives a
// change of vehicle -- which is exactly why it lives behind its own interface.
class RocketFlyByWire final : public FlyByWire {
public:
    struct Gains {
        // Attitude is slewed on a deceleration profile rather than a plain PD
        // term. A PD controller with saturating actuators overshoots badly on
        // large heading changes; a profile that asks for the fastest rate the
        // thrusters can still stop from does not.
        Real maxSlewRate{2.0};    // rad/s
        Real slewSafety{0.85};    // margin on the braking profile
        Real rateTau{0.15};       // s, first-order lag tracking the rate target

        // Velocity mode is a proportional law on velocity error.
        Real velGain{2.0};        // 1/s

        // Do not fire the main engine while pointing more than this far from the
        // requested direction, or the ship accelerates somewhere nobody asked
        // for. cos(60 degrees).
        Real thrustGateCos{0.5};
    };

    RocketFlyByWire() = default;
    explicit RocketFlyByWire(Gains gains) : gains_(gains) {}

    ControlInput resolve(const Intent& intent, const Observation& obs) override;

    // No state to clear: the rate profile reads angular velocity from the
    // observation instead of integrating an error term. Kept because the
    // interface promises it and a future gain-scheduled version will need it.
    void reset() override {}

    const Gains& gains() const { return gains_; }

private:
    Gains gains_{};
};

}  // namespace rf
