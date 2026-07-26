#pragma once

#include "control/Controller.hpp"

namespace rf {

// Does nothing, ever. The baseline every other controller is measured against,
// and the cleanest possible determinism fixture: any divergence between two runs
// of this is a bug in the physics, not in a policy.
class IdleController final : public Controller {
public:
    ControlInput evaluate(const Observation&) override { return {}; }
    void         reset(std::uint64_t, const WorldConfig&) override {}
};

// End-to-end: straight from Observation to actuators, no Intent anywhere. It
// exists to prove the interface is not secretly shaped around a human pilot --
// this one holds an open-loop spin while burning, which no Intent could express.
class SpinBurnController final : public Controller {
public:
    explicit SpinBurnController(Real throttle = Real(0.5), Real spin = Real(0.25))
        : throttle_(throttle), spin_(spin) {}

    ControlInput evaluate(const Observation&) override { return command_; }

    // The command depends on where this vehicle's thrusters are, so it is built
    // once the world is known rather than assumed at construction.
    void reset(std::uint64_t, const WorldConfig& env) override;

private:
    Real         throttle_;
    Real         spin_;
    ControlInput command_{};
};

// Intent-level policy: it says where it wants to go and lets the fly-by-wire
// work out the thrusters. Note what it does *not* mention -- gimbal range, RCS
// authority, inertia -- which is why the same policy would fly a different
// vehicle unchanged.
//
// A proportional-derivative law on position, not a velocity command aimed at
// the target. The obvious "fly at it, then brake" formulation fails on this
// vehicle: braking means physically rotating 180 degrees first, which takes a
// second or two, during which the ship sails past and the whole thing settles
// into a limit cycle around the target. A PD law is damped by construction and
// never asks for a manoeuvre the airframe cannot begin immediately.
class SeekPointIntentSource final : public IntentSource {
public:
    struct Gains {
        Real positionGain{0.15};  // 1/s^2
        Real velocityGain{0.90};  // 1/s -- overdamped, to absorb the turn delay
    };

    SeekPointIntentSource() = default;
    explicit SeekPointIntentSource(Vec2 target) : target_(target) {}
    SeekPointIntentSource(Vec2 target, Gains gains) : target_(target), gains_(gains) {}

    Intent intent(const Observation& obs) override;

    Vec2 target() const { return target_; }

private:
    Vec2  target_{};
    Gains gains_{};
};

}  // namespace rf
