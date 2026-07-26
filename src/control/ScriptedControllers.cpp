#include "control/ScriptedControllers.hpp"

#include <algorithm>
#include <cmath>

namespace rf {

void SpinBurnController::reset(std::uint64_t, const WorldConfig& env) {
    command_ = ControlInput{};

    if (env.rockets.empty()) return;
    const RocketSpec& spec = env.rockets.front().spec;

    for (std::size_t i = 0; i < spec.count(); ++i) {
        const Thruster& t       = spec[i];
        const Real      forward = dot(t.unitDirection(), Vec2{Real(1), Real(0)});

        if (forward > Real(0.7)) {
            command_.level[i] = throttle_;
        } else if (t.leverArm() > Real(0)) {
            // Only the thrusters that push it one way round, so it really does
            // spin rather than sitting in a balanced stall.
            command_.level[i] = spin_;
        }
    }
}

Intent SeekPointIntentSource::intent(const Observation& obs) {
    const Vec2 toTarget = target_ - obs.pos;

    // Pull towards the target, damped by current velocity. Deliberately
    // overdamped: the ship cannot reverse thrust without first rotating, so the
    // effective control lag is far larger than the actuators alone suggest.
    const Vec2 desiredAccel = toTarget * gains_.positionGain - obs.vel * gains_.velocityGain;

    Intent out;
    out.mode = Intent::Mode::Acceleration;
    // Intent is normalised: a fraction of whatever this vehicle can do. The
    // policy never needs to know what that is in m/s^2.
    out.vector = clampLength(desiredAccel / obs.maxAccel, Real(1));
    return out;
}

}  // namespace rf
