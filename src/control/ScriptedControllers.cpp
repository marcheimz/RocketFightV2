#include "control/ScriptedControllers.hpp"

#include <algorithm>
#include <cmath>

namespace rf {

ControlInput SpinBurnController::evaluate(const Observation&) {
    ControlInput out;
    out.throttle = throttle_;
    out.rcs      = rcs_;
    return out;
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
