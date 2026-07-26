#include "control/RocketFlyByWire.hpp"

#include <algorithm>
#include <cmath>

namespace rf {

ControlInput RocketFlyByWire::resolve(const Intent& intent, const Observation& obs) {
    ControlInput out;
    out.fire = intent.fire;

    // --- 1. What acceleration does the pilot actually want, in m/s^2? --------
    Vec2 desiredAccel{};
    switch (intent.mode) {
        case Intent::Mode::Acceleration:
            desiredAccel = clampLength(intent.vector, Real(1)) * obs.maxAccel;
            break;
        case Intent::Mode::Velocity: {
            const Vec2 velError = intent.vector - obs.vel;
            desiredAccel = clampLength(velError * gains_.velGain, obs.maxAccel);
            break;
        }
    }

    const Real accelMag = length(desiredAccel);

    // --- 2. Where should the nose point? ------------------------------------
    // An explicit facing wins. Otherwise the nose follows the thrust, because
    // that is the only direction this vehicle can push. With neither, hold
    // attitude rather than drifting.
    Real desiredHeading = obs.angle;
    if (intent.facing) {
        desiredHeading = *intent.facing;
    } else if (accelMag > Real(1e-9)) {
        desiredHeading = angleOf(desiredAccel);
    }

    // --- 3. Throttle ---------------------------------------------------------
    // Thrust only once the nose is near the requested direction, and ease it in
    // with the cosine so the ship does not lurch sideways while still rotating.
    // If the pilot has commanded a facing that fights the requested
    // acceleration, this correctly refuses to thrust: the vehicle physically
    // cannot do both, and quietly doing the wrong one would be worse.
    if (accelMag > Real(0)) {
        const Real thrustError = wrapPi(angleOf(desiredAccel) - obs.angle);
        const Real alignment   = std::cos(thrustError);
        if (alignment > gains_.thrustGateCos) {
            out.throttle = clamp(accelMag / obs.maxAccel, Real(0), Real(1)) * alignment;
        }
    }

    // --- 4. Attitude: a rate target the thrusters can still stop from --------
    const Real headingError = wrapPi(desiredHeading - obs.angle);

    // The braking profile uses RCS authority only. Gimbal authority is
    // proportional to throttle, and throttle can drop to zero at any moment --
    // promising a slew rate that only a lit engine could arrest would overshoot
    // exactly when the pilot lets go.
    const Real brakeAccel = obs.maxRcsAngAccel * gains_.slewSafety;
    const Real rateTarget =
        sign(headingError) * std::min(gains_.maxSlewRate,
                                      std::sqrt(Real(2) * brakeAccel * std::abs(headingError)));

    Real angAccelWanted = (rateTarget - obs.angVel) / gains_.rateTau;

    // --- 5. Control allocation ----------------------------------------------
    // Two actuators, different authority. Spend the gimbal first: it is free
    // while the engine is already burning, and stronger than the RCS at full
    // throttle. Whatever torque it cannot supply falls through to the thrusters.
    const Real gimbalAuthority = out.throttle * obs.maxGimbalAngAccel;
    if (gimbalAuthority > Real(0)) {
        // A positive nozzle deflection produces a *negative* torque: the nozzle
        // is behind the centre of mass, so the tail swings the opposite way.
        out.gimbal = clamp(-angAccelWanted / gimbalAuthority, Real(-1), Real(1));
        angAccelWanted += out.gimbal * gimbalAuthority;
    }

    if (obs.maxRcsAngAccel > Real(0)) {
        out.rcs = clamp(angAccelWanted / obs.maxRcsAngAccel, Real(-1), Real(1));
    }

    return out;
}

}  // namespace rf
