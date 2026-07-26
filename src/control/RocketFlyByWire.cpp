#include "control/RocketFlyByWire.hpp"

#include <algorithm>
#include <cmath>

namespace rf {

RocketFlyByWire::RocketFlyByWire(const RocketSpec& spec) : RocketFlyByWire(spec, Gains{}) {}

RocketFlyByWire::RocketFlyByWire(const RocketSpec& spec, Gains gains)
    : spec_(spec), allocator_(spec), gains_(gains) {
    bodyMustPoint_ = spec_.mustPointToThrust();

    // Braking authority deliberately excludes the gimbal for vehicles that have
    // attitude thrusters: gimbal authority is proportional to throttle, and
    // promising a slew rate only a lit engine could arrest would overshoot
    // exactly when the pilot lets go.
    brakeAuthority_ = spec_.maxAngAccel();
    gimbalOnly_     = brakeAuthority_ <= Real(1e-9);
    if (gimbalOnly_) brakeAuthority_ = spec_.maxGimbalAngAccel() * gains_.gimbalReliance;
    brakeAuthority_ = std::max(brakeAuthority_, Real(1e-6)) * gains_.slewSafety;

    // The dead time between deciding to correct and the correction existing:
    // one control period to notice, plus however long the attitude thrusters
    // take to light.
    attitudeLag_ = kControlDt + spec_.attitudeIgnitionTime();

    // For a vehicle that steers only by gimballing, that is not the dominant
    // delay. Braking a turn means swinging the nozzle all the way to the other
    // extreme, and until it gets there the vehicle keeps rotating. Ignoring that
    // is exactly how a rate profile talks itself into an overshoot it can never
    // recover from.
    if (gimbalOnly_) attitudeLag_ += spec_.gimbalReversalTime();

    ignitionLead_ = spec_.attitudeIgnitionTime() * gains_.leadFraction;

    // The tightest attitude this vehicle can actually hold. With a thrust floor
    // the smallest available correction is not arbitrarily small, and it cannot
    // be applied for less than one lag's worth of time -- so the finest angle it
    // can resolve is roughly that acceleration acting over that lag. Chasing an
    // error smaller than that means a correction bigger than the error, which is
    // a limit cycle dressed up as control. Better to name the limit and stop
    // inside it.
    deadband_ = std::max(gains_.minDeadband,
                         spec_.minAngAccelImpulse() * attitudeLag_ * attitudeLag_ *
                             gains_.deadbandFactor);
}

ControlInput RocketFlyByWire::resolve(const Intent& intent, const Observation& obs) {
    ControlInput out;
    out.fire = intent.fire;

    // --- 1. What acceleration does the pilot want, in m/s^2? ----------------
    Vec2 desiredAccel{};
    switch (intent.mode) {
        case Intent::Mode::Acceleration:
            desiredAccel = clampLength(intent.vector, Real(1)) * obs.maxAccel;
            break;
        case Intent::Mode::Velocity:
            desiredAccel = clampLength((intent.vector - obs.vel) * gains_.velGain, obs.maxAccel);
            break;
    }
    const Real accelMag = length(desiredAccel);

    // --- 2. Where should the nose point? ------------------------------------
    // Only a vehicle that cannot push sideways is obliged to turn towards its
    // thrust. A lander with real lateral authority keeps its attitude and
    // translates, which is both correct and what makes it feel different to fly.
    Real desiredHeading = obs.angle;
    if (intent.facing) {
        desiredHeading = *intent.facing;
    } else if (bodyMustPoint_ && accelMag > Real(1e-9)) {
        desiredHeading = angleOf(desiredAccel);
    }

    // --- 3. Body-frame force to ask the allocator for ------------------------
    Vec2 forceBody{};
    if (accelMag > Real(0)) {
        if (bodyMustPoint_) {
            // Ease thrust in with the cosine so the ship does not lurch sideways
            // while still rotating, and refuse entirely past the gate. If the
            // pilot has commanded a facing that fights the requested
            // acceleration, this correctly does nothing: the vehicle physically
            // cannot do both, and quietly doing the wrong one would be worse.
            const Real alignment = std::cos(wrapPi(angleOf(desiredAccel) - obs.angle));
            if (alignment > gains_.thrustGateCos) {
                forceBody = {spec_.mass * accelMag * alignment, Real(0)};
            }
        } else {
            forceBody = rotate(desiredAccel, -obs.angle) * spec_.mass;
        }
    }

    // --- 4. Attitude: a rate target the thrusters can still stop from --------
    // Predict where the heading will be by the time a correction commanded now
    // has actually lit. Judging the current error instead means chasing a state
    // that is already stale by an ignition delay.
    const Real predictedAngle = obs.angle + obs.angVel * ignitionLead_;
    const Real headingError   = wrapPi(desiredHeading - predictedAngle);

    Real torqueWanted = 0;
    const bool insideDeadband =
        std::abs(headingError) < deadband_ && std::abs(obs.angVel) < deadband_ * Real(4);

    if (!insideDeadband) {
        // Reserve the angle the vehicle will sweep through before any braking
        // takes effect. A profile that spends the whole error budget on
        // decelerating -- as if torque appeared the instant it was asked for --
        // commits to a rate it cannot stop from, overshoots, and does the same
        // thing coming back. That is a limit cycle, and on a gimbal-only vehicle
        // it is a large one.
        const Real deadTimeSweep = std::abs(obs.angVel) * attitudeLag_;
        const Real usableError   = std::max(Real(0), std::abs(headingError) - deadTimeSweep);

        const Real rateTarget =
            sign(headingError) *
            std::min(gains_.maxSlewRate, std::sqrt(Real(2) * brakeAuthority_ * usableError));

        torqueWanted = (rateTarget - obs.angVel) / gains_.rateTau * spec_.inertia();
    }

    // --- 5. A gimbal-only vehicle has to burn in order to steer --------------
    const bool attitudeWorkPending =
        !insideDeadband || std::abs(obs.angVel) > deadband_ * Real(4);
    if (gimbalOnly_ && attitudeWorkPending) {
        forceBody.x =
            std::max(forceBody.x, spec_.mass * spec_.maxForwardAccel() * gains_.steeringThrottle);
    }

    // --- 6. Hand the wrench to the layout ------------------------------------
    allocator_.solve(forceBody, torqueWanted, out);
    return out;
}

std::unique_ptr<FlyByWire> makeFlyByWire(const RocketSpec& spec) {
    return std::make_unique<RocketFlyByWire>(spec);
}

}  // namespace rf
