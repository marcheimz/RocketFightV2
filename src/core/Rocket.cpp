#include "core/Rocket.hpp"

#include <algorithm>

namespace rf {

namespace {

// A thruster commanded below this fraction of its own minimum is treated as
// commanded off. Without a gap between "shut down" and "light up" a demand
// hovering near the floor would chatter the engine on and off every tick.
constexpr Real kShutdownHysteresis = Real(0.5);

// Move `value` towards `target` at no more than `rate` per second. A rate of
// zero means instant, which is how a spec opts out of the constraint.
Real slew(Real value, Real target, Real rate, Real dt) {
    if (rate <= Real(0)) return target;
    const Real step = rate * dt;
    if (target > value) return std::min(target, value + step);
    return std::max(target, value - step);
}

}  // namespace

Rocket::Rocket(const RocketSpec& spec, Vec2 pos, Vec2 vel, Real angle, Real angVel) : spec_(spec) {
    body_.pos        = pos;
    body_.vel        = vel;
    body_.angle      = wrapPi(angle);
    body_.angVel     = angVel;
    body_.invMass    = Real(1) / spec.mass;
    body_.invInertia = Real(1) / spec.inertia();
}

Real Rocket::demandFor(std::size_t i) const {
    const Thruster& t     = spec_[i];
    const Real      level = clamp(ctrl_.level[i], Real(0), Real(1));
    const Real      asked = level * t.maxThrust;

    // Below half the floor, treat it as "off". Between there and the floor, the
    // thruster simply cannot comply: it delivers its minimum or nothing, and the
    // controller has to live with the difference. This is the whole reason
    // attitude control needs a deadband.
    if (asked < t.minThrust * kShutdownHysteresis) return Real(0);
    return clamp(asked, t.minThrust, t.maxThrust);
}

void Rocket::stepActuators(Real dt) {
    // Fixed iteration order over a dense array: one of the places determinism is
    // actually won or lost.
    for (std::size_t i = 0; i < spec_.count(); ++i) {
        const Thruster& t = spec_[i];
        ThrusterState&  a = actuators_[i];

        const Real demand = demandFor(i);

        if (demand > Real(0)) {
            if (!a.lit) {
                // Commanded on. Start the ignition clock if it is not already
                // running, and produce nothing at all until it expires.
                if (a.ignitionTimer <= Real(0) && a.thrust <= Real(0)) {
                    a.ignitionTimer = t.ignitionTime;
                }
                a.ignitionTimer -= dt;
                if (a.ignitionTimer <= Real(0)) {
                    a.ignitionTimer = Real(0);
                    a.lit           = true;
                    // Lighting produces the floor immediately; ramping from
                    // there is what the rate limit governs.
                    a.thrust = std::max(a.thrust, t.minThrust);
                }
            }

            if (a.lit) {
                a.thrust = slew(a.thrust, demand, t.rampUpRate, dt);
                a.thrust = std::max(a.thrust, std::min(t.minThrust, demand));
            }
        } else {
            // Commanded off: no delay going down, but thrust still decays at the
            // ramp rate rather than vanishing.
            a.lit           = false;
            a.ignitionTimer = Real(0);
            a.thrust        = slew(a.thrust, Real(0), t.rampDownRate, dt);
            if (a.thrust < Real(1e-9)) a.thrust = Real(0);
        }

        // The nozzle slews towards its commanded deflection whether or not the
        // thruster is burning -- and it is the *actual* angle that steers.
        const Real commanded = clamp(ctrl_.gimbal[i], Real(-1), Real(1)) * t.maxGimbal;
        a.gimbalAngle        = slew(a.gimbalAngle, commanded, t.gimbalRate, dt);
    }
}

void Rocket::applyControls() {
    for (std::size_t i = 0; i < spec_.count(); ++i) {
        const ThrusterState& a = actuators_[i];
        if (a.thrust <= Real(0)) continue;

        // Force in body frame from what the actuator is *actually* doing, then
        // rotated into the world. Applying it at the mount point is what turns
        // an off-centre thruster into a torque -- no separate attitude model,
        // and no way for the two to disagree.
        const Vec2 forceLocal = spec_[i].forceLocal(a.thrust, a.gimbalAngle);
        body_.applyForceAt(rotate(forceLocal, body_.angle), spec_[i].mountLocal);
    }
}

Vec2 Rocket::nozzleWorld(std::size_t i) const {
    return body_.pos + rotate(spec_[i].mountLocal, body_.angle);
}

Real Rocket::actualThrust() const {
    Real total = 0;
    for (std::size_t i = 0; i < spec_.count(); ++i) total += actuators_[i].thrust;
    return total;
}

}  // namespace rf
