#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <type_traits>

#include "core/Name.hpp"
#include "core/Thruster.hpp"

namespace rf {

// Fixed capacity rather than a vector: the spec is copied into every snapshot
// and every control input is sized to match, and keeping both trivially
// copyable means publishing at 1000 Hz never allocates.
inline constexpr std::size_t kMaxThrusters = 16;

// A rocket type's physical definition. Everything that differs between vehicles
// lives here, and nothing that differs between vehicles lives anywhere else.
// Loaded from JSON at startup -- see src/data -- so adding a vehicle needs no
// recompilation at all.
struct RocketSpec {
    RocketName name{};

    Real mass{1000.0};    // kg
    Real length{12.0};    // m, nose to tail
    Real width{3.0};      // m

    std::size_t                          thrusterCount{};
    std::array<Thruster, kMaxThrusters>  thrusters{};

    bool add(const Thruster& t) {
        if (thrusterCount >= kMaxThrusters) return false;
        thrusters[thrusterCount++] = t;
        return true;
    }

    const Thruster& operator[](std::size_t i) const { return thrusters[i]; }
    std::size_t     count() const { return thrusterCount; }

    // Solid rectangle about its centre.
    constexpr Real inertia() const { return mass * (width * width + length * length) / Real(12); }

    // Best acceleration achievable in a given body-frame direction, ignoring the
    // rotation that firing an unbalanced set would also cause. An upper bound,
    // which is what Intent needs in order to normalise itself.
    Real maxAccelAlong(Real bodyDirection) const {
        const Vec2 want = fromAngle(bodyDirection);
        Real total = 0;
        for (std::size_t i = 0; i < thrusterCount; ++i) {
            total += std::max(Real(0), dot(thrusters[i].unitDirection(), want)) *
                     thrusters[i].maxThrust;
        }
        return total / mass;
    }

    Real maxForwardAccel() const { return maxAccelAlong(Real(0)); }

    // Symmetric lateral capability: the weaker of left and right, because a
    // vehicle that can only shove itself one way sideways cannot be flown as if
    // it were omnidirectional.
    Real maxLateralAccel() const {
        return std::min(maxAccelAlong(kPi / Real(2)), maxAccelAlong(-kPi / Real(2)));
    }

    // Angular authority from thruster placement alone, taking the weaker
    // direction. A rocket with no off-axis thrusters returns zero, which is the
    // honest answer: it cannot turn without gimballing a lit engine.
    Real maxAngAccel() const {
        Real ccw = 0;
        Real cw  = 0;
        for (std::size_t i = 0; i < thrusterCount; ++i) {
            const Real torque = thrusters[i].leverArm() * thrusters[i].maxThrust;
            (torque > Real(0) ? ccw : cw) += std::abs(torque);
        }
        return std::min(ccw, cw) / inertia();
    }

    // The *smallest* angular acceleration the attitude thrusters can produce, as
    // opposed to the largest. With a minimum thrust floor this is not zero, and
    // it is what sets how tight an attitude deadband can possibly be.
    Real minAngAccelImpulse() const {
        Real smallest = 0;
        for (std::size_t i = 0; i < thrusterCount; ++i) {
            const Thruster& t   = thrusters[i];
            const Real      arm = std::abs(t.leverArm());
            if (arm <= Real(0) || t.minThrust <= Real(0)) continue;
            const Real step = arm * t.minThrust / inertia();
            if (smallest == Real(0) || step < smallest) smallest = step;
        }
        return smallest;
    }

    // Angular authority available from deflecting nozzles, at full throttle.
    // Unlike the above this is only there while the engine is burning, which is
    // why the fly-by-wire may not rely on it to stop a slew.
    Real maxGimbalAngAccel() const {
        Real total = 0;
        for (std::size_t i = 0; i < thrusterCount; ++i) {
            const Thruster& t = thrusters[i];
            total += std::abs(t.gimbalArm()) * t.maxThrust * std::sin(t.maxGimbal);
        }
        return total / inertia();
    }

    // Slowest thruster to light. The fly-by-wire has to plan around this: a
    // correction commanded now does not begin for this many seconds.
    Real worstIgnitionTime() const {
        Real worst = 0;
        for (std::size_t i = 0; i < thrusterCount; ++i) {
            worst = std::max(worst, thrusters[i].ignitionTime);
        }
        return worst;
    }

    // The delay that matters for *attitude* specifically: how long the thrusters
    // that can actually turn the vehicle take to light. On a vehicle whose main
    // engine is slow but whose attitude quad is quick, using the main engine's
    // delay would make the controller sluggish for no reason.
    Real attitudeIgnitionTime() const {
        Real worst  = 0;
        bool anyArm = false;
        for (std::size_t i = 0; i < thrusterCount; ++i) {
            if (std::abs(thrusters[i].leverArm()) <= Real(0)) continue;
            anyArm = true;
            worst  = std::max(worst, thrusters[i].ignitionTime);
        }
        // No attitude thrusters at all: steering means lighting a main engine,
        // so its delay is the one that governs.
        return anyArm ? worst : worstIgnitionTime();
    }

    // The smallest velocity change this vehicle can actually commit to.
    //
    // A thruster cannot burn for less than the time it takes to light, and it
    // cannot burn softer than its floor, so the finest available nudge is that
    // acceleration acting over that delay. No controller can null a velocity
    // more precisely than this, however good it is -- which is exactly the kind
    // of limit that ought to be stated rather than discovered.
    Real minImpulseDeltaV() const {
        Real best = 0;
        for (std::size_t i = 0; i < thrusterCount; ++i) {
            const Thruster& t = thrusters[i];
            if (t.minThrust <= Real(0)) continue;
            if (dot(t.unitDirection(), Vec2{Real(1), Real(0)}) < Real(0.7)) continue;
            const Real dv = (t.minThrust / mass) * (t.ignitionTime + kControlDt);
            if (best == Real(0) || dv < best) best = dv;
        }
        return best;
    }

    // How long it takes to swing a nozzle from one extreme to the other. For a
    // vehicle that steers only by gimballing, this is the dead time between
    // deciding to stop turning and being able to: the nozzle has to travel all
    // the way across before it produces torque the other way.
    Real gimbalReversalTime() const {
        Real worst = 0;
        for (std::size_t i = 0; i < thrusterCount; ++i) {
            const Thruster& t = thrusters[i];
            if (!t.gimballed() || t.gimbalRate <= Real(0)) continue;
            worst = std::max(worst, Real(2) * t.maxGimbal / t.gimbalRate);
        }
        return worst;
    }

    // A vehicle that can barely push sideways has to point its nose where it
    // wants to go; one with real lateral authority does not. The fly-by-wire
    // reads this rather than assuming, so a lander does not fly like a missile.
    bool mustPointToThrust() const {
        const Real forward = maxForwardAccel();
        return forward > Real(0) && maxLateralAccel() < forward * Real(0.2);
    }
};

static_assert(std::is_trivially_copyable_v<RocketSpec>);

}  // namespace rf
