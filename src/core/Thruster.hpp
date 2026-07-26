#pragma once

#include <type_traits>

#include "core/Name.hpp"
#include "core/Vec2.hpp"

namespace rf {

// The only way anything is ever actuated.
//
// There is deliberately no separate concept of "RCS": an attitude thruster is
// just one mounted off the centreline, and its torque falls out of where it sits
// rather than being postulated as a special case.
//
// A thruster is not an instantaneous, arbitrarily-adjustable force. It takes
// time to light, it cannot throttle below a floor once burning, it ramps rather
// than jumps, and its nozzle slews at a finite rate. Every one of those is a
// constraint the fly-by-wire has to fly around instead of wishing away.
struct Thruster {
    // What the crew would call it: "main", "nose-left", "main-stbd". Purely a
    // label -- nothing in the physics or the allocator reads it, and nothing may
    // start to, or layouts would stop being interchangeable. It exists so that a
    // diagnostic can say *which* nozzle is misbehaving instead of "index 3".
    ThrusterName name{};

    // --- geometry ---------------------------------------------------------
    // Where it is bolted to the hull, body frame, metres from the centre of
    // mass. This is what produces torque; a thruster whose line of action
    // passes through the CoM produces none.
    Vec2 mountLocal{};

    // Direction of the force applied *to the hull*, body frame, radians. A main
    // engine at the tail pushes forward, so its direction is 0 even though its
    // exhaust goes the other way.
    Real directionLocal{};

    // --- thrust envelope ---------------------------------------------------
    // Once lit, a thruster cannot be throttled below minThrust. It is either off
    // or somewhere in [minThrust, maxThrust]. This is what turns allocation from
    // a continuous problem into a discrete one, and it is the single biggest
    // reason attitude control needs a deadband.
    Real minThrust{};
    Real maxThrust{};

    // Seconds between being commanded on and producing any thrust at all.
    Real ignitionTime{};

    // How fast thrust can change once burning, in newtons per second.
    Real rampUpRate{};
    Real rampDownRate{};

    // --- gimbal ------------------------------------------------------------
    Real maxGimbal{};    // rad of deflection either side; 0 means a fixed nozzle
    Real gimbalRate{};   // rad/s the nozzle can slew; 0 with a gimbal means instant

    bool gimballed() const { return maxGimbal > Real(0); }

    Vec2 unitDirection() const { return fromAngle(directionLocal); }

    // Torque produced per newton of thrust with the nozzle undeflected. Sign is
    // the sign of the rotation: positive is counter-clockwise.
    Real leverArm() const { return cross(mountLocal, unitDirection()); }

    // Torque produced per newton per radian of nozzle deflection. Zero for a
    // fixed nozzle, and zero for a gimballed one mounted at the centre of mass.
    Real gimbalArm() const { return cross(mountLocal, perp(unitDirection())); }

    // The force this thruster applies to the hull, in body frame, at a given
    // *actual* thrust and *actual* nozzle deflection. Note both are actual, not
    // commanded: what the controller asked for is not what is happening yet.
    Vec2 forceLocal(Real thrust, Real gimbalAngle) const {
        return fromAngle(directionLocal + gimbalAngle) * thrust;
    }
};

// What a thruster is doing right now, as opposed to what it was told to do.
// This is simulation state: it is integrated every tick, it goes into the state
// hash, and it survives into the snapshot so the renderer draws real plumes.
struct ThrusterState {
    Real thrust{};        // N, currently being produced
    Real gimbalAngle{};   // rad, where the nozzle actually is
    Real ignitionTimer{}; // s remaining before it lights
    bool lit{};           // burning, or at least committed to burning

    bool idle() const { return !lit && thrust <= Real(0) && ignitionTimer <= Real(0); }
};

// Both of these are copied wholesale into a Snapshot a thousand times a second.
// Asserted rather than hoped for, because the failure mode of adding a
// std::string field here is not a compile error -- it is a heap allocation on
// the publish path that nothing would report.
static_assert(std::is_trivially_copyable_v<Thruster>);
static_assert(std::is_trivially_copyable_v<ThrusterState>);

}  // namespace rf
