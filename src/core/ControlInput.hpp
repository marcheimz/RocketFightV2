#pragma once

#include <array>
#include <type_traits>

#include "core/RocketSpec.hpp"

namespace rf {

// Layer 3: the actuator commands, one entry per thruster. This is the only
// thing the physics reads, and the only thing a Controller ever produces.
//
// Per-thruster rather than the old throttle/gimbal/rcs triple, because that
// triple silently assumed one gimballed engine and an abstract torque source.
// Fixed capacity keeps it trivially copyable and allocation-free.
struct ControlInput {
    std::array<Real, kMaxThrusters> level{};    // [0, 1] per thruster
    std::array<Real, kMaxThrusters> gimbal{};   // [-1, 1] per thruster, ignored if fixed
    bool fire{};

    void setLevel(std::size_t i, Real v) {
        if (i < kMaxThrusters) level[i] = clamp(v, Real(0), Real(1));
    }
    void setGimbal(std::size_t i, Real v) {
        if (i < kMaxThrusters) gimbal[i] = clamp(v, Real(-1), Real(1));
    }

    // Total commanded thrust as a fraction of everything the vehicle has. Used
    // by HUDs and by the renderer's plume, not by the physics.
    Real totalLevel(const RocketSpec& spec) const {
        Real sum = 0;
        Real max = 0;
        for (std::size_t i = 0; i < spec.count(); ++i) {
            sum += level[i] * spec[i].maxThrust;
            max += spec[i].maxThrust;
        }
        return max > Real(0) ? sum / max : Real(0);
    }
};

static_assert(std::is_trivially_copyable_v<ControlInput>);

}  // namespace rf
