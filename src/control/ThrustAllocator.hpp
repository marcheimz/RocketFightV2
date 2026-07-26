#pragma once

#include <array>

#include "core/ControlInput.hpp"
#include "core/RocketSpec.hpp"

namespace rf {

// Turns "I want this force and this torque" into per-thruster commands, for any
// layout, by solving the least-squares problem the layout defines.
//
// Each thruster contributes a fixed direction of a 3D wrench (force x, force y,
// torque) scaled by its level. Stack those into a 3xN matrix B and the question
// becomes: find levels u with B u = w. That is generally over- or
// under-determined, so this uses the damped pseudo-inverse
//
//     u = B^T (B B^T + lambda I)^-1 w
//
// which returns the *smallest* set of thruster levels achieving the wrench, and
// degrades gracefully to "as close as this vehicle can get" when the wrench is
// simply not achievable -- which is exactly right for a rocket being asked to
// push sideways when it has no side thrusters.
//
// Minimum-norm is also what makes attitude control fall out for free: asking for
// pure torque on a symmetric layout produces the balanced opposed pair, because
// firing one thruster alone would need a larger level *and* leave a residual
// force the solution is penalised for.
//
// The 3x3 inverse depends only on the layout, so it is computed once per rocket
// type rather than per control tick.
class ThrustAllocator {
public:
    explicit ThrustAllocator(const RocketSpec& spec);

    // Body-frame force in newtons, torque in newton-metres. Writes levels and
    // gimbal commands into `out`, leaving any other fields alone.
    //
    // The result respects each thruster's minimum: a level the vehicle could not
    // hold is rounded to off or to the floor, whichever is nearer. That makes
    // allocation partly discrete, and is why fine attitude control needs a
    // deadband rather than an ever-smaller correction.
    void solve(Vec2 desiredForceBody, Real desiredTorque, ControlInput& out) const;

    const RocketSpec& spec() const { return spec_; }

    // What the solved commands would achieve at full compliance, for tests and
    // diagnostics. Actuator lag means the real vehicle gets there later, and the
    // thrust floor means it may not get there exactly at all.
    void achieved(const ControlInput& in, Vec2& forceBody, Real& torque) const;

private:
    struct Column {
        Real fx{}, fy{}, tau{};  // per unit level, with tau row-scaled
    };

    // Symmetric 3x3, stored as the six distinct entries.
    struct Sym3 {
        Real xx{}, xy{}, xz{}, yy{}, yz{}, zz{};
    };

    void applyGimbals(Real residualTorque, ControlInput& out) const;

    // Round levels the vehicle physically cannot hold to off-or-minimum.
    void snapToThrustFloor(ControlInput& out) const;

    // Multiply the precomputed inverse by a wrench.
    void mulInverse(Real wx, Real wy, Real wz, Real& ox, Real& oy, Real& oz) const;

    RocketSpec                            spec_{};
    std::array<Column, kMaxThrusters>     columns_{};
    Sym3                                  inverse_{};

    // The torque row is divided by a characteristic length so that newtons and
    // newton-metres are numerically comparable. Without it the solve is badly
    // conditioned and quietly favours whichever row happens to carry the larger
    // units.
    Real torqueScale_{1};
};

}  // namespace rf
