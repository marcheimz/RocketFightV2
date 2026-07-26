#include "control/ThrustAllocator.hpp"

#include <algorithm>
#include <cmath>

namespace rf {

namespace {

// Thrusters cannot pull, so the raw least-squares answer routinely asks the
// opposite member of an opposed pair for a negative level. Clamping that to zero
// throws away real authority: ask a symmetric layout for 80% of its torque and
// a single clamped pass delivers 40%.
//
// Re-solving on the residual recovers it, but only geometrically -- each pass
// roughly halves what is left -- so two passes stall at 87%. Enough passes to
// converge, with an early exit once the residual is negligible, costs a few
// dozen multiply-adds at 100 Hz and gets the full authority the layout has.
constexpr int  kMaxRefinementPasses = 16;
constexpr Real kResidualEpsilon     = Real(1e-9);

}  // namespace

ThrustAllocator::ThrustAllocator(const RocketSpec& spec) : spec_(spec) {
    torqueScale_ = std::max(spec.length / Real(2), Real(1e-3));

    Sym3 m{};
    for (std::size_t i = 0; i < spec_.count(); ++i) {
        const Thruster& t   = spec_[i];
        const Vec2      dir = t.unitDirection();

        Column c;
        c.fx  = dir.x * t.maxThrust;
        c.fy  = dir.y * t.maxThrust;
        c.tau = t.leverArm() * t.maxThrust / torqueScale_;
        columns_[i] = c;

        m.xx += c.fx * c.fx;
        m.xy += c.fx * c.fy;
        m.xz += c.fx * c.tau;
        m.yy += c.fy * c.fy;
        m.yz += c.fy * c.tau;
        m.zz += c.tau * c.tau;
    }

    // Damping. A vehicle missing a whole axis of authority -- no side thrusters,
    // or no attitude thrusters at all -- gives a singular B B^T, and without a
    // lambda the inverse would be meaningless rather than merely limited.
    const Real trace  = (m.xx + m.yy + m.zz) / Real(3);
    const Real lambda = std::max(trace, Real(1)) * Real(1e-6);
    m.xx += lambda;
    m.yy += lambda;
    m.zz += lambda;

    // Cofactor inverse of a symmetric 3x3.
    const Real c00 = m.yy * m.zz - m.yz * m.yz;
    const Real c01 = m.xz * m.yz - m.xy * m.zz;
    const Real c02 = m.xy * m.yz - m.xz * m.yy;
    const Real det = m.xx * c00 + m.xy * c01 + m.xz * c02;

    if (std::abs(det) < Real(1e-30)) {
        inverse_ = Sym3{};  // no authority at all: every solve returns zero
        return;
    }

    const Real invDet = Real(1) / det;
    inverse_.xx = c00 * invDet;
    inverse_.xy = c01 * invDet;
    inverse_.xz = c02 * invDet;
    inverse_.yy = (m.xx * m.zz - m.xz * m.xz) * invDet;
    inverse_.yz = (m.xy * m.xz - m.xx * m.yz) * invDet;
    inverse_.zz = (m.xx * m.yy - m.xy * m.xy) * invDet;
}

void ThrustAllocator::mulInverse(Real wx, Real wy, Real wz, Real& ox, Real& oy, Real& oz) const {
    ox = inverse_.xx * wx + inverse_.xy * wy + inverse_.xz * wz;
    oy = inverse_.xy * wx + inverse_.yy * wy + inverse_.yz * wz;
    oz = inverse_.xz * wx + inverse_.yz * wy + inverse_.zz * wz;
}

void ThrustAllocator::solve(Vec2 desiredForceBody, Real desiredTorque, ControlInput& out) const {
    const std::size_t n = spec_.count();
    for (std::size_t i = 0; i < kMaxThrusters; ++i) {
        out.level[i]  = Real(0);
        out.gimbal[i] = Real(0);
    }
    if (n == 0) return;

    Real wx = desiredForceBody.x;
    Real wy = desiredForceBody.y;
    Real wz = desiredTorque / torqueScale_;

    // Magnitude below which a residual is not worth another pass, scaled to the
    // vehicle so it means the same thing for a lander and an interceptor.
    Real scale = 0;
    for (std::size_t i = 0; i < n; ++i) scale += spec_[i].maxThrust;
    const Real tolerance = std::max(scale, Real(1)) * kResidualEpsilon;

    for (int pass = 0; pass < kMaxRefinementPasses; ++pass) {
        Real px = 0;
        Real py = 0;
        Real pz = 0;
        mulInverse(wx, wy, wz, px, py, pz);

        for (std::size_t i = 0; i < n; ++i) {
            const Column& c  = columns_[i];
            const Real    du = c.fx * px + c.fy * py + c.tau * pz;
            out.level[i] = clamp(out.level[i] + du, Real(0), Real(1));
        }

        // What the clamped levels actually deliver, and what is left over.
        Real ax = 0;
        Real ay = 0;
        Real az = 0;
        for (std::size_t i = 0; i < n; ++i) {
            const Column& c = columns_[i];
            ax += c.fx * out.level[i];
            ay += c.fy * out.level[i];
            az += c.tau * out.level[i];
        }
        wx = desiredForceBody.x - ax;
        wy = desiredForceBody.y - ay;
        wz = desiredTorque / torqueScale_ - az;

        // Converged, or asking for something this vehicle simply cannot do --
        // either way there is nothing more to gain from another pass.
        const Real residual = std::abs(wx) + std::abs(wy) + std::abs(wz);
        if (residual < tolerance) break;
    }

    snapToThrustFloor(out);

    // Torque still missing after the levels are fixed goes to the nozzles, which
    // can steer thrust the levels cannot.
    Real achievedTorque = 0;
    for (std::size_t i = 0; i < n; ++i) {
        achievedTorque += columns_[i].tau * torqueScale_ * out.level[i];
    }
    applyGimbals(desiredTorque - achievedTorque, out);
}

void ThrustAllocator::snapToThrustFloor(ControlInput& out) const {
    // A thruster cannot be throttled below its floor, so a level between "off"
    // and "minimum" is not a command the vehicle can obey. Left alone, the
    // simulation would round it for us -- silently, and in a direction the
    // allocator never accounted for.
    //
    // Snapping here instead keeps the controller honest about what it is really
    // asking for: round to whichever of off-or-minimum is nearer, so a demand
    // that is mostly noise shuts the thruster down rather than committing it to
    // a burn it did not want.
    for (std::size_t i = 0; i < spec_.count(); ++i) {
        const Thruster& t = spec_[i];
        if (t.minThrust <= Real(0)) continue;

        const Real floor = t.minThrust / t.maxThrust;
        if (out.level[i] <= Real(0) || out.level[i] >= floor) continue;

        out.level[i] = (out.level[i] >= floor * Real(0.5)) ? floor : Real(0);
    }
}

void ThrustAllocator::applyGimbals(Real residualTorque, ControlInput& out) const {
    // Torque each nozzle could add at full deflection, given how hard it is
    // currently burning. A cold engine has no gimbal authority at all, which is
    // why the fly-by-wire must not rely on it to stop a slew.
    std::array<Real, kMaxThrusters> authority{};
    Real total = 0;

    for (std::size_t i = 0; i < spec_.count(); ++i) {
        const Thruster& t = spec_[i];
        if (!t.gimballed() || out.level[i] <= Real(0)) continue;
        authority[i] = out.level[i] * t.maxThrust * t.gimbalArm() * std::sin(t.maxGimbal);
        total += std::abs(authority[i]);
    }

    if (total <= Real(0)) return;

    // Deflect every gimballed nozzle by the same fraction of its range, each in
    // whichever direction helps. Their contributions then sum to the residual
    // without any one of them saturating before the others.
    const Real fraction = clamp(residualTorque / total, Real(-1), Real(1));
    for (std::size_t i = 0; i < spec_.count(); ++i) {
        if (authority[i] == Real(0)) continue;
        out.gimbal[i] = fraction * sign(authority[i]);
    }
}

void ThrustAllocator::achieved(const ControlInput& in, Vec2& forceBody, Real& torque) const {
    forceBody = {};
    torque    = 0;
    for (std::size_t i = 0; i < spec_.count(); ++i) {
        const Thruster& t = spec_[i];
        const Real      deflection = clamp(in.gimbal[i], Real(-1), Real(1)) * t.maxGimbal;
        const Vec2      f = t.forceLocal(in.level[i] * t.maxThrust, deflection);
        forceBody += f;
        torque += cross(t.mountLocal, f);
    }
}

}  // namespace rf
