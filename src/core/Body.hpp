#pragma once

#include "core/Vec2.hpp"

namespace rf {

// A 2D rigid body: position and orientation, both integrated.
//
// Mass and inertia are stored inverted. Zero then means "immovable" without a
// branch or a division anywhere in the integration path.
struct Body {
    Vec2 pos{};
    Vec2 vel{};
    Real angle{};
    Real angVel{};

    Real invMass{};
    Real invInertia{};

    // Accumulated over a tick, cleared once integrated.
    Vec2 force{};
    Real torque{};

    void applyForce(Vec2 f) { force += f; }

    void applyTorque(Real t) { torque += t; }

    // Apply a world-frame force at a body-local offset from the centre of mass.
    // This is what makes a gimballed nozzle steer the rocket instead of just
    // pushing it: the offset turns part of the thrust into torque.
    void applyForceAt(Vec2 worldForce, Vec2 localOffset) {
        const Vec2 r = rotate(localOffset, angle);
        force  += worldForce;
        torque += cross(r, worldForce);
    }

    void clearForces() {
        force  = {};
        torque = Real(0);
    }

    // Semi-implicit (symplectic) Euler: velocity is updated first, then position
    // is advanced using the *new* velocity. That one ordering detail is why this
    // stays stable in an orbit where explicit Euler spirals outwards.
    void integrate(Real dt) {
        vel    += force * (invMass * dt);
        pos    += vel * dt;
        angVel += torque * (invInertia * dt);
        angle   = wrapPi(angle + angVel * dt);
    }
};

}  // namespace rf
