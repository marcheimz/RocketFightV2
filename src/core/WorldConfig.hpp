#pragma once

#include <vector>

#include "core/RocketSpec.hpp"
#include "core/Types.hpp"
#include "core/Vec2.hpp"

namespace rf {

// A dense planet. Gravity is data, not a constant buried in the integrator:
// an empty attractor list is zero-g, which is the default world.
struct Attractor {
    Vec2 pos{};
    Real mu{};       // G*M, the only gravitational parameter that matters
    Real radius{};   // for rendering, and for collisions once they exist
};

struct RocketInit {
    Vec2 pos{};
    Vec2 vel{};
    Real angle{};
    Real angVel{};

    // The vehicle, by value. Core never looks a rocket up: specs are loaded from
    // JSON by the data layer and handed in, which is what keeps the simulation
    // free of any notion of a filesystem.
    RocketSpec spec{};
};

struct WorldConfig {
    std::vector<Attractor>  attractors{};   // empty -> zero-g
    std::vector<RocketInit> rockets{};

    Real boundsRadius{kWorldRadius};

    // Plummer softening. Keeps gravity finite if a body passes through a
    // singularity, which matters while there is nothing to collide with.
    Real softening{1.0};

    std::uint64_t seed{0x9E3779B97F4A7C15ull};
};

// The default playground: one rocket at rest at the origin, empty space.
WorldConfig defaultWorld(const RocketSpec& rocket);

// One rocket in a circular orbit around a single planet. Used by the eval
// harness and by the integrator accuracy test, which needs a trajectory with a
// known closed-form answer.
WorldConfig orbitWorld(const RocketSpec& rocket);

}  // namespace rf
