#pragma once

#include <vector>

#include "core/ControlInput.hpp"
#include "core/RocketSpec.hpp"
#include "core/Types.hpp"
#include "core/Vec2.hpp"
#include "core/WorldConfig.hpp"

namespace rf {

// Everything the renderer needs about one rocket, and nothing else. Not a
// reference to the Body: the renderer must not be able to reach into the world.
//
// The layout travels with the state because the renderer has to draw a plume per
// thruster, wherever this particular vehicle happens to have them. Both members
// are trivially copyable and fixed size, so publishing still never allocates.
struct RocketView {
    Vec2 pos{};
    Vec2 vel{};
    Real angle{};
    Real angVel{};

    RocketSpec   spec{};
    ControlInput ctrl{};   // what was demanded

    // ...and what the actuators are actually doing. The renderer draws plumes
    // from this, so an engine that has been commanded on but has not lit yet
    // correctly shows nothing.
    std::array<ThrusterState, kMaxThrusters> actuators{};
};

// Diagnostics measured by the simulation loop, not by the world. World::snapshot
// leaves these zeroed -- they are wall-clock facts, and the world does not have
// a clock.
struct SimStats {
    Real tickRate{};     // measured ticks per second
    Real loadFactor{};   // fraction of the 1 ms budget actually spent stepping
    std::uint64_t catchUpResyncs{};  // times the loop gave up catching up
};

struct Snapshot {
    Tick tick{};
    Real time{};

    std::vector<RocketView> rockets;
    std::vector<Attractor>  attractors;

    Real     boundsRadius{};
    SimStats stats{};
};

}  // namespace rf
