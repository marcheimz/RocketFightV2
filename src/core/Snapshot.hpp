#pragma once

#include <type_traits>
#include <vector>

#include "core/ControlInput.hpp"
#include "core/Intent.hpp"
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

// The snapshot itself owns vectors, whose capacity the triple buffer reuses; the
// per-rocket payload is what must never grow an owning member, or filling those
// vectors would start allocating per element.
static_assert(std::is_trivially_copyable_v<RocketView>);

// Diagnostics measured by the simulation loop, not by the world. World::snapshot
// leaves these zeroed -- they are wall-clock facts, and the world does not have
// a clock.
struct SimStats {
    Real tickRate{};     // measured ticks per second
    Real loadFactor{};   // fraction of the 1 ms budget actually spent stepping
    std::uint64_t catchUpResyncs{};  // times the loop gave up catching up
};

// What the pilot last asked for. Filled in by SimulationLoop from the active
// Controller::lastIntent(), not by World, for exactly the reason SimStats is
// not: a commanded intent is a fact about whoever is flying, and the world has
// no controller any more than it has a clock.
//
// `valid` is not a convenience. A controller that does not think in terms of
// Intent has no answer -- direct manual flight, an end-to-end policy -- and a
// zeroed Intent is indistinguishable from a real command for zero acceleration,
// so a renderer without this flag would draw a demand nobody made.
struct CommandView {
    bool   valid{};
    Intent intent{};
};

// Same reason as RocketView: this rides in a snapshot published at 1000 Hz and
// must never allocate. Intent holds a std::optional<Real>, which qualifies -- so
// this makes a future change that stops qualifying a compile error rather than a
// silent allocation on the publish path.
static_assert(std::is_trivially_copyable_v<CommandView>);

// Which layer 2 is actually flying. Filled in by SimulationLoop for the same
// reason CommandView is: the choice belongs to the loop, not to the world, and
// the renderer must not reach across the thread boundary to ask.
//
// A submission that failed to load is reported rather than hidden -- the loop
// falls back to the built-in so the ship still flies, and silently substituting
// a different controller than the one on screen would be the worst of both.
struct FlyByWireView {
    RocketName name{};        // "built-in", or the submission's manifest name
    bool       submission{};
    bool       loadFailed{};
};

static_assert(std::is_trivially_copyable_v<FlyByWireView>);

struct Snapshot {
    Tick tick{};
    Real time{};

    std::vector<RocketView> rockets;
    std::vector<Attractor>  attractors;

    Real     boundsRadius{};
    SimStats stats{};

    // Which fly-by-wire is flying rockets.front().
    FlyByWireView flyByWire{};

    // The command flown by rockets.front(), the one vehicle the loop controls.
    CommandView command{};
};

}  // namespace rf
