#pragma once

#include <cstdint>
#include <vector>

#include "core/ControlInput.hpp"
#include "core/Observation.hpp"
#include "core/Rng.hpp"
#include "core/Rocket.hpp"
#include "core/Snapshot.hpp"
#include "core/WorldConfig.hpp"

namespace rf {

// The simulation. Pure C++: no SFML, no threads, no I/O, and above all no
// clock -- the only notion of time is the tick count and the dt handed to step.
//
// One World is never touched by two threads. Parallelism comes from running
// many Worlds, not from sharing one.
class World {
public:
    explicit World(WorldConfig cfg);

    // Advance by exactly dt. Always called with kTickDt; the parameter exists so
    // tests can probe the integrator, not so callers can pass a frame time.
    void step(Real dt);

    // Restore the initial configuration. Same object, same identity, tick zero.
    void reset();

    void setControl(EntityId id, const ControlInput& in);

    Observation observe(EntityId id) const;

    // Fills the target rather than returning one, so the triple buffer's slots
    // keep their capacity and publishing at 1000 Hz does not allocate.
    void snapshot(Snapshot& out) const;

    // Bit-level fingerprint of the dynamic state.
    std::uint64_t hash() const;

    bool anyOutOfBounds() const;

    Tick tick() const { return tick_; }
    Real time() const { return static_cast<Real>(tick_) * kTickDt; }

    const WorldConfig&         config() const { return cfg_; }
    const std::vector<Rocket>& rockets() const { return rockets_; }
    std::size_t                rocketCount() const { return rockets_.size(); }

private:
    void accumulateGravity(Rocket& r) const;

    WorldConfig         cfg_;
    std::vector<Rocket> rockets_;
    Tick                tick_{0};
    Rng                 rng_;
};

}  // namespace rf
