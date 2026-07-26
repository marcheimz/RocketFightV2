#pragma once

#include <cstdint>
#include <string>

#include "core/Snapshot.hpp"
#include "core/WorldConfig.hpp"

namespace rf {

struct Scenario {
    std::string   name{"unnamed"};
    WorldConfig   world{};
    std::uint64_t seed{0};
    std::uint32_t maxTicks{60'000};  // 60 s of simulated time
};

enum class EndReason : std::uint8_t {
    Timeout,       // ran the full tick budget
    OutOfBounds,   // left the 1000 km world
};

struct EpisodeResult {
    std::string   scenario;
    std::string   controller;
    std::uint32_t ticks{};
    EndReason     reason{EndReason::Timeout};

    // A bit-level fingerprint of the final state. Two runs that agree here
    // agreed at every step, which is what makes determinism cheap enough to
    // assert on every episode instead of only in a test.
    std::uint64_t stateHash{};

    Snapshot finalState{};
};

const char* toString(EndReason r);

}  // namespace rf
