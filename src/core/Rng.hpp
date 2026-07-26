#pragma once

#include <cstdint>

#include "core/Types.hpp"

namespace rf {

// splitmix64. Explicitly seeded, explicitly owned by whoever needs randomness.
// The simulation never touches a global generator, because a global generator
// is shared mutable state and would break determinism the moment two worlds run
// on two threads.
class Rng {
public:
    constexpr explicit Rng(std::uint64_t seed = 0) : state_(seed) {}

    constexpr std::uint64_t next() {
        state_ += 0x9E3779B97F4A7C15ull;
        std::uint64_t z = state_;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return z ^ (z >> 31);
    }

    // Uniform in [0, 1).
    Real nextReal() {
        return static_cast<Real>(next() >> 11) * (Real(1) / Real(9007199254740992.0));
    }

    Real nextRange(Real lo, Real hi) { return lo + (hi - lo) * nextReal(); }

    constexpr std::uint64_t state() const { return state_; }

private:
    std::uint64_t state_;
};

}  // namespace rf
