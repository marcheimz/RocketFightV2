#pragma once

#include <bit>
#include <cstdint>

#include "core/Types.hpp"

namespace rf {

// FNV-1a over the bit patterns of the state. Hashing the *bits* rather than
// comparing values is the point: it turns "did these two runs diverge at all"
// into a single integer comparison, which is what makes determinism cheap
// enough to assert on every episode instead of only in tests.
class Hasher {
public:
    constexpr void feed(std::uint64_t v) {
        for (int i = 0; i < 8; ++i) {
            h_ ^= (v >> (i * 8)) & 0xFFull;
            h_ *= 0x100000001B3ull;
        }
    }

    void feed(Real v) {
        // -0.0 and +0.0 compare equal but have different bits. Normalise, so a
        // sign flip through zero is not reported as a divergence.
        if (v == Real(0)) v = Real(0);
        feed(std::bit_cast<std::uint64_t>(static_cast<double>(v)));
    }

    constexpr std::uint64_t value() const { return h_; }

private:
    std::uint64_t h_{0xCBF29CE484222325ull};
};

}  // namespace rf
