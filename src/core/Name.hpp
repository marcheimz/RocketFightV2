#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstring>
#include <string_view>

namespace rf {

// A short, fixed-capacity, inline name.
//
// Names live inside RocketSpec and Thruster, which live inside every Snapshot,
// which is published to the render thread a thousand times a second. A
// std::string here would make that publish allocate, so the storage is inline
// and the type stays trivially copyable. Names are labels for humans, so
// truncating an over-long one is the right failure mode -- there is nothing to
// report and nothing downstream cares.
template <std::size_t Capacity>
struct InlineName {
    static_assert(Capacity >= 2, "a name needs at least one character and a terminator");

    // Always zero-initialised, so `view()` always finds a terminator.
    std::array<char, Capacity> chars{};

    static InlineName from(std::string_view text) {
        InlineName        out;
        const std::size_t n = std::min(text.size(), Capacity - 1);
        if (n > 0) std::memcpy(out.chars.data(), text.data(), n);
        return out;
    }

    std::string_view view() const { return std::string_view(chars.data()); }
    bool             empty() const { return chars[0] == '\0'; }
};

// Distinct capacities rather than one size for both: there are up to sixteen
// thruster names per spec and only one rocket name, so the per-thruster one is
// the one worth keeping small.
using RocketName   = InlineName<24>;
using ThrusterName = InlineName<20>;

}  // namespace rf
