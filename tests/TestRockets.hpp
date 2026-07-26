#pragma once

#include <doctest/doctest.h>

#include <string_view>

#include "data/RocketCatalogue.hpp"

namespace rf::testing {

// Tests fly the same JSON definitions the game does. That is deliberate: it
// means a data file that is broken, or a rocket whose numbers stop making sense,
// fails the suite rather than only failing in the window.
inline const RocketSpec& rocket(std::string_view name) {
    const RocketSpec* spec = RocketCatalogue::instance().find(name);
    REQUIRE_MESSAGE(spec != nullptr, "rocket definition not found: ", std::string(name));
    return *spec;
}

inline const RocketSpec& classic() { return rocket("classic"); }

}  // namespace rf::testing
