#pragma once

#include <optional>

#include "core/Vec2.hpp"

namespace rf {

// Layer 1: what the pilot wants, expressed with no reference to thrusters,
// gimbals or torque. The same Intent is meaningful for any vehicle -- which is
// the entire point of having this layer.
struct Intent {
    enum class Mode : std::uint8_t {
        Acceleration,   // vector is a world-frame direction, |v| <= 1,
                        // as a fraction of the vehicle's maximum acceleration
        Velocity,       // vector is a world-frame target velocity, in m/s
    };

    Mode mode{Mode::Acceleration};
    Vec2 vector{};

    // Desired heading. Empty means "don't care" -- the fly-by-wire is then free
    // to point the nose wherever thrusting requires.
    std::optional<Real> facing{};

    bool fire{};
};

}  // namespace rf
