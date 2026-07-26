#pragma once

#include <SFML/Graphics/View.hpp>
#include <SFML/System/Vector2.hpp>

#include "core/Vec2.hpp"

namespace rf {

// World space is y-up and measured in metres; SFML's is y-down and measured in
// pixels. That conversion happens here and nowhere else, so no other file has
// to remember which way up it is.
inline sf::Vector2f toView(Vec2 world) {
    return {static_cast<float>(world.x), static_cast<float>(-world.y)};
}

// Angles flip sign along with the y axis.
inline float toViewDegrees(Real worldRadians) {
    return static_cast<float>(-worldRadians * Real(180) / kPi);
}

class Camera {
public:
    void setViewportPixels(sf::Vector2u px) { viewport_ = px; }

    void follow(Vec2 target) { center_ = target; }

    // Multiplicative, so a wheel notch feels the same at every scale.
    void zoomBy(Real factor);

    void setWidthMeters(Real metres);

    sf::View view() const;

    Real widthMeters() const { return widthMeters_; }
    Real metersPerPixel() const;

    Vec2 center() const { return center_; }

private:
    static constexpr Real kMinWidth = Real(20);        // 20 m across
    static constexpr Real kMaxWidth = Real(4.0e6);     // twice the world bound

    Vec2         center_{};
    Real         widthMeters_{300};
    sf::Vector2u viewport_{1280, 720};
};

}  // namespace rf
