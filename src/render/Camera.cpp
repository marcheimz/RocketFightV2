#include "render/Camera.hpp"

namespace rf {

void Camera::setViewportPixels(sf::Vector2u px) {
    if (px.x > 0 && viewport_.x > 0 && px.x != viewport_.x) {
        setWidthMeters(widthMeters_ * static_cast<Real>(px.x) / static_cast<Real>(viewport_.x));
    }
    viewport_ = px;
}

void Camera::zoomBy(Real factor) {
    widthMeters_ = clamp(widthMeters_ * factor, kMinWidth, kMaxWidth);
}

void Camera::setWidthMeters(Real metres) {
    widthMeters_ = clamp(metres, kMinWidth, kMaxWidth);
}

Real Camera::metersPerPixel() const {
    return widthMeters_ / static_cast<Real>(viewport_.x == 0 ? 1 : viewport_.x);
}

sf::View Camera::view() const {
    const Real aspect = viewport_.x == 0
                            ? Real(1)
                            : static_cast<Real>(viewport_.y) / static_cast<Real>(viewport_.x);

    sf::View v;
    v.setCenter(toView(center_));
    v.setSize({static_cast<float>(widthMeters_), static_cast<float>(widthMeters_ * aspect)});
    return v;
}

}  // namespace rf
