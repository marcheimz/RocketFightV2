#include "render/Camera.hpp"

namespace rf {

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
