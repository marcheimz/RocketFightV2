#include <doctest/doctest.h>

#include "core/Vec2.hpp"

using namespace rf;

TEST_CASE("cross product is rotation invariant") {
    // The whole torque calculation leans on this: applyForceAt rotates the lever
    // arm and the force into world space, and the resulting torque must not
    // depend on the body's orientation.
    const Vec2 a{3, 1};
    const Vec2 b{-2, 4};
    for (Real angle = -3.0; angle < 3.0; angle += 0.37) {
        CHECK(cross(rotate(a, angle), rotate(b, angle)) == doctest::Approx(cross(a, b)));
    }
}

TEST_CASE("wrapPi maps onto the short way round") {
    CHECK(wrapPi(Real(0)) == doctest::Approx(0));
    CHECK(wrapPi(kPi * Real(2)) == doctest::Approx(0));
    CHECK(wrapPi(-kPi * Real(1.5)) == doctest::Approx(kPi * Real(0.5)));

    // Range is [-pi, pi), so exactly half a turn lands on -pi rather than +pi.
    CHECK(wrapPi(kPi * Real(3)) == doctest::Approx(-kPi));
    CHECK(wrapPi(kPi) == doctest::Approx(-kPi));

    // A heading error just past half a turn must come back as a small negative
    // angle, not a nearly-full positive one -- otherwise the fly-by-wire would
    // slew the long way round.
    CHECK(wrapPi(kPi + Real(0.1)) == doctest::Approx(-kPi + Real(0.1)));
}

TEST_CASE("normalizedOr does not produce NaN for a zero vector") {
    const Vec2 fallback{1, 0};
    CHECK(normalizedOr(Vec2{}, fallback) == fallback);
    CHECK(length(normalizedOr(Vec2{3, 4}, fallback)) == doctest::Approx(1.0));
}

TEST_CASE("clampLength preserves direction") {
    const Vec2 v{30, 40};  // length 50
    const Vec2 c = clampLength(v, Real(10));
    CHECK(length(c) == doctest::Approx(10.0));
    CHECK(cross(v, c) == doctest::Approx(0.0));
    CHECK(clampLength(v, Real(100)) == v);
}

TEST_CASE("fromAngle and angleOf round-trip") {
    for (Real a = -3.0; a < 3.0; a += 0.41) {
        CHECK(angleOf(fromAngle(a)) == doctest::Approx(a));
    }
}
