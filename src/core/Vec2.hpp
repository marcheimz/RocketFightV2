#pragma once

#include <cmath>

#include "core/Types.hpp"

namespace rf {

// World space is y-up (the usual mathematical convention, which keeps angles,
// cross products and orbital mechanics sane). The renderer is the only place
// that flips to SFML's y-down screen space.
struct Vec2 {
    Real x{};
    Real y{};

    constexpr Vec2() = default;
    constexpr Vec2(Real xx, Real yy) : x(xx), y(yy) {}

    constexpr Vec2& operator+=(Vec2 o) { x += o.x; y += o.y; return *this; }
    constexpr Vec2& operator-=(Vec2 o) { x -= o.x; y -= o.y; return *this; }
    constexpr Vec2& operator*=(Real s) { x *= s;   y *= s;   return *this; }
    constexpr Vec2& operator/=(Real s) { x /= s;   y /= s;   return *this; }
};

constexpr Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
constexpr Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
constexpr Vec2 operator*(Vec2 v, Real s) { return {v.x * s, v.y * s}; }
constexpr Vec2 operator*(Real s, Vec2 v) { return {v.x * s, v.y * s}; }
constexpr Vec2 operator/(Vec2 v, Real s) { return {v.x / s, v.y / s}; }
constexpr Vec2 operator-(Vec2 v)         { return {-v.x, -v.y}; }

constexpr bool operator==(Vec2 a, Vec2 b) { return a.x == b.x && a.y == b.y; }

constexpr Real dot(Vec2 a, Vec2 b)   { return a.x * b.x + a.y * b.y; }
constexpr Real cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }
constexpr Real lengthSq(Vec2 v)      { return dot(v, v); }

inline Real length(Vec2 v) { return std::sqrt(lengthSq(v)); }

// Perpendicular, rotated +90 degrees.
constexpr Vec2 perp(Vec2 v) { return {-v.y, v.x}; }

// Safe normalise: a zero vector has no direction, so the caller says what it
// wants instead of getting a NaN.
inline Vec2 normalizedOr(Vec2 v, Vec2 fallback) {
    const Real len = length(v);
    return len > Real(0) ? v / len : fallback;
}

inline Vec2 fromAngle(Real a)        { return {std::cos(a), std::sin(a)}; }
inline Real angleOf(Vec2 v)          { return std::atan2(v.y, v.x); }

inline Vec2 rotate(Vec2 v, Real a) {
    const Real c = std::cos(a);
    const Real s = std::sin(a);
    return {v.x * c - v.y * s, v.x * s + v.y * c};
}

// Wrap to [-pi, pi). Used on every angle the simulation stores, so heading
// errors are always the short way round and stored angles never drift unbounded.
// The half-open end is a convention: exactly half a turn is reported as -pi.
inline Real wrapPi(Real a) {
    a = std::fmod(a + kPi, Real(2) * kPi);
    if (a < Real(0)) a += Real(2) * kPi;
    return a - kPi;
}

constexpr Real clamp(Real v, Real lo, Real hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// Clamp a vector's magnitude without changing its direction.
inline Vec2 clampLength(Vec2 v, Real maxLen) {
    const Real len = length(v);
    return (len > maxLen && len > Real(0)) ? v * (maxLen / len) : v;
}

constexpr Real sign(Real v) { return v < Real(0) ? Real(-1) : (v > Real(0) ? Real(1) : Real(0)); }

}  // namespace rf
