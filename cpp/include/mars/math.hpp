#pragma once

#include <algorithm>
#include <cmath>

namespace mars {

struct Vec2 {
  float x = 0.0f;
  float y = 0.0f;

  constexpr Vec2() = default;
  constexpr Vec2(float x_, float y_) : x(x_), y(y_) {}

  constexpr Vec2 operator+(Vec2 r) const { return {x + r.x, y + r.y}; }
  constexpr Vec2 operator-(Vec2 r) const { return {x - r.x, y - r.y}; }
  constexpr Vec2 operator-() const { return {-x, -y}; }
  constexpr Vec2 operator*(float s) const { return {x * s, y * s}; }
  constexpr Vec2 operator/(float s) const { return {x / s, y / s}; }
  Vec2& operator+=(Vec2 r) {
    x += r.x;
    y += r.y;
    return *this;
  }
  Vec2& operator-=(Vec2 r) {
    x -= r.x;
    y -= r.y;
    return *this;
  }
  Vec2& operator*=(float s) {
    x *= s;
    y *= s;
    return *this;
  }
};

inline float dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
inline float cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }
inline Vec2 perp(Vec2 v) { return {-v.y, v.x}; }
inline float length(Vec2 v) { return std::sqrt(dot(v, v)); }
inline Vec2 normalized(Vec2 v) {
  const float len = length(v);
  return len > 1.0e-6f ? v / len : Vec2{0.0f, 1.0f};
}
inline Vec2 rotate(Vec2 v, float angle) {
  const float c = std::cos(angle);
  const float s = std::sin(angle);
  return {c * v.x - s * v.y, s * v.x + c * v.y};
}
inline float clamp(float v, float lo, float hi) {
  return std::max(lo, std::min(hi, v));
}

}
