#pragma once

#include <vector>

#include "mars/math.hpp"

namespace mars {

enum class CollisionType { None, Box, Circle };

struct CollisionShapeConfig {
  CollisionType type = CollisionType::None;
  Vec2 size{};
  float radius = 0.0f;
};

struct BodyRigConfig {
  float mass = 12.0f;
  float inertia = 4.0f;
  Vec2 size{1.8f, 0.5f};
  CollisionShapeConfig collision{CollisionType::Box, {1.8f, 0.5f}, 0.0f};
  Vec2 local_position{};
};

struct SuspensionRigConfig {
  float rest_length = 0.28f;
  float min_length = 0.18f;
  float max_length = 0.36f;
  float stiffness = 120.0f;
  float damping = 12.0f;
};

struct WheelRigConfig {
  float radius = 0.24f;
  float mass = 1.0f;
  Vec2 local_anchor{};
  SuspensionRigConfig suspension{};
};

struct RoverRig {
  BodyRigConfig body{};
  std::vector<WheelRigConfig> wheels;

  static RoverRig default_two_wheel();
};

}
