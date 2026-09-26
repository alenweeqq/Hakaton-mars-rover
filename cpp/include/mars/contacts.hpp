#pragma once

#include <array>

#include "mars/math.hpp"
#include "mars/state.hpp"

namespace mars {

struct WheelContact {
  bool active = false;
  int wheel_index = -1;
  Vec2 point{};
  Vec2 normal{0.0f, 1.0f};
  Vec2 tangent{1.0f, 0.0f};
  float penetration = 0.0f;
  float ground_height = 0.0f;
  float slope = 0.0f;
  float normal_force = 0.0f;
  float slip = 0.0f;
};

using ContactArray = std::array<WheelContact, kMaxWheels>;

struct DeformationContact {
  bool active = false;
  float x = 0.0f;
  float penetration = 0.0f;



  float drive_effort = 0.0f;
  float slip = 0.0f;
};

using DeformationContactArray = std::array<DeformationContact, kMaxWheels>;

}
