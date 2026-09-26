#include "mars/rover_rig.hpp"

namespace mars {

RoverRig RoverRig::default_two_wheel() {
  RoverRig rig{};
  rig.body.mass = 12.0f;
  rig.body.inertia = 4.0f;
  rig.body.size = {1.8f, 0.5f};
  rig.body.collision = {CollisionType::Box, {1.8f, 0.5f}, 0.0f};
  rig.wheels = {
      {0.24f, 1.0f, {0.65f, -0.30f}, {0.28f, 0.18f, 0.36f, 360.0f, 36.0f}},
      {0.24f, 1.0f, {-0.65f, -0.30f}, {0.28f, 0.18f, 0.36f, 360.0f, 36.0f}},
  };
  return rig;
}

}
