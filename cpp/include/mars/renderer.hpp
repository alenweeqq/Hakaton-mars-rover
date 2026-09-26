#pragma once

#include <cstdint>
#include "mars/env.hpp"

namespace mars {

struct RenderConfig {
  int width = 640;
  int height = 360;
  float pixels_per_meter = 80.0f;
  bool debug_overlay = false;
};

class Renderer {
 public:
  explicit Renderer(RenderConfig config = {});
  void set_debug_overlay(bool value) { config_.debug_overlay = value; }
  void render_rgb(const Env& env, uint8_t* rgb_out, int width, int height);

 private:
  RenderConfig config_{};
};

}
