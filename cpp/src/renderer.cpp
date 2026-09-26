#include "mars/renderer.hpp"
#include "mars/biome_bank.hpp"

#include <algorithm>
#include <cmath>

namespace mars {
namespace {

struct Color { uint8_t r, g, b; };

constexpr Color kWaterShadow{8, 17, 28};
constexpr Color kWaterHighlight{184, 226, 238};

struct SurfaceStyle {
  Color ground;
  Color dust;
  Color liquid_color;
  BiomeVisuals particles;
  bool liquid;
};

Color color(BiomeColor c) { return {c.r, c.g, c.b}; }

Color mix(Color from, Color to, float amount) {
  const float t = clamp(amount, 0.0f, 1.0f);
  return {
      static_cast<uint8_t>(from.r + (static_cast<float>(to.r) - from.r) * t),
      static_cast<uint8_t>(from.g + (static_cast<float>(to.g) - from.g) * t),
      static_cast<uint8_t>(from.b + (static_cast<float>(to.b) - from.b) * t),
  };
}

SurfaceStyle surface_style(int biome_id) {
  const auto visuals = biome_by_id(biome_id).visuals();
  return {color(visuals.ground), color(visuals.particles), color(visuals.liquid),
          visuals, visuals.liquid_surface};
}

inline void put_pixel(uint8_t* rgb, int width, int height, int x, int y, Color c) {
  if (static_cast<unsigned>(x) >= static_cast<unsigned>(width) ||
      static_cast<unsigned>(y) >= static_cast<unsigned>(height)) return;
  const size_t idx = static_cast<size_t>((y * width + x) * 3);
  rgb[idx] = c.r; rgb[idx + 1] = c.g; rgb[idx + 2] = c.b;
}

void draw_line(uint8_t* rgb, int width, int height, int x0, int y0, int x1, int y1, Color c) {
  const int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  const int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  for (;;) {
    put_pixel(rgb, width, height, x0, y0, c);
    if (x0 == x1 && y0 == y1) break;
    const int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
}

void draw_circle(uint8_t* rgb, int width, int height, int cx, int cy, int radius, Color c) {
  const int r2 = radius * radius;
  const int y0 = std::max(0, cy - radius), y1 = std::min(height - 1, cy + radius);
  for (int y = y0; y <= y1; ++y) {
    const int dy = y - cy;
    const int span = static_cast<int>(std::sqrt(static_cast<float>(r2 - dy * dy)));
    const int x0 = std::max(0, cx - span), x1 = std::min(width - 1, cx + span);
    for (int x = x0; x <= x1; ++x) put_pixel(rgb, width, height, x, y, c);
  }
}

uint32_t hash_u32(uint32_t x) {
  x ^= x >> 16;
  x *= 0x7feb352du;
  x ^= x >> 15;
  x *= 0x846ca68bu;
  x ^= x >> 16;
  return x;
}

void draw_rotated_box(uint8_t* rgb, int width, int height, int cx, int cy, float angle,
                      float hw, float hh, Color c) {
  const float ca = std::cos(-angle), sa = std::sin(-angle);
  const int radius = static_cast<int>(std::ceil(std::sqrt(hw * hw + hh * hh)));
  const int y0 = std::max(0, cy - radius), y1 = std::min(height - 1, cy + radius);
  const int x0 = std::max(0, cx - radius), x1 = std::min(width - 1, cx + radius);
  for (int y = y0; y <= y1; ++y) for (int x = x0; x <= x1; ++x) {
    const float dx = static_cast<float>(x - cx), dy = static_cast<float>(y - cy);
    const float lx = ca * dx - sa * dy, ly = sa * dx + ca * dy;
    if (std::abs(lx) <= hw && std::abs(ly) <= hh) put_pixel(rgb, width, height, x, y, c);
  }
}

}

Renderer::Renderer(RenderConfig config) : config_(config) {}

void Renderer::render_rgb(const Env& env, uint8_t* rgb, int width, int height) {
  if (!rgb || width <= 0 || height <= 0) return;
  const auto& terrain = env.terrain();
  const auto& state = env.state();
  const auto& body_zone = env.mechanic_at(state.body.position.x);
  const auto body_style = surface_style(body_zone.biome_id);
  const Color sky = color(body_style.particles.sky);
  for (size_t i = 0; i < static_cast<size_t>(width) * height; ++i) {
    rgb[i * 3] = sky.r; rgb[i * 3 + 1] = sky.g; rgb[i * 3 + 2] = sky.b;
  }
  const auto& rig = env.config().rig;
  const float ppm = config_.pixels_per_meter;
  const float camera_x = state.render_camera_position.x - 3.0f;
  const float camera_y = state.render_camera_position.y - 1.8f;
  auto screen = [&](Vec2 p) {
    return Vec2{(p.x - camera_x) * ppm,
                static_cast<float>(height - 1) - (p.y - camera_y) * ppm};
  };

  for (int x = 0; x < width; ++x) {
    const float wx = camera_x + static_cast<float>(x) / ppm;
    const auto& zone = env.mechanic_at(wx);
    const float ground_height = terrain.query_near(wx, 1.0e6f).height;
    SurfaceStyle style = surface_style(zone.biome_id);
    if (zone.type == MechanicType::Liquid) style.liquid = true;
    if (zone.type == MechanicType::Liquid && zone.liquid_level - ground_height < 0.025f) {
      style = surface_style(builtin_biome_id(MechanicType::Sand));
    }
    const int gy = height - 1 - static_cast<int>((ground_height - camera_y) * ppm);
    const int start = std::max(0, gy);
    if (style.liquid) {
      const int water_start = std::max(
          0, height - 1 - static_cast<int>((zone.liquid_level - camera_y) * ppm));
      const int water_end = std::min(height, std::max(water_start, gy));
      for (int y = water_start; y < water_end; ++y) {
        const float depth = static_cast<float>(y - water_start) /
                            std::max(1.0f, static_cast<float>(water_end - water_start));
        const float ripple = 0.5f + 0.5f * std::sin(wx * 2.15f +
            static_cast<float>(state.step_index) * 0.075f + depth * 5.0f);
        const Color surface_tone = mix(style.liquid_color, kWaterHighlight, 0.18f);
        const Color shaded = mix(surface_tone, kWaterShadow, 0.10f + 0.46f * depth);
        put_pixel(rgb, width, height, x, y, mix(shaded, kWaterHighlight,
                                                (1.0f - depth) * ripple * 0.18f));
      }
    }
    for (int y = start; y < height; ++y) {
      const size_t i = static_cast<size_t>((y * width + x) * 3);
      rgb[i] = style.ground.r; rgb[i + 1] = style.ground.g; rgb[i + 2] = style.ground.b;
    }
  }




  if (state.thruster_thrust > 0.0f) {
    const Vec2 down = rotate({0.0f, -1.0f}, state.body.angle);
    const Vec2 side = rotate({1.0f, 0.0f}, state.body.angle);
    const Vec2 nozzle = state.body.position + down * 0.22f;
    for (int i = 0; i < 90; ++i) {
      const uint32_t h = hash_u32(static_cast<uint32_t>(i * 2657 + state.step_index * 61));
      const float along = static_cast<float>(h % 1000u) / 1000.0f;
      const float spread = (static_cast<float>((h >> 10) % 1000u) / 1000.0f - 0.5f) *
                           (0.10f + 0.34f * along);
      const Vec2 world = nozzle + down * (along * 1.5f) + side * spread;
      const Vec2 screen_point = screen(world);
      const Color hot{255, static_cast<uint8_t>(230 - 120 * along),
                      static_cast<uint8_t>(140 - 120 * along)};
      const Color cool{140, 120, 190};
      put_pixel(rgb, width, height, static_cast<int>(screen_point.x),
                static_cast<int>(screen_point.y), mix(hot, cool, along * along));
    }
  }

  if (body_style.particles.ambient_particles > 0) {


    for (int p = 0; p < body_style.particles.ambient_particles; ++p) {
      const uint32_t h = hash_u32(static_cast<uint32_t>(p * 977 + state.step_index / 2));
      const int drift = static_cast<int>(state.step_index * body_style.particles.ambient_drift);
      const int x = (static_cast<int>(h & 1023u) + drift) % (width + 24) - 12;
      const int y = static_cast<int>((h >> 10) % static_cast<uint32_t>(std::max(1, height)));
      const int length = 2 + static_cast<int>((h >> 20) & 7u);
      draw_line(rgb, width, height, x, y, x + length, y - 1, body_style.dust);
    }
  }

  for (int i = 0; i < state.wheel_count; ++i) {
    const auto& wheel = state.wheels[static_cast<size_t>(i)];
    const Vec2 wheel_position = wheel.position;
    if (i < static_cast<int>(rig.wheels.size())) {
      const auto& wr = rig.wheels[static_cast<size_t>(i)];
      const Vec2 anchor =
          screen(state.body.position + rotate(wr.local_anchor, state.body.angle));
      const Vec2 wp = screen(wheel_position);
      draw_line(rgb, width, height, static_cast<int>(anchor.x), static_cast<int>(anchor.y),
                static_cast<int>(wp.x), static_cast<int>(wp.y), {70, 70, 70});
    }
    const Vec2 wp = screen(wheel_position);
    const int radius = std::max(2, static_cast<int>(wheel.radius * ppm));
    draw_circle(rgb, width, height, static_cast<int>(wp.x), static_cast<int>(wp.y), radius,
                {25, 25, 25});
    draw_circle(rgb, width, height, static_cast<int>(wp.x), static_cast<int>(wp.y),
                std::max(1, radius / 2), {120, 120, 120});


    const float spoke_angle = wheel.angle;
    const int sx = static_cast<int>(wp.x + std::cos(spoke_angle) * radius);
    const int sy = static_cast<int>(wp.y - std::sin(spoke_angle) * radius);
    draw_line(rgb, width, height, static_cast<int>(wp.x), static_cast<int>(wp.y), sx, sy,
              {230, 230, 230});

    if (wheel.in_contact) {
      const auto& wheel_zone = env.mechanic_at(wheel_position.x);
      const float wheel_ground_height = terrain.query(wheel_position.x).height;
      const int wheel_biome_id =
          wheel_zone.type == MechanicType::Liquid &&
                  wheel_zone.liquid_level - wheel_ground_height < 0.025f
              ? builtin_biome_id(MechanicType::Sand)
              : wheel_zone.biome_id;
      const auto wheel_style = surface_style(wheel_biome_id);
      const auto ground = terrain.query(wheel_position.x);
      const Vec2 tangent = normalized({ground.normal.y, -ground.normal.x});
      const float rolling_speed = dot(state.body.velocity, tangent);
      const float dust_speed = std::abs(rolling_speed);
      const Vec2 spray_dir = tangent * (rolling_speed >= 0.0f ? -1.0f : 1.0f);
      const bool in_water = wheel_zone.type == MechanicType::Liquid &&
          wheel_zone.liquid_level - ground.height >= 0.03f;
      if (in_water) {


        const int foam_count = std::clamp(1 + static_cast<int>(dust_speed * 4.0f), 1, 10);
        const Vec2 origin{wheel_position.x, wheel_zone.liquid_level - 0.015f};
        for (int p = 0; p < foam_count; ++p) {
          const uint32_t h = hash_u32(static_cast<uint32_t>(state.step_index * 53 + i * 197 + p * 31));
          const float along = 0.08f + static_cast<float>(h & 255u) / 255.0f *
                               (0.25f + dust_speed * 0.16f);
          const float lift = static_cast<float>((h >> 8) & 63u) / 63.0f * 0.07f;
          const Vec2 pp = screen(origin + spray_dir * along + Vec2{0.0f, lift});
          draw_circle(rgb, width, height, static_cast<int>(pp.x), static_cast<int>(pp.y),
                      p % 4 == 0 ? 2 : 1, mix(wheel_style.liquid_color, kWaterHighlight, 0.72f));
        }
      } else {
        const float motion = clamp((dust_speed - 0.20f) / 2.2f, 0.0f, 1.0f);
        const float slip = clamp(wheel.slip, 0.0f, 1.0f);
        const int particle_count = std::clamp(
            static_cast<int>(motion * (2.0f + wheel_style.particles.base_particles * 0.35f +
                                       dust_speed * wheel_style.particles.particle_rate *
                                           (0.25f + 0.75f * slip))),
            0, wheel_style.particles.max_particles);
        const Vec2 origin{wheel_position.x, ground.height + wheel.radius * 0.10f};
        for (int p = 0; p < particle_count; ++p) {
          const uint32_t h = hash_u32(static_cast<uint32_t>(state.step_index * 37 + i * 101 + p * 17));
          const float rx = static_cast<float>(h & 255u) / 255.0f;
          const float ry = static_cast<float>((h >> 8) & 255u) / 255.0f;
          const float distance = wheel.radius + 0.05f +
                                 rx * (0.14f + dust_speed * 0.08f) *
                                     wheel_style.particles.particle_spread;
          const float lift = ry * (0.03f + dust_speed * 0.024f) *
                             wheel_style.particles.particle_lift;
          const Vec2 pp = screen(origin + spray_dir * distance + ground.normal * lift);
          const int px = static_cast<int>(pp.x);
          const int py = static_cast<int>(pp.y);
          put_pixel(rgb, width, height, px, py, wheel_style.dust);
          if (p % 5 == 0) put_pixel(rgb, width, height, px + 1, py, wheel_style.dust);
        }
      }
    }
  }

  const Vec2 bp = screen(state.body.position);
  draw_rotated_box(rgb, width, height, static_cast<int>(bp.x), static_cast<int>(bp.y),
                   -state.body.angle, rig.body.size.x * 0.5f * ppm,
                   rig.body.size.y * 0.5f * ppm, {210, 105, 25});

  if (state.ballast_blowing && env.mechanic_at(state.body.position.x).type == MechanicType::Liquid) {
    for (int bubble = 0; bubble < 7; ++bubble) {
      const float phase = static_cast<float>(state.step_index) * 0.045f + bubble * 0.83f;
      const Vec2 bubble_world = state.body.position +
          Vec2{-0.45f + 0.15f * bubble + 0.06f * std::sin(phase * 1.7f),
               0.18f + std::fmod(phase, 1.1f)};
      const Vec2 bubble_screen = screen(bubble_world);
      draw_circle(rgb, width, height, static_cast<int>(bubble_screen.x),
                  static_cast<int>(bubble_screen.y), 2, {155, 220, 245});
    }
  }

  if (state.roof_piston_extension > 0.001f) {
    const Vec2 axis = rotate({0.0f, 1.0f}, state.body.angle);
    for (int piston = 0; piston < 2; ++piston) {
      if ((state.roof_piston_mask & (1 << piston)) == 0) continue;
      const float local_x = piston == 0 ? rig.body.size.x * 0.34f : -rig.body.size.x * 0.34f;
      const Vec2 base_world = state.body.position +
          rotate({local_x, rig.body.size.y * 0.5f}, state.body.angle);
      const Vec2 tip_world = base_world + axis * (0.14f + 0.36f * state.roof_piston_extension);
      const Vec2 tip = screen(tip_world);
      const Color rod_color = state.roof_piston_contact ? Color{196, 142, 48} : Color{82, 89, 100};
      const Vec2 cylinder_center_world = base_world + axis * 0.09f;
      const Vec2 cylinder_center = screen(cylinder_center_world);
      draw_rotated_box(rgb, width, height, static_cast<int>(cylinder_center.x),
                       static_cast<int>(cylinder_center.y), -state.body.angle,
                       std::max(2.0f, 0.075f * ppm), std::max(3.0f, 0.10f * ppm),
                       {75, 84, 98});
      const Vec2 rod_base = screen(base_world + axis * 0.14f);
      draw_line(rgb, width, height, static_cast<int>(rod_base.x), static_cast<int>(rod_base.y),
                static_cast<int>(tip.x), static_cast<int>(tip.y), rod_color);
      draw_circle(rgb, width, height, static_cast<int>(tip.x), static_cast<int>(tip.y), 3, rod_color);
    }
  }

  {
    const Vec2 hub_world = state.body.position +
        rotate({-rig.body.size.x * 0.58f, 0.02f}, state.body.angle);
    const Vec2 hub = screen(hub_world);
    const float blade_length = 0.06f * ppm + 0.28f * ppm * state.propeller_deployment;
    for (int blade = 0; blade < 3; ++blade) {
      const float angle = state.propeller_phase + static_cast<float>(blade) * 2.0943951f - state.body.angle;
      const int bx = static_cast<int>(hub.x + std::cos(angle) * blade_length);
      const int by = static_cast<int>(hub.y - std::sin(angle) * blade_length);
      draw_line(rgb, width, height, static_cast<int>(hub.x), static_cast<int>(hub.y), bx, by,
                {64, 70, 78});
    }
    draw_circle(rgb, width, height, static_cast<int>(hub.x), static_cast<int>(hub.y), 4, {148, 154, 166});
  }

  if (state.solar_panel_deployment > 0.001f) {
    const float deployment = state.solar_panel_deployment;
    const Vec2 mast_base_world =
        state.body.position + rotate({0.0f, rig.body.size.y * 0.5f}, state.body.angle);
    const Vec2 panel_center_world =
        state.body.position +
        rotate({0.0f, rig.body.size.y * 0.5f + 0.08f + 0.34f * deployment},
               state.body.angle);
    const Vec2 mast_base = screen(mast_base_world);
    const Vec2 panel_center = screen(panel_center_world);
    draw_line(rgb, width, height, static_cast<int>(mast_base.x), static_cast<int>(mast_base.y),
              static_cast<int>(panel_center.x), static_cast<int>(panel_center.y), {80, 80, 88});
    const float panel_half_width = (0.12f + 0.72f * deployment) * ppm;
    draw_rotated_box(rgb, width, height, static_cast<int>(panel_center.x),
                     static_cast<int>(panel_center.y), -state.body.angle,
                     panel_half_width, std::max(2.0f, 0.055f * ppm), {36, 82, 142});
    const Vec2 panel_left =
        screen(panel_center_world + rotate({-panel_half_width / ppm, 0.0f}, state.body.angle));
    const Vec2 panel_right =
        screen(panel_center_world + rotate({panel_half_width / ppm, 0.0f}, state.body.angle));
    draw_line(rgb, width, height, static_cast<int>(panel_left.x), static_cast<int>(panel_left.y),
              static_cast<int>(panel_right.x), static_cast<int>(panel_right.y), {105, 175, 225});
  }



  for (int x = 0; x < width; ++x) {
    const float wx = camera_x + static_cast<float>(x) / ppm;
    const auto& zone = env.mechanic_at(wx);
    if (zone.type != MechanicType::Liquid) continue;
    const int surface_y = std::max(
        0, height - 1 - static_cast<int>((zone.liquid_level - camera_y) * ppm));
    const float wave = std::sin(wx * 2.15f + static_cast<float>(state.step_index) * 0.075f) +
                       0.45f * std::sin(wx * 5.7f - static_cast<float>(state.step_index) * 0.11f);
    const int wave_y = surface_y + static_cast<int>(std::round(wave * 1.6f));
    const auto style = surface_style(zone.biome_id);
    put_pixel(rgb, width, height, x, wave_y, mix(style.liquid_color, kWaterHighlight, 0.58f));
    const uint32_t h = hash_u32(static_cast<uint32_t>(x * 193 + state.step_index / 3));
    if ((h & 31u) == 0u) {
      draw_line(rgb, width, height, x, wave_y + 3,
                x + 3 + static_cast<int>((h >> 8) & 7u), wave_y + 3,
                mix(style.liquid_color, kWaterHighlight, 0.32f));
    }
  }





  constexpr float kMinVisibleBrightness = 0.70f;
  const float brightness =
      clamp(body_style.particles.screen_brightness, kMinVisibleBrightness, 1.5f);
  for (size_t i = 0; i < static_cast<size_t>(width) * height * 3; ++i) {
    rgb[i] = static_cast<uint8_t>(clamp(static_cast<float>(rgb[i]) * brightness, 0.0f, 255.0f));
  }

  if (state.lidar_active_steps > 0 && state.lidar_range > 0.0f && state.airborne) {
    const Vec2 origin = screen(state.body.position);
    const float span = state.lidar_landing_valid
                           ? state.lidar_landing_x - state.body.position.x
                           : 0.0f;
    constexpr int kRays = 18;
    for (int ray = 1; ray <= kRays; ++ray) {
      const float x = state.body.position.x + span * static_cast<float>(ray) / kRays;
      const auto sample = terrain.query(x);
      if (!sample.solid) continue;
      const Vec2 hit = screen({x, sample.height});
      draw_line(rgb, width, height, static_cast<int>(origin.x), static_cast<int>(origin.y),
                static_cast<int>(hit.x), static_cast<int>(hit.y), {62, 255, 124});
    }
    if (state.lidar_landing_valid) {
      const Vec2 mark = screen({state.lidar_landing_x, state.lidar_landing_y});
      const int mx = static_cast<int>(mark.x);
      const int my = static_cast<int>(mark.y);
      draw_line(rgb, width, height, mx - 9, my, mx + 9, my, {255, 96, 96});
      draw_line(rgb, width, height, mx, my - 9, mx, my + 9, {255, 96, 96});
    }
  } else if (state.lidar_active_steps > 0 && state.lidar_range > 0.0f) {
    const Vec2 local_dir{1.0f, 0.0f};
    const Vec2 world_dir = rotate(local_dir, state.body.angle);
    const Vec2 origin = screen(state.body.position + world_dir * (rig.body.size.x * 0.45f));
    constexpr int kRays = 18;
    const float near_range = std::max(0.0f, env.config().physics.near_sense_range);
    const float far_span = std::max(0.0f, state.lidar_range - near_range);
    for (int ray = 1; ray <= kRays; ++ray) {
      const float distance = near_range + far_span * static_cast<float>(ray) / kRays;
      const Vec2 sample = state.body.position + world_dir * distance;
      const float x = sample.x;
      const Vec2 hit = screen({x, terrain.query(x).height});
      draw_line(rgb, width, height, static_cast<int>(origin.x), static_cast<int>(origin.y),
                static_cast<int>(hit.x), static_cast<int>(hit.y), {62, 255, 124});
    }
  }

}

}
