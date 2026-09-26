#include "mars/env.hpp"
#include "mars/biome_bank.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <vector>

namespace mars {

void Env::update_world_latents() {
  std::array<const MechanicZone*, kMaxActiveMechanisms> active{};
  const int n = mechanic_layout_.active_layers(state_.body.position.x, active);
  state_.active_layer_count = n;
  float weight_sum = 0.0f;
  float sand_weight = 0.0f;

  MechanicParams mixed{};
  mixed.moisture = 0.22f;
  mixed.ambient_temperature = -21.0f;
  mixed.base_friction_mul = 1.0f;
  mixed.base_viscosity = 0.0f;
  mixed.base_energy_drain_mul = 1.0f;
  mixed.base_thermal_transfer = 1.0f;
  for (int i = 0; i < n; ++i) {
    const auto& z = *active[static_cast<size_t>(i)];
    const float feather = std::min(25.0f, std::max(15.0f, (z.end_x - z.begin_x) * 0.16f));
    const float enter = clamp((state_.body.position.x - z.begin_x) / feather, 0.0f, 1.0f);
    const float leave = clamp((z.end_x - state_.body.position.x) / feather, 0.0f, 1.0f);
    const float w = enter * enter * (3.0f - 2.0f * enter) * leave * leave * (3.0f - 2.0f * leave);
    weight_sum += w;
    if (config_.biome_split == 1 && z.type == MechanicType::Sand) sand_weight += w;
    const auto& p = z.params;
    mixed.moisture += w * (p.moisture - 0.22f);
    mixed.sink_rate += w * p.sink_rate;
    mixed.ambient_temperature += w * (p.ambient_temperature + 21.0f);
    mixed.base_friction_mul += w * (p.base_friction_mul - 1.0f);
    mixed.base_viscosity += w * p.base_viscosity;
    mixed.base_energy_drain_mul += w * (p.base_energy_drain_mul - 1.0f);
    mixed.base_thermal_transfer += w * (p.base_thermal_transfer - 1.0f);
    mixed.gravity_mul += w * (p.gravity_mul - 1.0f);
    mixed.wind_force += w * p.wind_force;
    mixed.solar_charge_rate += w * (p.solar_charge_rate - 1.0f);
    mixed.lidar_energy_mul += w * (p.lidar_energy_mul - 1.0f);
    mixed.lidar_range_mul += w * (p.lidar_range_mul - 1.0f);
  }
  apply_generation_influences(mixed);
  const float speed = std::abs(state_.body.velocity.x);
  if (config_.biome_split == 1 && sand_weight > 0.0f) {
    const float stillness = clamp((0.70f - speed) / 0.70f, 0.0f, 1.0f);
    sand_burial_ = clamp(sand_burial_ + config_.physics.dt *
        (0.060f * clamp(sand_weight, 0.0f, 1.0f) * stillness -
         0.14f * speed), 0.0f, 1.0f);
  } else {
    sand_burial_ = std::max(0.0f, sand_burial_ - config_.physics.dt * 0.30f);
  }
  const auto& current_zone = mechanic_layout_.at(state_.body.position.x);
  mixed.lidar_range_mul = std::min(mixed.lidar_range_mul,
                                   current_zone.params.lidar_range_mul);
  state_.latent_moisture = mixed.moisture;
  state_.latent_heat = clamp((mixed.ambient_temperature + 58.0f) / 130.0f, 0.0f, 1.0f);
  state_.latent_viscosity = mixed.viscosity + sand_burial_ * 8.0f;
  state_.latent_sink = mixed.sink_rate;
  state_.latent_charge_reserve = clamp(state_.energy / std::max(1.0f, config_.physics.energy_capacity), 0.0f, 1.0f);
  state_.latent_suspension = clamp(1.0f - mixed.sink_rate * 0.25f - sand_burial_ * 0.18f +
                                           (state_.climb_mode ? 0.12f : 0.0f),
                                   0.65f, 1.25f);
  state_.latent_traction = mixed.friction_mul * (1.0f - sand_burial_ * 0.70f);
  state_.latent_energy_resistance = mixed.energy_drain_mul * (1.0f + sand_burial_ * 0.40f);
  state_.latent_gravity_multiplier = mixed.gravity_mul;
  state_.latent_wind_force = mixed.wind_force;
  state_.latent_ambient_temperature = mixed.ambient_temperature;
  state_.latent_thermal_transfer = mixed.thermal_transfer;
  state_.latent_solar_rate = mixed.solar_charge_rate;
  state_.latent_lidar_energy_multiplier = mixed.lidar_energy_mul;
  state_.latent_lidar_range_multiplier = mixed.lidar_range_mul;
  state_.active_layer_weight = weight_sum;
}

float Env::course_difficulty(float x) const {
  const float course_length = std::max(1.0f, config_.terrain.length);
  const float progress = clamp(x / course_length, 0.0f, 1.0f);
  if (progress <= difficulty_safe_fraction_) return 0.0f;
  const float t = clamp((progress - difficulty_safe_fraction_) /
                            std::max(0.01f, 1.0f - difficulty_safe_fraction_),
                        0.0f, 1.0f);
  const float exponent = clamp(config_.difficulty_exponent, 0.5f, 4.0f);
  return clamp(0.68f * std::pow(t, exponent) +
                   0.32f * std::pow(t, exponent * 2.35f),
               0.0f, 1.0f);
}

}
