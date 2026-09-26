#include "mars/env.hpp"
#include "mars/biome_bank.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <vector>

namespace mars {

void Env::build_observation(float* obs_out) const {
  if (!obs_out) {
    return;
  }
  int k = 0;
  const float finish_scale = 1000.0f;
  const float energy_scale = std::max(1.0f, config_.physics.energy_capacity);
  obs_out[k++] = state_.body.position.x / finish_scale;
  obs_out[k++] = state_.body.position.y / 10.0f;
  obs_out[k++] = state_.body.velocity.x / 20.0f;
  obs_out[k++] = state_.body.velocity.y / 20.0f;
  obs_out[k++] = state_.body.angle;
  obs_out[k++] = state_.body.angular_velocity / 10.0f;
  obs_out[k++] = state_.energy / energy_scale;

  obs_out[k++] = state_.lidar_landing_valid
                     ? clamp((state_.lidar_landing_x - state_.body.position.x) / 50.0f,
                             -1.0f, 1.0f)
                     : 0.0f;
  for (int i = 0; i < kMaxWheels; ++i) {
    const auto& w = state_.wheels[static_cast<size_t>(i)];
    obs_out[k++] = (i < state_.wheel_count && w.in_contact) ? 1.0f : 0.0f;
    obs_out[k++] = i < state_.wheel_count ? w.slip : 0.0f;
    obs_out[k++] = i < state_.wheel_count ? w.normal_force / 200.0f : 0.0f;
  }
  const bool lidar_active = state_.lidar_active_steps > 0;
  const float near_range = std::max(0.0f, config_.physics.near_sense_range);
  const int height_base = k;
  const int slope_base = k + kTerrainSamplesAhead;
  for (int i = 0; i < kTerrainSamplesAhead; ++i) {

    const float distance = i < 12 ? 0.5f * static_cast<float>(i + 1)
                                  : 6.0f + 1.5f * static_cast<float>(i - 11);
    const auto sample = terrain_.query_near(state_.body.position.x + distance, 1.0e6f);
    const bool near_visible = distance <= near_range;
    const bool far_visible = lidar_active && distance > near_range &&
                             distance <= state_.lidar_range;
    const bool visible = near_visible || far_visible;
    obs_out[height_base + i] = visible && sample.solid
                                   ? sample.height - state_.body.position.y
                                   : 0.0f;
    obs_out[slope_base + i] = visible && sample.solid ? sample.slope : 0.0f;
  }
  k += kTerrainSamplesAhead * 2;
  for (int i = 0; i < kBiomeSamplesAhead; ++i) {
    const float distance = 3.0f * static_cast<float>(i + 1);
    const float sample_x = state_.body.position.x + distance;
    const bool near_visible = distance <= near_range;
    const bool far_visible = lidar_active && distance > near_range &&
                             distance <= state_.lidar_range;
    const bool visible = near_visible || far_visible;
    const auto sample = terrain_.query_near(sample_x, 1.0e6f);
    const auto behind = terrain_.query_near(sample_x - 0.25f, 1.0e6f);
    const auto ahead = terrain_.query_near(sample_x + 0.25f, 1.0e6f);

    obs_out[k++] = visible && sample.solid
                       ? 1.0f / (1.0f + std::abs(sample.slope))
                       : 0.0f;
    obs_out[k++] = visible
                       ? (sample.solid ? clamp((sample.height - state_.body.position.y) / 10.0f,
                                               -1.0f, 1.0f)
                                       : -1.0f)
                       : 0.0f;
    obs_out[k++] = visible && sample.solid && behind.solid && ahead.solid
                       ? clamp(std::abs(ahead.slope - behind.slope), 0.0f, 1.0f)
                       : 0.0f;
  }
  obs_out[k++] = static_cast<float>(state_.gear_index + 1) / 8.0f;
  obs_out[k++] = state_.driveline_load_factor;
  obs_out[k++] = state_.gear_energy_multiplier / 3.1f;
  obs_out[k++] = state_.engine_rpm / 9000.0f;
  obs_out[k++] = static_cast<float>(state_.drive_mode) * 0.5f;
  obs_out[k++] = state_.engine_stalled ? 1.0f : 0.0f;
  obs_out[k++] = state_.clutch_engagement;
  obs_out[k++] = state_.engine_temperature / 120.0f;
  obs_out[k++] = state_.heater_active ? 1.0f : 0.0f;

  obs_out[k++] = state_.engine_overheated ? 1.0f : 0.0f;
  obs_out[k++] = state_.engine_cold_locked ? 1.0f : 0.0f;
  obs_out[k++] = state_.solar_panel_deployment;
  obs_out[k++] = state_.charging_active ? 1.0f : 0.0f;
  obs_out[k++] = state_.solar_charge_rate / 3.0f;
  obs_out[k++] = lidar_active ? 1.0f : 0.0f;
  obs_out[k++] = state_.lidar_cooldown_steps > 0
                     ? static_cast<float>(state_.lidar_cooldown_steps) * config_.physics.dt
                     : 0.0f;
  obs_out[k++] = static_cast<float>(state_.previous_action) / 16777215.0f;
  obs_out[k++] = state_.last_reward;

  obs_out[k++] = clamp(state_.imu_acceleration.x / 20.0f, -4.0f, 4.0f);
  obs_out[k++] = clamp(state_.imu_acceleration.y / 20.0f, -4.0f, 4.0f);
  obs_out[k++] = clamp(state_.imu_angular_acceleration / 20.0f, -4.0f, 4.0f);
  obs_out[k++] = clamp(state_.imu_impact / 10.0f, 0.0f, 4.0f);

  obs_out[k++] = state_.body_contact_front ? 1.0f : 0.0f;
  obs_out[k++] = state_.body_contact_belly ? 1.0f : 0.0f;
  obs_out[k++] = state_.body_contact_rear ? 1.0f : 0.0f;

  for (int i = 0; i < kMaxWheels; ++i) {
    obs_out[k++] = i < state_.wheel_count
                       ? clamp(state_.wheels[static_cast<size_t>(i)].angular_velocity / 60.0f,
                               -4.0f, 4.0f)
                       : 0.0f;
  }
  for (int i = 0; i < kMaxWheels; ++i) {
    if (i >= state_.wheel_count || i >= static_cast<int>(config_.rig.wheels.size())) {
      obs_out[k++] = 0.0f;
      continue;
    }
    const auto& wheel = state_.wheels[static_cast<size_t>(i)];
    const auto& wheel_rig = config_.rig.wheels[static_cast<size_t>(i)];
    const Vec2 anchor = state_.body.position + rotate(wheel_rig.local_anchor, state_.body.angle);
    const Vec2 axis = rotate({0.0f, -1.0f}, state_.body.angle);
    const float travel = dot(wheel.position - anchor, axis);
    const float span = std::max(0.01f, wheel_rig.suspension.max_length -
                                           wheel_rig.suspension.min_length);
    obs_out[k++] = clamp((wheel_rig.suspension.max_length - travel) / span, 0.0f, 1.0f);
  }

  obs_out[k++] = state_.climb_mode ? 1.0f : 0.0f;
  obs_out[k++] = state_.propeller_mode ? 1.0f : 0.0f;
  obs_out[k++] = state_.jump_cooldown_steps * config_.physics.dt;
  obs_out[k++] = state_.recovery_state;
  obs_out[k++] = state_.energy / energy_scale;
  obs_out[k++] = clamp(state_.drive_fuel_rate, 0.0f, 4.0f) / 4.0f;
  obs_out[k++] = state_.airborne ? 1.0f : 0.0f;
  obs_out[k++] = state_.body_contact_front ? 1.0f : 0.0f;
  obs_out[k++] = state_.body_contact_belly ? 1.0f : 0.0f;
  obs_out[k++] = state_.body_contact_rear ? 1.0f : 0.0f;
  obs_out[k++] = state_.propeller_deployment;
  obs_out[k++] = state_.suspension_jump_charge;
  obs_out[k++] = state_.roof_piston_extension;
  obs_out[k++] = clamp(state_.energy_cost_rate, 0.0f, 32.0f) / 32.0f;
  obs_out[k++] = state_.ballast_air;
  assert(k == kObservationDim);
}

}
