#include "mars/env.hpp"
#include "mars/biome_bank.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <vector>

namespace mars {

int Env::trial_step_budget() const {
  if (config_.termination.trial_time_limit <= 0.0f) {
    return 0;
  }
  const float dt = std::max(0.0001f, config_.physics.dt);
  return std::max(1, static_cast<int>(std::lround(config_.termination.trial_time_limit / dt)));
}

bool Env::trial_exhausted() const {
  const int budget = trial_step_budget();
  return budget > 0 && trial_steps_used_ >= budget;
}

float Env::trial_time_left() const {
  const int budget = trial_step_budget();
  if (budget <= 0) {
    return -1.0f;
  }
  return static_cast<float>(std::max(0, budget - trial_steps_used_)) * config_.physics.dt;
}

void Env::recover_from_pit(float recovery_x) {
  const float recovery_y = terrain_.query(recovery_x).height + 1.0f;
  state_.body.position = {recovery_x, recovery_y};
  state_.body.velocity = {};
  state_.body.angle = 0.0f;
  state_.body.angular_velocity = 0.0f;
  state_.render_camera_position = state_.body.position;
  for (int i = 0; i < state_.wheel_count; ++i) {
    const auto& wr = config_.rig.wheels[static_cast<size_t>(i)];
    auto& wheel = state_.wheels[static_cast<size_t>(i)];
    wheel.position = state_.body.position + wr.local_anchor +
                     Vec2{0.0f, -wr.suspension.rest_length};
    wheel.velocity = {};
    wheel.angular_velocity = 0.0f;
    wheel.in_contact = false;
    wheel.normal_force = 0.0f;
    wheel.slip = 0.0f;
  }
  state_.previous_x = recovery_x;
  state_.airborne = false;
  state_.has_grounded = false;
  state_.airborne_steps = 0;
  state_.landing_event = false;
  state_.landing_fatal = false;
  state_.fatal_error = false;
  state_.pit_recovery_event = true;
  ++state_.pit_recovery_count;
}

void Env::update_lidar_landing() {
  state_.lidar_landing_valid = false;
  state_.lidar_landing_x = state_.body.position.x;
  state_.lidar_landing_y = state_.body.position.y;
  if (state_.lidar_active_steps <= 0 || state_.lidar_range <= 0.0f || !state_.airborne) {
    return;
  }
  const float gravity = config_.physics.gravity * state_.latent_gravity_multiplier;
  if (gravity >= 0.0f) return;
  const float dt = std::max(0.0001f, config_.physics.dt);
  Vec2 position = state_.body.position;
  Vec2 velocity = state_.body.velocity;
  const int max_steps = static_cast<int>(10.0f / dt);
  const float damping = 1.0f - clamp(config_.physics.linear_damping, 0.0f, 1.0f);
  const float wind_acceleration =
      state_.latent_wind_force / std::max(0.001f, state_.body.mass);
  for (int i = 0; i < max_steps; ++i) {
    velocity.y += gravity * dt;
    velocity.x += wind_acceleration * dt;
    velocity *= damping;
    position.x += velocity.x * dt;
    position.y += velocity.y * dt;
    if (std::abs(position.x - state_.body.position.x) > state_.lidar_range) return;
    if (position.x < 0.0f || position.x > terrain_.length()) return;
    const auto sample = terrain_.query(position.x);
    if (!sample.solid) continue;
    if (position.y <= sample.height) {
      state_.lidar_landing_x = position.x;
      state_.lidar_landing_y = sample.height;
      state_.lidar_landing_valid = true;
      return;
    }
  }
}

bool Env::is_flipped() const {
  const float angle = std::atan2(std::sin(state_.body.angle), std::cos(state_.body.angle));
  return std::abs(angle) >= config_.termination.flip_angle;
}

bool Env::is_upright_on_wheels() const {
  const float angle = std::atan2(std::sin(state_.body.angle), std::cos(state_.body.angle));
  const float upright_limit = std::min(0.35f, config_.termination.flip_angle * 0.25f);
  if (std::abs(angle) > upright_limit || state_.body_contact_roof || state_.wheel_count <= 0) {
    return false;
  }
  for (int i = 0; i < state_.wheel_count; ++i) {
    if (!state_.wheels[static_cast<size_t>(i)].in_contact) return false;
  }
  return true;
}

bool Env::is_stuck() const {
  return state_.step_index > 120 &&
         stuck_counter_ > config_.termination.stuck_steps;
}

}
