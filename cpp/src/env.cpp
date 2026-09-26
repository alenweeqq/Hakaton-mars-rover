#include "mars/env.hpp"
#include "mars/biome_bank.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace mars {

Env::Env(EnvConfig config) : config_(std::move(config)), physics_(config_.physics) {
  if (config_.biome_split < 0 || config_.biome_split > 3 || config_.biome_split == 2) {
    throw std::invalid_argument("student biome_split must be 0, 1, or 3");
  }
  terrain_.configure(config_.terrain);
}

void Env::reset(uint64_t seed, bool trial_start, float* obs_out) {
  rng_.seed(seed);
  pit_recovery_latched_ = false;
  sand_burial_ = 0.0f;
  const int next_episode_in_trial = trial_start ? 0 : state_.episode_in_trial + 1;
  if (trial_start || !has_trial_mechanic_seed_) {
    trial_mechanic_seed_ = seed;
    has_trial_mechanic_seed_ = true;
    trial_steps_used_ = 0;
  }

  select_mechanic_layout(trial_mechanic_seed_);
  const auto& picked_params = mechanic_layout_.zones[0].params;
  TerrainConfig scaled_terrain = config_.terrain;

  scaled_terrain.amplitude *= clamp(picked_params.terrain_amplitude_mul, 0.5f, 1.8f);
  scaled_terrain.roughness *= clamp(picked_params.terrain_roughness_mul, 0.5f, 2.5f);
  scaled_terrain.crater_count = static_cast<int>(std::lround(
      static_cast<float>(scaled_terrain.crater_count) *
      clamp(picked_params.terrain_crater_mul, 0.3f, 3.0f)));
  scaled_terrain.step_count = static_cast<int>(std::lround(
      static_cast<float>(scaled_terrain.step_count) *
      clamp(picked_params.terrain_step_mul, 0.3f, 3.0f)));
  scaled_terrain.safe_start_fraction = difficulty_safe_fraction_;
  scaled_terrain.difficulty_exponent = config_.difficulty_exponent;
  terrain_.configure(scaled_terrain);
  terrain_.generate(seed ^ 0x9e3779b97f4a7c15ULL);
  finalize_mechanic_layout();
  const float spawn_y = terrain_.query(1.0f).height + 1.0f;
  physics_.initialize_state(config_.rig, state_, {1.0f, spawn_y});
  state_.world_seed = seed;
  update_world_latents();
  state_.ambient_temperature = state_.latent_ambient_temperature;
  state_.solar_irradiance = state_.latent_solar_rate;
  state_.trial_start = trial_start;
  state_.episode_in_trial = next_episode_in_trial;
  stuck_counter_ = 0;
  flip_latched_ = false;
  best_progress_x_ = state_.body.position.x;
  build_observation(obs_out);
}

StepOutput Env::step(int action, float* obs_out) {
  state_.previous_x = state_.body.position.x;
  state_.pit_recovery_event = false;
  update_world_latents();
  const auto stats = physics_.step(config_.rig, terrain_, state_, action, mechanic_layout_);

  const auto& current_zone = mechanic_layout_.at(state_.body.position.x);
  mechanic_type_ = current_zone.type;
  mechanic_params_ = current_zone.params;
  state_.route_branch = 0;
  if (current_zone.type == MechanicType::Liquid) {
    const float lower = terrain_.query(state_.body.position.x).height;
    const float contacted = terrain_.query_near(state_.body.position.x, state_.body.position.y).height;
    state_.route_branch = contacted > lower + 0.30f ? 2 : 1;
  }

  for (int i = 0; i < state_.wheel_count; ++i) {
    const auto& c = stats.deformation_contacts[static_cast<size_t>(i)];
    if (c.active) {
      const auto& zone = mechanic_layout_.at(c.x);
      float deform_scale = 0.0f;
      if (zone.type == MechanicType::Crust) {
        deform_scale = c.penetration > 0.008f ? 0.025f + zone.params.crust_deform * 2.0f : 0.0f;
      } else if (zone.type == MechanicType::Mud) {
        deform_scale = 0.012f + zone.params.viscosity * 0.003f;
      }
      if (deform_scale > 0.0f) {
        terrain_.deform(c.x, state_.wheels[static_cast<size_t>(i)].radius * 1.15f,
                        deform_scale * c.penetration);
      }
    }
  }
  state_.damage += stats.hard_contact > 1500.0f ? (stats.hard_contact - 1500.0f) * 0.000001f : 0.0f;

  const bool finished = false;
  const bool flipped = is_flipped();
  const bool flip_started = flipped && state_.body_contact_roof && !flip_latched_;
  if (flip_started) {
    flip_latched_ = true;
  } else if (flip_latched_ && is_upright_on_wheels()) {
    flip_latched_ = false;
  }

  if (state_.body.position.x >= best_progress_x_ + 0.5f) {
    best_progress_x_ = state_.body.position.x;
    stuck_counter_ = 0;
  } else {
    stuck_counter_ += 1;
  }
  const bool stuck = false;
  trial_steps_used_ += 1;
  if (flip_started && trial_step_budget() > 0) {
    const int penalty_steps = std::max(
        1, static_cast<int>(std::lround(10.0f / std::max(0.0001f, config_.physics.dt))));
    trial_steps_used_ = std::min(trial_step_budget(), trial_steps_used_ + penalty_steps);
  }

  const bool over_gap = !terrain_.query(state_.body.position.x).solid;
  const bool fatal_pit = over_gap && state_.body.position.y < config_.termination.fatal_fall_y;
  if (fatal_pit && !pit_recovery_latched_) {
    pit_recovery_latched_ = true;
    const float pit_recovery_x = terrain_.previous_solid_x(state_.body.position.x, 2.5f);
    recover_from_pit(pit_recovery_x);
    if (!flip_started && trial_step_budget() > 0) {
      const int penalty_steps = std::max(
          1, static_cast<int>(std::lround(10.0f / std::max(0.0001f, config_.physics.dt))));
      trial_steps_used_ = std::min(trial_step_budget(), trial_steps_used_ + penalty_steps);
    }
  } else if (!fatal_pit) {
    pit_recovery_latched_ = false;
  }
  StepOutput out{};
  out.terminated = false;
  out.truncated = trial_exhausted();

  if (!out.truncated && config_.termination.trial_time_limit <= 0.0f &&
      config_.termination.max_steps > 0 &&
      state_.step_index + 1 >= config_.termination.max_steps) {
    out.truncated = true;
  }
  state_.termination_reason = 0;
  if (out.truncated) state_.termination_reason = 7;
  out.reward =
      compute_reward(config_.reward, state_, stats.energy_cost, finished, flip_started, stuck);
  state_.last_reward = out.reward;
  state_.previous_action = action;
  state_.step_index += 1;
  update_lidar_landing();
  build_observation(obs_out);
  return out;
}

}
