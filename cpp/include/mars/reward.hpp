#pragma once

#include "mars/state.hpp"

namespace mars {

struct RewardConfig {
  float progress_scale = 1.0f;
  float energy_cost_scale = 0.01f;
  float flip_penalty = 0.0f;
  float stuck_penalty = 0.0f;
  float hard_contact_penalty = 0.0f;
  float finish_bonus = 0.0f;
};

struct TerminationConfig {
  float finish_x = 0.0f;
  float min_energy = 0.0f;
  float flip_angle = 2.2f;
  int stuck_steps = 600;
  int max_steps = 0;
  float fatal_fall_y = -8.0f;
  float trial_time_limit = 120.0f;
};

float compute_reward(const RewardConfig& config, const RoverState& state, float energy_cost,
                     bool finished, bool flipped, bool stuck);

}
