#include "mars/reward.hpp"

namespace mars {

float compute_reward(const RewardConfig& config, const RoverState& state, float energy_cost,
                     bool finished, bool flipped, bool stuck) {
  const float progress_delta = state.body.position.x - state.previous_x;
  float reward = progress_delta * config.progress_scale - energy_cost * config.energy_cost_scale;
  if (flipped) {
    reward -= config.flip_penalty;
  }
  if (stuck) {
    reward -= config.stuck_penalty;
  }
  if (finished) {
    reward += config.finish_bonus;
  }
  reward -= config.hard_contact_penalty * state.damage;
  return reward;
}

}
