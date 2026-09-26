#pragma once

#include <cstdint>
#include <vector>

#include "mars/env.hpp"
#include "mars/renderer.hpp"

namespace mars {

class BatchEnv {
 public:
  BatchEnv(int num_envs, EnvConfig config);

  void reset_all(uint64_t seed, float* obs_out);
  void reset_at(int env_id, uint64_t seed, bool trial_start, float* obs_out);

  void step_batch(const int* actions, float* obs_out, float* rewards_out,
                  uint8_t* terminated_out, uint8_t* truncated_out);
  void render_rgb(int env_id, uint8_t* rgb_out, int width, int height, bool debug_overlay) const;

  int num_envs() const { return static_cast<int>(envs_.size()); }
  int obs_dim() const { return kObservationDim; }
  int action_dim() const { return 1 << 24; }

  Env& env_at(int env_id) { return envs_[env_id]; }
  const Env& env_at(int env_id) const { return envs_[env_id]; }

 private:
  std::vector<Env> envs_;
  mutable Renderer renderer_{};
};

}
