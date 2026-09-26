#include "mars/batch_env.hpp"

#include <algorithm>
#include <stdexcept>

#include "mars/renderer.hpp"

namespace mars {

BatchEnv::BatchEnv(int num_envs, EnvConfig config) {
  if (num_envs <= 0) {
    throw std::invalid_argument("num_envs must be positive");
  }
  envs_.reserve(static_cast<size_t>(num_envs));
  for (int i = 0; i < num_envs; ++i) {
    envs_.emplace_back(config);
  }
}

void BatchEnv::reset_all(uint64_t seed, float* obs_out) {
  const int n = num_envs();
  const int observation_dim = obs_dim();
#if defined(MARS_ROVER_HAS_OPENMP)




  const int parallel_threads = std::min(16, n);
#pragma omp parallel for schedule(static) if (n >= 64) num_threads(parallel_threads)
#endif
  for (int i = 0; i < n; ++i) {
    envs_[static_cast<size_t>(i)].reset(seed + static_cast<uint64_t>(i) * 9973ULL, true,
                                       obs_out + i * observation_dim);
  }
}

void BatchEnv::reset_at(int env_id, uint64_t seed, bool trial_start, float* obs_out) {
  envs_.at(static_cast<size_t>(env_id)).reset(seed, trial_start, obs_out);
}

void BatchEnv::step_batch(const int* actions, float* obs_out, float* rewards_out,
                          uint8_t* terminated_out, uint8_t* truncated_out) {
  const int n = num_envs();
  const int observation_dim = obs_dim();
#if defined(MARS_ROVER_HAS_OPENMP)
  const int parallel_threads = std::min(16, n);
#pragma omp parallel for schedule(static) if (n >= 64) num_threads(parallel_threads)
#endif
  for (int i = 0; i < n; ++i) {
    const auto out =
        envs_[static_cast<size_t>(i)].step(actions[i], obs_out + i * observation_dim);
    rewards_out[i] = out.reward;
    terminated_out[i] = out.terminated ? 1 : 0;
    truncated_out[i] = out.truncated ? 1 : 0;
  }
}

void BatchEnv::render_rgb(int env_id, uint8_t* rgb_out, int width, int height,
                          bool debug_overlay) const {
  renderer_.set_debug_overlay(debug_overlay);
  renderer_.render_rgb(envs_.at(static_cast<size_t>(env_id)), rgb_out, width, height);
}

}
