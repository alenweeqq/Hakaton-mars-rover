#pragma once

#include <array>
#include <cstdint>
#include <random>
#include <string_view>

#include "mars/observation.hpp"
#include "mars/physics.hpp"
#include "mars/reward.hpp"
#include "mars/version.hpp"

namespace mars {

struct EnvConfig {
  TerrainConfig terrain{};
  PhysicsConfig physics{};
  RewardConfig reward{};
  TerminationConfig termination{};
  RoverRig rig = RoverRig::default_two_wheel();
  int episodes_per_trial = 0;

  int biome_split = 1;
  int fixed_biome_id = -1;
  bool debug = false;

  bool chain_biomes = false;
  int chain_zone_count = 8;
  float chain_segment_min_length = 120.0f;
  float chain_segment_max_length = 250.0f;
  float terrain_surprise_probability = 0.22f;
  float terrain_surprise_strength = 1.0f;
  float difficulty_safe_fraction_min = 0.02f;
  float difficulty_safe_fraction_max = 0.02f;
  float difficulty_exponent = 0.7125f;
  float terrain_profile_frequency_growth = 0.75f;

  std::array<MechanicType, 4> evaluation_stack_types{};
  int evaluation_stack_count = 0;
};

struct StepOutput {
  float reward = 0.0f;
  bool terminated = false;
  bool truncated = false;
};

class Env {
 public:
  explicit Env(EnvConfig config = {});

  void reset(uint64_t seed, bool trial_start, float* obs_out);
  StepOutput step(int action, float* obs_out);
  void build_observation(float* obs_out) const;

  const RoverState& state() const { return state_; }
  const Terrain& terrain() const { return terrain_; }
  const EnvConfig& config() const { return config_; }
  MechanicType mechanic_type() const { return mechanic_type_; }
  const MechanicParams& mechanic_params() const { return mechanic_params_; }
  const MechanicZone& mechanic_at(float x) const { return mechanic_layout_.at(x); }
  const MechanicLayout& mechanic_layout() const { return mechanic_layout_; }

  int obs_dim() const { return kObservationDim; }
  int action_dim() const { return 1 << 24; }

  int trial_steps_used() const { return trial_steps_used_; }
  int trial_step_budget() const;
  bool trial_exhausted() const;
  float trial_time_left() const;
  float best_progress() const { return best_progress_x_; }

 private:
  void select_mechanic_layout(uint64_t seed);
  void finalize_mechanic_layout();
  bool is_flipped() const;
  bool is_upright_on_wheels() const;
  bool is_stuck() const;
  void update_world_latents();
  void update_lidar_landing();
  float course_difficulty(float x) const;
  void recover_from_pit(float recovery_x);

  EnvConfig config_{};
  Terrain terrain_{};
  PhysicsEngine physics_{};
  RoverState state_{};
  std::mt19937_64 rng_{1};
  MechanicType mechanic_type_ = MechanicType::Normal;
  MechanicParams mechanic_params_{};
  MechanicLayout mechanic_layout_{};
  uint64_t trial_mechanic_seed_ = 0;
  bool has_trial_mechanic_seed_ = false;
  int stuck_counter_ = 0;
  int trial_steps_used_ = 0;
  bool flip_latched_ = false;
  bool pit_recovery_latched_ = false;
  float best_progress_x_ = 0.0f;
  std::array<float, kMaxMechanicZones> pending_basin_depth_{};
  float difficulty_safe_fraction_ = 0.02f;
  float sand_burial_ = 0.0f;
};

}
