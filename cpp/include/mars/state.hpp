#pragma once

#include <array>
#include <cstdint>

#include "mars/math.hpp"

namespace mars {

constexpr int kMaxWheels = 8;
constexpr int kDefaultWheelCount = 2;

struct RigidBodyState {
  Vec2 position{};
  Vec2 velocity{};
  float angle = 0.0f;
  float angular_velocity = 0.0f;
  float mass = 1.0f;
  float inv_mass = 1.0f;
  float inertia = 1.0f;
  float inv_inertia = 1.0f;
};

struct WheelState {
  Vec2 position{};
  Vec2 velocity{};
  float angle = 0.0f;
  float angular_velocity = 0.0f;
  float radius = 0.24f;
  float mass = 1.0f;
  float inv_mass = 1.0f;
  bool in_contact = false;
  float slip = 0.0f;
  float normal_force = 0.0f;
};

struct RoverState {
  RigidBodyState body{};
  std::array<WheelState, kMaxWheels> wheels{};
  int wheel_count = kDefaultWheelCount;
  float energy = 100.0f;
  float damage = 0.0f;
  float previous_x = 0.0f;
  float last_reward = 0.0f;
  int previous_action = 0;
  int gear_index = 0;
  float engine_rpm = 1100.0f;
  float engine_temperature = 20.0f;
  float ambient_temperature = -45.0f;
  float thermal_transfer = 1.0f;
  float drive_fuel_rate = 0.0f;
  float energy_cost_rate = 0.0f;
  float energy_gain_rate = 0.0f;
  float cold_power_factor = 1.0f;
  float clutch_engagement = 1.0f;
  float drivetrain_slip = 0.0f;
  bool drivetrain_grounded = false;
  bool solar_panel_requested = false;
  bool charging_active = false;
  float solar_panel_deployment = 0.0f;
  float solar_charge_rate = 0.0f;
  float passive_charge_rate = 0.0f;
  float solar_irradiance = 0.0f;
  float ballast_air = 0.0f;
  bool ballast_blowing = false;
  bool ballast_flooding = false;
  bool solar_panel_stationary = false;
  int lidar_active_steps = 0;
  int lidar_cooldown_steps = 0;
  float lidar_last_energy_cost = 0.0f;
  float lidar_range = 0.0f;
  float lidar_landing_x = 0.0f;
  float lidar_landing_y = 0.0f;
  bool lidar_landing_valid = false;
  Vec2 imu_acceleration{};
  float imu_angular_acceleration = 0.0f;
  float imu_impact = 0.0f;
  bool body_contact_front = false;
  bool body_contact_belly = false;
  bool body_contact_rear = false;
  bool body_contact_roof = false;
  int drive_mode = 0;
  bool engine_running = true;
  bool engine_stalled = false;
  bool engine_overheated = false;
  bool engine_cold_locked = false;
  bool heater_active = false;
  float recommended_upshift_rpm = 0.0f;
  float minimum_upshift_rpm = 0.0f;
  float projected_upshift_rpm = 0.0f;
  bool upshift_speed_ok = false;
  bool upshift_recommended = false;
  bool can_shift_up = false;
  bool can_shift_down = false;
  bool should_shift_down = false;
  int shift_cooldown_steps = 0;
  int shift_clutch_cut_steps = 0;
  int shift_up_buffer_steps = 0;
  int shift_down_buffer_steps = 0;
  float last_shift_energy_cost = 0.0f;
  int engine_lug_steps = 0;
  float driveline_load_factor = 0.0f;
  float gear_energy_multiplier = 1.0f;
  int step_index = 0;
  int episode_in_trial = 0;
  bool trial_start = true;



  bool airborne = false;
  bool has_grounded = false;
  int airborne_steps = 0;
  bool landing_event = false;
  bool landing_fatal = false;
  bool fatal_error = false;
  bool pit_recovery_event = false;
  int pit_recovery_count = 0;


  int termination_reason = 0;
  float last_impact_speed = 0.0f;
  float last_landing_angle = 0.0f;


  float latent_traction = 1.0f;
  float latent_moisture = 0.25f;
  float latent_heat = 0.25f;
  float latent_charge_reserve = 1.0f;
  float latent_viscosity = 0.0f;
  float latent_sink = 0.0f;
  float latent_energy_resistance = 1.0f;
  float latent_suspension = 1.0f;
  float latent_gravity_multiplier = 1.0f;
  float latent_wind_force = 0.0f;
  float latent_ambient_temperature = -45.0f;
  float latent_thermal_transfer = 1.0f;
  float latent_solar_rate = 1.0f;
  float latent_lidar_energy_multiplier = 1.0f;
  float latent_lidar_range_multiplier = 1.0f;
  float thruster_thrust = 0.0f;
  int active_layer_count = 0;
  float active_layer_weight = 0.0f;
  bool climb_mode = false;
  bool propeller_mode = false;
  float propeller_deployment = 0.0f;
  float propeller_phase = 0.0f;
  float propeller_thrust = 0.0f;
  int jump_cooldown_steps = 0;
  int suspension_jump_phase = 0;
  int suspension_jump_phase_steps = 0;
  float suspension_jump_charge = 0.0f;



  float suspension_jump_preload_velocity = 0.0f;
  int suspension_jump_mask = 3;
  float roof_piston_extension = 0.0f;
  int roof_piston_mask = 0;
  bool roof_piston_contact = false;
  float recovery_state = 0.0f;
  uint64_t world_seed = 0;
  int route_branch = 0;
  Vec2 render_camera_position{};
};

struct EpisodeResult {
  float reward = 0.0f;
  bool terminated = false;
  bool truncated = false;
};

}
