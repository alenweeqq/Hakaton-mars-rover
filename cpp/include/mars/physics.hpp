#pragma once

#include "mars/action.hpp"
#include "mars/contacts.hpp"
#include "mars/mechanics.hpp"
#include "mars/rover_rig.hpp"
#include "mars/state.hpp"
#include "mars/terrain.hpp"

namespace mars {

struct PhysicsConfig {
  float dt = 1.0f / 60.0f;
  float gravity = -3.71f;
  float wheel_friction = 1.2f;
  float motor_torque = 115.0f;
  float final_drive_ratio = 4.5f;
  float engine_inertia = 0.48f;
  float engine_drag_torque = 7.0f;
  float clutch_sync_rate = 3200.0f;



  float shift_energy_base = 0.75f;
  float shift_energy_sync_per_krpm = 0.32f;
  float initial_engine_temperature = 20.0f;
  float cold_start_temperature = -25.0f;
  float minimum_operating_temperature = -38.0f;
  float full_power_temperature = 65.0f;
  float overheat_temperature = 115.0f;
  float overheat_restart_temperature = 92.0f;
  float engine_thermal_mass = 8.0f;
  float engine_idle_heat = 2.2f;
  float engine_heat_per_fuel = 3.0f;
  float engine_cooling_conductance = 0.10f;
  float engine_cooling_airflow = 0.08f;
  float engine_heater_energy_rate = 0.15f;
  float engine_heater_heat = 42.0f;
  float initial_energy = 60.0f;
  float energy_capacity = 75.0f;
  float panel_deploy_time = 4.0f;
  float panel_retract_time = 3.0f;
  float lidar_energy_cost = 0.65f;
  float lidar_scan_duration = 2.0f;
  float lidar_cooldown = 1.25f;
  float lidar_base_range = 24.0f;
  float near_sense_range = 3.0f;
  float brake_strength = 20.0f;
  float body_tilt_torque = 8.0f;
  float linear_damping = 0.005f;
  float angular_damping = 0.02f;
  float safe_landing_speed = 4.0f;
  float safe_landing_angle = 0.55f;
  int ballistic_min_air_steps = 6;
  float fatal_landing_flip_angle = 2.35f;
};

struct PhysicsStepStats {
  float energy_cost = 0.0f;
  float drive_energy_cost = 0.0f;
  float energy_gain = 0.0f;
  float hard_contact = 0.0f;
  DeformationContactArray deformation_contacts{};
};

class PhysicsEngine {
 public:
  explicit PhysicsEngine(PhysicsConfig config = {});

  void initialize_state(const RoverRig& rig, RoverState& state, Vec2 spawn) const;
  PhysicsStepStats step(const RoverRig& rig, const Terrain& terrain, RoverState& state,
                        int discrete_action, const MechanicLayout& mechanics) const;

 private:
  void apply_body_impulse(RigidBodyState& body, Vec2 impulse, Vec2 point) const;

  PhysicsConfig config_{};
};

}
