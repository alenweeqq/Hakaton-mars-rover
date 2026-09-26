#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cmath>

#include "mars/batch_env.hpp"
#include "mars/biome_bank.hpp"
#include "mars/renderer.hpp"

namespace py = pybind11;

namespace {

template <typename T>
T* checked_ptr(py::array_t<T, py::array::c_style>& arr, py::ssize_t expected) {
  if (arr.size() < expected) {
    throw std::runtime_error("array is smaller than expected");
  }
  return static_cast<T*>(arr.mutable_data());
}

const char* mechanic_name(mars::MechanicType type) {
  switch (type) {
    case mars::MechanicType::Normal:
      return "Normal";
    case mars::MechanicType::Sand:
      return "Sand";
    case mars::MechanicType::Ice:
      return "Ice";
    case mars::MechanicType::Mud:
      return "Mud";
    case mars::MechanicType::Wind:
      return "Wind";
    case mars::MechanicType::LowGravity:
      return "LowGravity";
    case mars::MechanicType::Crust:
      return "Crust";
    case mars::MechanicType::Liquid:
      return "Liquid";
  }
  return "Unknown";
}

constexpr int kDebugGearCount = 8;

}

PYBIND11_MODULE(_mars_rover_cpp, m) {
  m.attr("__version__") = std::string(mars::kRepositoryVersion);
  m.def("biome_bank_version", [] { return std::string(mars::kBiomeBankVersion); });
  m.def("environment_version", [] { return std::string(mars::kEnvironmentVersion); });
  py::enum_<mars::CollisionType>(m, "CollisionType")
      .value("None_", mars::CollisionType::None)
      .value("Box", mars::CollisionType::Box)
      .value("Circle", mars::CollisionType::Circle);

  py::enum_<mars::MechanicType>(m, "MechanicType")
      .value("Normal", mars::MechanicType::Normal)
      .value("Sand", mars::MechanicType::Sand)
      .value("Ice", mars::MechanicType::Ice)
      .value("Mud", mars::MechanicType::Mud)
      .value("Wind", mars::MechanicType::Wind)
      .value("LowGravity", mars::MechanicType::LowGravity)
      .value("Crust", mars::MechanicType::Crust)
      .value("Liquid", mars::MechanicType::Liquid);

  py::class_<mars::Vec2>(m, "Vec2")
      .def(py::init<>())
      .def(py::init<float, float>())
      .def_readwrite("x", &mars::Vec2::x)
      .def_readwrite("y", &mars::Vec2::y);

  py::class_<mars::TerrainConfig>(m, "TerrainConfig")
      .def(py::init<>())
      .def_readwrite("sample_count", &mars::TerrainConfig::sample_count)
      .def_readwrite("dx", &mars::TerrainConfig::dx)
      .def_readwrite("base_height", &mars::TerrainConfig::base_height)
      .def_readwrite("amplitude", &mars::TerrainConfig::amplitude)
      .def_readwrite("roughness", &mars::TerrainConfig::roughness)
      .def_readwrite("crater_count", &mars::TerrainConfig::crater_count)
      .def_readwrite("step_count", &mars::TerrainConfig::step_count)
      .def_readwrite("length", &mars::TerrainConfig::length)
      .def_readwrite("safe_start_fraction", &mars::TerrainConfig::safe_start_fraction)
      .def_readwrite("difficulty_exponent", &mars::TerrainConfig::difficulty_exponent)
      .def_readwrite("difficulty_distance_offset", &mars::TerrainConfig::difficulty_distance_offset)
      .def_readwrite("preserve_spawn_safety", &mars::TerrainConfig::preserve_spawn_safety)
      .def_readwrite("jagged_scale", &mars::TerrainConfig::jagged_scale);

  py::class_<mars::PhysicsConfig>(m, "PhysicsConfig")
      .def(py::init<>())
      .def_readwrite("dt", &mars::PhysicsConfig::dt)
      .def_readwrite("gravity", &mars::PhysicsConfig::gravity)
      .def_readwrite("wheel_friction", &mars::PhysicsConfig::wheel_friction)
      .def_readwrite("motor_torque", &mars::PhysicsConfig::motor_torque)
      .def_readwrite("final_drive_ratio", &mars::PhysicsConfig::final_drive_ratio)
      .def_readwrite("engine_inertia", &mars::PhysicsConfig::engine_inertia)
      .def_readwrite("engine_drag_torque", &mars::PhysicsConfig::engine_drag_torque)
      .def_readwrite("clutch_sync_rate", &mars::PhysicsConfig::clutch_sync_rate)
      .def_readwrite("shift_energy_base", &mars::PhysicsConfig::shift_energy_base)
      .def_readwrite("shift_energy_sync_per_krpm",
                     &mars::PhysicsConfig::shift_energy_sync_per_krpm)
      .def_readwrite("initial_engine_temperature",
                     &mars::PhysicsConfig::initial_engine_temperature)
      .def_readwrite("cold_start_temperature", &mars::PhysicsConfig::cold_start_temperature)
      .def_readwrite("minimum_operating_temperature",
                     &mars::PhysicsConfig::minimum_operating_temperature)
      .def_readwrite("full_power_temperature", &mars::PhysicsConfig::full_power_temperature)
      .def_readwrite("overheat_temperature", &mars::PhysicsConfig::overheat_temperature)
      .def_readwrite("overheat_restart_temperature",
                     &mars::PhysicsConfig::overheat_restart_temperature)
      .def_readwrite("engine_thermal_mass", &mars::PhysicsConfig::engine_thermal_mass)
      .def_readwrite("engine_idle_heat", &mars::PhysicsConfig::engine_idle_heat)
      .def_readwrite("engine_heat_per_fuel", &mars::PhysicsConfig::engine_heat_per_fuel)
      .def_readwrite("engine_cooling_conductance",
                     &mars::PhysicsConfig::engine_cooling_conductance)
      .def_readwrite("engine_cooling_airflow", &mars::PhysicsConfig::engine_cooling_airflow)
      .def_readwrite("engine_heater_energy_rate", &mars::PhysicsConfig::engine_heater_energy_rate)
      .def_readwrite("engine_heater_heat", &mars::PhysicsConfig::engine_heater_heat)
      .def_readwrite("initial_energy", &mars::PhysicsConfig::initial_energy)
      .def_readwrite("energy_capacity", &mars::PhysicsConfig::energy_capacity)
      .def_readwrite("panel_deploy_time", &mars::PhysicsConfig::panel_deploy_time)
      .def_readwrite("panel_retract_time", &mars::PhysicsConfig::panel_retract_time)
      .def_readwrite("lidar_energy_cost", &mars::PhysicsConfig::lidar_energy_cost)
      .def_readwrite("lidar_scan_duration", &mars::PhysicsConfig::lidar_scan_duration)
      .def_readwrite("lidar_cooldown", &mars::PhysicsConfig::lidar_cooldown)
      .def_readwrite("lidar_base_range", &mars::PhysicsConfig::lidar_base_range)
      .def_readwrite("near_sense_range", &mars::PhysicsConfig::near_sense_range)
      .def_readwrite("brake_strength", &mars::PhysicsConfig::brake_strength)
      .def_readwrite("body_tilt_torque", &mars::PhysicsConfig::body_tilt_torque)
      .def_readwrite("linear_damping", &mars::PhysicsConfig::linear_damping)
      .def_readwrite("angular_damping", &mars::PhysicsConfig::angular_damping)
      .def_readwrite("safe_landing_speed", &mars::PhysicsConfig::safe_landing_speed)
      .def_readwrite("safe_landing_angle", &mars::PhysicsConfig::safe_landing_angle)
      .def_readwrite("ballistic_min_air_steps", &mars::PhysicsConfig::ballistic_min_air_steps)
      .def_readwrite("fatal_landing_flip_angle", &mars::PhysicsConfig::fatal_landing_flip_angle);

  py::class_<mars::RewardConfig>(m, "RewardConfig")
      .def(py::init<>())
      .def_readwrite("progress_scale", &mars::RewardConfig::progress_scale)
      .def_readwrite("energy_cost_scale", &mars::RewardConfig::energy_cost_scale)
      .def_readwrite("flip_penalty", &mars::RewardConfig::flip_penalty)
      .def_readwrite("stuck_penalty", &mars::RewardConfig::stuck_penalty)
      .def_readwrite("hard_contact_penalty", &mars::RewardConfig::hard_contact_penalty)
      .def_readwrite("finish_bonus", &mars::RewardConfig::finish_bonus);

  py::class_<mars::TerminationConfig>(m, "TerminationConfig")
      .def(py::init<>())
      .def_readwrite("finish_x", &mars::TerminationConfig::finish_x)
      .def_readwrite("min_energy", &mars::TerminationConfig::min_energy)
      .def_readwrite("flip_angle", &mars::TerminationConfig::flip_angle)
      .def_readwrite("stuck_steps", &mars::TerminationConfig::stuck_steps)
      .def_readwrite("max_steps", &mars::TerminationConfig::max_steps)
      .def_readwrite("fatal_fall_y", &mars::TerminationConfig::fatal_fall_y)
      .def_readwrite("trial_time_limit", &mars::TerminationConfig::trial_time_limit);

  py::class_<mars::CollisionShapeConfig>(m, "CollisionShapeConfig")
      .def(py::init<>())
      .def_readwrite("type", &mars::CollisionShapeConfig::type)
      .def_readwrite("size", &mars::CollisionShapeConfig::size)
      .def_readwrite("radius", &mars::CollisionShapeConfig::radius);

  py::class_<mars::BodyRigConfig>(m, "BodyRigConfig")
      .def(py::init<>())
      .def_readwrite("mass", &mars::BodyRigConfig::mass)
      .def_readwrite("inertia", &mars::BodyRigConfig::inertia)
      .def_readwrite("size", &mars::BodyRigConfig::size)
      .def_readwrite("collision", &mars::BodyRigConfig::collision)
      .def_readwrite("local_position", &mars::BodyRigConfig::local_position);

  py::class_<mars::SuspensionRigConfig>(m, "SuspensionRigConfig")
      .def(py::init<>())
      .def_readwrite("rest_length", &mars::SuspensionRigConfig::rest_length)
      .def_readwrite("min_length", &mars::SuspensionRigConfig::min_length)
      .def_readwrite("max_length", &mars::SuspensionRigConfig::max_length)
      .def_readwrite("stiffness", &mars::SuspensionRigConfig::stiffness)
      .def_readwrite("damping", &mars::SuspensionRigConfig::damping);

  py::class_<mars::WheelRigConfig>(m, "WheelRigConfig")
      .def(py::init<>())
      .def_readwrite("radius", &mars::WheelRigConfig::radius)
      .def_readwrite("mass", &mars::WheelRigConfig::mass)
      .def_readwrite("local_anchor", &mars::WheelRigConfig::local_anchor)
      .def_readwrite("suspension", &mars::WheelRigConfig::suspension);

  py::class_<mars::RoverRig>(m, "RoverRig")
      .def(py::init<>())
      .def_static("default_two_wheel", &mars::RoverRig::default_two_wheel)
      .def_readwrite("body", &mars::RoverRig::body)
      .def_readwrite("wheels", &mars::RoverRig::wheels)
      ;

  py::class_<mars::EnvConfig>(m, "EnvConfig")
      .def(py::init<>())
      .def_readwrite("terrain", &mars::EnvConfig::terrain)
      .def_readwrite("physics", &mars::EnvConfig::physics)
      .def_readwrite("reward", &mars::EnvConfig::reward)
      .def_readwrite("termination", &mars::EnvConfig::termination)
      .def_readwrite("rig", &mars::EnvConfig::rig)
      .def_readwrite("episodes_per_trial", &mars::EnvConfig::episodes_per_trial)
      .def_readwrite("biome_split", &mars::EnvConfig::biome_split)
      .def_readwrite("fixed_biome_id", &mars::EnvConfig::fixed_biome_id)
      .def_readwrite("chain_biomes", &mars::EnvConfig::chain_biomes)
      .def_readwrite("chain_zone_count", &mars::EnvConfig::chain_zone_count)
      .def_readwrite("chain_segment_min_length", &mars::EnvConfig::chain_segment_min_length)
      .def_readwrite("chain_segment_max_length", &mars::EnvConfig::chain_segment_max_length)
      .def_readwrite("terrain_surprise_probability",
                     &mars::EnvConfig::terrain_surprise_probability)
      .def_readwrite("terrain_surprise_strength", &mars::EnvConfig::terrain_surprise_strength)
      .def_readwrite("difficulty_safe_fraction_min", &mars::EnvConfig::difficulty_safe_fraction_min)
      .def_readwrite("difficulty_safe_fraction_max", &mars::EnvConfig::difficulty_safe_fraction_max)
      .def_readwrite("difficulty_exponent", &mars::EnvConfig::difficulty_exponent)
      .def_readwrite("terrain_profile_frequency_growth",
                     &mars::EnvConfig::terrain_profile_frequency_growth)
      .def_readwrite("evaluation_stack_types", &mars::EnvConfig::evaluation_stack_types)
      .def_readwrite("evaluation_stack_count", &mars::EnvConfig::evaluation_stack_count)
      .def_readwrite("debug", &mars::EnvConfig::debug);

  py::class_<mars::BatchEnv>(m, "MarsRoverBatchEnv")
      .def(py::init<int, mars::EnvConfig>(), py::arg("num_envs"), py::arg("config") = mars::EnvConfig{})
      .def_property_readonly("num_envs", &mars::BatchEnv::num_envs)
      .def_property_readonly("obs_dim", &mars::BatchEnv::obs_dim)
      .def_property_readonly("action_dim", &mars::BatchEnv::action_dim)
      .def("reset_all",
           [](mars::BatchEnv& self, uint64_t seed, py::array_t<float, py::array::c_style> obs) {
             auto* obs_ptr = checked_ptr(obs, self.num_envs() * self.obs_dim());
             py::gil_scoped_release release;
             self.reset_all(seed, obs_ptr);
           })
      .def("reset_at",
           [](mars::BatchEnv& self, int env_id, uint64_t seed, bool trial_start,
              py::array_t<float, py::array::c_style> obs) {
             auto* obs_ptr = checked_ptr(obs, self.obs_dim());
             py::gil_scoped_release release;
             self.reset_at(env_id, seed, trial_start, obs_ptr);
           })
      .def("step",
           [](mars::BatchEnv& self, py::array_t<int, py::array::c_style | py::array::forcecast> actions,
              py::array_t<float, py::array::c_style> obs,
              py::array_t<float, py::array::c_style> rewards,
              py::array_t<uint8_t, py::array::c_style> terminated,
              py::array_t<uint8_t, py::array::c_style> truncated) {
             if (actions.size() < self.num_envs()) {
               throw std::runtime_error("actions array is smaller than num_envs");
             }
             auto* obs_ptr = checked_ptr(obs, self.num_envs() * self.obs_dim());
             auto* rewards_ptr = checked_ptr(rewards, self.num_envs());
             auto* term_ptr = checked_ptr(terminated, self.num_envs());
             auto* trunc_ptr = checked_ptr(truncated, self.num_envs());
             const int* actions_ptr = static_cast<const int*>(actions.data());
             py::gil_scoped_release release;
             self.step_batch(actions_ptr, obs_ptr, rewards_ptr, term_ptr, trunc_ptr);
           })
      .def("render_rgb",
           [](mars::BatchEnv& self, int env_id, py::array_t<uint8_t, py::array::c_style> rgb,
              int width, int height, bool debug_overlay) {
             auto* rgb_ptr = checked_ptr(rgb, width * height * 3);
             self.render_rgb(env_id, rgb_ptr, width, height, debug_overlay);
           })
      .def("trial_exhausted",
           [](mars::BatchEnv& self, int env_id) { return self.env_at(env_id).trial_exhausted(); })
      .def("trial_steps_used",
           [](mars::BatchEnv& self, int env_id) { return self.env_at(env_id).trial_steps_used(); })
      .def("trial_step_budget",
           [](mars::BatchEnv& self, int env_id) { return self.env_at(env_id).trial_step_budget(); })
      .def("debug_info",
           [](mars::BatchEnv& self, int env_id) {
             const auto& env = self.env_at(env_id);
             const auto& state = env.state();
             const auto& params = env.mechanic_params();
             const auto& zone = env.mechanic_at(state.body.position.x);
             const auto visuals = mars::biome_by_id(zone.biome_id).visuals();
             py::dict d;
             d["mechanic"] = std::string(mars::biome_by_id(zone.biome_id).display_name());
             d["zone_begin_x"] = zone.begin_x;
             d["zone_end_x"] = zone.end_x;
             d["water_depth"] = zone.type == mars::MechanicType::Liquid
                                      ? std::max(0.0f, zone.liquid_level -
                                                          env.terrain().query(state.body.position.x).height)
                                      : 0.0f;
             d["thruster_thrust"] = state.thruster_thrust;
             d["drive_fuel_rate"] = state.drive_fuel_rate;
             d["energy_cost_rate"] = state.energy_cost_rate;
             d["energy_gain_rate"] = state.energy_gain_rate;
             d["trial_steps_used"] = env.trial_steps_used();
             d["trial_step_budget"] = env.trial_step_budget();
             d["trial_time_left"] = env.trial_time_left();
             d["trial_exhausted"] = env.trial_exhausted();
             d["x"] = state.body.position.x;
             d["distance_m"] = std::max(0.0f, state.body.position.x - 1.0f);
             d["best_distance_m"] = std::max(0.0f, env.best_progress() - 1.0f);
             d["course_difficulty"] = env.terrain().difficulty_at(state.body.position.x);
             d["safe_start_m"] = env.terrain().safe_start_fraction() * env.terrain().length();
             d["generated_pit_count"] = env.terrain().generated_pit_count();
             d["generated_step_count"] = env.terrain().generated_step_count();
             d["terrain_profile"] = zone.terrain_profile;
             d["terrain_frequency_scale"] = zone.terrain_frequency_scale;
             {
               static const char* kJagged[] = {"none", "washboard", "scree", "sawtooth", "rubble"};
               d["jagged_mode"] = kJagged[std::clamp(zone.jagged_mode, 0, 4)];
               d["jagged_amplitude"] = zone.jagged_amplitude;
             }
             d["y"] = state.body.position.y;
             d["ground_height"] = env.terrain().query(state.body.position.x).height;
             d["vx"] = state.body.velocity.x;
             d["vy"] = state.body.velocity.y;
             d["angle"] = state.body.angle;
             d["energy"] = state.energy;
             d["energy_capacity"] = env.config().physics.energy_capacity;
             d["damage"] = state.damage;
             d["airborne"] = state.airborne;
             d["airborne_steps"] = state.airborne_steps;
             d["landing_event"] = state.landing_event;
             d["landing_fatal"] = state.landing_fatal;
             d["fatal_error"] = state.fatal_error;
             d["pit_recovery_event"] = state.pit_recovery_event;
             d["pit_recovery_count"] = state.pit_recovery_count;
             d["impact_speed"] = state.last_impact_speed;
             d["landing_angle"] = state.last_landing_angle;
             d["episode_in_trial"] = state.episode_in_trial;
             d["gear"] = state.gear_index < 0 ? py::cast("N") : py::cast(state.gear_index + 1);
             d["gear_count"] = kDebugGearCount;
             d["driveline_load_factor"] = state.driveline_load_factor;
             d["gear_energy_multiplier"] = state.gear_energy_multiplier;
             const float speed = std::abs(state.body.velocity.x);
             d["speed"] = speed;
             d["speed_kmh"] = speed * 3.6f;
             d["gravity"] = env.config().physics.gravity * state.latent_gravity_multiplier;
             d["gravity_multiplier"] = state.latent_gravity_multiplier;
             d["engine_rpm"] = state.engine_rpm;
             d["engine_temperature"] = state.engine_temperature;
             d["ambient_temperature"] = state.ambient_temperature;
             d["thermal_transfer"] = state.thermal_transfer;
             d["cold_power_factor"] = state.cold_power_factor;
             d["engine_running"] = state.engine_running;
             d["engine_stalled"] = state.engine_stalled;
             d["engine_overheated"] = state.engine_overheated;
             d["engine_cold_locked"] = state.engine_cold_locked;
             d["heater_active"] = state.heater_active;
             d["heater_energy_rate"] = env.config().physics.engine_heater_energy_rate;
             d["cold_start_temperature"] = env.config().physics.cold_start_temperature;
             d["minimum_operating_temperature"] =
                 env.config().physics.minimum_operating_temperature;
             d["overheat_temperature"] = env.config().physics.overheat_temperature;
             d["overheat_restart_temperature"] =
                 env.config().physics.overheat_restart_temperature;
             d["final_drive_ratio"] = env.config().physics.final_drive_ratio;
             constexpr const char* kDriveNames[] = {"RWD", "FWD", "AWD"};
             d["drive_layout"] = kDriveNames[std::clamp(state.drive_mode, 0, 2)];
             const bool clutch_down = (state.previous_action & mars::ControlClutchPedal) != 0;
             d["clutch_down"] = clutch_down;
             d["clutch_engagement"] = state.clutch_engagement;
             d["clutch_time_remaining"] = clutch_down
                 ? state.clutch_engagement / 10.0f
                 : (1.0f - state.clutch_engagement) / 3.5f;
             d["drivetrain_slip"] = state.drivetrain_slip;
             py::list wheel_angular_velocities;
             for (int i = 0; i < state.wheel_count; ++i) {
               wheel_angular_velocities.append(state.wheels[static_cast<size_t>(i)].angular_velocity);
             }
             d["wheel_angular_velocities"] = wheel_angular_velocities;
             d["solar_panel_requested"] = state.solar_panel_requested;
             d["solar_panel_deployment"] = state.solar_panel_deployment;
             d["charging_active"] = state.charging_active;
             d["solar_charge_rate"] = state.solar_charge_rate;
             d["ballast_air"] = state.ballast_air;
             d["ballast_blowing"] = state.ballast_blowing;
             d["ballast_flooding"] = state.ballast_flooding;
             d["solar_panel_stationary"] = state.solar_panel_stationary;
             d["propeller_deployment"] = state.propeller_deployment;
             d["propeller_thrust"] = state.propeller_thrust;
             d["passive_charge_rate"] = state.passive_charge_rate;
             d["solar_irradiance"] = state.solar_irradiance;
             d["lidar_active"] = state.lidar_active_steps > 0;
             d["lidar_cooldown"] = static_cast<float>(state.lidar_cooldown_steps) *
                                     env.config().physics.dt;
             d["lidar_range"] = state.lidar_range;
             d["lidar_landing_valid"] = state.lidar_landing_valid;
             d["lidar_landing_x"] = state.lidar_landing_x;
             d["lidar_landing_y"] = state.lidar_landing_y;
             d["lidar_energy_cost"] = state.lidar_last_energy_cost;
             d["imu_acceleration_x"] = state.imu_acceleration.x;
             d["imu_acceleration_y"] = state.imu_acceleration.y;
             d["imu_angular_acceleration"] = state.imu_angular_acceleration;
             d["imu_impact"] = state.imu_impact;
             d["body_contact_front"] = state.body_contact_front;
             d["body_contact_belly"] = state.body_contact_belly;
             d["body_contact_rear"] = state.body_contact_rear;
             d["body_contact_roof"] = state.body_contact_roof;
             d["terrain_surprise_mode"] = zone.terrain_surprise_mode;
             constexpr const char* kTerminationReasons[] = {
                 "RUNNING", "FINISH REACHED", "FATAL LANDING", "ROVER ROLLOVER",
                 "FELL OUT OF COURSE", "BATTERY DEPLETED", "NO PROGRESS", "120 SECOND TIMER",
             };
             d["termination_reason"] = kTerminationReasons[
                 std::clamp(state.termination_reason, 0, 7)];
             d["screen_brightness"] = visuals.screen_brightness;
             const auto& hazard_biome = mars::biome_by_id(zone.biome_id);
             d["hazard"] = hazard_biome.hazard_at(state.step_index);


             d["hazard_ahead"] = hazard_biome.hazard_at(state.step_index + 55);
             d["sky_rgb"] = py::make_tuple(visuals.sky.r, visuals.sky.g, visuals.sky.b);
             d["can_shift_up"] = state.can_shift_up;
             d["can_shift_down"] = state.can_shift_down;
             d["should_shift_down"] = state.should_shift_down;
             d["shift_up_rpm"] = state.recommended_upshift_rpm;
             d["minimum_shift_up_rpm"] = state.minimum_upshift_rpm;
             d["next_gear_rpm"] = state.projected_upshift_rpm;
             d["upshift_speed_ok"] = state.upshift_speed_ok;
             d["upshift_recommended"] = state.upshift_recommended;
             d["shift_cooldown"] = static_cast<float>(state.shift_cooldown_steps) / 60.0f;
             d["shift_clutch_cut"] = static_cast<float>(state.shift_clutch_cut_steps) *
                 env.config().physics.dt;
             d["shift_energy_cost"] = state.last_shift_energy_cost;
             d["shift_buffered"] = state.shift_up_buffer_steps > 0 ||
                                     state.shift_down_buffer_steps > 0;
             d["top_gear"] = state.gear_index >= kDebugGearCount - 1;
             d["friction_mul"] = params.friction_mul;
             d["sink_rate"] = params.sink_rate;
             d["viscosity"] = params.viscosity;
             d["wind_force"] = params.wind_force;
             d["gravity_mul"] = params.gravity_mul;
             d["active_layers"] = state.active_layer_count;
             py::list active_layer_names;
             std::array<const mars::MechanicZone*, mars::kMaxActiveMechanisms> active_layers{};
             const int active_count = env.mechanic_layout().active_layers(
                 state.body.position.x, active_layers);
             for (int i = 0; i < active_count; ++i) {
               active_layer_names.append(mechanic_name(active_layers[static_cast<size_t>(i)]->type));
             }
             d["active_layer_names"] = active_layer_names;
             d["seed"] = state.world_seed;
             d["layer_weight"] = state.active_layer_weight;
             d["latent_traction"] = state.latent_traction;
             d["latent_moisture"] = state.latent_moisture;
             d["latent_heat"] = state.latent_heat;
             d["latent_charge_reserve"] = state.latent_charge_reserve;
             d["latent_viscosity"] = state.latent_viscosity;
             d["latent_sink"] = state.latent_sink;
             d["latent_energy_resistance"] = state.latent_energy_resistance;
             d["latent_suspension"] = state.latent_suspension;
             d["climb_mode"] = state.climb_mode;
             d["propeller_mode"] = state.propeller_mode;
             d["jump_cooldown"] = state.jump_cooldown_steps * env.config().physics.dt;
             d["suspension_jump_phase"] = state.suspension_jump_phase;
             d["suspension_jump_charge"] = state.suspension_jump_charge;
             d["suspension_jump_preload_velocity"] = state.suspension_jump_preload_velocity;
             d["suspension_jump_mask"] = state.suspension_jump_mask;
             d["roof_piston_extension"] = state.roof_piston_extension;
             d["roof_piston_mask"] = state.roof_piston_mask;
             d["roof_piston_contact"] = state.roof_piston_contact;
             d["recovery_state"] = state.recovery_state;
             d["route_branch"] = state.route_branch == 2 ? py::cast("upper_dry") :
                                 state.route_branch == 1 ? py::cast("lower_water") : py::cast("terrain");
             py::list wheels;
             for (int i = 0; i < state.wheel_count; ++i) {
               const auto& wheel = state.wheels[static_cast<size_t>(i)];
               py::dict wd;
               wd["x"] = wheel.position.x;
               wd["y"] = wheel.position.y;
               wd["vx"] = wheel.velocity.x;
               wd["vy"] = wheel.velocity.y;
               wd["angle"] = wheel.angle;
               wd["angular_velocity"] = wheel.angular_velocity;
               wd["in_contact"] = wheel.in_contact;
               wd["normal_force"] = wheel.normal_force;
               const auto& wheel_zone = env.mechanic_at(wheel.position.x);
               const float wheel_water_depth =
                   wheel_zone.type == mars::MechanicType::Liquid
                       ? wheel_zone.liquid_level - env.terrain().query(wheel.position.x).height
                       : 0.0f;
               wd["surface"] = mechanic_name(
                   wheel_zone.type == mars::MechanicType::Liquid && wheel_water_depth < 0.08f
                       ? mars::MechanicType::Sand
                       : wheel_zone.type);
               wheels.append(wd);
             }
             d["wheels"] = wheels;
             return d;
           });

  m.def("biome_catalog", [] {
    py::list out;
    const auto& bank = mars::biome_registry();
    for (int i = 0; i < static_cast<int>(bank.size()); ++i) {
      const auto& biome = *bank[static_cast<size_t>(i)];
      py::dict item;
      item["index"] = i;
      item["id"] = std::string(biome.id());
      item["name"] = std::string(biome.display_name());
      item["skill_stratum"] = std::string(biome.skill_stratum());
      item["is_anchor"] = biome.is_anchor();
      item["split"] = static_cast<int>(biome.split());
      item["visual_type"] = static_cast<int>(biome.visual_type());
      const auto params = biome.sample_params(0x4d415253ULL);
      py::dict values;
      values["friction"] = params.friction_mul;
      values["sink"] = params.sink_rate;
      values["viscosity"] = params.viscosity;
      values["wind"] = params.wind_force;
      values["gravity"] = params.gravity_mul;
      values["energy"] = params.energy_drain_mul;
      values["temperature"] = params.ambient_temperature;
      values["thermal"] = params.thermal_transfer;
      values["solar"] = params.solar_charge_rate;
      values["lidar_energy"] = params.lidar_energy_mul;
      values["lidar_range"] = params.lidar_range_mul;
      values["ledge_gap_width"] = params.ledge_gap_width;
      values["ledge_spacing"] = params.ledge_spacing;
      values["ledge_ramp_length"] = params.ledge_ramp_length;
      values["ledge_ramp_height"] = params.ledge_ramp_height;
      values["ledge_start_x"] = params.ledge_start_x;
      item["parameters"] = values;
      const auto visuals = biome.visuals();
      py::dict visual_values;
      visual_values["ground_rgb"] = py::make_tuple(
          visuals.ground.r, visuals.ground.g, visuals.ground.b);
      visual_values["sky_rgb"] = py::make_tuple(
          visuals.sky.r, visuals.sky.g, visuals.sky.b);
      visual_values["particle_rgb"] = py::make_tuple(
          visuals.particles.r, visuals.particles.g, visuals.particles.b);
      visual_values["particle_rate"] = visuals.particle_rate;
      visual_values["particle_lift"] = visuals.particle_lift;
      visual_values["particle_spread"] = visuals.particle_spread;
      visual_values["ambient_particles"] = visuals.ambient_particles;
      visual_values["screen_brightness"] = visuals.screen_brightness;
      item["visuals"] = visual_values;
      out.append(item);
    }
    return out;
  });
}
