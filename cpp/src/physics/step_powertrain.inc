  const float drivetrain_load =
      clamp(std::abs(std::sin(state.body.angle)) * 2.2f, 0.0f, 1.0f);
  float driven_wheel_radius = 0.0f;
  for (const auto& wheel : rig.wheels) driven_wheel_radius += wheel.radius;
  driven_wheel_radius /= std::max(1.0f, static_cast<float>(rig.wheels.size()));
  driven_wheel_radius = std::max(0.05f, driven_wheel_radius);
  constexpr float kRadPerSecToRpm = 60.0f / 6.28318530718f;

  constexpr float kEffectiveDrivelineRatio = 1.0f;

  float half_wheelbase = 0.65f;
  float com_height = 0.30f;
  for (const auto& wheel : rig.wheels) {
    half_wheelbase = std::max(half_wheelbase, std::abs(wheel.local_anchor.x));
    com_height = std::max(com_height, std::abs(wheel.local_anchor.y));
  }
  const float rover_mass =
      state.body.mass + static_cast<float>(state.wheel_count) *
                            (rig.wheels.empty() ? 1.0f : rig.wheels.front().mass);
  const float tip_limited_drive = 0.80f * rover_mass * std::abs(config_.gravity) *
                                  (half_wheelbase / std::max(0.05f, com_height));
  const auto gear_top_speed = [&](int gear_index) {
    if (gear_index < 0 || gear_index >= kGearCount) return 1.0e9f;
    const float rpm_per_speed = kGearRatios[gear_index] * config_.final_drive_ratio *
                                kEffectiveDrivelineRatio * kRadPerSecToRpm /
                                driven_wheel_radius;
    return kRedlineRpm / std::max(1.0f, rpm_per_speed);
  };
  const auto road_rpm_for_gear = [&](int gear_index) {
    if (gear_index < 0 || gear_index >= kGearCount) return kIdleRpm;
    return shift_speed / driven_wheel_radius * kGearRatios[gear_index] *
           config_.final_drive_ratio * kEffectiveDrivelineRatio * kRadPerSecToRpm;
  };

  const float minimum_post_shift_rpm = 1150.0f + drivetrain_load * 450.0f;
  float next_gear_rpm = kIdleRpm;
  float recommended_upshift_rpm = 0.0f;
  float minimum_upshift_rpm = 0.0f;
  if (state.gear_index >= 0 && state.gear_index < kGearCount - 1) {
    next_gear_rpm = road_rpm_for_gear(state.gear_index + 1);

    const float speed_step = kGearRatios[state.gear_index] /
                             kGearRatios[state.gear_index + 1];
    recommended_upshift_rpm = 0.78f * kRedlineRpm;
    minimum_upshift_rpm =
        clamp(minimum_post_shift_rpm * speed_step, 1800.0f, 8500.0f);
  }
  const bool upshift_speed_ok =
      state.gear_index < 0 || next_gear_rpm >= minimum_post_shift_rpm * 0.98f;

  const float next_gear_floor =
      state.gear_index >= 0 && state.gear_index < kGearCount - 1
          ? std::max(minimum_post_shift_rpm, kMinimumLoadedRpm[state.gear_index + 1])
          : minimum_post_shift_rpm;
  const bool upshift_recommended =
      state.gear_index >= 0 && state.engine_rpm >= recommended_upshift_rpm &&
      next_gear_rpm >= next_gear_floor;
  const bool can_shift_up =
      state.shift_up_buffer_steps > 0 &&
      (clutch_pedal_down || powered_upshift_request || std::abs(control.throttle) > 0.5f) &&
      state.shift_cooldown_steps == 0 &&
      state.gear_index < kGearCount - 1 &&
      upshift_speed_ok;
  const float lower_gear_rpm =
      state.gear_index > 0
          ? road_rpm_for_gear(state.gear_index - 1)
          : kIdleRpm;
  const bool can_shift_down = state.shift_down_buffer_steps > 0 &&
                              (clutch_pedal_down || powered_downshift_request) &&
                              state.shift_cooldown_steps == 0 &&
                              state.gear_index >= 0 &&
                              (state.gear_index == 0 || lower_gear_rpm <= kSafeDownshiftRpm);
  const float minimum_loaded_rpm = 1500.0f + drivetrain_load * 1800.0f;
  state.recommended_upshift_rpm = recommended_upshift_rpm;
  state.minimum_upshift_rpm = minimum_upshift_rpm;
  state.projected_upshift_rpm = next_gear_rpm;
  state.upshift_speed_ok = upshift_speed_ok;
  state.upshift_recommended = upshift_recommended;
  state.can_shift_up = can_shift_up && !state.climb_mode;
  state.can_shift_down = can_shift_down && !state.climb_mode;
  state.should_shift_down = state.gear_index > 0 && state.engine_rpm < minimum_loaded_rpm;
  if (state.shift_up_buffer_steps > 0 && can_shift_up) {
    const float target_rpm = road_rpm_for_gear(state.gear_index + 1);
    const float mismatch_krpm = std::abs(state.engine_rpm - target_rpm) * 0.001f;
    state.last_shift_energy_cost = config_.shift_energy_base +
        config_.shift_energy_sync_per_krpm * mismatch_krpm;
    stats.energy_cost += state.last_shift_energy_cost;
    state.gear_index = std::min(state.gear_index + 1, kGearCount - 1);
    state.shift_cooldown_steps = 45;
    state.shift_clutch_cut_steps = std::max(1, static_cast<int>(0.16f / dt));
    state.clutch_engagement = 0.0f;
    state.shift_up_buffer_steps = 0;
    state.shift_down_buffer_steps = 0;
    state.can_shift_up = false;
    state.can_shift_down = false;
  }
  if (state.shift_down_buffer_steps > 0 && can_shift_down) {
    const float target_rpm = road_rpm_for_gear(state.gear_index - 1);
    const float mismatch_krpm = std::abs(state.engine_rpm - target_rpm) * 0.001f;
    state.last_shift_energy_cost = config_.shift_energy_base +
        config_.shift_energy_sync_per_krpm * mismatch_krpm;
    stats.energy_cost += state.last_shift_energy_cost;
    state.gear_index = std::max(state.gear_index - 1, -1);
    state.shift_cooldown_steps = 45;
    state.shift_clutch_cut_steps = std::max(1, static_cast<int>(0.16f / dt));
    state.clutch_engagement = 0.0f;
    state.shift_up_buffer_steps = 0;
    state.shift_down_buffer_steps = 0;
    state.can_shift_up = false;
    state.can_shift_down = false;
  }
  const bool in_neutral = state.gear_index < 0;
  const float gear_ratio = in_neutral ? 0.0f : kGearRatios[state.gear_index];
  const float gear_max_speed = in_neutral ? 1.0e9f : gear_top_speed(state.gear_index);
  const float road_coupled_rpm =
      in_neutral ? state.engine_rpm
                 : road_rpm_for_gear(state.gear_index);
  const float driveline_load_factor =
      in_neutral ? 0.0f
                 : (state.gear_index == 0
                        ? 1.0f
                        : clamp((road_coupled_rpm -
                                 kMinimumLoadedRpm[state.gear_index] * 0.85f) /
                                    (kMinimumLoadedRpm[state.gear_index] * 0.15f),
                                0.0f, 1.0f));
  state.driveline_load_factor = driveline_load_factor;
  state.gear_energy_multiplier =
      in_neutral ? 0.0f : kGearEnergyMul[state.gear_index];
  state.should_shift_down =
      state.should_shift_down || (state.gear_index > 0 && driveline_load_factor < 0.35f);
  int front_wheel_index = 0;
  int rear_wheel_index = 0;
  for (int i = 1; i < state.wheel_count; ++i) {
    if (rig.wheels[static_cast<size_t>(i)].local_anchor.x >
        rig.wheels[static_cast<size_t>(front_wheel_index)].local_anchor.x) {
      front_wheel_index = i;
    }
    if (rig.wheels[static_cast<size_t>(i)].local_anchor.x <
        rig.wheels[static_cast<size_t>(rear_wheel_index)].local_anchor.x) {
      rear_wheel_index = i;
    }
  }
  const auto is_driven_wheel = [&](int i) {
    return state.drive_mode == 2 || (state.drive_mode == 1 && i == front_wheel_index) ||
           (state.drive_mode == 0 && i == rear_wheel_index);
  };
  const float tyre_coupling = state.drivetrain_grounded
                                  ? 1.0f - clamp(state.drivetrain_slip * 0.95f, 0.0f, 0.95f)
                                  : 0.05f;
  const float drivetrain_coupling = state.clutch_engagement * tyre_coupling;

  if (state.engine_overheated &&
      state.engine_temperature <= config_.overheat_restart_temperature) {
    state.engine_overheated = false;
  }
  if (state.engine_running &&
      state.engine_temperature <= config_.minimum_operating_temperature) {
    state.engine_running = false;
  }
  state.engine_cold_locked =
      state.engine_temperature < config_.cold_start_temperature;

  const bool battery_empty = state.energy <= 0.0f;
  if (state.engine_running && battery_empty) {
    state.engine_running = false;
  }
  if (!state.engine_running && control.ignition && (in_neutral || clutch_pedal_down) &&
      !state.engine_overheated && !battery_empty &&
      !state.engine_cold_locked) {
    state.engine_running = true;
    state.engine_rpm = std::max(state.engine_rpm, 700.0f);
  }
  state.cold_power_factor =
      clamp((state.engine_temperature - config_.minimum_operating_temperature) /
                std::max(1.0f, config_.full_power_temperature -
                                   config_.minimum_operating_temperature),
            0.0f, 1.0f);

  const float preliminary_rpm01 =
      clamp((state.engine_rpm - kIdleRpm) / (kRedlineRpm - kIdleRpm), 0.0f, 1.0f);
  const float preliminary_torque_curve =
      0.62f + 0.58f * (1.0f - std::abs(preliminary_rpm01 - 0.55f) / 0.55f);
  const float throttle_torque =
      state.engine_running
          ? std::abs(control.throttle) * config_.motor_torque * preliminary_torque_curve *
                (0.35f + state.cold_power_factor * 0.65f)
          : 0.0f;
  const float drag_torque =
      config_.engine_drag_torque * (0.35f + 0.65f * state.engine_rpm / kRedlineRpm);
  const float idle_governor =
      state.engine_running && (in_neutral || clutch_pedal_down) &&
              state.engine_rpm < kIdleRpm
          ? clamp((kIdleRpm - state.engine_rpm) * 0.08f, 0.0f,
                  config_.motor_torque * 0.45f)
          : 0.0f;
  const float free_rpm_acceleration =
      (throttle_torque + idle_governor - drag_torque) /
      std::max(0.05f, config_.engine_inertia) * kRadPerSecToRpm;
  float rpm_acceleration = free_rpm_acceleration;
  if (!in_neutral && state.clutch_engagement > 0.001f) {
    const float coupled_rpm = state.gear_index == 0 && std::abs(control.throttle) > 0.0f
                                  ? std::max(kIdleRpm, road_coupled_rpm)
                                  : road_coupled_rpm;

    const float clutch_sync_gain = 2.4f + 1.2f * state.clutch_engagement;
    const float clutch_acceleration =
        clamp((coupled_rpm - state.engine_rpm) * clutch_sync_gain,
              -config_.clutch_sync_rate * 1.6f, config_.clutch_sync_rate);

    rpm_acceleration = clutch_acceleration * drivetrain_coupling +
                       free_rpm_acceleration * (1.0f - drivetrain_coupling * 0.88f);
  }
  state.engine_rpm =
      clamp(state.engine_rpm + rpm_acceleration * dt, 0.0f, kRedlineRpm);
  if (state.gear_index > 0 && state.clutch_engagement > 0.85f &&
      driveline_load_factor < 0.12f) {
    ++state.engine_lug_steps;
  } else {
    state.engine_lug_steps = 0;
  }
  if (state.engine_running && !in_neutral && !clutch_pedal_down &&
      state.gear_index > 0 &&
      (state.engine_rpm < 650.0f || state.engine_lug_steps >= 45)) {
    state.engine_running = false;
  }
  const float rpm01 = clamp((state.engine_rpm - kIdleRpm) / (kRedlineRpm - kIdleRpm), 0.0f, 1.0f);
  const float torque_curve = 0.62f + 0.58f * (1.0f - std::abs(rpm01 - 0.55f) / 0.55f);
  const float lug_factor = state.gear_index <= 0
                               ? 1.0f
                               : clamp((state.engine_rpm - 650.0f) / 1550.0f, 0.0f, 1.0f);

  const float battery_fraction = clamp(
      state.energy / std::max(1.0f, config_.energy_capacity), 0.0f, 1.0f);
  const float reserve_power = 0.006f + 0.994f * std::sqrt(battery_fraction);

  const float climb_torque = state.climb_mode ? 1.05f : 1.0f;

  const auto& body_zone = mechanics.at(state.body.position.x);
  state.solar_charge_rate = state.charging_active ? state.latent_solar_rate * 10.0f : 0.0f;
  state.solar_irradiance = state.latent_solar_rate;
  stats.energy_gain = state.solar_charge_rate * dt;
  float ambient_temperature = state.latent_ambient_temperature;
  float thermal_transfer = state.latent_thermal_transfer;
  if (body_zone.type == MechanicType::Liquid) {
    const float engine_immersion =
        clamp((body_zone.liquid_level -
               (state.body.position.y - rig.body.size.y * 0.25f)) /
                  std::max(0.1f, rig.body.size.y),
              0.0f, 1.0f);
    constexpr float kMarsAirTemperature = -50.0f;
    ambient_temperature = kMarsAirTemperature +
                          (ambient_temperature - kMarsAirTemperature) * engine_immersion;
    thermal_transfer = 1.0f + (thermal_transfer - 1.0f) * engine_immersion;
  }
  state.ambient_temperature = ambient_temperature;
  state.thermal_transfer = thermal_transfer;
  const float speed = std::abs(state.body.velocity.x);
  const float airflow_factor = speed / (speed + 6.0f);
  const float conductance =
      (config_.engine_cooling_conductance +
       config_.engine_cooling_airflow * airflow_factor) * thermal_transfer;
  const float heat_removed = conductance * (state.engine_temperature - ambient_temperature);
  const float fuel_power = state.drive_fuel_rate;
  const float rpm_heat_factor =
      0.55f + 0.45f * clamp(state.engine_rpm / kRedlineRpm, 0.0f, 1.0f);
  const float combustion_heat =
      state.engine_running
          ? config_.engine_idle_heat * (0.6f + 0.4f * rpm01) +
                config_.engine_heat_per_fuel * fuel_power * rpm_heat_factor +
                config_.engine_idle_heat * 1.6f * std::abs(control.throttle) * rpm01
          : 0.0f;
  const float thermal_mass = std::max(0.01f, config_.engine_thermal_mass);
  const float heater_energy =
      std::max(0.0f, config_.engine_heater_energy_rate) * dt;
  state.heater_active = control.heater &&
                        !state.engine_overheated && state.energy >= heater_energy;
  if (state.heater_active) {
    stats.energy_cost += heater_energy;
  }
  constexpr float kThrusterHeat = 90.0f;
  const float thruster_heat = control.thruster && state.energy > 0.0f &&
                                      !state.engine_overheated
                                  ? kThrusterHeat
                                  : 0.0f;
  const float total_heat = combustion_heat +
                           (state.heater_active
                                ? std::max(0.0f, config_.engine_heater_heat)
                                : 0.0f) +
                           thruster_heat;
  state.engine_temperature += (total_heat - heat_removed) / thermal_mass * dt;
  if (state.engine_temperature >= config_.overheat_temperature) {
    state.engine_temperature = std::min(state.engine_temperature,
                                        config_.overheat_temperature + 15.0f);
    state.engine_overheated = true;
    state.engine_running = false;
  }
  if (state.engine_temperature <= config_.minimum_operating_temperature) {
    state.engine_running = false;
  }
  state.cold_power_factor =
      clamp((state.engine_temperature - config_.minimum_operating_temperature) /
                std::max(1.0f, config_.full_power_temperature -
                                   config_.minimum_operating_temperature),
            0.0f, 1.0f);
  state.engine_cold_locked =
      !state.engine_running &&
      state.engine_temperature < config_.cold_start_temperature;
  state.engine_stalled = !state.engine_running;

  float gravity = config_.gravity * state.latent_gravity_multiplier;
  if (state.energy <= 0.0f) gravity *= 4.0f;

  Vec2 body_force{0.0f, state.body.mass * gravity};
  const bool spiked_in = state.climb_mode && state.drivetrain_grounded;
  body_force.x += state.latent_wind_force * (spiked_in ? 0.2f : 1.0f);
  if (spiked_in) {

    body_force.y += state.body.mass * -gravity * 0.4f;
    body_force.x -= state.body.velocity.x * state.body.mass * 0.75f;
  }

  if (state.energy <= 0.0f) {
    constexpr float kLimpSpeedLimit = 0.35f;
    const float excess_speed =
        std::max(0.0f, std::abs(state.body.velocity.x) - kLimpSpeedLimit);
    if (excess_speed > 0.0f) {
      body_force.x -= std::copysign(state.body.mass * 0.80f * excess_speed,
                                    state.body.velocity.x);
    }
  }
  float body_torque = control.body_torque;
  if (!charging_lockout && state.solar_panel_deployment > 0.01f &&
      std::abs(state.body.velocity.x) > 0.3f) {
    const Vec2 panel_axis = rotate({0.0f, 1.0f}, state.body.angle);
    const float panel_lift = state.solar_panel_deployment *
        std::min(22.0f, std::abs(state.body.velocity.x) * std::abs(state.body.velocity.x) * 0.55f);
    const Vec2 panel_force = panel_axis * panel_lift;
    body_force += panel_force;
    body_torque += cross(rotate({0.0f, rig.body.size.y * 0.5f + 0.25f}, state.body.angle), panel_force);
  }
  if (jump_active && state.jump_cooldown_steps == 0 && !state.airborne) {

    if (state.suspension_jump_phase != 1) {
      state.suspension_jump_mask = control.jump ? 3 : (control.jump_front ? 1 : 2);
      state.suspension_jump_preload_velocity = 0.0f;
    }
    state.suspension_jump_phase = 1;
    constexpr float kPreloadAngularFrequency = 7.5f;
    constexpr float kPreloadDampingRatio = 1.0f;
    const float preload_acceleration =
        kPreloadAngularFrequency * kPreloadAngularFrequency *
            (1.0f - state.suspension_jump_charge) -
        2.0f * kPreloadDampingRatio * kPreloadAngularFrequency *
            state.suspension_jump_preload_velocity;
    state.suspension_jump_preload_velocity += preload_acceleration * dt;
    state.suspension_jump_charge = clamp(
        state.suspension_jump_charge + state.suspension_jump_preload_velocity * dt, 0.0f, 1.0f);
    if ((state.suspension_jump_charge <= 0.0f && state.suspension_jump_preload_velocity < 0.0f) ||
        (state.suspension_jump_charge >= 1.0f && state.suspension_jump_preload_velocity > 0.0f)) {
      state.suspension_jump_preload_velocity = 0.0f;
    }
    stats.energy_cost += 2.50f * dt;
  } else if (jump_released && state.suspension_jump_phase == 1 &&
             state.suspension_jump_charge >= 0.06f) {
    state.suspension_jump_phase = 2;
    state.suspension_jump_preload_velocity = 0.0f;
    state.suspension_jump_phase_steps = std::max(
        1, static_cast<int>((0.06f + 0.15f * state.suspension_jump_charge) / dt));
    state.jump_cooldown_steps = std::max(1, static_cast<int>(0.45f / dt));
  }
  if (std::abs(state.body.angle) > 1.35f) {
    state.recovery_state = clamp((std::abs(state.body.angle) - 1.35f) / 1.6f, 0.0f, 1.0f);

    if (jump_active) {
      const float recovery_torque = 16.0f * state.recovery_state;
      const float damping_torque = -state.body.angular_velocity * 6.0f;
      body_torque += (state.body.angle > 0.0f ? -recovery_torque : recovery_torque) + damping_torque;
    }
  } else {
    state.recovery_state = 0.0f;
  }
  MechanicBodyContext body_context{};
  body_context.body_force = &body_force;
  body_context.body_torque = &body_torque;
  body_context.energy_cost = &stats.energy_cost;
  body_context.velocity = state.body.velocity;
  body_context.angular_velocity = state.body.angular_velocity;
  body_context.mass = state.body.mass;
  body_context.gravity = gravity;
  body_context.dt = dt;
  body_context.step_index = state.step_index;

  (void)body_context;
  const float body_cos = std::cos(state.body.angle);
  const float body_sin = std::sin(state.body.angle);
