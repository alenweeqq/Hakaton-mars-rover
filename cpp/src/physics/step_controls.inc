  PhysicsStepStats stats{};
  const float dt = config_.dt;
  const Vec2 previous_body_velocity = state.body.velocity;
  const float previous_body_angular_velocity = state.body.angular_velocity;
  state.landing_event = false;
  state.landing_fatal = false;
  state.body_contact_front = false;
  state.body_contact_belly = false;
  state.body_contact_rear = false;
  state.body_contact_roof = false;
  const bool was_airborne = state.airborne;
  const int previous_airborne_steps = state.airborne_steps;
  const float pre_contact_vertical_speed = state.body.velocity.y;
  ControlInput control = decode_discrete_action(discrete_action, config_.body_tilt_torque);
  const int jump_action_mask = ControlJump | ControlJumpFront | ControlJumpRear;
  bool jump_active = control.jump || control.jump_front || control.jump_rear;
  bool piston_active = control.roof_piston_front || control.roof_piston_rear;
  bool jump_released = !jump_active && (state.previous_action & jump_action_mask) != 0;
  const bool climb_pressed = control.toggle_climb &&
      (state.previous_action & ControlToggleClimb) == 0;
  const bool propeller_pressed = control.toggle_propeller &&
      (state.previous_action & ControlTogglePropeller) == 0;
  if (climb_pressed) state.climb_mode = !state.climb_mode;
  if (propeller_pressed) state.propeller_mode = !state.propeller_mode;
  state.propeller_deployment = clamp(
      state.propeller_deployment + (state.propeller_mode ? 1.0f : -1.0f) * dt / 0.65f,
      0.0f, 1.0f);
  if (state.propeller_mode) {
    state.propeller_phase = std::fmod(
        state.propeller_phase + state.propeller_deployment * dt *
        (11.0f + 21.0f * std::abs(control.throttle)), 6.2831853f);
  }
  if (state.jump_cooldown_steps > 0) --state.jump_cooldown_steps;
  if (state.suspension_jump_phase_steps > 0) --state.suspension_jump_phase_steps;
  if (state.suspension_jump_phase == 2 && state.suspension_jump_phase_steps == 0) {
    state.suspension_jump_phase = 0;
    state.suspension_jump_charge = 0.0f;
    state.suspension_jump_preload_velocity = 0.0f;
  }
  if (piston_active) {
    state.roof_piston_mask = (control.roof_piston_front ? 1 : 0) |
        (control.roof_piston_rear ? 2 : 0);
  }
  const float piston_speed = piston_active ? 1.0f / 0.18f : -1.0f / 0.10f;
  state.roof_piston_extension = clamp(state.roof_piston_extension + piston_speed * dt, 0.0f, 1.0f);
  state.roof_piston_contact = false;

  if (state.lidar_active_steps > 0) --state.lidar_active_steps;
  if (state.lidar_cooldown_steps > 0) --state.lidar_cooldown_steps;
  if (state.lidar_active_steps == 0) state.lidar_range = 0.0f;
  state.lidar_last_energy_cost = 0.0f;
  float lidar_energy_cost = 0.0f;
  const bool lidar_pressed = control.lidar && (state.previous_action & ControlLidar) == 0;
  if (lidar_pressed && state.lidar_cooldown_steps == 0) {
    const float requested_cost =
        config_.lidar_energy_cost * state.latent_lidar_energy_multiplier;
    if (state.energy >= requested_cost) {
      lidar_energy_cost = requested_cost;
      state.lidar_last_energy_cost = requested_cost;
      state.lidar_active_steps =
          std::max(1, static_cast<int>(config_.lidar_scan_duration / std::max(0.0001f, dt)));
      state.lidar_cooldown_steps =
          std::max(state.lidar_active_steps,
                   static_cast<int>(config_.lidar_cooldown / std::max(0.0001f, dt)));
      state.lidar_range = config_.lidar_base_range * state.latent_lidar_range_multiplier;
    }
  }
  const bool toggle_charge_pressed = control.toggle_charge &&
      (state.previous_action & ControlToggleCharge) == 0;
  if (toggle_charge_pressed) state.solar_panel_requested = !state.solar_panel_requested;
  if (state.solar_panel_requested) {
    state.solar_panel_deployment = std::min(
        1.0f, state.solar_panel_deployment + dt / std::max(0.1f, config_.panel_deploy_time));
  } else {
    state.solar_panel_deployment = std::max(
        0.0f, state.solar_panel_deployment - dt / std::max(0.1f, config_.panel_retract_time));
  }
  const bool charging_lockout = state.solar_panel_requested;
  if (charging_lockout) {
    control.throttle = 0.0f;
    control.brake = 1.0f;
    control.body_torque = 0.0f;
    control.jump = control.jump_front = control.jump_rear = false;
    control.roof_piston_front = control.roof_piston_rear = false;
    control.ballast_blow = control.ballast_flood = false;
    control.thruster = false;
    jump_active = false;
    jump_released = false;
    piston_active = false;
    state.suspension_jump_phase = 0;
    state.suspension_jump_charge = 0.0f;
    state.suspension_jump_preload_velocity = 0.0f;
    state.propeller_mode = false;
  }
  state.ballast_blowing = control.ballast_blow && !control.ballast_flood;
  state.ballast_flooding = control.ballast_flood && !control.ballast_blow;
  if (state.ballast_blowing) {
    state.ballast_air = clamp(state.ballast_air + 0.55f * dt, 0.0f, 1.0f);
    stats.energy_cost += 0.35f * dt;
  } else if (state.ballast_flooding) {
    state.ballast_air = clamp(state.ballast_air - 0.75f * dt, 0.0f, 1.0f);
  }
  const bool rover_stationary = length(state.body.velocity) < 0.06f &&
                                std::abs(state.body.angular_velocity) < 0.08f;
  state.solar_panel_stationary = rover_stationary;
  state.charging_active = state.solar_panel_requested &&
                          state.solar_panel_deployment >= 0.999f &&
                          (rover_stationary || state.airborne);
  const bool shift_up_pressed = control.shift_up && (state.previous_action & ControlShiftUp) == 0;
  const bool shift_down_pressed =
      control.shift_down && (state.previous_action & ControlShiftDown) == 0;
  const bool toggle_drive_pressed =
      control.toggle_drive && (state.previous_action & ControlToggleDrive) == 0;
  if (toggle_drive_pressed) {
    state.drive_mode = (state.drive_mode + 1) % 3;
  }
  if (state.shift_up_buffer_steps > 0) --state.shift_up_buffer_steps;
  if (state.shift_down_buffer_steps > 0) --state.shift_down_buffer_steps;
  if (shift_up_pressed) {
    state.shift_up_buffer_steps = 60;
    state.shift_down_buffer_steps = 0;
  }
  if (shift_down_pressed) {
    state.shift_down_buffer_steps = 60;
    state.shift_up_buffer_steps = 0;
  }
  const float shift_speed = std::abs(state.body.velocity.x);
  const bool clutch_pedal_down = control.clutch < 0.5f;
  const bool powered_upshift_request =
      shift_up_pressed && std::abs(control.throttle) > 0.5f && !clutch_pedal_down;
  const bool powered_downshift_request =
      shift_down_pressed && std::abs(control.throttle) > 0.5f && !clutch_pedal_down;

  const float clutch_target = state.shift_clutch_cut_steps > 0 ? 0.0f : control.clutch;
  const float clutch_rate = clutch_target > state.clutch_engagement ? 3.5f : 10.0f;
  state.clutch_engagement +=
      clamp(clutch_target - state.clutch_engagement, -clutch_rate * dt, clutch_rate * dt);
  state.clutch_engagement = clamp(state.clutch_engagement, 0.0f, 1.0f);
  if (state.shift_cooldown_steps > 0) {
    --state.shift_cooldown_steps;
  }
  if (state.shift_clutch_cut_steps > 0) {
    --state.shift_clutch_cut_steps;
  }
  state.last_shift_energy_cost = 0.0f;
