  float next_drivetrain_slip = 0.0f;
  bool next_drivetrain_grounded = false;
  bool any_wheel_grounded = false;
  for (int i = 0; i < state.wheel_count; ++i) {
    auto& wheel = state.wheels[static_cast<size_t>(i)];
    const auto& wr = rig.wheels[static_cast<size_t>(i)];
    const auto& wheel_zone = mechanics.at(wheel.position.x);
    const float liquid_immersion = wheel_zone.type == MechanicType::Liquid
                                       ? clamp((wheel_zone.liquid_level -
                                                (wheel.position.y - wheel.radius)) /
                                                   std::max(0.05f, wheel.radius * 2.0f),
                                               0.0f, 1.0f)
                                       : 0.0f;
    Vec2 wheel_force{0.0f, wheel.mass * gravity};
    if (liquid_immersion > 0.0f) {
      const float water_speed = length(wheel.velocity);
      wheel_force += wheel.velocity *
                     (-wheel.mass * liquid_immersion * (0.55f + water_speed * 0.85f));
      wheel_force.y += wheel.mass * -gravity * 0.48f * liquid_immersion;
    }
    const bool driven_wheel = is_driven_wheel(i);
    const bool jump_selected = state.suspension_jump_mask == 3 ||
        (state.suspension_jump_mask == 1 && i == front_wheel_index) ||
        (state.suspension_jump_mask == 2 && i == rear_wheel_index);
    const int driven_wheel_count = state.drive_mode == 2 ? std::max(1, state.wheel_count) : 1;
    const float torque_split = driven_wheel ? 1.0f / static_cast<float>(driven_wheel_count) : 0.0f;

    const Vec2 anchor_offset = rotate_body(wr.local_anchor);
    const Vec2 anchor_world = state.body.position + anchor_offset;
    const Vec2 anchor_velocity = state.body.velocity + perp(anchor_offset) * state.body.angular_velocity;
    const Vec2 suspension_axis = rotate_body({0.0f, -1.0f});
    const float min_len = std::max(0.02f, std::min(wr.suspension.min_length, wr.suspension.rest_length));
    const float max_len = std::max(min_len + 0.01f,
                                   std::max(wr.suspension.max_length, wr.suspension.rest_length));
    auto constrain_to_suspension = [&]() {
      const Vec2 offset = wheel.position - anchor_world;
      float travel = dot(offset, suspension_axis);
      const Vec2 lateral = offset - suspension_axis * travel;
      if (dot(lateral, lateral) > 1.0e-10f) {
        wheel.position -= lateral;
      }

      const Vec2 rel_velocity = wheel.velocity - anchor_velocity;
      const Vec2 lateral_velocity =
          rel_velocity - suspension_axis * dot(rel_velocity, suspension_axis);
      if (dot(lateral_velocity, lateral_velocity) > 1.0e-10f) {
        const Vec2 delta_v = -lateral_velocity;
        wheel.velocity += delta_v;
        if (wheel.inv_mass > 0.0f) {
          const Vec2 impulse_on_wheel = delta_v / wheel.inv_mass;
          apply_body_impulse(state.body, -impulse_on_wheel, anchor_world);
        }
      }

      travel = dot(wheel.position - anchor_world, suspension_axis);
      const float clamped_travel = clamp(travel, min_len, max_len);
      if (clamped_travel != travel) {
        wheel.position = anchor_world + suspension_axis * clamped_travel;
        const float travel_speed = dot(wheel.velocity - anchor_velocity, suspension_axis);
        const bool moving_past_min = travel < min_len && travel_speed < 0.0f;
        const bool moving_past_max = travel > max_len && travel_speed > 0.0f;
        if (moving_past_min || moving_past_max) {
          const Vec2 delta_v = suspension_axis * (-travel_speed);
          wheel.velocity += delta_v;
          if (wheel.inv_mass > 0.0f) {
            const Vec2 impulse_on_wheel = delta_v / wheel.inv_mass;
            apply_body_impulse(state.body, -impulse_on_wheel, anchor_world);
          }
        }
      }
    };

    constrain_to_suspension();

    const float suspension_travel = dot(wheel.position - anchor_world, suspension_axis);
    const float spring_vel = dot(wheel.velocity - anchor_velocity, suspension_axis);
    float jump_rest_length = wr.suspension.rest_length;
    float jump_stiffness = wr.suspension.stiffness;
    if (jump_selected && state.suspension_jump_phase == 1) {
      jump_rest_length -= 0.11f * state.suspension_jump_charge;
      jump_stiffness *= 1.0f + 2.5f * state.suspension_jump_charge;
    } else if (jump_selected && state.suspension_jump_phase == 2) {
      jump_rest_length += 0.16f * state.suspension_jump_charge;
      jump_stiffness *= 1.0f + 4.5f * state.suspension_jump_charge;
    }
    const float spring_force_mag =
        -jump_stiffness * (suspension_travel - jump_rest_length) -
        wr.suspension.damping * spring_vel;

    constexpr float kStopTravel = 0.045f;
    float stop_force_mag = 0.0f;
    const float compression_stop = (min_len + kStopTravel) - suspension_travel;
    if (compression_stop > 0.0f) {
      stop_force_mag += 2400.0f * compression_stop * compression_stop;
      if (spring_vel < 0.0f) stop_force_mag += -spring_vel * 70.0f;
    }
    const float rebound_stop = suspension_travel - (max_len - kStopTravel);
    if (rebound_stop > 0.0f) {
      stop_force_mag -= 1200.0f * rebound_stop * rebound_stop;
      if (spring_vel > 0.0f) stop_force_mag -= spring_vel * 40.0f;
    }
    const Vec2 spring_force = suspension_axis * (spring_force_mag + stop_force_mag);
    wheel_force += spring_force;
    body_force -= spring_force;
    body_torque += cross(anchor_world - state.body.position, -spring_force);

    auto terrain_sample = terrain.query_near(wheel.position.x, wheel.position.y);
    float contact_x = wheel.position.x;
    float penetration = terrain_sample.height + wheel.radius - wheel.position.y;

    const float motion_sign = control.throttle != 0.0f
                                  ? (control.throttle > 0.0f ? 1.0f : -1.0f)
                                  : (anchor_velocity.x >= 0.0f ? 1.0f : -1.0f);
    const float probe_dx = motion_sign * wheel.radius * 0.72f;
    const auto leading_sample = terrain.query_near(wheel.position.x + probe_dx, wheel.position.y);
    const float tyre_half_height = wheel.radius * 0.693974f;
    const float leading_penetration =
        leading_sample.height + tyre_half_height - wheel.position.y;
    if (leading_penetration > penetration) {
      terrain_sample = leading_sample;
      contact_x = wheel.position.x + probe_dx;
      penetration = leading_penetration;
    }
    WheelContact contact{};
    contact.wheel_index = i;
    float deformation_drive_effort = 0.0f;

    constexpr float kContactSlop = 0.018f;
    if (terrain_sample.solid && penetration > -kContactSlop) {
      contact.active = true;
      contact.point = {contact_x, terrain_sample.height};
      contact.normal = terrain_sample.normal;
      contact.tangent = normalized({contact.normal.y, -contact.normal.x});
      contact.penetration = penetration;
      contact.ground_height = terrain_sample.height;
      contact.slope = terrain_sample.slope;

      const float normal_vel = dot(wheel.velocity, contact.normal);
      const float stable_penetration = clamp(penetration, 0.0f, 0.14f);
      const float impact_speed = clamp(-normal_vel, 0.0f, 4.0f);
      const float normal_force = stable_penetration * 1400.0f + impact_speed * 55.0f;
      contact.normal_force = clamp(normal_force, 0.0f, 700.0f);
      wheel_force += contact.normal * contact.normal_force;
      const float axle_speed = dot(anchor_velocity, contact.tangent);
      const float driven_speed = axle_speed * control.throttle;
      const bool top_gear = state.gear_index == kGearCount - 1;
      const float top_gear_max_speed = gear_top_speed(kGearCount - 1);
      float speed_fade;
      if (!top_gear && driven_speed > gear_max_speed) {

        speed_fade = 0.0f;
      } else if (!top_gear || driven_speed <= top_gear_max_speed) {
        speed_fade = 1.0f / (1.0f + std::max(0.0f, driven_speed) /
                                    std::max(3.0f, gear_max_speed * 3.5f));
      } else {

        const float base_fade = 1.0f / (1.0f + top_gear_max_speed /
                                                    (top_gear_max_speed * 3.5f));
        const float excess = driven_speed - top_gear_max_speed;
        speed_fade = base_fade / (1.0f + std::log1p(excess /
                                                     std::max(1.0f, top_gear_max_speed * 0.4f)));
      }

      const float submerged_drive_coupling = 1.0f - 0.94f * liquid_immersion;

      const float nose_up = state.body.angle;
      const float wheelie_guard =
          nose_up > 0.60f && state.body.angular_velocity > 0.0f
              ? clamp(1.0f - (nose_up - 0.60f) / 0.50f, 0.45f, 1.0f)
              : 1.0f;
      const float drive_torque =
          (state.engine_running ? config_.motor_torque : 0.0f) * state.cold_power_factor *
          torque_curve * driveline_load_factor *
          lug_factor * reserve_power * climb_torque * gear_ratio *
          config_.final_drive_ratio *
          torque_split * speed_fade * control.throttle * state.clutch_engagement *
          submerged_drive_coupling * wheelie_guard;
      const float drive_force = drive_torque / std::max(0.05f, wheel.radius);
      Vec2 traction_force{0.0f, 0.0f};
      MechanicContext ctx{};

      const float supported_mass =
          (state.body.mass + static_cast<float>(state.wheel_count) * wheel.mass) /
          static_cast<float>(std::max(1, state.wheel_count));
      contact.normal_force = std::max(contact.normal_force, supported_mass * -gravity);
      ctx.contact = &contact;
      ctx.wheel_force = &traction_force;
      ctx.body_force = &body_force;
      ctx.energy_cost = &stats.energy_cost;
      ctx.dt = dt;
      ctx.wheel_radius = wheel.radius;
      ctx.base_friction = config_.wheel_friction * state.latent_traction *
                          (state.climb_mode ? 2.40f : 1.0f);
      ctx.drive_force = drive_force;
      ctx.immersion = liquid_immersion;

      ctx.minimum_drive_limit =
          state.engine_running && driven_wheel && !in_neutral && state.gear_index == 0 &&
                  shift_speed < 1.5f &&
                  state.clutch_engagement > 0.5f
              ? supported_mass * -gravity * state.latent_traction
              : 0.0f;
      ctx.wheel_speed = dot(wheel.velocity, contact.tangent);
      ctx.step_index = state.step_index;
      const float traction_limit = std::max(ctx.minimum_drive_limit,
                                            contact.normal_force * ctx.base_friction);
      const float limit = std::min(traction_limit,
                                   tip_limited_drive /
                                       static_cast<float>(std::max(1, driven_wheel_count)));
      const float applied_drive = clamp(ctx.drive_force, -limit, limit);
      deformation_drive_effort = driven_wheel
                                     ? clamp(std::abs(ctx.drive_force) /
                                                 std::max(1.0f, limit),
                                             0.0f, 4.0f)
                                     : 0.0f;
      traction_force += contact.tangent * applied_drive;
      contact.slip = std::abs(ctx.drive_force - applied_drive) /
                     (std::abs(ctx.drive_force) + 1.0f);

      traction_force += contact.tangent *
                        (-state.latent_viscosity * 2.2f * ctx.wheel_speed);
      stats.energy_cost += (0.025f + state.latent_sink * 0.015f +
                            state.latent_viscosity * 0.005f * std::abs(ctx.wheel_speed)) * dt;
      body_force += traction_force;
      body_torque += cross(anchor_world - state.body.position, traction_force);

      if (control.brake > 0.0f) {
        float brake_surface_scale = state.latent_traction;
        const float max_brake = contact.normal_force * config_.wheel_friction *
                                brake_surface_scale * 1.5f;
        const float brake_force = -clamp(axle_speed * config_.brake_strength * control.brake,
                                        -max_brake, max_brake);
        const Vec2 brake = contact.tangent * brake_force;
        body_force += brake;
        body_torque += cross(anchor_world - state.body.position, brake);
      }
    }

    wheel.in_contact = contact.active;
    wheel.normal_force = contact.normal_force;
    wheel.slip = contact.slip;
    auto& deformation_contact = stats.deformation_contacts[static_cast<size_t>(i)];
    deformation_contact.active = contact.active;
    deformation_contact.x = contact.point.x;
    deformation_contact.penetration = contact.penetration;
    deformation_contact.drive_effort = deformation_drive_effort;
    deformation_contact.slip = contact.slip;

    wheel.velocity += wheel_force * wheel.inv_mass * dt;
    wheel.velocity *= (1.0f - config_.linear_damping);
    wheel.position += wheel.velocity * dt;
    constrain_to_suspension();

    auto post_ground = terrain.query_near(wheel.position.x, wheel.position.y);
    const float post_penetration = post_ground.height + wheel.radius - wheel.position.y;
    if (post_ground.solid && post_penetration > 0.0f) {
      const float correction = std::min(post_penetration, 0.08f);
      wheel.position += post_ground.normal * correction;
      const float normal_speed = dot(wheel.velocity, post_ground.normal);
      if (normal_speed < 0.0f) {
        wheel.velocity -= post_ground.normal * normal_speed;
      }
      wheel.in_contact = true;
      contact.active = true;
      contact.point = {wheel.position.x, post_ground.height};
      contact.normal = post_ground.normal;
      contact.tangent = normalized({post_ground.normal.y, -post_ground.normal.x});
      contact.ground_height = post_ground.height;
      contact.slope = post_ground.slope;
      contact.penetration = std::max(contact.penetration, post_penetration);
      deformation_contact.active = true;
      deformation_contact.x = contact.point.x;
      deformation_contact.penetration = contact.penetration;
    }
    constrain_to_suspension();

    if (wheel.in_contact) {
      const float rolling_speed = dot(state.body.velocity, contact.tangent);

      const float target_angular_velocity = -rolling_speed / std::max(0.05f, wheel.radius);
      if (control.brake > 0.0f) {
        wheel.angular_velocity = 0.0f;
      } else if (liquid_immersion > 0.35f) {

        wheel.angular_velocity =
            std::abs(rolling_speed) <= 0.12f ? 0.0f : target_angular_velocity;
      } else if (driven_wheel && state.engine_running && !in_neutral &&
                 std::abs(control.throttle) > 0.0f) {
        const float powered_speed =
            control.throttle * (state.engine_rpm / kRedlineRpm) * gear_max_speed *
            driveline_load_factor;
        const float powered_angular_velocity =
            -powered_speed / std::max(0.05f, wheel.radius);
        const float visual_slip = clamp(wheel.slip, 0.0f, 1.0f);
        wheel.angular_velocity = target_angular_velocity +
                                 (powered_angular_velocity - target_angular_velocity) * visual_slip;
      } else {
        wheel.angular_velocity =
            std::abs(rolling_speed) <= 0.12f ? 0.0f : target_angular_velocity;
      }
    } else {

      wheel.angular_velocity = 0.0f;
    }
    wheel.angle += wheel.angular_velocity * dt;
    if (driven_wheel) {
      next_drivetrain_slip = std::max(next_drivetrain_slip, wheel.slip);
      next_drivetrain_grounded = next_drivetrain_grounded || wheel.in_contact;
    }
    any_wheel_grounded = any_wheel_grounded || wheel.in_contact;

    const float top_gear_max_speed = gear_top_speed(kGearCount - 1);
    const float overspeed = std::max(0.0f, speed - top_gear_max_speed);
    const float overspeed_ratio = overspeed / std::max(1.0f, top_gear_max_speed * 0.5f);
    const float overspeed_cost_mul = 1.0f + 9.0f * overspeed_ratio * overspeed_ratio * overspeed_ratio;
    const float wheel_fuel = driven_wheel && state.engine_running && !in_neutral
                                 ? std::abs(config_.motor_torque * control.throttle) *
                                       torque_split * state.clutch_engagement *
                                       kGearEnergyMul[state.gear_index] *
                                       (1.0f + 0.06f * speed * speed + state.latent_viscosity * 0.8f +
                                        (state.climb_mode ? 0.65f : 0.0f)) *
                                       overspeed_cost_mul *
                                       state.latent_energy_resistance *
                                       (1.0f - 0.94f * liquid_immersion) * 0.00025f * dt
                                 : 0.0f;
    stats.energy_cost += wheel_fuel;
    stats.drive_energy_cost += wheel_fuel;
  }
  state.drivetrain_slip = next_drivetrain_slip;
  state.drivetrain_grounded = next_drivetrain_grounded;
