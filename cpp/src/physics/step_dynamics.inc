  const auto rotate_body = [body_cos, body_sin](Vec2 v) {
    return Vec2{body_cos * v.x - body_sin * v.y, body_sin * v.x + body_cos * v.y};
  };
  const auto register_body_contact = [&](Vec2 local) {
    const float half_width = std::max(0.01f, rig.body.collision.size.x * 0.5f);
    if (local.y > 0.0f) {
      state.body_contact_roof = true;
    }
    if (local.x > half_width * 0.55f) {
      state.body_contact_front = true;
    } else if (local.x < -half_width * 0.55f) {
      state.body_contact_rear = true;
    } else {
      state.body_contact_belly = true;
    }
  };

  if (piston_active && state.roof_piston_extension > 0.02f && rig.body.size.y > 0.0f) {
    const Vec2 roof_axis = rotate_body({0.0f, 1.0f});
    for (int piston = 0; piston < 2; ++piston) {
      const int piston_bit = 1 << piston;
      if ((state.roof_piston_mask & piston_bit) == 0) continue;
      const float local_x = piston == 0 ? rig.body.size.x * 0.34f : -rig.body.size.x * 0.34f;
      const Vec2 base = state.body.position + rotate_body({local_x, rig.body.size.y * 0.5f});
      const float rod_length = 0.14f + 0.36f * state.roof_piston_extension;
      const Vec2 tip = base + roof_axis * rod_length;
      const auto surface = terrain.query_near(tip.x, tip.y);
      if (!surface.solid || surface.height < tip.y - 0.015f) continue;
      const float penetration = std::min(0.12f, surface.height - tip.y + 0.015f);
      const Vec2 tip_velocity = state.body.velocity +
          perp(tip - state.body.position) * state.body.angular_velocity;
      const float tip_speed = dot(tip_velocity, roof_axis);

      const float force = state.roof_piston_extension *
          std::max(0.0f, 507.0f * penetration - 12.0f * tip_speed);
      const Vec2 reaction = -roof_axis * force;
      body_force += reaction;
      body_torque += cross(base - state.body.position, reaction);
      stats.hard_contact = std::max(stats.hard_contact, force);
      stats.energy_cost += 8.0f * state.roof_piston_extension * dt;
      state.roof_piston_contact = true;
    }
  }

  if (rig.body.size.y > 0.0f) {
    const float half_width = rig.body.size.x * 0.5f;
    const float half_height = rig.body.size.y * 0.5f;
    constexpr float sample_x[] = {-0.72f, 0.0f, 0.72f};
    const float sample_mass = state.body.mass / 3.0f;
    for (float nx : sample_x) {
      const Vec2 r = rotate_body({nx * half_width, -half_height});
      const Vec2 point = state.body.position + r;
      const auto& water_zone = mechanics.at(point.x);
      if (water_zone.type != MechanicType::Liquid) continue;
      const float submerged =
          clamp((water_zone.liquid_level - point.y) / rig.body.size.y, 0.0f, 1.0f);
      if (submerged <= 0.0f) continue;
      const Vec2 point_velocity = state.body.velocity + perp(r) * state.body.angular_velocity;
      const float point_speed = length(point_velocity);
      const float ballast_buoyancy = 0.45f + 1.10f * state.ballast_air;
      const Vec2 buoyancy{0.0f, sample_mass * -gravity * ballast_buoyancy * submerged};
      const Vec2 water_drag =
          point_velocity * (-sample_mass * submerged * (0.32f + point_speed * 0.52f));
      const Vec2 water_force = buoyancy + water_drag;
      body_force += water_force;
      body_torque += cross(r, water_force);
      stats.energy_cost += submerged * point_speed * point_speed * 0.006f * dt;
    }
  }
  state.propeller_thrust = 0.0f;
  const Vec2 propeller_hub = state.body.position +
      rotate_body({-rig.body.size.x * 0.58f, 0.02f});
  const auto& propeller_zone = mechanics.at(propeller_hub.x);
  const bool propeller_submerged = propeller_zone.type == MechanicType::Liquid &&
      propeller_zone.liquid_level >= propeller_hub.y - 0.03f;
  const bool propeller_upright = rotate_body({0.0f, 1.0f}).y >= 0.35f;
  if (state.propeller_deployment >= 0.99f && control.throttle > 0.0f &&
      propeller_upright) {

    const float rpm_threshold = 1800.0f;
    const float rpm_span = 2200.0f;
    const float propeller_ready = clamp((state.engine_rpm - rpm_threshold) /
                                            std::max(1.0f, rpm_span),
                                        0.0f, 1.0f);
    const float thrust = 108.0f * reserve_power * propeller_ready;
    body_force += rotate_body({thrust, 0.0f});
    state.propeller_thrust = thrust;
    const float medium_cost = propeller_submerged ? 1.0f : 1.25f;
    stats.energy_cost += medium_cost * (2.0f + 6.0f * control.throttle) * dt;
  }

  state.thruster_thrust = 0.0f;
  if (control.thruster && state.energy > 0.0f && !state.engine_overheated) {
    constexpr float kThrusterWeightRatio = 2.4f;
    const float rover_weight = rover_mass * std::abs(gravity);
    const float thrust = kThrusterWeightRatio * rover_weight;
    body_force += rotate_body({0.0f, 1.0f}) * thrust;
    state.thruster_thrust = thrust;
    stats.energy_cost += 8.0f * dt;
  }
