  bool body_ground_contact = false;
  if (rig.body.collision.type == CollisionType::Box) {
    const float hw = rig.body.collision.size.x * 0.5f;
    const float hh = rig.body.collision.size.y * 0.5f;

    const Vec2 samples[8] = {
        {-hw, -hh}, {0.0f, -hh}, {hw, -hh}, {hw, 0.0f},
        {hw, hh}, {0.0f, hh}, {-hw, hh}, {-hw, 0.0f},
    };
    for (const Vec2 local : samples) {
      const Vec2 point = state.body.position + rotate_body(local);
      const auto ground = terrain.query_near(point.x, point.y);
      if (!ground.solid) continue;
      const float penetration = ground.height - point.y;
      if (penetration > 0.0f) {
        body_ground_contact = true;
        register_body_contact(local);
        const Vec2 ground_normal = ground.normal;
        const Vec2 r = point - state.body.position;
        const Vec2 point_velocity = state.body.velocity + perp(r) * state.body.angular_velocity;
        const float normal_vel = dot(point_velocity, ground_normal);
        const float force_mag = std::max(0.0f, penetration * 3000.0f - normal_vel * 120.0f);
        const Vec2 force = ground_normal * force_mag;
        body_force += force;
        body_torque += cross(r, force);
        stats.hard_contact = std::max(stats.hard_contact, force_mag);
      }
    }
  }

  state.body.velocity += body_force * state.body.inv_mass * dt;
  state.body.velocity *= (1.0f - config_.linear_damping);
  state.body.position += state.body.velocity * dt;
  state.body.angular_velocity += body_torque * state.body.inv_inertia * dt;
  state.body.angular_velocity *= (1.0f - config_.angular_damping);
  state.body.angle += state.body.angular_velocity * dt;
  const float camera_follow = clamp(dt * 7.0f, 0.0f, 1.0f);
  state.render_camera_position +=
      (state.body.position - state.render_camera_position) * camera_follow;

  float body_contact_impulse = 0.0f;
  if (rig.body.collision.type == CollisionType::Box) {
    const float hw = rig.body.collision.size.x * 0.5f;
    const float hh = rig.body.collision.size.y * 0.5f;
    const Vec2 samples[8] = {
        {-hw, -hh}, {0.0f, -hh}, {hw, -hh}, {hw, 0.0f},
        {hw, hh}, {0.0f, hh}, {-hw, hh}, {-hw, 0.0f},
    };
    constexpr float kBodyFriction = 0.72f;
    constexpr float kRestitution = 0.03f;
    constexpr float kContactSlop = 0.001f;
    for (int iteration = 0; iteration < 4; ++iteration) {
      float deepest = 0.0f;
      Vec2 deepest_local{};
      TerrainSample deepest_ground{};
      bool found = false;
      for (const Vec2 local : samples) {
        const Vec2 point = state.body.position + rotate(local, state.body.angle);
        const auto ground = terrain.query_near(point.x, point.y);
        if (!ground.solid) continue;
        const float penetration = ground.height - point.y;
        if (penetration > deepest) {
          deepest = penetration;
          deepest_local = local;
          deepest_ground = ground;
          found = true;
        }
      }
      if (!found) break;

      body_ground_contact = true;
      register_body_contact(deepest_local);
      const Vec2 normal = deepest_ground.normal;
      state.body.position += normal * (deepest + kContactSlop);
      const Vec2 r = rotate(deepest_local, state.body.angle);
      Vec2 point_velocity = state.body.velocity + perp(r) * state.body.angular_velocity;
      const float normal_speed = dot(point_velocity, normal);
      float normal_impulse = 0.0f;
      if (normal_speed < 0.0f) {
        const float lever = cross(r, normal);
        const float effective_inv_mass =
            state.body.inv_mass + lever * lever * state.body.inv_inertia;
        if (effective_inv_mass > 1.0e-8f) {
          normal_impulse = -(1.0f + kRestitution) * normal_speed / effective_inv_mass;
          body_contact_impulse = std::max(body_contact_impulse, normal_impulse);
          apply_body_impulse(state.body, normal * normal_impulse,
                             state.body.position + r);
        }
      }

      point_velocity = state.body.velocity + perp(r) * state.body.angular_velocity;
      const Vec2 tangent = normalized({normal.y, -normal.x});
      const float tangent_speed = dot(point_velocity, tangent);
      const float tangent_lever = cross(r, tangent);
      const float tangent_inv_mass =
          state.body.inv_mass + tangent_lever * tangent_lever * state.body.inv_inertia;
      if (tangent_inv_mass > 1.0e-8f) {
        const float desired_impulse = -tangent_speed / tangent_inv_mass;
        const float gravity_support = state.body.mass * std::abs(gravity) * dt;
        const float friction_limit =
            kBodyFriction * std::max(normal_impulse, gravity_support);
        const float friction_impulse =
            clamp(desired_impulse, -friction_limit, friction_limit);
        apply_body_impulse(state.body, tangent * friction_impulse,
                           state.body.position + r);
      }
    }
  }

  const bool grounded_now = any_wheel_grounded || body_ground_contact;
  if (!grounded_now) {

    state.airborne = state.has_grounded;
    state.airborne_steps = state.airborne
                               ? (was_airborne ? previous_airborne_steps + 1 : 1)
                               : 0;
  } else {
    state.has_grounded = true;
    state.airborne = false;
    state.airborne_steps = 0;
    if (was_airborne && previous_airborne_steps >= config_.ballistic_min_air_steps) {
      state.landing_event = true;
      state.last_impact_speed = std::max(0.0f, -pre_contact_vertical_speed);
      state.last_landing_angle = std::abs(state.body.angle);
      const bool bad_speed = state.last_impact_speed > config_.safe_landing_speed;
      const bool bad_angle = state.last_landing_angle > config_.safe_landing_angle;
      if (bad_speed || bad_angle || body_ground_contact) {
        state.landing_fatal = true;
        state.fatal_error = true;
      }
    }
  }
  const Vec2 world_acceleration =
      (state.body.velocity - previous_body_velocity) / std::max(0.0001f, dt);
  state.imu_acceleration = rotate(world_acceleration, -state.body.angle);
  state.imu_angular_acceleration =
      (state.body.angular_velocity - previous_body_angular_velocity) /
      std::max(0.0001f, dt);
  state.imu_impact = std::max(stats.hard_contact * dt, body_contact_impulse) /
                     std::max(0.1f, state.body.mass);
  stats.energy_cost += lidar_energy_cost;
  state.passive_charge_rate = 0.0f;
  state.drive_fuel_rate = stats.drive_energy_cost / std::max(0.0001f, dt);
  state.energy_cost_rate = stats.energy_cost / std::max(0.0001f, dt);
  state.energy_gain_rate = stats.energy_gain / std::max(0.0001f, dt);
  state.energy = clamp(state.energy - stats.energy_cost + stats.energy_gain,
                       0.0f, config_.energy_capacity);
  return stats;
