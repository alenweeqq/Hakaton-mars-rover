#pragma once

namespace generated_biomes {

class GravityShelfLug final : public Biome {
 public:
  std::string_view id() const noexcept override { return "gravity_shelf_lug"; }
  std::string_view display_name() const noexcept override { return "Gravity Shelf Lug"; }
  std::string_view skill_stratum() const noexcept override { return "gravity_change"; }
  BiomeSplit split() const noexcept override { return BiomeSplit::Train; }
  MechanicType visual_type() const noexcept override { return MechanicType::LowGravity; }

  int hazard_at(int step) const noexcept override { return shelving(step) ? 1 : 0; }

  bool shelving(int step) const noexcept {
    int phase = (step + kPhase) % kCycle;
    if (phase < 0) phase += kCycle;
    return phase < kShelfSteps;
  }

  MechanicParams sample_params(uint64_t s) const noexcept override {
    MechanicParams p;

    p.friction_mul = 0.95f + 0.10f * biome_random01(s);
    p.sink_rate = 0.001f + 0.002f * biome_random01(s, 1);
    p.viscosity = 0.0f;
    p.energy_drain_mul = 1.10f + 0.15f * biome_random01(s, 2);
    p.wind_force = 0.5f + 0.5f * biome_random01(s, 3);

    p.ambient_temperature = -46.0f + 15.0f * biome_random01(s, 4);
    p.thermal_transfer = 0.50f + 0.15f * biome_random01(s, 5);
    p.solar_charge_rate = 0.04f + 0.03f * biome_random01(s, 6);
    p.gravity_mul = 0.85f + 0.10f * biome_random01(s, 7);
    p.crust_deform = 0.003f + 0.004f * biome_random01(s, 8);

    p.lidar_energy_mul = 6.0f + 1.0f * biome_random01(s, 9);
    p.lidar_range_mul = 0.08f + 0.03f * biome_random01(s, 10);

    p.terrain_amplitude_mul = 0.65f + 0.15f * biome_random01(s, 11);
    p.terrain_roughness_mul = 0.40f + 0.10f * biome_random01(s, 12);
    p.terrain_crater_mul = 0.80f + 0.20f * biome_random01(s, 13);
    p.terrain_step_mul = 0.40f + 0.10f * biome_random01(s, 14);
    return p;
  }

  float friction_scale(const MechanicParams& p) const noexcept override {

    return p.friction_mul * 1.05f;
  }

  void apply_effects(const MechanicParams& p, MechanicContext& c) const noexcept override {
    if (c.wheel_force && c.contact) {

      *c.wheel_force += c.contact->tangent * (-0.10f * c.wheel_speed);
      *c.wheel_force -= c.contact->normal * (c.contact->normal_force * 0.006f);
    }
    if (c.energy_cost) {

      *c.energy_cost += (0.004f + 0.002f * std::abs(c.wheel_speed)) * p.energy_drain_mul * c.dt;
    }
  }

  void apply_body_effects(const MechanicParams& p, MechanicBodyContext& c) const noexcept override {
    const float t = static_cast<float>(c.step_index);
    const float speed = std::abs(c.velocity.x);
    const float speed_norm = std::tanh(speed * 0.10f);
    const bool shelf = shelving(c.step_index);

    if (c.body_force) {
      if (shelf) {

        const float heavy = 3.8f * c.mass * c.gravity;
        c.body_force->y += heavy;

        if (speed > kShelfCreep) {
          const float over = clamp((speed - kShelfCreep) * 1.5f, 0.0f, 5.0f);
          const float drag = (2.5f + 2.0f * over) * c.mass * (c.velocity.x >= 0.0f ? 1.0f : -1.0f);
          c.body_force->x -= drag;

          const float lateral = speed_norm * 0.6f * c.mass * std::sin(t * 0.013f + 0.6f);
          c.body_force->x += lateral;

          c.body_force->x -= c.velocity.x * c.mass * 2.0f;
        } else {

          c.body_force->x -= c.velocity.x * c.mass * 0.8f;
        }

        c.body_force->y -= c.velocity.y * c.mass * 1.2f;
      } else {

        const float assist = 0.018f * c.mass * c.gravity;
        const float damping = 0.03f + 0.02f * speed_norm;
        c.body_force->x += assist - c.velocity.x * c.mass * damping;
        c.body_force->y -= c.velocity.y * c.mass * 0.03f;
      }

      if (c.body_torque) {
        if (shelf) {

          const float pitch = (0.05f + 0.35f * speed_norm) * c.mass;
          *c.body_torque += pitch * (c.velocity.x >= 0.0f ? 1.0f : -1.0f);

          *c.body_torque -= c.angular_velocity * c.mass * 0.9f;
        } else {

          *c.body_torque -= c.angular_velocity * c.mass * 0.03f;
        }
      }
    }

    if (c.energy_cost) {
      if (shelf) {
        if (speed <= kShelfCreep) {

          *c.energy_cost += 0.012f * p.energy_drain_mul * c.dt;
        } else {

          const float over = clamp((speed - kShelfCreep) * 1.5f, 0.0f, 5.0f);
          *c.energy_cost += (0.30f + 0.90f * over * over) * p.energy_drain_mul * c.dt;
        }
      } else {

        *c.energy_cost += (0.005f + 0.003f * speed_norm) * p.energy_drain_mul * c.dt;
      }
    }
  }

  BiomeVisuals visuals() const noexcept override {
    BiomeVisuals v;

    v.ground = {104, 112, 124};
    v.particles = {178, 186, 198};
    v.liquid = {48, 56, 66};
    v.sky = {64, 74, 88};
    v.particle_rate = 4.0f;
    v.particle_lift = 1.1f;
    v.particle_spread = 0.6f;
    v.base_particles = 1;
    v.max_particles = 14;
    v.particle_size = 1;
    v.ambient_particles = 6;
    v.ambient_drift = 0.8f;
    v.screen_brightness = 0.06f;
    v.liquid_surface = false;
    return v;
  }

 private:
  static constexpr int kCycle = 720;
  static constexpr int kShelfSteps = 150;
  static constexpr int kPhase = 170;
  static constexpr float kShelfCreep = 0.30f;
};

class ThermalSurgeRelay final : public Biome {
 public:
  std::string_view id() const noexcept override { return "thermal_surge_relay"; }
  std::string_view display_name() const noexcept override { return "Thermal Surge Relay"; }
  std::string_view skill_stratum() const noexcept override { return "energy_mode"; }
  MechanicType visual_type() const noexcept override { return MechanicType::Liquid; }
  BiomeSplit split() const noexcept override { return BiomeSplit::Train; }

  int hazard_at(int step) const noexcept override { return hot_surge(step) ? 1 : 0; }

  bool hot_surge(int step) const noexcept {
    int phase = (step + kPhase) % kCycle;
    if (phase < 0) phase += kCycle;
    return phase < kHotSteps;
  }

  MechanicParams sample_params(uint64_t s) const noexcept override {
    MechanicParams p;

    p.friction_mul = 0.55f + 0.20f * biome_random01(s);
    p.sink_rate = 0.006f + 0.010f * biome_random01(s, 1);
    p.viscosity = 0.60f + 0.80f * biome_random01(s, 2);
    p.energy_drain_mul = 1.25f + 0.25f * biome_random01(s, 3);
    p.wind_force = 0.2f + 0.5f * biome_random01(s, 4);

    p.ambient_temperature = 58.0f + 12.0f * biome_random01(s, 5);
    p.thermal_transfer = 3.4f + 0.6f * biome_random01(s, 6);
    p.solar_charge_rate = 0.12f + 0.06f * biome_random01(s, 7);
    p.gravity_mul = 0.90f + 0.10f * biome_random01(s, 8);
    p.crust_deform = 0.004f + 0.006f * biome_random01(s, 9);

    p.lidar_energy_mul = 5.8f + 1.0f * biome_random01(s, 10);
    p.lidar_range_mul = 0.10f + 0.04f * biome_random01(s, 11);

    p.terrain_amplitude_mul = 0.60f + 0.15f * biome_random01(s, 12);
    p.terrain_roughness_mul = 0.35f + 0.10f * biome_random01(s, 13);
    p.terrain_crater_mul = 0.40f + 0.10f * biome_random01(s, 14);
    p.terrain_step_mul = 0.20f + 0.10f * biome_random01(s, 15);
    return p;
  }

  float friction_scale(const MechanicParams& p) const noexcept override {

    return p.friction_mul * 0.72f;
  }

  void apply_effects(const MechanicParams& p, MechanicContext& c) const noexcept override {
    if (c.wheel_force && c.contact) {
      float depth = c.contact->penetration * 20.0f;
      float drag = (0.20f + p.viscosity * 1.4f * (1.0f + depth) * c.immersion) * c.wheel_speed;
      *c.wheel_force += c.contact->tangent * (-drag);
      *c.wheel_force -= c.contact->normal * (c.contact->normal_force * (0.02f + 0.08f * depth * c.immersion));
    }
    if (c.energy_cost) {
      *c.energy_cost += (0.008f + std::abs(c.wheel_speed) * 0.004f) * p.energy_drain_mul * c.dt;
    }
  }

  void apply_body_effects(const MechanicParams& p, MechanicBodyContext& c) const noexcept override {
    const float t = static_cast<float>(c.step_index);
    const float speed = std::abs(c.velocity.x);
    const float speed_norm = std::tanh(speed * 0.10f);
    const bool hot = hot_surge(c.step_index);

    if (c.body_force) {
      if (hot) {

        const float churn = 0.12f + 0.25f * speed_norm;
        c.body_force->x -= c.velocity.x * c.mass * (0.5f + 1.2f * speed_norm);
        c.body_force->y += churn * c.mass * c.gravity * std::sin(t * 0.017f);

        c.body_force->x += speed_norm * 0.22f * c.mass * c.gravity * std::sin(t * 0.013f + 0.7f);

        c.body_force->x -= c.velocity.x * c.mass * 1.4f;
        c.body_force->y -= c.velocity.y * c.mass * 0.9f;
      } else {

        const float assist = 0.014f * c.mass * c.gravity;
        const float damping = 0.03f + 0.02f * speed_norm;
        c.body_force->x += assist - c.velocity.x * c.mass * damping;
        c.body_force->y -= c.velocity.y * c.mass * 0.03f;
      }

      if (c.body_torque) {
        if (hot) {

          const float pitch = (0.05f + 0.35f * speed_norm) * c.mass * std::sin(t * 0.015f + 0.4f);
          *c.body_torque += pitch;

          *c.body_torque -= c.angular_velocity * c.mass * 1.2f;
        } else {
          *c.body_torque -= c.angular_velocity * c.mass * 0.03f;
        }
      }
    }

    if (c.energy_cost) {
      if (hot) {
        if (speed <= 0.14f) {

          *c.energy_cost -= 2.4f * c.dt;

          *c.energy_cost += 0.04f * c.dt;
        } else {

          *c.energy_cost += (0.35f + 0.85f * speed_norm * speed_norm) * p.energy_drain_mul * c.dt;
        }
      } else {

        *c.energy_cost += (0.006f + 0.004f * speed_norm) * p.energy_drain_mul * c.dt;
      }
    }
  }

  BiomeVisuals visuals() const noexcept override {
    BiomeVisuals v;

    v.ground = {20, 28, 34};
    v.particles = {120, 160, 170};
    v.liquid = {14, 40, 50};
    v.sky = {32, 42, 52};
    v.particle_rate = 4.0f;
    v.particle_lift = 1.6f;
    v.particle_spread = 0.8f;
    v.base_particles = 2;
    v.max_particles = 18;
    v.particle_size = 2;
    v.ambient_particles = 8;
    v.ambient_drift = 1.3f;
    v.screen_brightness = 0.07f;
    v.liquid_surface = true;
    return v;
  }

 private:
  static constexpr int kCycle = 700;
  static constexpr int kHotSteps = 180;
  static constexpr int kPhase = 220;
};

class RimeQuarryDawn final : public Biome {
 public:
  std::string_view id() const noexcept override { return "rime_quarry_dawn"; }
  std::string_view display_name() const noexcept override { return "Rime Quarry Dawn"; }
  std::string_view skill_stratum() const noexcept override { return "dynamic_obstacle"; }
  MechanicType visual_type() const noexcept override { return MechanicType::LowGravity; }
  BiomeSplit split() const noexcept override { return BiomeSplit::Train; }

  bool rime_phase(int step) const noexcept {
    int phase = (step + kPhase) % kCycle;
    if (phase < 0) phase += kCycle;
    return phase < kRimeSteps;
  }

  int hazard_at(int step) const noexcept override { return rime_phase(step) ? 2 : 1; }

  MechanicParams sample_params(uint64_t s) const noexcept override {
    MechanicParams p;

    p.friction_mul = 0.85f + 0.15f * biome_random01(s);
    p.sink_rate = 0.002f + 0.003f * biome_random01(s, 1);
    p.viscosity = 0.0f;
    p.energy_drain_mul = 1.20f + 0.20f * biome_random01(s, 2);
    p.wind_force = 0.5f + 0.5f * biome_random01(s, 3);

    p.ambient_temperature = -58.0f + 15.0f * biome_random01(s, 4);
    p.thermal_transfer = 1.4f + 0.3f * biome_random01(s, 5);
    p.solar_charge_rate = 0.04f + 0.03f * biome_random01(s, 6);
    p.gravity_mul = 0.55f + 0.15f * biome_random01(s, 7);
    p.crust_deform = 0.004f + 0.006f * biome_random01(s, 8);

    p.lidar_energy_mul = 5.5f + 1.0f * biome_random01(s, 9);
    p.lidar_range_mul = 0.10f + 0.04f * biome_random01(s, 10);

    p.terrain_amplitude_mul = 0.60f + 0.15f * biome_random01(s, 11);
    p.terrain_roughness_mul = 0.55f + 0.15f * biome_random01(s, 12);
    p.terrain_crater_mul = 0.90f + 0.20f * biome_random01(s, 13);
    p.terrain_step_mul = 0.70f + 0.20f * biome_random01(s, 14);
    return p;
  }

  float friction_scale(const MechanicParams& p) const noexcept override {

    return p.friction_mul * 1.20f;
  }

  void apply_effects(const MechanicParams& p, MechanicContext& c) const noexcept override {
    if (c.wheel_force && c.contact) {

      *c.wheel_force += c.contact->tangent * (-0.10f * c.wheel_speed);
      *c.wheel_force -= c.contact->normal * (c.contact->normal_force * 0.006f);
    }
    if (c.energy_cost) {

      *c.energy_cost += (0.004f + 0.002f * std::abs(c.wheel_speed)) * p.energy_drain_mul * c.dt;
    }
  }

  void apply_body_effects(const MechanicParams& p, MechanicBodyContext& c) const noexcept override {
    const float t = static_cast<float>(c.step_index);
    const float speed = std::abs(c.velocity.x);
    const float speed_norm = std::tanh(speed * 0.10f);
    const bool rime = rime_phase(c.step_index);

    if (c.body_force) {
      if (rime) {

        const float release = 1.10f;
        if (speed < release) {
          const float bite = clamp((release - speed) / release, 0.0f, 1.0f);
          const float bite_force = (0.6f + 2.8f * bite) * c.mass *
                                   (c.velocity.x >= 0.0f ? 1.0f : -1.0f);
          c.body_force->x -= bite_force;

          c.body_force->x += (0.2f + 0.5f * bite) * c.mass * std::sin(t * 0.017f + 0.6f);

          c.body_force->x -= c.velocity.x * c.mass * (0.8f + 1.5f * bite);
        } else {

          c.body_force->x += 0.025f * c.mass * c.gravity;
          c.body_force->x -= c.velocity.x * c.mass * 0.03f;
        }

        c.body_force->y -= c.velocity.y * c.mass * 0.9f;
      } else {

        const float thaw_damping = 0.04f + 0.02f * speed_norm;
        c.body_force->x -= c.velocity.x * c.mass * thaw_damping;
        if (speed <= 0.2f) {

          c.body_force->x += 0.012f * c.mass * c.gravity;
        }
        c.body_force->y -= c.velocity.y * c.mass * 0.03f;
      }

      if (c.body_torque) {
        if (rime) {
          if (speed < 1.10f) {

            const float bite = clamp((1.10f - speed) / 1.10f, 0.0f, 1.0f);
            const float pitch = (0.02f + 0.30f * bite) * c.mass;
            *c.body_torque -= pitch * (c.velocity.x >= 0.0f ? 1.0f : -1.0f);
            *c.body_torque -= c.angular_velocity * c.mass * 0.9f;
          } else {
            *c.body_torque -= c.angular_velocity * c.mass * 0.03f;
          }
        } else {
          *c.body_torque -= c.angular_velocity * c.mass * 0.03f;
        }
      }
    }

    if (c.energy_cost) {
      if (rime) {
        if (speed < 1.10f) {

          const float bite = clamp((1.10f - speed) / 1.10f, 0.0f, 1.0f);
          *c.energy_cost += (0.15f + 0.75f * bite * bite) * p.energy_drain_mul * c.dt;
        } else {

          *c.energy_cost += (0.005f + 0.004f * speed_norm) * p.energy_drain_mul * c.dt;
        }
      } else {
        if (speed <= 0.2f) {

          *c.energy_cost -= 0.8f * c.dt;
        } else {

          *c.energy_cost += (0.006f + 0.004f * speed_norm) * p.energy_drain_mul * c.dt;
        }
      }
    }
  }

  BiomeVisuals visuals() const noexcept override {
    BiomeVisuals v;

    v.ground = {102, 110, 122};
    v.particles = {196, 204, 214};
    v.liquid = {44, 52, 62};
    v.sky = {56, 66, 80};
    v.particle_rate = 4.0f;
    v.particle_lift = 0.7f;
    v.particle_spread = 0.5f;
    v.base_particles = 1;
    v.max_particles = 14;
    v.particle_size = 1;
    v.ambient_particles = 6;
    v.ambient_drift = 0.7f;
    v.screen_brightness = 0.06f;
    v.liquid_surface = false;
    return v;
  }

 private:
  static constexpr int kCycle = 760;
  static constexpr int kRimeSteps = 210;
  static constexpr int kPhase = 240;
};

}
