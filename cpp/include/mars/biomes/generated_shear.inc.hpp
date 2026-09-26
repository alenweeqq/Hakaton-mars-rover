#pragma once

namespace generated_biomes {

class GravityShearEscarpment final : public Biome {
 public:
  std::string_view id() const noexcept override { return "gravity_shear_escarpment"; }
  std::string_view display_name() const noexcept override { return "Gravity Shear Escarpment"; }
  std::string_view skill_stratum() const noexcept override { return "gravity_change"; }
  BiomeSplit split() const noexcept override { return BiomeSplit::Train; }
  MechanicType visual_type() const noexcept override { return MechanicType::LowGravity; }

  int hazard_at(int step) const noexcept override { return shear_phase(step) ? 1 : 0; }

  bool shear_phase(int step) const noexcept {
    int phase = (step + kPhase) % kCycle;
    return phase < 0 ? phase + kCycle : phase < kShearSteps;
  }

  MechanicParams sample_params(uint64_t s) const noexcept override {
    MechanicParams p;

    p.friction_mul = 0.85f + 0.15f * biome_random01(s);
    p.sink_rate = 0.002f + 0.003f * biome_random01(s, 1);
    p.viscosity = 0.0f;
    p.energy_drain_mul = 1.15f + 0.20f * biome_random01(s, 2);
    p.wind_force = 0.8f + 0.6f * biome_random01(s, 3);

    p.ambient_temperature = -56.0f + 15.0f * biome_random01(s, 4);
    p.thermal_transfer = 0.55f + 0.15f * biome_random01(s, 5);
    p.solar_charge_rate = 0.05f + 0.03f * biome_random01(s, 6);
    p.gravity_mul = 0.60f + 0.15f * biome_random01(s, 7);
    p.crust_deform = 0.003f + 0.004f * biome_random01(s, 8);

    p.lidar_energy_mul = 6.0f + 1.0f * biome_random01(s, 9);
    p.lidar_range_mul = 0.0f;

    p.terrain_amplitude_mul = 1.50f + 0.30f * biome_random01(s, 11);
    p.terrain_roughness_mul = 0.80f + 0.20f * biome_random01(s, 12);
    p.terrain_crater_mul = 0.60f + 0.20f * biome_random01(s, 13);
    p.terrain_step_mul = 1.80f + 0.40f * biome_random01(s, 14);
    return p;
  }

  float friction_scale(const MechanicParams& p) const noexcept override {

    return p.friction_mul * 1.10f;
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
    const float speed_norm = std::tanh(speed * 0.09f);
    const bool shear = shear_phase(c.step_index);

    if (c.body_force) {
      if (shear) {

        const float creep_limit = 0.60f;
        if (speed > creep_limit) {
          const float over = clamp((speed - creep_limit) * 1.8f, 0.0f, 5.0f);

          const float shove = (2.2f + 2.8f * over) * c.mass * (c.velocity.x >= 0.0f ? 1.0f : -1.0f);
          c.body_force->x -= shove;

          c.body_force->x += (0.3f + 0.7f * over) * c.mass * std::sin(t * 0.013f + 0.6f);

          c.body_force->x -= c.velocity.x * c.mass * (0.8f + 1.2f * over);

          c.body_force->y += 0.15f * over * c.mass * c.gravity;
        } else {

          c.body_force->x += 0.020f * c.mass * c.gravity;
          c.body_force->x -= c.velocity.x * c.mass * 0.04f;
        }

        c.body_force->y -= c.velocity.y * c.mass * 0.9f;
      } else {

        const float assist = 0.030f * c.mass * c.gravity;
        const float damping = 0.025f + 0.02f * speed_norm;
        c.body_force->x += assist - c.velocity.x * c.mass * damping;
        c.body_force->y -= c.velocity.y * c.mass * 0.03f;
      }

      if (c.body_torque) {
        if (shear) {
          if (speed > 0.60f) {

            const float over = clamp((speed - 0.60f) * 1.8f, 0.0f, 5.0f);
            const float pitch = (0.04f + 0.28f * over) * c.mass;
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
      if (shear) {
        if (speed > 0.60f) {

          const float over = clamp((speed - 0.60f) * 1.8f, 0.0f, 5.0f);
          *c.energy_cost += (0.20f + 0.70f * over * over) * p.energy_drain_mul * c.dt;
        } else {

          *c.energy_cost += (0.005f + 0.003f * speed_norm) * p.energy_drain_mul * c.dt;
        }
      } else {

        *c.energy_cost += (0.004f + 0.003f * speed_norm) * p.energy_drain_mul * c.dt;
      }
    }
  }

  BiomeVisuals visuals() const noexcept override {
    BiomeVisuals v;

    v.ground = {118, 122, 128};
    v.particles = {188, 192, 200};
    v.liquid = {46, 52, 62};
    v.sky = {72, 78, 90};
    v.particle_rate = 4.0f;
    v.particle_lift = 1.0f;
    v.particle_spread = 0.7f;
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
  static constexpr int kCycle = 800;
  static constexpr int kShearSteps = 220;
  static constexpr int kPhase = 190;
};

}
