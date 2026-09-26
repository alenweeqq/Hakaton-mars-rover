#pragma once

namespace generated_biomes {

inline constexpr int kGeneratedBiomeBankStart = 0;

class LateralShearBelt final : public Biome {
 public:
  std::string_view id() const noexcept override { return "lateral_shear_belt"; }
  std::string_view display_name() const noexcept override { return "Lateral Shear Belt"; }
  std::string_view skill_stratum() const noexcept override { return "lateral_force"; }
  BiomeSplit split() const noexcept override { return BiomeSplit::Train; }
  MechanicType visual_type() const noexcept override { return MechanicType::Wind; }

  int hazard_at(int) const noexcept override { return 2; }

  MechanicParams sample_params(uint64_t s) const noexcept override {
    MechanicParams p;

    p.friction_mul = 0.78f + 0.15f * biome_random01(s);
    p.sink_rate = 0.002f + 0.003f * biome_random01(s, 1);
    p.viscosity = 0.0f;
    p.energy_drain_mul = 1.35f + 0.25f * biome_random01(s, 2);
    p.wind_force = 4.0f + 2.0f * biome_random01(s, 3);

    p.ambient_temperature = -31.0f + 15.0f * biome_random01(s, 4);
    p.thermal_transfer = 0.60f + 0.20f * biome_random01(s, 5);
    p.solar_charge_rate = 0.10f + 0.06f * biome_random01(s, 6);
    p.gravity_mul = 1.05f + 0.08f * biome_random01(s, 7);
    p.crust_deform = 0.004f + 0.006f * biome_random01(s, 8);

    p.lidar_energy_mul = 6.0f + 1.0f * biome_random01(s, 9);
    p.lidar_range_mul = 0.08f + 0.03f * biome_random01(s, 10);

    p.terrain_amplitude_mul = 0.80f + 0.15f * biome_random01(s, 11);
    p.terrain_roughness_mul = 0.40f + 0.10f * biome_random01(s, 12);
    p.terrain_crater_mul = 0.50f + 0.15f * biome_random01(s, 13);
    p.terrain_step_mul = 0.25f + 0.10f * biome_random01(s, 14);
    return p;
  }

  float friction_scale(const MechanicParams& p) const noexcept override {

    return p.friction_mul * 1.15f;
  }

  void apply_effects(const MechanicParams& p, MechanicContext& c) const noexcept override {
    if (c.wheel_force && c.contact) {

      *c.wheel_force += c.contact->tangent * (-0.12f * c.wheel_speed);
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

    const float shear_phase = t * 0.0044f + c.velocity.x * 0.011f;
    const float cross = 0.5f + 0.5f * std::sin(shear_phase);
    const float straight = 1.0f - cross;

    const float pulse_phase = t * 0.031f + c.velocity.x * 0.073f;
    const float pulse = 0.5f + 0.5f * std::sin(pulse_phase);
    const float pulse_narrow = pulse * pulse;

    if (c.body_force) {

      const float cross_lateral = cross * std::sin(shear_phase + 1.2f) *
          (0.08f + 0.30f * speed_norm * speed_norm) * c.mass * c.gravity * 0.35f;

      const float pulse_lateral = cross * pulse_narrow *
          (0.05f + 0.18f * speed_norm) * c.mass * c.gravity * std::sin(pulse_phase + 0.9f);

      const float straight_assist = straight * 0.020f * c.mass * c.gravity;

      const float precession = std::sin(shear_phase * 0.5f + 0.6f) *
          (0.03f + 0.10f * speed_norm) * c.mass * c.gravity;

      const float damping = 0.04f + 0.02f * cross + 0.02f * speed_norm;

      c.body_force->x += straight_assist + cross_lateral + pulse_lateral + precession;
      c.body_force->x -= c.velocity.x * c.mass * damping;
      c.body_force->y -= c.velocity.y * c.mass * (0.04f + 0.02f * cross);

      if (c.body_torque) {

        const float yaw_cross = cross * std::cos(shear_phase + 1.5f) *
            (0.05f + 0.18f * speed_norm) * c.mass * c.gravity * 0.22f;

        const float pulse_torque = cross * pulse_narrow *
            (0.03f + 0.10f * speed_norm) * c.mass * c.gravity * std::sin(pulse_phase + 0.6f);

        const float pitch_damping = 0.025f + 0.02f * straight + 0.01f * speed_norm;
        *c.body_torque += yaw_cross + pulse_torque;
        *c.body_torque -= c.angular_velocity * c.mass * pitch_damping;
      }
    }

    if (c.energy_cost) {

      const bool in_cross = cross > 0.5f;
      if (in_cross) {
        if (speed > 0.25f) {

          *c.energy_cost += (0.014f + 0.030f * speed_norm * speed_norm) * p.energy_drain_mul * c.dt;
        } else {

          *c.energy_cost += 0.018f * p.energy_drain_mul * c.dt;
        }
      } else {

        *c.energy_cost += (0.005f + 0.003f * speed_norm) * p.energy_drain_mul * c.dt;
      }
    }
  }

  BiomeVisuals visuals() const noexcept override {
    BiomeVisuals v;

    v.ground = {28, 30, 36};
    v.particles = {96, 106, 124};
    v.liquid = {16, 18, 24};
    v.sky = {48, 54, 68};
    v.particle_rate = 3.0f;
    v.particle_lift = 0.7f;
    v.particle_spread = 0.5f;
    v.base_particles = 1;
    v.max_particles = 12;
    v.particle_size = 1;
    v.ambient_particles = 4;
    v.ambient_drift = 0.8f;
    v.screen_brightness = 0.06f;
    v.liquid_surface = false;
    return v;
  }
};

class HysteresisSurgeBog final : public Biome {
 public:
  std::string_view id() const noexcept override { return "hysteresis_surge_bog"; }
  std::string_view display_name() const noexcept override { return "Hysteresis Surge Bog"; }
  std::string_view skill_stratum() const noexcept override { return "inertia_hysteresis"; }
  MechanicType visual_type() const noexcept override { return MechanicType::Mud; }
  BiomeSplit split() const noexcept override { return BiomeSplit::Train; }

  int hazard_at(int) const noexcept override { return 2; }

  MechanicParams sample_params(uint64_t s) const noexcept override {
    MechanicParams p;

    p.friction_mul = 0.42f + 0.18f * biome_random01(s);
    p.sink_rate = 0.014f + 0.020f * biome_random01(s, 1);
    p.viscosity = 1.4f + 1.6f * biome_random01(s, 2);
    p.energy_drain_mul = 1.60f + 0.45f * biome_random01(s, 3);
    p.wind_force = 0.0f;

    p.ambient_temperature = -1.0f + 15.0f * biome_random01(s, 4);
    p.thermal_transfer = 2.2f + 0.5f * biome_random01(s, 5);
    p.solar_charge_rate = 0.05f + 0.04f * biome_random01(s, 6);
    p.gravity_mul = 0.95f + 0.10f * biome_random01(s, 7);
    p.crust_deform = 0.012f + 0.016f * biome_random01(s, 8);

    p.lidar_energy_mul = 5.8f + 1.2f * biome_random01(s, 9);
    p.lidar_range_mul = 0.09f + 0.04f * biome_random01(s, 10);

    p.terrain_amplitude_mul = 1.35f + 0.30f * biome_random01(s, 11);
    p.terrain_roughness_mul = 0.60f + 0.20f * biome_random01(s, 12);
    p.terrain_crater_mul = 0.40f + 0.15f * biome_random01(s, 13);
    p.terrain_step_mul = 0.25f + 0.10f * biome_random01(s, 14);
    return p;
  }

  float friction_scale(const MechanicParams& p) const noexcept override {

    return p.friction_mul * 0.55f;
  }

  void apply_effects(const MechanicParams& p, MechanicContext& c) const noexcept override {
    if (c.contact) {
      float speed = std::abs(c.wheel_speed);

      c.contact->penetration += p.sink_rate * c.dt * (1.0f + 1.8f * std::tanh(speed * 0.20f));
    }
    if (c.wheel_force && c.contact) {
      float depth = c.contact->penetration * 28.0f;

      float drag = (0.28f + p.viscosity * 1.6f * (1.0f + depth) + 0.10f * depth) * c.wheel_speed;
      *c.wheel_force += c.contact->tangent * (-drag);
      *c.wheel_force -= c.contact->normal * (c.contact->normal_force * (0.03f + 0.09f * depth));
    }
    if (c.energy_cost) {
      float depth = c.contact->penetration * 28.0f;

      *c.energy_cost += (0.015f + depth * 0.20f + std::abs(c.wheel_speed) * 0.005f) * p.energy_drain_mul * c.dt;
    }
  }

  void apply_body_effects(const MechanicParams& p, MechanicBodyContext& c) const noexcept override {
    const float t = static_cast<float>(c.step_index);
    const float speed = std::abs(c.velocity.x);
    const float speed_norm = std::tanh(speed * 0.10f);

    const float surge_phase = t * 0.0045f + c.velocity.x * 0.012f;
    const float surge = 0.5f + 0.5f * std::sin(surge_phase);
    const float release = 1.0f - surge;

    const float pulse_phase = t * 0.035f + c.velocity.x * 0.085f;
    const float pulse = 0.5f + 0.5f * std::sin(pulse_phase);
    const float pulse_narrow = pulse * pulse;

    const float memory = 0.5f + 0.5f * std::tanh((speed_norm - 0.25f) * 5.0f);
    const float hysteresis = surge * memory * (0.5f + 0.5f * pulse_narrow);

    if (c.body_force) {

      const float suck_drag = surge * (0.08f + 0.16f * memory) * c.mass * speed_norm *
          (c.velocity.x >= 0.0f ? 1.0f : -1.0f);

      const float release_assist = release * (0.02f + 0.04f * memory) * c.mass * c.gravity;

      const float pulse_drag = hysteresis * (0.10f + 0.14f * speed_norm) * c.mass *
          (c.velocity.x >= 0.0f ? 1.0f : -1.0f);

      const float lateral = std::sin(t * 0.021f + c.velocity.x * 0.049f) *
          (0.04f + 0.12f * std::abs(surge - 0.5f) * 2.0f + 0.08f * speed_norm) * c.mass * c.gravity;

      const float damping = 0.05f + 0.04f * surge + 0.02f * speed_norm;

      c.body_force->x += release_assist + lateral - suck_drag - pulse_drag;
      c.body_force->x -= c.velocity.x * c.mass * damping;
      c.body_force->y -= c.velocity.y * c.mass * (0.04f + 0.02f * surge);

      if (c.body_torque) {

        const float suck_torque = surge * (0.2f + 0.4f * speed_norm) * c.mass;
        const float release_torque = release * (0.1f + 0.3f * speed_norm) * c.mass;
        const float pulse_torque = hysteresis * (0.3f + 0.35f * speed_norm) * c.mass *
            std::sin(pulse_phase + 0.8f);
        *c.body_torque -= suck_torque;
        *c.body_torque += release_torque + pulse_torque;
        *c.body_torque -= c.angular_velocity * c.mass * (0.03f + 0.02f * surge);
      }
    }

    if (c.energy_cost) {

      const float suck_cost = surge * (0.012f + 0.028f * memory) * p.energy_drain_mul;
      const float pulse_penalty = hysteresis * (0.014f + 0.020f * speed_norm) * p.energy_drain_mul;
      const float release_cost = release * (0.006f + 0.004f * speed_norm) * p.energy_drain_mul;
      const float low_speed_cost = std::max(0.0f, 0.25f - speed_norm) * 0.012f * p.energy_drain_mul;

      const float base_cost = 0.008f;

      *c.energy_cost += (base_cost + suck_cost + pulse_penalty + release_cost + low_speed_cost) * c.dt;
    }
  }

  BiomeVisuals visuals() const noexcept override {
    BiomeVisuals v;

    v.ground = {34, 36, 28};
    v.particles = {120, 124, 88};
    v.liquid = {18, 20, 14};
    v.sky = {42, 44, 34};
    v.particle_rate = 8.0f;
    v.particle_lift = 0.7f;
    v.particle_spread = 1.0f;
    v.base_particles = 2;
    v.max_particles = 22;
    v.particle_size = 2;
    v.ambient_particles = 8;
    v.ambient_drift = 1.2f;
    v.screen_brightness = 0.07f;
    v.liquid_surface = false;
    return v;
  }
};

}
