#pragma once

namespace handcrafted_biomes {

class MoltenWindowBiome : public Biome {
 public:
  MechanicType visual_type() const noexcept override { return MechanicType::Crust; }
  BiomeSplit split() const noexcept override { return BiomeSplit::Train; }

  virtual int cycle_steps() const noexcept = 0;
  virtual int melt_steps() const noexcept = 0;
  virtual int phase_offset() const noexcept { return 0; }

  int hazard_at(int step) const noexcept override { return molten(step) ? 1 : 0; }

  bool molten(int step) const noexcept {
    const int cycle = cycle_steps();
    int phase = (step + phase_offset()) % cycle;
    if (phase < 0) phase += cycle;
    return phase < melt_steps();
  }

  MechanicParams sample_params(uint64_t s) const noexcept override {
    MechanicParams p;
    p.friction_mul = 0.95f + 0.10f * biome_random01(s);
    p.crust_deform = 0.010f + 0.006f * biome_random01(s, 1);
    p.ambient_temperature = 29.0f + 12.0f * biome_random01(s, 2);
    p.thermal_transfer = 1.1f + 0.3f * biome_random01(s, 3);
    p.solar_charge_rate = 1.4f + 0.4f * biome_random01(s, 4);

    p.lidar_range_mul = 0.35f;
    p.lidar_energy_mul = 2.0f;
    p.terrain_amplitude_mul = 0.8f;
    p.terrain_step_mul = 0.6f;
    return p;
  }

  void apply_effects(const MechanicParams& p, MechanicContext& c) const noexcept override {
    if (!c.contact || !c.contact->active) return;
    if (!molten(c.step_index)) return;
    const float speed = std::abs(c.wheel_speed);
    const float excess = speed - kCreepSpeed;
    if (excess <= 0.0f) return;

    const float severity = clamp(excess * 1.6f, 0.0f, 6.0f);
    c.contact->penetration += (0.30f + 0.55f * severity) * c.dt;
    if (c.wheel_force) {
      *c.wheel_force += c.contact->tangent * (-9.0f * c.wheel_speed * (1.0f + severity));
      *c.wheel_force -= c.contact->normal * (c.contact->normal_force * (0.9f + 0.5f * severity));
    }
    if (c.energy_cost)
      *c.energy_cost += (0.10f + 0.16f * severity) * p.energy_drain_mul * c.dt;
  }

  void apply_body_effects(const MechanicParams&, MechanicBodyContext& c) const noexcept override {
    if (!molten(c.step_index)) return;
    const float speed = std::abs(c.velocity.x);
    if (speed <= kCreepSpeed) return;
    const float severity = clamp((speed - kCreepSpeed) * 1.4f, 0.0f, 5.0f);
    if (c.body_force) c.body_force->y -= c.mass * std::abs(c.gravity) * (0.55f + 0.30f * severity);

  }

 protected:
  static constexpr float kCreepSpeed = 0.55f;
};

class CollapseWindowFlats final : public MoltenWindowBiome {
 public:
  std::string_view id() const noexcept override { return "collapse_window_flats"; }
  std::string_view display_name() const noexcept override { return "Collapse Window Flats"; }
  std::string_view skill_stratum() const noexcept override { return "dynamic_obstacle"; }
  int cycle_steps() const noexcept override { return 480; }
  int melt_steps() const noexcept override { return 110; }
  BiomeVisuals visuals() const noexcept override {
    BiomeVisuals v;
    v.sky = {196, 126, 88};
    v.ground = {150, 84, 52};
    v.particles = {224, 146, 92};
    v.particle_rate = 9.0f;
    v.screen_brightness = 0.95f;
    return v;
  }
};

class CollapseWindowTerrace final : public MoltenWindowBiome {
 public:
  std::string_view id() const noexcept override { return "collapse_window_terrace"; }
  std::string_view display_name() const noexcept override { return "Collapse Window Terrace"; }
  std::string_view skill_stratum() const noexcept override { return "dynamic_obstacle"; }
  int cycle_steps() const noexcept override { return 700; }
  int melt_steps() const noexcept override { return 140; }
  int phase_offset() const noexcept override { return 350; }
  BiomeVisuals visuals() const noexcept override {
    BiomeVisuals v;
    v.sky = {172, 138, 108};
    v.ground = {118, 96, 74};
    v.particles = {206, 178, 138};
    v.particle_rate = 7.0f;
    v.screen_brightness = 0.90f;
    return v;
  }
};

class SetpointRimeShelf final : public Biome {
 public:
  std::string_view id() const noexcept override { return "setpoint_rime_shelf"; }
  std::string_view display_name() const noexcept override { return "Setpoint Rime Shelf"; }
  std::string_view skill_stratum() const noexcept override { return "traction_loss"; }
  MechanicType visual_type() const noexcept override { return MechanicType::Ice; }
  BiomeSplit split() const noexcept override { return BiomeSplit::Train; }

  int hazard_at(int) const noexcept override { return 2; }

  MechanicParams sample_params(uint64_t s) const noexcept override {
    MechanicParams p;
    p.friction_mul = 0.85f + 0.15f * biome_random01(s);
    p.ambient_temperature = -46.0f + 10.0f * biome_random01(s, 1);
    p.thermal_transfer = 1.6f + 0.4f * biome_random01(s, 2);
    p.solar_charge_rate = 0.9f + 0.3f * biome_random01(s, 3);
    p.energy_drain_mul = 1.05f + 0.15f * biome_random01(s, 4);
    p.lidar_range_mul = 0.40f;
    p.lidar_energy_mul = 1.8f;
    p.terrain_amplitude_mul = 0.9f;
    return p;
  }

  BiomeVisuals visuals() const noexcept override {
    BiomeVisuals v;
    v.sky = {148, 176, 200};
    v.ground = {166, 186, 198};
    v.particles = {226, 238, 246};
    v.particle_rate = 6.0f;
    v.ambient_particles = 40;
    v.ambient_drift = 2.0f;
    v.screen_brightness = 1.0f;
    return v;
  }

  void apply_effects(const MechanicParams& p, MechanicContext& c) const noexcept override {
    if (!c.contact || !c.contact->active) return;
    const float speed = std::abs(c.wheel_speed);
    if (speed >= kReleaseSpeed) return;

    const float bite = clamp((kReleaseSpeed - speed) / kReleaseSpeed, 0.0f, 1.0f);
    c.contact->penetration += 0.16f * bite * c.dt;
    if (c.wheel_force) *c.wheel_force += c.contact->tangent * (-14.0f * bite * c.wheel_speed);
    if (c.energy_cost) *c.energy_cost += 0.09f * bite * p.energy_drain_mul * c.dt;
  }

  void apply_body_effects(const MechanicParams&, MechanicBodyContext& c) const noexcept override {
    const float speed = std::abs(c.velocity.x);
    if (speed >= kReleaseSpeed) return;
    const float bite = clamp((kReleaseSpeed - speed) / kReleaseSpeed, 0.0f, 1.0f);
    if (c.body_force) c.body_force->x -= c.mass * 2.2f * bite * (c.velocity.x >= 0.0f ? 1.0f : -1.0f);
    if (c.energy_cost) *c.energy_cost += 0.05f * bite * c.dt;
  }

 private:
  static constexpr float kReleaseSpeed = 0.85f;
};

class CollapseWindowGulch final : public MoltenWindowBiome {
 public:
  std::string_view id() const noexcept override { return "collapse_window_gulch"; }
  std::string_view display_name() const noexcept override { return "Collapse Window Gulch"; }
  std::string_view skill_stratum() const noexcept override { return "inertia_hysteresis"; }
  int cycle_steps() const noexcept override { return 600; }
  int melt_steps() const noexcept override { return 130; }
  int phase_offset() const noexcept override { return 200; }
  BiomeVisuals visuals() const noexcept override {
    BiomeVisuals v;
    v.sky = {182, 150, 96};
    v.ground = {126, 104, 60};
    v.particles = {214, 190, 130};
    v.particle_rate = 8.0f;
    v.screen_brightness = 0.92f;
    return v;
  }
};

class CollapseWindowPlaya final : public MoltenWindowBiome {
 public:
  std::string_view id() const noexcept override { return "collapse_window_playa"; }
  std::string_view display_name() const noexcept override { return "Collapse Window Playa"; }
  std::string_view skill_stratum() const noexcept override { return "inertia_hysteresis"; }
  BiomeSplit split() const noexcept override { return BiomeSplit::Train; }
  int cycle_steps() const noexcept override { return 820; }
  int melt_steps() const noexcept override { return 175; }
  int phase_offset() const noexcept override { return 410; }
  MechanicParams sample_params(uint64_t s) const noexcept override {
    MechanicParams p = MoltenWindowBiome::sample_params(s);
    p.friction_mul = 1.22f + 0.14f * biome_random01(s, 11);
    p.gravity_mul = 0.74f + 0.08f * biome_random01(s, 12);
    p.viscosity = 0.45f + 0.20f * biome_random01(s, 13);
    p.ambient_temperature = 50.0f + 12.0f * biome_random01(s, 14);
    p.thermal_transfer = 2.4f + 0.5f * biome_random01(s, 15);
    p.solar_charge_rate = 2.3f + 0.4f * biome_random01(s, 16);
    p.terrain_amplitude_mul = 0.62f;
    p.terrain_crater_mul = 2.10f;
    p.terrain_step_mul = 0.45f;
    return p;
  }
  void apply_effects(const MechanicParams& p, MechanicContext& c) const noexcept override {
    MoltenWindowBiome::apply_effects(p, c);

    if (c.wheel_force && c.contact && c.contact->active)
      *c.wheel_force += c.contact->tangent * (-p.viscosity * 2.8f * c.wheel_speed);
  }
  BiomeVisuals visuals() const noexcept override {
    BiomeVisuals v;
    v.sky = {166, 146, 122};
    v.ground = {132, 118, 96};
    v.particles = {212, 196, 164};
    v.particle_rate = 6.0f;
    v.screen_brightness = 0.90f;
    return v;
  }
};

class PulseGravityReef final : public Biome {
 public:
  std::string_view id() const noexcept override { return "pulse_gravity_reef"; }
  std::string_view display_name() const noexcept override { return "Pulse Gravity Reef"; }
  std::string_view skill_stratum() const noexcept override { return "gravity_change"; }
  MechanicType visual_type() const noexcept override { return MechanicType::LowGravity; }
  BiomeSplit split() const noexcept override { return BiomeSplit::Train; }

  bool pulsing(int step) const noexcept {
    int phase = (step + kPhase) % kCycle;
    if (phase < 0) phase += kCycle;
    return phase < kPulseSteps;
  }
  int hazard_at(int step) const noexcept override { return pulsing(step) ? 1 : 0; }

  MechanicParams sample_params(uint64_t s) const noexcept override {
    MechanicParams p;
    p.friction_mul = 0.68f + 0.12f * biome_random01(s);
    p.gravity_mul = 0.55f + 0.10f * biome_random01(s, 1);
    p.ambient_temperature = -28.0f + 12.0f * biome_random01(s, 2);
    p.thermal_transfer = 0.75f + 0.20f * biome_random01(s, 3);
    p.solar_charge_rate = 1.35f + 0.35f * biome_random01(s, 4);
    p.energy_drain_mul = 0.95f + 0.15f * biome_random01(s, 5);
    p.lidar_range_mul = 0.30f;
    p.lidar_energy_mul = 2.6f;
    p.terrain_amplitude_mul = 1.30f;
    p.terrain_crater_mul = 1.80f;
    return p;
  }

  BiomeVisuals visuals() const noexcept override {
    BiomeVisuals v;
    v.sky = {132, 128, 176};
    v.ground = {104, 100, 138};
    v.particles = {186, 182, 222};
    v.particle_rate = 5.0f;
    v.particle_lift = 2.6f;
    v.screen_brightness = 0.94f;
    return v;
  }

  void apply_effects(const MechanicParams&, MechanicContext& c) const noexcept override {
    if (!c.contact || !c.contact->active || !pulsing(c.step_index)) return;

    if (c.wheel_force)
      *c.wheel_force -= c.contact->normal * (c.contact->normal_force * 0.72f);
  }

  void apply_body_effects(const MechanicParams&, MechanicBodyContext& c) const noexcept override {
    if (!pulsing(c.step_index)) return;
    const float speed = std::abs(c.velocity.x);
    if (speed <= kSafeSpeed) return;
    const float over = clamp((speed - kSafeSpeed) * 1.4f, 0.0f, 4.0f);
    if (c.body_force) c.body_force->y += c.mass * std::abs(c.gravity) * (0.30f + 0.22f * over);
    if (c.body_torque) *c.body_torque += c.mass * (1.1f + 0.9f * over);
  }

 private:
  static constexpr int kCycle = 690;
  static constexpr int kPulseSteps = 165;
  static constexpr int kPhase = 120;
  static constexpr float kSafeSpeed = 0.70f;
};

class CommitmentLedgeField final : public Biome {
 public:
  std::string_view id() const noexcept override { return "commitment_ledge_field"; }
  std::string_view display_name() const noexcept override { return "Commitment Ledge Field"; }
  std::string_view skill_stratum() const noexcept override { return "dynamic_obstacle"; }
  MechanicType visual_type() const noexcept override { return MechanicType::Normal; }
  BiomeSplit split() const noexcept override { return BiomeSplit::Train; }

  int hazard_at(int) const noexcept override { return 2; }

  MechanicParams sample_params(uint64_t s) const noexcept override {
    MechanicParams p;
    p.friction_mul = 1.15f + 0.10f * biome_random01(s);
    p.gravity_mul = 0.92f + 0.06f * biome_random01(s, 1);
    p.energy_drain_mul = 0.85f + 0.10f * biome_random01(s, 2);
    p.ambient_temperature = -18.0f + 8.0f * biome_random01(s, 3);
    p.thermal_transfer = 0.9f + 0.2f * biome_random01(s, 4);
    p.solar_charge_rate = 1.2f + 0.2f * biome_random01(s, 5);
    p.lidar_energy_mul = 0.30f;
    p.lidar_range_mul = 1.0f;
    p.terrain_amplitude_mul = 0.55f;
    p.terrain_roughness_mul = 0.55f;
    p.terrain_crater_mul = 0.30f;
    p.terrain_step_mul = 0.30f;
    p.ledge_start_x = 18.0f;
    p.ledge_spacing = 25.0f;
    p.ledge_gap_width = 3.2f + 0.4f * biome_random01(s, 6);
    p.ledge_ramp_length = 2.8f;
    p.ledge_ramp_height = 0.72f;
    return p;
  }

  BiomeVisuals visuals() const noexcept override {
    BiomeVisuals v;
    v.sky = {152, 112, 92};
    v.ground = {102, 82, 72};
    v.particles = {188, 154, 128};
    v.particle_rate = 5.0f;
    v.screen_brightness = 0.95f;
    return v;
  }
};

inline void append(std::vector<const Biome*>& out) {
  static const CollapseWindowFlats collapse_flats; out.push_back(&collapse_flats);
  static const CollapseWindowGulch collapse_gulch; out.push_back(&collapse_gulch);
  static const SetpointRimeShelf rime_shelf; out.push_back(&rime_shelf);
  static const CollapseWindowPlaya collapse_playa; out.push_back(&collapse_playa);
  static const PulseGravityReef pulse_reef; out.push_back(&pulse_reef);
  static const CommitmentLedgeField commitment_ledge; out.push_back(&commitment_ledge);
}

}
