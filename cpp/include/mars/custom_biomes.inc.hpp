#pragma once

namespace custom_biomes {

class ExampleTrainingBiome final : public Biome {
 public:
  std::string_view id() const noexcept override { return "example_training"; }
  std::string_view display_name() const noexcept override {
    return "Example training biome";
  }
  std::string_view skill_stratum() const noexcept override {
    return "custom";
  }
  MechanicType visual_type() const noexcept override {
    return MechanicType::Sand;
  }
  BiomeSplit split() const noexcept override { return BiomeSplit::Train; }

  MechanicParams sample_params(uint64_t seed) const noexcept override {
    MechanicParams params;
    params.friction_mul = 0.55f + 0.20f * biome_random01(seed);
    params.energy_drain_mul = 1.10f + 0.30f * biome_random01(seed, 1);
    params.terrain_amplitude_mul = 0.90f + 0.35f * biome_random01(seed, 2);
    params.terrain_roughness_mul = 0.85f + 0.30f * biome_random01(seed, 3);
    return params;
  }

  BiomeVisuals visuals() const noexcept override {
    BiomeVisuals visuals;
    visuals.sky = {170, 100, 70};
    visuals.ground = {145, 95, 55};
    visuals.particles = {205, 155, 95};
    visuals.particle_rate = 8.0f;
    return visuals;
  }
};

inline void append(std::vector<const Biome*>& out) {
  (void)out;
}

}
