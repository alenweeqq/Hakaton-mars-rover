#pragma once

namespace generated_biomes {

inline void append(std::vector<const Biome*>& out) {
  static const LateralShearBelt biome_0; out.push_back(&biome_0);
  static const HysteresisSurgeBog biome_1; out.push_back(&biome_1);
  static const GravityShelfLug biome_2; out.push_back(&biome_2);
  static const ThermalSurgeRelay biome_3; out.push_back(&biome_3);
  static const RimeQuarryDawn biome_4; out.push_back(&biome_4);
  static const GravityShearEscarpment biome_5; out.push_back(&biome_5);
}

inline constexpr int kGeneratedBiomeBankEnd = 0;

}
