#include "mars/biome_bank.hpp"

#include <array>
#include <cmath>

namespace mars {

namespace {

float normalized_source(const MechanicParams& p, GenerationParameter source) {
  switch (source) {
    case GenerationParameter::Moisture: return clamp(p.moisture, 0.0f, 1.0f);
    case GenerationParameter::Sink: return clamp(p.sink_rate, 0.0f, 1.0f);
    case GenerationParameter::Temperature:
      return clamp((p.ambient_temperature + 58.0f) / 130.0f, 0.0f, 1.0f);
    case GenerationParameter::Traction:
      return clamp((p.friction_mul - 0.12f) / (1.45f - 0.12f), 0.0f, 1.0f);
    case GenerationParameter::Viscosity: return clamp(p.viscosity / 2.5f, 0.0f, 1.0f);
    case GenerationParameter::EnergyResistance:
      return clamp((p.energy_drain_mul - 0.4f) / 2.6f, 0.0f, 1.0f);
    case GenerationParameter::ThermalTransfer:
      return clamp((p.thermal_transfer - 0.25f) / 2.75f, 0.0f, 1.0f);
  }
  return 0.0f;
}

float target_scale(GenerationParameter target) {
  switch (target) {
    case GenerationParameter::Traction: return 0.38f;
    case GenerationParameter::Viscosity: return 0.34f;
    case GenerationParameter::EnergyResistance: return 0.68f;
    case GenerationParameter::ThermalTransfer: return 0.62f;
    default: return 0.0f;
  }
}

void apply_target(MechanicParams& p, GenerationParameter target) {
  float sum = 0.0f;
  for (const auto& influence : kGenerationInfluences) {
    if (influence.target == target) {
      sum += influence.weight * normalized_source(p, influence.source);
    }
  }
  const float adjustment = std::copysign(std::log1p(std::abs(sum)), sum);
  const float scaled = target_scale(target) * adjustment;
  switch (target) {
    case GenerationParameter::Traction:
      p.friction_mul = clamp(p.base_friction_mul + scaled, 0.12f, 1.45f);
      break;
    case GenerationParameter::Viscosity:
      p.viscosity = clamp(p.base_viscosity + scaled, 0.0f, 2.5f);
      break;
    case GenerationParameter::EnergyResistance:
      p.energy_drain_mul = clamp(p.base_energy_drain_mul + scaled, 0.4f, 3.0f);
      break;
    case GenerationParameter::ThermalTransfer:
      p.thermal_transfer = clamp(p.base_thermal_transfer + scaled, 0.25f, 3.0f);
      break;
    default:
      break;
  }
}

float sampled_moisture(MechanicType type, uint64_t seed) {
  const float u = biome_random01(seed, 0x4d4f4953ULL);
  switch (type) {
    case MechanicType::Sand: return 0.03f + 0.17f * u;
    case MechanicType::Ice: return 0.08f + 0.27f * u;
    case MechanicType::Mud: return 0.62f + 0.35f * u;
    case MechanicType::Liquid: return 0.82f + 0.18f * u;
    case MechanicType::Crust: return 0.06f + 0.24f * u;
    default: return 0.10f + 0.30f * u;
  }
}

}

void apply_generation_influences(MechanicParams& p) {


  apply_target(p, GenerationParameter::Traction);
  apply_target(p, GenerationParameter::Viscosity);
  apply_target(p, GenerationParameter::EnergyResistance);
  apply_target(p, GenerationParameter::ThermalTransfer);

  p.moisture = clamp(p.moisture, 0.0f, 1.0f);
  p.sink_rate = clamp(p.sink_rate, 0.0f, 1.0f);


  p.gravity_mul = clamp(p.gravity_mul, 0.25f, 1.60f);
  p.wind_force = clamp(p.wind_force, -80.0f, 80.0f);
  p.ambient_temperature = clamp(p.ambient_temperature, -100.0f, 72.0f);
  p.solar_charge_rate = clamp(p.solar_charge_rate, 0.0f, 8.0f);
  p.lidar_energy_mul = clamp(p.lidar_energy_mul, 0.4f, 3.0f);
  p.lidar_range_mul = clamp(p.lidar_range_mul, 0.0f, 1.5f);
}

void prepare_generation_params(MechanicParams& p, MechanicType type, uint64_t seed) {
  p.moisture = sampled_moisture(type, seed);


  p.ambient_temperature = clamp(p.ambient_temperature, -100.0f, 72.0f);
  p.base_friction_mul = clamp(p.friction_mul, 0.12f, 1.45f);



  p.base_viscosity = p.viscosity > 0.0f
                         ? clamp(p.viscosity, 0.0f, 2.5f)
                         : 0.18f + 0.18f * biome_random01(seed, 0x56495343ULL);
  p.base_energy_drain_mul = clamp(p.energy_drain_mul, 0.4f, 3.0f);
  p.base_thermal_transfer = clamp(p.thermal_transfer, 0.25f, 6.0f);
  apply_generation_influences(p);
}

const MechanicZone& MechanicLayout::at(float x) const {
  int lo = 0, hi = std::max(0, count - 1);
  while (lo < hi) { const int mid = (lo + hi) / 2; if (x < zones[static_cast<size_t>(mid)].end_x) hi = mid; else lo = mid + 1; }
  return zones[static_cast<size_t>(lo)];
}

MechanicParams MechanicLayout::thermal_at(float x) const { const auto& current = at(x); return thermal_at(x, current); }

MechanicParams MechanicLayout::thermal_at(float x, const MechanicZone& current) const {
  const int index = static_cast<int>(&current - zones.data()); MechanicParams result = current.params;
  const auto blend = [&](const MechanicParams& a, const MechanicParams& b, float t) {
    t = t * t * (3.0f - 2.0f * t);
    result.ambient_temperature = a.ambient_temperature + (b.ambient_temperature - a.ambient_temperature) * t;
    result.thermal_transfer = a.thermal_transfer + (b.thermal_transfer - a.thermal_transfer) * t;
  };
  if (index > 0) { const auto& previous = zones[static_cast<size_t>(index - 1)];
    const float width = std::min(4.0f, std::min(previous.end_x - previous.begin_x, current.end_x - current.begin_x) * 0.25f);
    if (width > 0.0f && x < current.begin_x + width) { blend(previous.params, current.params, clamp((x - (current.begin_x - width)) / (2.0f * width), 0.0f, 1.0f)); return result; }
  }
  if (index + 1 < count) { const auto& next = zones[static_cast<size_t>(index + 1)];
    const float width = std::min(4.0f, std::min(current.end_x - current.begin_x, next.end_x - next.begin_x) * 0.25f);
    if (width > 0.0f && x > current.end_x - width) blend(current.params, next.params, clamp((x - (current.end_x - width)) / (2.0f * width), 0.0f, 1.0f));
  }
  return result;
}

int MechanicLayout::active_layers(
    float x, std::array<const MechanicZone*, kMaxActiveMechanisms>& out) const {
  int n = 0;


  for (int i = 0; i < layer_count && n < kMaxActiveMechanisms; ++i) {
    const auto& layer = layers[static_cast<size_t>(i)];
    if (x >= layer.begin_x && x <= layer.end_x) out[static_cast<size_t>(n++)] = &layer;
  }
  return n;
}

int builtin_biome_id(MechanicType type) { return static_cast<int>(type); }
float mechanic_friction_scale(int biome_id, const MechanicParams& params) { return biome_by_id(biome_id).friction_scale(params); }
void apply_mechanic(int biome_id, const MechanicParams& params, MechanicContext& ctx) { biome_by_id(biome_id).apply(params, ctx); }
void apply_body_mechanic(int biome_id, const MechanicParams& params, MechanicBodyContext& ctx) {
  biome_by_id(biome_id).apply_body_effects(params, ctx);
}

}
