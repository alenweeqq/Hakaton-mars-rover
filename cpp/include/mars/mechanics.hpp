#pragma once

#include <array>
#include <cstdint>

#include "mars/contacts.hpp"
#include "mars/math.hpp"

namespace mars {

enum class MechanicType : int {
  Normal = 0,
  Sand = 1,
  Ice = 2,
  Mud = 3,
  Wind = 4,
  LowGravity = 5,
  Crust = 6,
  Liquid = 7,
};

enum class GenerationParameter : int {
  Moisture,
  Sink,
  Temperature,
  Traction,
  Viscosity,
  EnergyResistance,
  ThermalTransfer,
};

struct GenerationInfluence {
  GenerationParameter source;
  GenerationParameter target;
  float weight;
};



inline constexpr std::array<GenerationInfluence, 5> kGenerationInfluences{{
    {GenerationParameter::Moisture, GenerationParameter::Traction, -0.72f},
    {GenerationParameter::Sink, GenerationParameter::Traction, -0.58f},
    {GenerationParameter::Temperature, GenerationParameter::Viscosity, -0.78f},
    {GenerationParameter::Viscosity, GenerationParameter::EnergyResistance, 0.88f},
    {GenerationParameter::Temperature, GenerationParameter::ThermalTransfer, -0.72f},
}};

struct MechanicParams {
  float friction_mul = 1.0f;
  float moisture = 0.22f;
  float sink_rate = 0.0f;
  float viscosity = 0.0f;
  float wind_force = 0.0f;
  float gravity_mul = 1.0f;
  float energy_drain_mul = 1.0f;
  float crust_deform = 0.0f;
  float ambient_temperature = 20.0f;
  float thermal_transfer = 1.0f;
  float solar_charge_rate = 1.0f;
  float lidar_energy_mul = 1.0f;
  float lidar_range_mul = 1.0f;


  float base_friction_mul = 1.0f;
  float base_viscosity = 0.0f;
  float base_energy_drain_mul = 1.0f;
  float base_thermal_transfer = 1.0f;





  float terrain_amplitude_mul = 1.0f;
  float terrain_roughness_mul = 1.0f;
  float terrain_crater_mul = 1.0f;
  float terrain_step_mul = 1.0f;



  float ledge_gap_width = 0.0f;
  float ledge_spacing = 0.0f;
  float ledge_ramp_length = 0.0f;
  float ledge_ramp_height = 0.0f;
  float ledge_start_x = 18.0f;
};

constexpr int kMaxMechanicZones = 96;
constexpr int kMaxActiveMechanisms = 4;

struct MechanicZone {
  float begin_x = 0.0f;
  float end_x = 1.0e9f;
  MechanicType type = MechanicType::Normal;
  int biome_id = 0;
  uint64_t terrain_seed = 0;
  int terrain_surprise_mode = 0;
  float terrain_surprise_strength = 0.0f;
  int terrain_profile = 0;
  float terrain_frequency_scale = 1.0f;

  int jagged_mode = 0;
  float jagged_amplitude = 0.0f;
  MechanicParams params{};
  float liquid_level = -1.0e9f;
};

struct MechanicLayout {
  std::array<MechanicZone, kMaxMechanicZones> zones{};
  int count = 1;


  std::array<MechanicZone, kMaxMechanicZones> layers{};
  int layer_count = 0;

  const MechanicZone& at(float x) const;
  MechanicParams thermal_at(float x) const;
  MechanicParams thermal_at(float x, const MechanicZone& current) const;
  int active_layers(float x, std::array<const MechanicZone*, kMaxActiveMechanisms>& out) const;
};

struct MechanicContext {
  WheelContact* contact = nullptr;
  Vec2* wheel_force = nullptr;
  Vec2* body_force = nullptr;
  float* energy_cost = nullptr;
  float dt = 1.0f / 60.0f;
  float wheel_radius = 0.24f;
  float base_friction = 1.0f;
  float drive_force = 0.0f;
  float minimum_drive_limit = 0.0f;
  float wheel_speed = 0.0f;
  float immersion = 0.0f;
  int step_index = 0;
};

struct MechanicBodyContext {
  Vec2* body_force = nullptr;
  float* body_torque = nullptr;
  float* energy_cost = nullptr;
  Vec2 velocity{};
  float angular_velocity = 0.0f;
  float mass = 1.0f;
  float gravity = -3.71f;
  float dt = 1.0f / 60.0f;
  int step_index = 0;
};

void apply_mechanic(int biome_id, const MechanicParams& params, MechanicContext& ctx);
float mechanic_friction_scale(int biome_id, const MechanicParams& params);
void apply_body_mechanic(int biome_id, const MechanicParams& params, MechanicBodyContext& ctx);
int builtin_biome_id(MechanicType type);
void prepare_generation_params(MechanicParams& params, MechanicType type, uint64_t seed);
void apply_generation_influences(MechanicParams& params);

}
