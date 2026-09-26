#include "mars/physics.hpp"
#include "mars/biome_bank.hpp"

#include <cmath>

namespace mars {
namespace {

constexpr float kGearRatios[] = {12.30f, 7.40f, 5.30f, 4.10f, 3.36f, 2.85f};

constexpr float kMinimumLoadedRpm[] = {800.0f, 1350.0f, 1800.0f, 2300.0f,
                                       2800.0f, 3200.0f};
constexpr float kGearEnergyMul[] = {1.00f, 1.15f, 1.34f, 1.58f, 1.90f, 2.30f};

constexpr float kBasePostShiftRpm[] = {2600.0f, 2700.0f, 2800.0f, 2900.0f, 3000.0f};
constexpr int kGearCount = static_cast<int>(sizeof(kGearRatios) / sizeof(kGearRatios[0]));
constexpr float kWheelRadiusReference = 0.24f;
constexpr float kDrivelineReference = 2.25f;

constexpr float kIdleRpm = 1100.0f;
constexpr float kRedlineRpm = 9000.0f;
constexpr float kSafeDownshiftRpm = 8500.0f;

}

PhysicsEngine::PhysicsEngine(PhysicsConfig config) : config_(config) {}

void PhysicsEngine::initialize_state(const RoverRig& rig, RoverState& state, Vec2 spawn) const {
  state = RoverState{};
  state.body.position = spawn;
  state.render_camera_position = spawn;
  state.body.mass = rig.body.mass;
  state.body.inv_mass = rig.body.mass > 0.0f ? 1.0f / rig.body.mass : 0.0f;
  state.body.inertia = rig.body.inertia;
  state.body.inv_inertia = rig.body.inertia > 0.0f ? 1.0f / rig.body.inertia : 0.0f;
  state.wheel_count = static_cast<int>(rig.wheels.size());
  for (int i = 0; i < state.wheel_count; ++i) {
    const auto& wr = rig.wheels[static_cast<size_t>(i)];
    auto& w = state.wheels[static_cast<size_t>(i)];
    w.radius = wr.radius;
    w.mass = wr.mass;
    w.inv_mass = wr.mass > 0.0f ? 1.0f / wr.mass : 0.0f;
    w.position = spawn + wr.local_anchor + Vec2{0.0f, -wr.suspension.rest_length};
  }
  state.energy = clamp(config_.initial_energy, 0.0f, config_.energy_capacity);
  state.previous_x = state.body.position.x;
  state.engine_temperature = config_.initial_engine_temperature;
  state.engine_cold_locked =
      state.engine_temperature < config_.cold_start_temperature;
  state.engine_overheated =
      state.engine_temperature >= config_.overheat_temperature;
  state.engine_running = !state.engine_cold_locked && !state.engine_overheated;
  state.engine_stalled = !state.engine_running;
}

PhysicsStepStats PhysicsEngine::step(const RoverRig& rig, const Terrain& terrain, RoverState& state,
                                     int discrete_action, const MechanicLayout& mechanics) const {
#include "physics/step_controls.inc"
#include "physics/step_powertrain.inc"
#include "physics/step_dynamics.inc"
#include "physics/step_wheels.inc"
#include "physics/step_contacts.inc"

}

void PhysicsEngine::apply_body_impulse(RigidBodyState& body, Vec2 impulse, Vec2 point) const {
  body.velocity += impulse * body.inv_mass;
  body.angular_velocity += cross(point - body.position, impulse) * body.inv_inertia;
}

}
