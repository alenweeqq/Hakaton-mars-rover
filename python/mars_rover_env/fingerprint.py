from __future__ import annotations

import math
import os
import subprocess
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def fingerprint_of_compiled_biome(qualified_class: str, compiler: str = "g++") -> list[float]:


    return _fingerprint_trace(
        qualified_class.rsplit("::", 1)[-1],
        declarations="",
        construction=qualified_class,
        compiler=compiler,
    )


def _fingerprint_trace(
    name: str, *, declarations: str, construction: str, compiler: str
) -> list[float]:
    unit = f'''#include <cmath>
#include <iomanip>
#include <iostream>
#include "mars/biome_bank.hpp"
{declarations}
int main() {{
  using namespace mars;
  {construction} biome;
  std::cout << std::setprecision(9);
  for (uint64_t seed : {{11ULL, 29ULL, 47ULL}}) {{
    MechanicParams p = biome.sample_params(seed);
    std::cout << static_cast<int>(biome.visual_type()) << ' '
              << p.friction_mul << ' ' << p.sink_rate << ' ' << p.viscosity << ' '
              << p.wind_force << ' ' << p.gravity_mul << ' ' << p.energy_drain_mul << ' '
              << p.crust_deform << ' ' << p.ambient_temperature / 100.0f << ' '
              << p.thermal_transfer << ' ' << p.solar_charge_rate << ' '
              << p.lidar_energy_mul << ' ' << p.lidar_range_mul << ' '
              << p.terrain_amplitude_mul << ' ' << p.terrain_roughness_mul << ' '
              << p.terrain_crater_mul << ' ' << p.terrain_step_mul << ' '
              << p.ledge_gap_width << ' ' << p.ledge_spacing << ' '
              << p.ledge_ramp_length << ' ' << p.ledge_ramp_height << ' '
              << p.ledge_start_x << ' ' << biome.friction_scale(p) << ' ';
    for (float terrain_x : {{0.0f, 1.7f, 4.3f, 9.1f, 17.0f, 31.0f}}) {{
      std::cout << biome.terrain_height_delta(terrain_x, seed ^ 0x6a09e667ULL) << ' ';
    }}
    for (int hazard_step = 0; hazard_step <= 1260; hazard_step += 21) {{
      std::cout << biome.hazard_at(hazard_step) << ' ';
    }}
    for (int probe_step : {{0, 137, 389, 701, 1031}}) {{
      for (float probe_speed : {{0.15f, 0.85f, 2.20f, 4.00f}}) {{
        Vec2 probe_force{{0.0f, 0.0f}};
        float probe_torque = 0.0f, probe_cost = 0.0f;
        MechanicBodyContext probe;
        probe.body_force = &probe_force; probe.body_torque = &probe_torque;
        probe.energy_cost = &probe_cost; probe.velocity = Vec2{{probe_speed, 0.18f}};
        probe.angular_velocity = 0.24f; probe.mass = 12.0f;
        probe.gravity = -3.71f * p.gravity_mul; probe.dt = 1.0f / 60.0f;
        probe.step_index = probe_step;
        biome.apply_body_effects(p, probe);
        std::cout << probe_force.x << ' ' << probe_force.y << ' '
                  << probe_torque << ' ' << probe_cost << ' ';
      }}
    }}
    Vec2 velocity{{0.15f, 0.0f}};
    float angular_velocity = 0.0f, energy = 60.0f;
    for (int step = 0; step < 96; ++step) {{
      const float drive = step < 24 ? 70.0f : (step < 48 ? -35.0f : (step < 72 ? 0.0f : 95.0f));
      WheelContact contact;
      contact.active = true;
      contact.normal_force = 80.0f + static_cast<float>((step * 17) % 43);
      contact.penetration = 0.006f + static_cast<float>(step % 9) * 0.0015f;
      contact.normal = normalized(Vec2{{-0.08f * std::sin(step * 0.13f), 1.0f}});
      contact.tangent = normalized(Vec2{{contact.normal.y, -contact.normal.x}});
      Vec2 wheel_force{{0.0f, 0.0f}}, body_force{{0.0f, 0.0f}};
      float torque = 0.0f, cost = 0.0f;
      MechanicContext wheel;
      wheel.contact = &contact; wheel.wheel_force = &wheel_force; wheel.body_force = &body_force;
      wheel.energy_cost = &cost; wheel.dt = 1.0f / 60.0f; wheel.wheel_radius = 0.24f;
      wheel.base_friction = 1.2f; wheel.drive_force = drive; wheel.minimum_drive_limit = 0.0f;
      wheel.wheel_speed = velocity.x + std::sin(step * 0.19f); wheel.immersion = (step % 32) / 31.0f;
      const int biome_step = step * 13;
      wheel.step_index = biome_step;
      biome.apply(p, wheel);
      MechanicBodyContext body;
      body.body_force = &body_force; body.body_torque = &torque; body.energy_cost = &cost;
      body.velocity = velocity; body.angular_velocity = angular_velocity; body.mass = 12.0f;
      body.gravity = -3.71f * p.gravity_mul; body.dt = 1.0f / 60.0f; body.step_index = biome_step;
      biome.apply_body_effects(p, body);
      const Vec2 previous = velocity;
      velocity += (wheel_force + body_force) * (body.dt / 12.0f);
      angular_velocity += torque * body.dt / 4.0f;
      energy -= cost;
      if (step % 12 == 11) {{
        std::cout << velocity.x - previous.x << ' ' << velocity.y - previous.y << ' '
                  << angular_velocity << ' ' << energy << ' ' << contact.slip << ' ';
      }}
    }}
  }}
}}
'''
    with tempfile.TemporaryDirectory(prefix="mars-biome-fingerprint-") as temp:
        source_path = Path(temp) / "fingerprint.cpp"
        executable = Path(temp) / ("fingerprint.exe" if os.name == "nt" else "fingerprint")
        source_path.write_text(unit, encoding="utf-8")
        build = subprocess.run(
            [
                compiler,
                "-std=c++20",
                "-O2",
                "-I",
                str(ROOT / "cpp" / "include"),
                str(source_path),
                "-o",
                str(executable),
            ],
            capture_output=True,
            text=True,
        )
        if build.returncode:
            raise RuntimeError(f"Behavior fingerprint build failed for {name}:\n{build.stderr}")
        run = subprocess.run([str(executable)], capture_output=True, text=True, timeout=30)
        if run.returncode:
            raise RuntimeError(f"Behavior fingerprint failed for {name}:\n{run.stderr}")
    try:
        values = [float(item) for item in run.stdout.split()]
    except ValueError as exc:
        raise RuntimeError(f"Non-numeric behavior fingerprint for {name}") from exc
    if not values or not all(math.isfinite(item) for item in values):
        raise RuntimeError(f"Non-finite behavior fingerprint for {name}")
    return values
