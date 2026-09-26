def telemetry_lines(debug: dict, fps: float, seed: int) -> tuple[str, ...]:
    energy = debug.get("energy", 0.0)
    capacity = max(1.0, debug.get("energy_capacity", 1.0))
    engine = "RUNNING" if debug.get("engine_running") else "OFF"
    if debug.get("engine_overheated"):
        engine = "OVERHEATED"
    elif debug.get("engine_cold_locked"):
        engine = "TOO COLD"
    jump_phase = {0: "READY", 1: "PRELOAD", 2: "LAUNCH"}.get(
        debug.get("suspension_jump_phase", 0), "READY"
    )
    contacts = "".join(
        name if debug.get(field) else "-"
        for name, field in (
            ("F", "body_contact_front"),
            ("B", "body_contact_belly"),
            ("R", "body_contact_rear"),
            ("T", "body_contact_roof"),
        )
    )
    if debug.get("lidar_landing_valid"):
        drop = debug.get("lidar_landing_x", 0.0) - debug.get("x", 0.0)
        landing = f"{drop:+6.1f}m   h {debug.get('lidar_landing_y', 0.0):+6.1f}m"
    else:
        landing = "   --"
    return (
        f"FPS {fps:5.1f}   SEED {seed}",
        f"TIME {debug.get('trial_time_left', 0.0):6.1f}s   BIOME {debug.get('mechanic', '-')}",
        f"DIST {debug.get('x', 0.0):7.1f}m   SPEED {debug.get('speed_kmh', 0.0):5.1f} km/h",
        f"BATTERY {energy / capacity * 100:5.1f}%   NET {debug.get('energy_gain_rate', 0.0) - debug.get('energy_cost_rate', 0.0):+.2f}/s",
        f"ENGINE {engine}   RPM {debug.get('engine_rpm', 0.0):5.0f}",
        f"TEMP {debug.get('engine_temperature', 0.0):5.1f} C   ENV {debug.get('ambient_temperature', 0.0):5.1f} C",
        f"GEAR {debug.get('gear', '-')} / {debug.get('gear_count', '-')}   DRIVE {debug.get('drive_layout', '-')}",
        f"CLUTCH {debug.get('clutch_engagement', 0.0) * 100:3.0f}%   SLIP {debug.get('drivetrain_slip', 0.0):.2f}",
        f"SPRING {jump_phase} {debug.get('suspension_jump_charge', 0.0) * 100:3.0f}%",
        f"PISTON {debug.get('roof_piston_extension', 0.0) * 100:3.0f}%   CONTACT {contacts}",
        f"SOLAR {debug.get('solar_panel_deployment', 0.0) * 100:3.0f}%   +{debug.get('solar_charge_rate', 0.0):.2f}/s",
        f"LIDAR {debug.get('lidar_range', 0.0):5.1f}m",
        f"LANDING {landing}",
        f"PROPELLER {debug.get('propeller_thrust', 0.0):6.1f}N   ROCKET {debug.get('thruster_thrust', 0.0):6.1f}N",
        f"GRAVITY {debug.get('gravity', -3.71):+.2f}m/s²",
    )
