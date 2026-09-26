from _mars_rover_cpp import biome_catalog


VISIBLE_SPLITS = frozenset({0, 1})


def visible_catalog() -> list[dict]:
    return [dict(item) for item in biome_catalog() if int(item["split"]) in VISIBLE_SPLITS]


def create_environment(args):
    from mars_rover_env import MarsRoverEnv

    return MarsRoverEnv(
        config_path=args.config,
        rig_path=args.rig,
        biome_split=1,
        fixed_biome_id=args.biome_index if args.biome_index is not None else -1,
        render_mode="debug_rgb_array" if args.debug else "rgb_array",
        render_width=args.render_width,
        render_height=args.render_height,
    )
