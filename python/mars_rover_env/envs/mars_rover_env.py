from __future__ import annotations

import numpy as np

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError as exc:
    raise ImportError("gymnasium is required for MarsRoverEnv") from exc

from _mars_rover_cpp import MarsRoverBatchEnv

from mars_rover_env.config import load_env_config


class MarsRoverEnv(gym.Env):
    BIOME_MODE_ALL = 0
    BIOME_MODE_TRAIN = 1
    BIOME_MODE_ANCHOR = 3
    metadata = {"render_modes": [None, "rgb_array", "debug_rgb_array"], "render_fps": 60}

    def __init__(
        self,
        config_path: str | None = None,
        rig_path: str | None = None,
        render_mode: str | None = None,
        render_width: int = 640,
        render_height: int = 360,
        biome_split: int | None = None,
        fixed_biome_id: int | None = None,
        chain_biomes: bool | None = None,
        chain_zone_count: int | None = None,
        config_override=None,
    ):
        super().__init__()
        self.render_mode = render_mode
        self._render_width = int(render_width)
        self._render_height = int(render_height)
        self._rgb = np.zeros((self._render_height, self._render_width, 3), dtype=np.uint8)
        self._config = config_override or load_env_config(config_path, rig_path)
        if biome_split is not None:
            self._config.biome_split = int(biome_split)
        if fixed_biome_id is not None:
            self._config.fixed_biome_id = int(fixed_biome_id)
        if chain_biomes is not None:
            self._config.chain_biomes = bool(chain_biomes)
        if chain_zone_count is not None:
            self._config.chain_zone_count = int(chain_zone_count)
        self._episodes_per_trial = int(self._config.episodes_per_trial)
        self._trial_time_limit = float(self._config.termination.trial_time_limit)
        self._episodes_seen_in_trial = 0
        self._next_trial_start = True
        self._batch = MarsRoverBatchEnv(1, self._config)
        self._obs = np.zeros((1, self._batch.obs_dim), dtype=np.float32)
        self._reward = np.zeros((1,), dtype=np.float32)
        self._terminated = np.zeros((1,), dtype=np.uint8)
        self._truncated = np.zeros((1,), dtype=np.uint8)
        self._actions = np.zeros((1,), dtype=np.int32)

        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(self._batch.obs_dim,), dtype=np.float32
        )
        self.action_space = spaces.Discrete(self._batch.action_dim)

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        super().reset(seed=seed)
        if options is not None and "trial_start" in options:
            trial_start = bool(options["trial_start"])
        else:
            trial_start = self._next_trial_start
        seed_value = (
            int(self.np_random.integers(0, np.iinfo(np.uint32).max, dtype=np.uint32))
            if seed is None
            else int(seed)
        )
        self._batch.reset_at(0, seed_value, trial_start, self._obs[0])
        if trial_start:
            self._episodes_seen_in_trial = 1
        else:
            self._episodes_seen_in_trial += 1
        if self._trial_time_limit > 0.0:
            self._next_trial_start = bool(self._batch.trial_exhausted(0))
        else:
            self._next_trial_start = (
                self._episodes_per_trial > 0
                and self._episodes_seen_in_trial >= self._episodes_per_trial
            )
        if self._next_trial_start:
            self._episodes_seen_in_trial = 0
        return self._obs[0].copy(), {"trial_start": trial_start}

    def step(self, action: int):
        self._actions[0] = int(action)
        self._batch.step(self._actions, self._obs, self._reward, self._terminated, self._truncated)
        terminated = bool(self._terminated[0])
        truncated = bool(self._truncated[0])
        if self._trial_time_limit > 0.0 and self._batch.trial_exhausted(0):
            self._next_trial_start = True
        return self._obs[0].copy(), float(self._reward[0]), terminated, truncated, {}

    def render(self):
        if self.render_mode is None:
            return None
        debug = self.render_mode == "debug_rgb_array"
        self._batch.render_rgb(0, self._rgb, self._render_width, self._render_height, debug)
        return self._rgb.copy()

    def debug_info(self) -> dict:
        return dict(self._batch.debug_info(0))
