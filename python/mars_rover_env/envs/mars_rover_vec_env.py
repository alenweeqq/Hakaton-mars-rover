from __future__ import annotations

import numpy as np

from _mars_rover_cpp import MarsRoverBatchEnv

from mars_rover_env.config import load_env_config


class MarsRoverVecEnv:


    def __init__(
        self,
        num_envs: int,
        config_path: str | None = None,
        rig_path: str | None = None,
        biome_split: int | None = None,
        fixed_biome_id: int | None = None,
        config_override=None,
    ):
        config = config_override or load_env_config(config_path, rig_path)
        if biome_split is not None:
            config.biome_split = int(biome_split)
        if fixed_biome_id is not None:
            config.fixed_biome_id = int(fixed_biome_id)
        self.episodes_per_trial = int(config.episodes_per_trial)
        self.trial_time_limit = float(config.termination.trial_time_limit)
        self.max_steps = int(config.termination.max_steps)
        self.core = MarsRoverBatchEnv(int(num_envs), config)
        self.num_envs = self.core.num_envs
        self.obs_dim = self.core.obs_dim
        self.action_dim = self.core.action_dim
        self.obs = np.zeros((self.num_envs, self.obs_dim), dtype=np.float32)
        self.rewards = np.zeros((self.num_envs,), dtype=np.float32)
        self.terminated = np.zeros((self.num_envs,), dtype=np.uint8)
        self.truncated = np.zeros((self.num_envs,), dtype=np.uint8)
        self.terminated_bool = np.zeros((self.num_envs,), dtype=bool)
        self.truncated_bool = np.zeros((self.num_envs,), dtype=bool)
        self.episodes_seen_in_trial = np.zeros((self.num_envs,), dtype=np.int32)
        self.next_trial_start = np.ones((self.num_envs,), dtype=bool)

    def _trial_over(self, env_id: int) -> bool:
        if self.trial_time_limit > 0.0:
            return bool(self.core.trial_exhausted(int(env_id)))
        return bool(
            self.episodes_per_trial > 0
            and self.episodes_seen_in_trial[env_id] >= self.episodes_per_trial
        )

    def reset(self, seed: int = 0):
        self.core.reset_all(int(seed), self.obs)
        self.episodes_seen_in_trial.fill(1)
        if self.trial_time_limit > 0.0:
            self.next_trial_start.fill(False)
        else:
            self.next_trial_start.fill(0 < self.episodes_per_trial <= 1)
        return self.obs

    def reset_at(self, env_id: int, seed: int = 0, trial_start: bool = True):
        self.core.reset_at(int(env_id), int(seed), bool(trial_start), self.obs[int(env_id)])
        if trial_start:
            self.episodes_seen_in_trial[env_id] = 1
        else:
            self.episodes_seen_in_trial[env_id] += 1
        self.next_trial_start[env_id] = self._trial_over(env_id)
        if self.next_trial_start[env_id]:
            self.episodes_seen_in_trial[env_id] = 0
        return self.obs[int(env_id)]

    def step(self, actions: np.ndarray):
        actions = np.asarray(actions, dtype=np.int32)
        self.core.step(actions, self.obs, self.rewards, self.terminated, self.truncated)
        np.not_equal(self.terminated, 0, out=self.terminated_bool)
        np.not_equal(self.truncated, 0, out=self.truncated_bool)
        self._refresh_trial_flags()
        return self.obs, self.rewards, self.terminated_bool, self.truncated_bool, {}

    def _refresh_trial_flags(self) -> None:
        if self.trial_time_limit <= 0.0:
            return
        for env_id in range(self.num_envs):
            if self.terminated_bool[env_id] or self.truncated_bool[env_id]:
                self.next_trial_start[env_id] = bool(self.core.trial_exhausted(env_id))

    def step_uint8(self, actions: np.ndarray):

        actions = np.asarray(actions, dtype=np.int32)
        self.core.step(actions, self.obs, self.rewards, self.terminated, self.truncated)
        if self.trial_time_limit > 0.0:
            np.not_equal(self.terminated, 0, out=self.terminated_bool)
            np.not_equal(self.truncated, 0, out=self.truncated_bool)
            self._refresh_trial_flags()
        return self.obs, self.rewards, self.terminated, self.truncated, {}

    def debug_info(self, env_id: int) -> dict:
        return dict(self.core.debug_info(int(env_id)))
