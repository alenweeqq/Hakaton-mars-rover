from __future__ import annotations

import importlib
import os
import unittest

import numpy as np
import torch

from _mars_rover_cpp import (
    MarsRoverBatchEnv,
    __version__ as native_version,
    biome_bank_version,
    environment_version,
)
from mars_rover_env import __version__
from mars_rover_env.actions import ACTION_MACROS
from mars_rover_env.config import load_env_config
from model import Policy

EXPECTED_ACTION_MACROS = (
    0,
    1,
    1 << 1,
    1 | (1 << 2),
    (1 << 3) | (1 << 6),
    1 | (1 << 6),
    1 | (1 << 4) | (1 << 6),
    1 | (1 << 5) | (1 << 6),
    (1 << 3) | (1 << 7),
    1 << 9,
    1 | (1 << 4),
    1 | (1 << 5),
    1 << 11,
    1 << 8,
    1 << 10,
    1 << 13,
    1 << 14,
    1 << 15,
    1 << 16,
    1 << 17,
    1 << 18,
    1 << 19,
    1 << 20,
    1 << 21,
    1 << 22,
    1 << 23,
    1 | (1 << 23),
    1 << 12,
    1 | (1 << 12),
    1 | (1 << 4) | (1 << 12),
    1 | (1 << 5) | (1 << 12),
)


class SubmissionSmokeTest(unittest.TestCase):
    def test_versions_are_synchronized(self) -> None:
        self.assertEqual(__version__, "0.16.0")
        self.assertEqual(native_version, __version__)
        self.assertEqual(environment_version(), __version__)
        self.assertRegex(biome_bank_version(), r"^sha256:[0-9a-f]{64}$")

    def test_environment_contract_and_rollout(self) -> None:
        config = load_env_config()
        env = MarsRoverBatchEnv(2, config)

        self.assertEqual(env.obs_dim, 160)
        self.assertEqual(tuple(ACTION_MACROS), EXPECTED_ACTION_MACROS)
        self.assertEqual(len(ACTION_MACROS), 31)
        self.assertEqual(env.action_dim, 1 << 24)

        self.assertAlmostEqual(config.rig.body.mass, 18.0)
        self.assertAlmostEqual(config.rig.body.inertia, 6.0)
        self.assertEqual(len(config.rig.wheels), 2)
        for wheel in config.rig.wheels:
            self.assertAlmostEqual(wheel.mass, 1.4)

        observations = np.empty((2, env.obs_dim), dtype=np.float32)
        rewards = np.empty(2, dtype=np.float32)
        terminated = np.empty(2, dtype=np.uint8)
        truncated = np.empty(2, dtype=np.uint8)
        env.reset_all(2026, observations)
        self.assertTrue(np.isfinite(observations).all())

        actions = np.asarray([ACTION_MACROS[0], ACTION_MACROS[1]], dtype=np.int32)
        for _ in range(8):
            env.step(actions, observations, rewards, terminated, truncated)
            self.assertTrue(np.isfinite(observations).all())
            self.assertTrue(np.isfinite(rewards).all())

    def test_invalid_student_test_split_is_rejected(self) -> None:
        config = load_env_config()
        config.biome_split = 2
        with self.assertRaises((ValueError, RuntimeError)):
            MarsRoverBatchEnv(1, config)

    def test_recurrent_policy_shapes(self) -> None:
        batch = 3
        steps = 4
        policy = Policy(obs_dim=160, action_dim=len(ACTION_MACROS), hidden_size=32)
        memory = policy.initial(batch, torch.device("cpu"))
        logits, values, next_memory = policy.step(
            torch.zeros(batch, 160),
            torch.zeros(batch, dtype=torch.long),
            torch.zeros(batch),
            torch.zeros(batch),
            torch.zeros(batch),
            torch.ones(batch),
            memory,
        )
        self.assertEqual(tuple(logits.shape), (batch, len(ACTION_MACROS)))
        self.assertEqual(tuple(values.shape), (batch,))
        self.assertEqual(tuple(next_memory.shape), (batch, 32))
        logits, values = policy.sequence(
            torch.zeros(steps, batch, 160),
            torch.zeros(steps, batch, dtype=torch.long),
            torch.zeros(steps, batch),
            torch.zeros(steps, batch),
            torch.zeros(steps, batch),
            torch.zeros(steps, batch),
            memory,
        )
        self.assertEqual(tuple(logits.shape), (steps, batch, len(ACTION_MACROS)))
        self.assertEqual(tuple(values.shape), (steps, batch))

    def test_training_entrypoint_uses_platform_onnx_contract(self) -> None:
        try:
            train = importlib.import_module("train")
        except ModuleNotFoundError:
            if os.environ.get("MARS_ROVER_REQUIRE_PLATFORM") == "1":
                raise
            self.skipTest("arena-base platform modules are unavailable")
        self.assertIs(train.ppo.Agent, Policy)
        self.assertIs(train.ppo.save, train.save_weights)


if __name__ == "__main__":
    unittest.main()
