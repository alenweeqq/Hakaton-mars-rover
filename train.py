from __future__ import annotations

import os
import time
from pathlib import Path

import torch
from arena.protocol import export_agent_onnx
from vendor import meta_ppo as ppo

from model import Policy


started = time.monotonic()


def save_weights(path, model, optimizer, rms, config, frames):
    del path, optimizer, config, frames
    export_agent_onnx("/output/policy.onnx", model, rms)
    if time.monotonic() - started > 3_540:
        raise SystemExit(0)


ppo.Agent = Policy
ppo.save = save_weights


if __name__ == "__main__":
    torch.set_num_threads(int(os.environ.get("OMP_NUM_THREADS", "4")))
    config = ppo.Config(
        total_frames=int(os.environ.get("TOTAL_FRAMES", "1500000")),
        hidden_size=128,
        num_envs=64,
        rollout_steps=128,
        frame_skip=8,
        epochs=1,
        envs_per_batch=64,
        seed=int(os.environ.get("ARENA_SEED", "2026")),
        device="auto",
    )
    ppo.train(config, Path("/tmp/training"))
