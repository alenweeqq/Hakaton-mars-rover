"""PPO policy with a modest, trainable recovery-action prior.

Uses the original 31-action table unchanged. This is not a direct port of the
local raw-bit heuristic: the competition acts through action macros and frame skip.
"""
from __future__ import annotations

import math
import torch
from torch import nn
from torch.nn import functional as F


class Policy(nn.Module):
    RECOVERY_ACTION = 8  # Original macro: CLUTCH | DOWN (136).
    STALLED_OBS = 109
    ENERGY_OBS = 6
    OVERHEAT_OBS = 113

    def __init__(self, obs_dim: int, action_dim: int, hidden_size: int):
        super().__init__()
        self.obs_dim = obs_dim
        self.action_dim = action_dim
        self.hidden_size = hidden_size
        self.encoder = nn.Sequential(
            nn.Linear(obs_dim + action_dim + 3, hidden_size), nn.Tanh(),
        )
        self.memory = nn.GRUCell(hidden_size, hidden_size)
        self.actor = nn.Linear(hidden_size, action_dim)
        self.critic = nn.Linear(hidden_size, 1)
        for layer in (self.encoder[0], self.actor, self.critic):
            nn.init.orthogonal_(layer.weight, math.sqrt(2))
            nn.init.zeros_(layer.bias)
        nn.init.orthogonal_(self.actor.weight, 0.01)
        nn.init.orthogonal_(self.critic.weight, 1.0)

    def initial(self, batch: int, device: torch.device) -> torch.Tensor:
        return torch.zeros(batch, self.hidden_size, device=device)

    def step(self, observation, previous_action, previous_reward, previous_done,
             trial_progress, trial_start, memory):
        memory = memory * (1 - trial_start.float().unsqueeze(-1))
        encoded_action = F.one_hot(previous_action.long(), self.action_dim).float()
        features = torch.cat((
            observation, encoded_action,
            torch.tanh(previous_reward.unsqueeze(-1) / 10),
            previous_done.unsqueeze(-1), trial_progress.unsqueeze(-1),
        ), dim=-1)
        memory = self.memory(self.encoder(features), memory)
        logits = self.actor(memory)
        # Prior, not a forced action. Suppress immediate repeated downshift pulses.
        # No Python branching on tensor values: stays compatible with ONNX tracing.
        recovery = (
            (observation[:, self.STALLED_OBS] > 0.5)
            & (observation[:, self.ENERGY_OBS] > 0.07)
            & (observation[:, self.OVERHEAT_OBS] < 0.5)
            & (previous_action.long() != self.RECOVERY_ACTION)
        ).to(logits.dtype)
        boost = F.one_hot(
            torch.full_like(previous_action.long(), self.RECOVERY_ACTION),
            self.action_dim,
        ).to(logits.dtype)
        logits = logits + 2.0 * recovery.unsqueeze(-1) * boost
        return logits, self.critic(memory).squeeze(-1), memory

    def sequence(self, observation, previous_action, previous_reward, previous_done,
                 trial_progress, trial_start, memory):
        logits, values = [], []
        for index in range(len(observation)):
            current_logits, current_values, memory = self.step(
                observation[index], previous_action[index], previous_reward[index],
                previous_done[index], trial_progress[index], trial_start[index], memory,
            )
            logits.append(current_logits)
            values.append(current_values)
        return torch.stack(logits), torch.stack(values)
