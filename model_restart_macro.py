"""PPO policy with a stateful, legal-macro stalled-engine recovery controller.

The local pilot uses raw controls at every physics frame; here the controller
uses the original macro indices at PPO decision intervals (frame_skip=8).
This is an adaptation, not a claim of identical local trajectory/score.
"""
from __future__ import annotations
import math
import torch
from torch import nn
from torch.nn import functional as F

class Policy(nn.Module):
    RECOVERY_ACTION = 8  # CLUTCH | DOWN
    GAS_ACTION = 1
    IGNITION_ACTION = 9
    STALLED_OBS = 109
    ENERGY_OBS = 6
    OVERHEAT_OBS = 113
    def __init__(self, obs_dim: int, action_dim: int, hidden_size: int):
        super().__init__()
        if hidden_size < 2:
            raise ValueError('hidden_size must be at least 2')
        self.obs_dim, self.action_dim, self.hidden_size = obs_dim, action_dim, hidden_size
        self.core_size = hidden_size - 1
        self.encoder = nn.Sequential(nn.Linear(obs_dim + action_dim + 3, self.core_size), nn.Tanh())
        self.memory = nn.GRUCell(self.core_size, self.core_size)
        self.actor = nn.Linear(self.core_size, action_dim)
        self.critic = nn.Linear(self.core_size, 1)
        for layer in (self.encoder[0], self.actor, self.critic):
            nn.init.orthogonal_(layer.weight, math.sqrt(2))
            nn.init.zeros_(layer.bias)
        nn.init.orthogonal_(self.actor.weight, .01)
        nn.init.orthogonal_(self.critic.weight, 1.)

    def initial(self, batch: int, device: torch.device) -> torch.Tensor:
        return torch.zeros(batch, self.hidden_size, device=device)

    def step(self, observation, previous_action, previous_reward, previous_done,
             trial_progress, trial_start, memory):
        reset = trial_start.float().unsqueeze(-1)
        memory = memory * (1. - reset)
        core, phase = memory[:, :-1], memory[:, -1]
        prev = previous_action.long()
        features = torch.cat((observation, F.one_hot(prev, self.action_dim).float(),
                              torch.tanh(previous_reward.unsqueeze(-1) / 10),
                              previous_done.unsqueeze(-1), trial_progress.unsqueeze(-1)), dim=-1)
        core = self.memory(self.encoder(features), core)
        logits = self.actor(core)
        stalled = observation[:, self.STALLED_OBS] > .5
        can_drive = (observation[:, self.ENERGY_OBS] > .07) & (observation[:, self.OVERHEAT_OBS] < .5)
        # Six policy decisions are approximately 48 simulator frames with frame_skip=8.
        # At the beginning of each stalled period: one downshift, then gas/release.
        active = stalled & can_drive
        shift = active & (phase < .5) & (prev != self.RECOVERY_ACTION)
        ignite = active & (phase > 2.5) & (phase < 3.5) & (prev != self.IGNITION_ACTION)
        # Strong logits override for recovery, while the normal PPO policy remains trainable.
        target = torch.where(shift, torch.full_like(prev, self.RECOVERY_ACTION),
                             torch.where(ignite, torch.full_like(prev, self.IGNITION_ACTION),
                                         torch.full_like(prev, self.GAS_ACTION)))
        override = F.one_hot(target, self.action_dim).to(logits.dtype)
        logits = torch.where(active.unsqueeze(-1), override * 30., logits)
        next_phase = torch.where(active, torch.remainder(phase + 1., 6.), torch.zeros_like(phase))
        new_memory = torch.cat((core, next_phase.unsqueeze(-1)), dim=-1)
        return logits, self.critic(core).squeeze(-1), new_memory

    def sequence(self, observation, previous_action, previous_reward, previous_done,
                 trial_progress, trial_start, memory):
        logits, values = [], []
        for i in range(len(observation)):
            current_logits, current_values, memory = self.step(
                observation[i], previous_action[i], previous_reward[i],
                previous_done[i], trial_progress[i], trial_start[i], memory)
            logits.append(current_logits)
            values.append(current_values)
        return torch.stack(logits), torch.stack(values)
