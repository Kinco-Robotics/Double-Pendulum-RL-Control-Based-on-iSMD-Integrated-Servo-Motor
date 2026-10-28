# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import wrap_to_pi

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def joint_pos_target_l2(env: ManagerBasedRLEnv, target: float, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Penalize joint position deviation from a target value."""
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    # wrap the joint positions to (-pi, pi)
    joint_pos = wrap_to_pi(asset.data.joint_pos[:, asset_cfg.joint_ids])
    # compute the reward
    return torch.sum(torch.square(joint_pos - target), dim=1)


def rk_joint_alignment_reward(
    env: ManagerBasedRLEnv,
    linear_weight: float,
    exp_weight: float,
    target_joint_pos: list[float],
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    asset: Articulation = env.scene[asset_cfg.name]
    joint_pos = asset.data.joint_pos[:, asset_cfg.joint_ids]
    target_tensor = torch.tensor(target_joint_pos, device=joint_pos.device, dtype=joint_pos.dtype)
    error = torch.abs(wrap_to_pi(joint_pos - target_tensor.unsqueeze(0)))
    error = torch.clamp(error, 0.0, torch.pi)
    linear_reward = (torch.pi - error) / torch.pi
    exp_reward = torch.exp(-1.0 * error)
    combined = linear_reward * linear_weight + exp_reward * exp_weight
    return torch.mean(combined, dim=-1)


def rk_error_gated_vel_penalty(
    env: ManagerBasedRLEnv,
    error_threshold: float = 1,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    asset: Articulation = env.scene[asset_cfg.name]
    joint_pos = asset.data.joint_pos[:, asset_cfg.joint_ids]
    max_error = torch.max(torch.abs(wrap_to_pi(joint_pos)), dim=-1).values
    gate = torch.exp(-max_error / error_threshold)
    joint_vel = asset.data.joint_vel[:, asset_cfg.joint_ids]
    joint_vel = torch.nan_to_num(joint_vel, nan=0.0, posinf=0.0, neginf=0.0)
    penalty = torch.sum(torch.square(joint_vel), dim=1)
    return penalty * gate


def rk_joint_angle_penalty(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    angle_threshold: float = math.pi / 3,
) -> torch.Tensor:
    asset: Articulation = env.scene[asset_cfg.name]
    joint_pos = asset.data.joint_pos[:, asset_cfg.joint_ids]
    exceeded = torch.abs(wrap_to_pi(joint_pos)) > angle_threshold
    return torch.sum(exceeded.float(), dim=1)


def rk_joint1_speed_cap_penalty(
    env: ManagerBasedRLEnv,
    speed_limit: float = 4.7124,
    penalty_scale: float = 1.0,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    asset: Articulation = env.scene[asset_cfg.name]
    joint_vel = asset.data.joint_vel[:, asset_cfg.joint_ids]
    joint_vel = torch.nan_to_num(joint_vel, nan=0.0, posinf=0.0, neginf=0.0)
    j1_vel = torch.abs(joint_vel[:, 0])
    exceed = torch.clamp(j1_vel - speed_limit, min=0.0)
    return torch.square(exceed) * penalty_scale


def rk_joint_angle_deviation_penalty(
    env: ManagerBasedRLEnv,
    quad_scale: float = 1.0,
    angle_limit: float = math.pi,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    asset: Articulation = env.scene[asset_cfg.name]
    joint_pos = asset.data.joint_pos[:, asset_cfg.joint_ids]
    q = torch.abs(wrap_to_pi(joint_pos))
    q = torch.nan_to_num(q, nan=0.0, posinf=0.0, neginf=0.0)
    within = (q <= angle_limit).float()
    return quad_scale * torch.sum(q**2 * within, dim=-1)
