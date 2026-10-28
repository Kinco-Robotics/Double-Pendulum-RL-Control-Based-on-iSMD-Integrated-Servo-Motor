from __future__ import annotations

from typing import TYPE_CHECKING

import torch
import isaaclab.utils.math as math_utils
from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv


def rk_mirror_symmetric_reset(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor | None,
    asset_cfg: SceneEntityCfg,
    position_range: tuple[float, float],
    velocity_range: tuple[float, float],
):
    asset: Articulation = env.scene[asset_cfg.name]
    if env_ids is None:
        env_ids = torch.arange(env.scene.num_envs, device=asset.device)
    if isinstance(asset_cfg.joint_ids, slice):
        joint_ids = slice(None)
        n_joints = asset.num_joints
    else:
        joint_ids = list(asset_cfg.joint_ids)
        n_joints = len(joint_ids)
    n_envs = len(env_ids)
    pos_samples = math_utils.sample_uniform(
        *position_range, (n_envs, n_joints), device=asset.device
    )
    vel_samples = math_utils.sample_uniform(
        *velocity_range, (n_envs, n_joints), device=asset.device
    )
    sign = torch.where(
        env_ids % 2 == 0,
        torch.tensor(1.0, device=asset.device),
        torch.tensor(-1.0, device=asset.device),
    )
    pos_samples = pos_samples * sign.unsqueeze(-1)
    asset.write_joint_state_to_sim(
        pos_samples, vel_samples, joint_ids=joint_ids, env_ids=env_ids
    )
