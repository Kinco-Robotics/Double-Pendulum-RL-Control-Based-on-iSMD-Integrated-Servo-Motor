import math

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import UniformNoiseCfg

from . import mdp
from .ismd60d_robot_3dp_cfg import ISMD60D_DOUBLEPENDULUM_3DP_CFG

linear_weight = 10
exp_weight = 0


@configclass
class Ismd60dDoublependulum3dpSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(size=(100.0, 100.0)),
    )

    robot: ArticulationCfg = ISMD60D_DOUBLEPENDULUM_3DP_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=1000.0),
    )


@configclass
class ActionsCfg:
    joint_eff = mdp.JointEffortActionCfg(
        asset_name="robot",
        joint_names=["joint1"],
        scale=3.81,
        clip={"joint1": (-3.81, 3.81)},
    )


@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):

        history_length = 3

        joint_sincos = ObsTerm(
            func=mdp.rk_get_joint_sincos,
            noise=UniformNoiseCfg(n_min=-0.02, n_max=0.02),
            params={
                "asset_cfg": SceneEntityCfg("robot", joint_names=["joint1", "joint2"]),
            },
        )
        joint_vel = ObsTerm(
            func=mdp.joint_vel,
            noise=UniformNoiseCfg(n_min=-0.02, n_max=0.02),
            params={
                "asset_cfg": SceneEntityCfg("robot", joint_names=["joint1", "joint2"]),
            },
        )

        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self) -> None:
            self.enable_corruption = True
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    reset_link1_position = EventTerm(
        func=mdp.rk_mirror_symmetric_reset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=["joint1"]),
            "position_range": (0.0, 0.25 * math.pi),
            "velocity_range": (-0.5 * math.pi, 0.5 * math.pi),
        },
    )

    reset_link2_position = EventTerm(
        func=mdp.rk_mirror_symmetric_reset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=["joint2"]),
            "position_range": (0.0, math.pi),
            "velocity_range": (-0.5 * math.pi, 0.5 * math.pi),
        },
    )

    randomize_rigid_body_mass_link1 = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=["link1"]),
            "mass_distribution_params": (-0.05, +0.05),
            "operation": "add",
            "recompute_inertia": True,
        },
    )

    randomize_rigid_body_mass_link2 = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=["link2"]),
            "mass_distribution_params": (-0.01, +0.01),
            "operation": "add",
            "recompute_inertia": True,
        },
    )

    randomize_link1_com_positions = EventTerm(
        func=mdp.randomize_rigid_body_com,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=["link1"]),
            "com_range": {"x": (0.0, 0.0), "y": (-0.05, 0.05), "z": (-0.0, 0.0)},
        },
    )

    randomize_link2_com_positions = EventTerm(
        func=mdp.randomize_rigid_body_com,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=["link2"]),
            "com_range": {"x": (0.0, 0.0), "y": (-0.02, 0.02), "z": (-0.0, 0.0)},
        },
    )


@configclass
class TerminationsCfg:
    time_out = DoneTerm(func=mdp.time_out, time_out=True)

    angle_limit_link1 = DoneTerm(func=mdp.rk_joint_position_limit, params={
        "asset_cfg": SceneEntityCfg("robot", joint_names=["joint1"]),
        "limit": 0.44 * math.pi,
    })

    angle_limit_link2 = DoneTerm(func=mdp.rk_joint_position_limit, params={
        "asset_cfg": SceneEntityCfg("robot", joint_names=["joint2"]),
        "limit": 1.1 * math.pi,
    })


@configclass
class RewardsCfg:
    joint_alignment_reward = RewTerm(
        func=mdp.rk_joint_alignment_reward,
        weight=1.2,
        params={
            "linear_weight": linear_weight,
            "exp_weight": exp_weight,
            "target_joint_pos": [0.0, 0.0],
            "asset_cfg": SceneEntityCfg("robot", joint_names=["joint1", "joint2"]),
        },
    )

    joint_vel_penalty = RewTerm(
        func=mdp.rk_error_gated_vel_penalty,
        weight=-0.01,
        params={
            "error_threshold": 1,
            "asset_cfg": SceneEntityCfg("robot", joint_names=["joint1", "joint2"]),
        },
    )

    joint_angle_deviation_penalty = RewTerm(
        func=mdp.rk_joint_angle_deviation_penalty,
        weight=-1,
        params={
            "quad_scale": 1.0,
            "angle_limit": math.pi,
            "asset_cfg": SceneEntityCfg("robot", joint_names=["joint1", "joint2"]),
        },
    )

    joint2_angle_count_penalty = RewTerm(
        func=mdp.rk_joint_angle_penalty,
        weight=-1,
        params={
            "angle_threshold": math.pi / 3,
            "asset_cfg": SceneEntityCfg("robot", joint_names=["joint2"]),
        },
    )

    joint1_speed_cap_penalty = RewTerm(
        func=mdp.rk_joint1_speed_cap_penalty,
        weight=-0.15,
        params={
            "speed_limit": 4.7124,
            "penalty_scale": 1.0,
            "asset_cfg": SceneEntityCfg("robot", joint_names=["joint1"]),
        },
    )


@configclass
class Ismd60dDoublependulum3dpEnvCfg(ManagerBasedRLEnvCfg):
    scene: Ismd60dDoublependulum3dpSceneCfg = Ismd60dDoublependulum3dpSceneCfg(
        num_envs=4096,
        env_spacing=1.5,
    )

    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventCfg = EventCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    def __post_init__(self) -> None:
        self.decimation = 2
        self.episode_length_s = 6
        self.viewer.eye = (8.0, 0.0, 5.0)
        self.sim.dt = 1 / 180
        self.sim.render_interval = self.decimation
