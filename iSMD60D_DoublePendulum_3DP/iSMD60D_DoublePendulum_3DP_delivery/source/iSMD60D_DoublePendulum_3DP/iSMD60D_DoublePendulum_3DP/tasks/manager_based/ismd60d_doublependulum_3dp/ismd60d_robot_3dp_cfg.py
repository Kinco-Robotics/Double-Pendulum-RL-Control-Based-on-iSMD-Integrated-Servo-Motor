import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import DelayedPDActuatorCfg, ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.normpath(os.path.join(_THIS_DIR, *[".."] * 6))

ISMD60D_USD_PATH = os.path.join(_PROJECT_ROOT, "models/iSMD60D_DoublePendulum/iSMD60D_DoublePendulum.usd")

ISMD60D_DOUBLEPENDULUM_3DP_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=ISMD60D_USD_PATH,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=100.0,
            enable_gyroscopic_forces=True,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
            sleep_threshold=0,
            stabilization_threshold=0,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.5),
        joint_pos={
            "joint1": 0.0,
            "joint2": 0.0,
        },
    ),
    actuators={
        "base_actuator": DelayedPDActuatorCfg(
            joint_names_expr=["joint1"],
            effort_limit=3.81,
            stiffness=0.0,
            damping=0.0,
            min_delay=1,
            max_delay=8,
        ),
        "flex_actuator": ImplicitActuatorCfg(
            joint_names_expr=["joint2"],
            stiffness=0.0,
            damping=0.0,
            friction=0.0001,
        ),
    },
)
