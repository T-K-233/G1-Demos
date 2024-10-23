# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from omni.isaac.lab.utils import configclass
from omni.isaac.lab.sensors import FrameTransformerCfg
from omni.isaac.lab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR


import g1_demos.tasks.manipulation.lift.mdp as mdp

##
# Pre-defined configs
##
from omni.isaac.lab.markers.config import FRAME_MARKER_CFG  # isort: skip
from omni.isaac.lab_assets import FRANKA_PANDA_CFG  # isort: skip
from omni.isaac.lab_assets import G1_MINIMAL_CFG  # isort: skip

from g1_demos.tasks.manipulation.lift.lift_env_cfg import LiftCubeEnvCfg

##
# Environment configuration
##


@configclass
class G1LiftCubeEnvCfg(LiftCubeEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # switch robot to franka
        # self.scene.robot = FRANKA_PANDA_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.robot = G1_MINIMAL_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        self.scene.robot.spawn.articulation_props.fix_root_link = True


        # override rewards
        # self.rewards.end_effector_position_tracking.params["asset_cfg"].body_names = ["right_palm_link"]
        # self.rewards.end_effector_position_tracking_fine_grained.params["asset_cfg"].body_names = ["right_palm_link"]
        # self.rewards.end_effector_orientation_tracking.params["asset_cfg"].body_names = ["right_palm_link"]

        # override actions
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot", joint_names=[
                "right_shoulder_pitch_joint",
                "right_shoulder_roll_joint",
                "right_shoulder_yaw_joint",
                "right_elbow_pitch_joint",
                "right_elbow_roll_joint",
                # "right_five_joint",
                "right_three_joint",
                "right_zero_joint",
            ], scale=0.5, use_default_offset=True
        )
        self.actions.gripper_action = mdp.BinaryJointPositionActionCfg(
            asset_name="robot",
            joint_names=["right_five_joint"],
            open_command_expr={"right_five_joint": 0.04},
            close_command_expr={"right_five_joint": 0.0},
        )

        # override command generator body
        # end-effector is along z-direction
        self.commands.object_pose.body_name = "right_palm_link"
        # self.commands.ee_pose.body_name = "right_palm_link"

        # Listens to the required transforms
        marker_cfg = FRAME_MARKER_CFG.copy()
        marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
        marker_cfg.prim_path = "/Visuals/FrameTransformer"
        
        self.scene.ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/pelvis",
            debug_vis=False,
            visualizer_cfg=marker_cfg,
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/right_palm_link",
                    name="end_effector",
                    offset=OffsetCfg(
                        pos=[0.0, 0.0, 0.1034],
                    ),
                ),
            ],
        )


@configclass
class G1LiftCubeEnvCfg_PLAY(G1LiftCubeEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False
