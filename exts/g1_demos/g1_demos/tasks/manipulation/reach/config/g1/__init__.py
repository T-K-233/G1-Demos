# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import gymnasium as gym

from . import ik_abs_env_cfg, ik_rel_env_cfg, joint_pos_env_cfg
from .agents import rsl_rl_ppo_cfg

##
# Register Gym environments.
##

##
# Joint Position Control
##

gym.register(
    id="Reach-G1-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": joint_pos_env_cfg.G1ReachEnvCfg,
        "rsl_rl_cfg_entry_point": rsl_rl_ppo_cfg.G1ReachPPORunnerCfg,
    },
)
