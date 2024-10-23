import gymnasium as gym

from . import agents, env_cfg
from .agents import rsl_rl_ppo_cfg

##
# Register Gym environments.
##


gym.register(
    id="Velocity-G1-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": env_cfg.G1EnvCfg,
        "rsl_rl_cfg_entry_point": rsl_rl_ppo_cfg.G1PPORunnerCfg,
    },
)

gym.register(
    id="Velocity-G1-Play-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": env_cfg.G1EnvCfg_PLAY,
        "rsl_rl_cfg_entry_point": rsl_rl_ppo_cfg.G1PPORunnerCfg,
    },
)


gym.register(
    id="Velocity-G1-LegOnly-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": env_cfg.G1LegOnlyEnvCfg,
        "rsl_rl_cfg_entry_point": rsl_rl_ppo_cfg.G1PPORunnerCfg,
    },
)

gym.register(
    id="Velocity-G1-LegOnly-Play-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": env_cfg.G1LegOnlyEnvCfg_PLAY,
        "rsl_rl_cfg_entry_point": rsl_rl_ppo_cfg.G1PPORunnerCfg,
    },
)

gym.register(
    id="Velocity-G1-Reach-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": env_cfg.G1ReachEnvCfg,
        "rsl_rl_cfg_entry_point": rsl_rl_ppo_cfg.G1PPORunnerCfg,
    },
)
