from gym.envs.registration import register
# this is the base implementation od environment
register(
	id='BipedalPyBulletEnv-v0',
	entry_point='envs.bipedal_bullet_env:BipedalBulletEnv',
	max_episode_steps=1000,
	reward_threshold=950.0,
	)

# this is the implementation od environment for simbicon control
register(
	id='BipedalPyBulletEnv-v1',
	entry_point='envs.bipedal_bullet_env_simbicon:BipedalBulletSimbiconEnv',
	max_episode_steps=1000,
	reward_threshold=950.0,
	)

# this is the implementation od environment for RL control
register(
	id='BipedalPyBulletEnv-v2',
	entry_point='envs.bipedal_bullet_env_rl:BipedalBulletRLEnv',
	max_episode_steps=1000,
	reward_threshold=950.0,
	)
# action including 7 torques including crnter hip
register(
	id='BipedalPyBulletEnv-v3',
	entry_point='envs.bipedal_bullet_env_rl_a7:BipedalBulletRLEnvA7',
	max_episode_steps=1000,
	reward_threshold=950.0,
	)
# action including 7 torques including crnter hip, 2d dimension robot
register(
	id='BipedalPyBulletEnv-v4',
	entry_point='envs.bipedal_bullet_env_rl_a7_d2:BipedalBulletRLEnvA7D2',
	max_episode_steps=1000,
	reward_threshold=950.0,
	)

# action including 4 torques(upper leg, knee) , 2d dimension robot
register(
	id='BipedalPyBulletEnv-v5',
	entry_point='envs.bipedal_bullet_env_rl_a4_d2:BipedalBulletRLEnvA4D2',
	max_episode_steps=1000,
	reward_threshold=950.0,
	)

# action including 4 torques(upper leg, knee) , 2d dimension robot, not swing base 
register(
	id='BipedalPyBulletEnv-v6',
	entry_point='envs.bipedal_bullet_env_rl_flame5:BipedalBulletRLEnvFLAME5',
	max_episode_steps=1000,
	reward_threshold=950.0,
	)

# action including 4 torques(upper leg, knee) , 2d dimension robot, not swing base
register(
	id='BipedalPyBulletEnv-v7',
	entry_point='envs.bipedal_bullet_env_rl_flame7:BipedalBulletRLEnvFLAME7',
	max_episode_steps=1000,
	reward_threshold=950.0,
	)

# action including 4 torques(upper leg, knee) , 2d dimension robot, not swing base, revolute but not action ankle joint
register(
	id='BipedalPyBulletEnv-v8',
	entry_point='envs.bipedal_bullet_env_rl_flame8:BipedalBulletRLEnvFLAME8',
	max_episode_steps=1000,
	reward_threshold=950.0,
	)

# action including 6 torques(upper leg, knee) , 2d dimension robot, not swing base, action 6
register(
	id='BipedalPyBulletEnv-v9',
	entry_point='envs.bipedal_bullet_env_rl_flame8_a6:BipedalBulletRLEnvFLAME8A6',
	max_episode_steps=1000,
	reward_threshold=950.0,
	)

# action including 4 torques(upper leg, knee) , 3d dimension robot, not swing base, action 4
register(
	id='BipedalPyBulletEnv-v10',
	entry_point='envs.bipedal_bullet_env_rl_flame10:BipedalBulletRLEnvFLAME10',
	max_episode_steps=1000,
	reward_threshold=950.0,
	)
# action including 6 torques(upper leg, knee) , 3d dimension robot, not swing base, action6
register(
	id='BipedalPyBulletEnv-v11',
	entry_point='envs.bipedal_bullet_env_rl_flame10_a6:BipedalBulletRLEnvFLAME10A6',
	max_episode_steps=1000,
	reward_threshold=950.0,
	)

# action including 6 torques(upper leg, knee) , 3d dimension robot, not swing base, action6
register(
	id='BipedalPyBulletEnv-v12',
	entry_point='envs.bipedal_bullet_env_rl_flame10_stand:BipedalBulletRLEnvFLAME10Stand',
	max_episode_steps=1000,
	reward_threshold=950.0,
	)
