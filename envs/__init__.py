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