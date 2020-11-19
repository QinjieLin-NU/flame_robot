from gym.envs.registration import register
register(
	id='BipedalPyBulletEnv-v0',
	entry_point='envs.bipedal_bullet_env:BipedalBulletEnv',
	max_episode_steps=1000,
	reward_threshold=950.0,
	)