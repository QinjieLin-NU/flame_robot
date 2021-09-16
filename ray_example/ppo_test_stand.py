import envs
from envs.bipedal_bullet_env_rl_flame8_a6 import BipedalBulletRLEnvFLAME8A6 
from envs.bipedal_bullet_env_rl_flame10_a6 import BipedalBulletRLEnvFLAME10A6 
from ray import tune
from ray.rllib.agents.ppo import PPOTrainer
from ray.tune.registry import register_env
import ray
import ray.rllib.agents.ppo as ppo
from ray.tune.logger import pretty_print
import gym
import time

env_name = 'BipedalPyBulletEnv-v12'
# instantiate env class
def env_creator(env_config):
    import envs
    return gym.make(env_name)  # return an env instance
register_env("my_env", env_creator)

#trainer config
ray.init()
config = ppo.DEFAULT_CONFIG.copy()
config["num_gpus"] = 0
config["num_workers"] = 1
config["explore"] = False
config["framework"] = "torch"
config["seed"] = 12345
config["num_sgd_iter"] = 20
config["kl_coeff"]=1.0
config["lr"] = 0.0001
config["sgd_minibatch_size"] =2560#32768
config["train_batch_size"] = 25600#320000
config["batch_mode"] = "complete_episodes"
config["observation_filter"]  = "MeanStdFilter"
trainer = ppo.PPOTrainer(config=config, env="my_env")
# restore_path = "example/model/stand3/checkpoint-5296"
# restore_path = "example/model/stand2/checkpoint-186" #0.02
restore_path = "example/model/stand1/checkpoint-8046"
restore_path = "/root/ray_results/PPO_my_env_2021-09-16_04-54-24y01_6msn/checkpoint_000241/checkpoint-241"#greate
restore_path = "/root/ray_results/PPO_BipedalPyBulletEnv-v12_2021-09-16_04-56-48fjb9h5fn/checkpoint_000231/checkpoint-231"#good
trainer.restore(restore_path) 

# run until episode ends
env = gym.make(env_name)
for i in range(100):
    episode_reward = 0
    done = False
    env.render()
    obs = env.reset()
    while not done:
        action = trainer.compute_action(obs)
        # print(action)
        obs, reward, done, info = env.step(action)
        episode_reward += reward
        time.sleep(0.1)
    print("episode reward:",episode_reward)
