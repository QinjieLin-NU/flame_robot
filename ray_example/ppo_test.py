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

env_name = 'BipedalPyBulletEnv-v11'
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
# restore_path = "/root/ray_results/PPO_my_env_2021-08-12_19-51-40h75wmxfj/checkpoint_000101/checkpoint-101"
# restore_path = "/root/ray_results/PPO_my_env_2021-08-12_20-34-04u0ogc2pw/checkpoint_000201/checkpoint-201"
# restore_path = "/root/ray_results/PPO_my_env_2021-08-12_21-22-33k3b9h26n/checkpoint_000901/checkpoint-901"
# restore_path = "/root/ray_results/PPO_my_env_2021-08-17_16-52-10xw7fb_dm/checkpoint_000401/checkpoint-401" #v9 reach 1059 score
restore_path = "/root/ray_results/PPO_my_env_2021-08-22_16-47-27hbkjroyx/checkpoint_002351/checkpoint-2351" #** good result
restore_path = "/root/ray_results/PPO_my_env_2021-08-22_16-47-53sj4hb4b4/checkpoint_002551/checkpoint-2551" # walk 2 steps
restore_path = "/root/ray_results/PPO_my_env_2021-08-22_16-48-10wzv5tx55/checkpoint_002451/checkpoint-2451" #can walk
# restore_path = "/root/ray_results/PPO_my_env_2021-08-22_17-29-24u35kr8s1/checkpoint_002301/checkpoint-2301" #walk wiredly
restore_path = "/root/ray_results/PPO_my_env_2021-08-23_06-08-34lsdd50a8/checkpoint_001766/checkpoint-1766" #good
# restore_path = "/root/ray_results/PPO_my_env_2021-08-23_06-08-4838e_ayvb/checkpoint_001706/checkpoint-1706"
# restore_path = "/root/ray_results/PPO_my_env_2021-08-23_06-09-55bgyzobjy/checkpoint_001811/checkpoint-1811"
restore_path = "/root/ray_results/PPO_my_env_2021-08-23_06-08-34lsdd50a8/checkpoint_002996/checkpoint-2996" #good, walk normally
# restore_path = "/root/ray_results/PPO_my_env_2021-08-23_06-08-4838e_ayvb/checkpoint_002996/checkpoint-2996"
restore_path = "/root/ray_results/PPO_my_env_2021-08-23_22-00-43bnev_f58/checkpoint_002891/checkpoint-2891"
restore_path = "/root/ray_results/PPO_my_env_2021-08-23_22-01-05ubc_f77b/checkpoint_003186/checkpoint-3186"# walk several steps
restore_path = "/root/ray_results/PPO_my_env_2021-08-23_22-00-43bnev_f58/checkpoint_004676/checkpoint-4676" #walk normally
restore_path = "/root/ray_results/PPO_my_env_2021-08-23_22-00-43bnev_f58/checkpoint_008606/checkpoint-8606"
restore_path = "/root/ray_results/PPO_my_env_2021-08-23_22-00-43bnev_f58/checkpoint_009886/checkpoint-9886"
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
