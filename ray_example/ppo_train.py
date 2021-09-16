import envs
from ray import tune
from ray.rllib.agents.ppo import PPOTrainer
from ray.tune.registry import register_env
import ray
import ray.rllib.agents.ppo as ppo
from ray.tune.logger import pretty_print
import gym
import pickle

#register env
def env_creator(env_config):
    import envs
    return gym.make('BipedalPyBulletEnv-v12')   # return an env instance
register_env("my_env", env_creator)

#trainer config
ray.init()
config = ppo.DEFAULT_CONFIG.copy()
config["num_gpus"] = 0
config["num_workers"] = 20
config["framework"] = "torch"
config["seed"] = 321
config["num_sgd_iter"] = 10 #20
config["kl_coeff"]=1.0
config["lr"] = 0.0001 #0.0001
config["sgd_minibatch_size"] = 2560 #32768
config["train_batch_size"] = 25600 #3200000
config["batch_mode"] = "complete_episodes"
config["observation_filter"]  = "MeanStdFilter"
trainer = ppo.PPOTrainer(config=config, env="my_env")

# Can optionally call trainer.restore(path) to load a checkpoint.
# Perform one iteration of training the policy with PPO
file_dir = "ray_example/log/"
reward_array = []
for i in range(10000):
    result = trainer.train()
    reward_array.append(result["episode_reward_mean"])
    print("iteration",result["training_iteration"]," reward_mean: ",result["episode_reward_mean"])

    if i % 5 == 0:
        checkpoint = trainer.save()
        print("checkpoint saved at", checkpoint)

        #save learning curve
        file_name = file_dir + checkpoint.split("/")[3] + ".pkl"
        with open(file_name, 'wb') as f:
                pickle.dump(reward_array, f)
                print("data saved at", file_name)

