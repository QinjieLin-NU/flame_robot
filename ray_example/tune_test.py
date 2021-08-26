import ray
from ray import tune
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
    return gym.make('BipedalPyBulletEnv-v11')   # return an env instance
register_env("my_env", env_creator)

def on_train_result(info):
    result = info["result"]
    trainer = info["trainer"]
    print("training")
    # trainer.workers.foreach_worker(
        # lambda ev: ev.foreach_env(
            # lambda env: env.set_task(task)))

ray.init()
tune.run(
    "PPO",
    config={
        "env": "my_env",
        "framework": "torch",
        "lr": tune.grid_search([0.0001, 0.001, 0.01]),
        "num_sgd_iter": tune.choice([10, 20]),
        "seed" : 12345,
        "kl_coeff" : 1.0 ,
        "sgd_minibatch_size" : 2560 ,
        "train_batch_size" : 25600,
        "batch_mode" : "complete_episodes",
        "observation_filter"  : "MeanStdFilter",
        "callbacks": {
            "on_train_result": on_train_result,
        },
    },
)