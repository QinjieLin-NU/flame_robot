import ray
from ray import tune
from ray.rllib.agents.ppo import PPOTrainer

def train(config, reporter):
    trainer = PPOTrainer(config=config, env="CartPole-v0")
    while True:
        result = trainer.train()
        reporter(**result)
        if result["episode_reward_mean"] > 200:
            task = 2
        elif result["episode_reward_mean"] > 100:
            task = 1
        else:
            task = 0
        # trainer.workers.foreach_worker(
            # lambda ev: ev.foreach_env(
                # lambda env: env.set_task(task)))

num_gpus = 0
num_workers = 2

ray.init(num_cpus=8)
tune.run(
    train,
    config={
        "num_gpus": num_gpus,
        "num_workers": num_workers,
        "lr": 0.0001,
    },
    resources_per_trial=tune.PlacementGroupFactory(
        [{"CPU": 1}] 
    ),
)