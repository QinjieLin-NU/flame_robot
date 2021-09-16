import gym
import envs 
import time 
import numpy as np
if __name__ == "__main__":
    env=gym.make("BipedalPyBulletEnv-v12")
    env.render(mode="realtime")
    for i_episode in range(20):
        env.reset()
        for t in range(1000):
            action = env.action_space.sample()
            action = np.zeros(7,)
            observation,reward,done,info = env.step(action)
            # print(observation)
            # print(observation,reward,done,info)
            # if done:
                # break
            # time.sleep(0.01)
    env.close()
     