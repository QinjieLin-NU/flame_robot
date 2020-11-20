import gym
import envs 
import time 

if __name__ == "__main__":
    env=gym.make("BipedalPyBulletEnv-v0")
    env.render()
    for i_episode in range(20):
        env.reset()
        for t in range(1000):
            env.render()
            action = env.action_space.sample()
            observation,reward,done,info = env.step(action)
            print(observation,reward,done,info)
            if done:
                break
            # time.sleep(0.5)
    env.close()
     