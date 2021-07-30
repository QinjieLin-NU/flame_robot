import numpy as np
import torch
import torch.nn as nn
import numpy as np
import torch
import gym
import argparse
import os
import time
import pybullet
import envs 

from TD3_control.TwinDelayed import TD3, device


# pybullet.connect(pybullet.DIRECT)

# env = gym.make('BipedalPyBulletEnv-v5')
env = gym.make('BipedalPyBulletEnv-v6')
env.render(mode="realtime")
file_prefix = "biped-v6-2d-17"#"biped-v5-2d-11"
# Set seeds
seed = 12345
env.seed(seed)
torch.manual_seed(seed)
np.random.seed(seed)

state_size = env.observation_space.shape[0]
action_size=env.action_space.shape[0]
action_high= float(env.action_space.high[0])
print('state_size: ', state_size, ', action_size: ', action_size, ', action_high: ', action_high)
    
agent = TD3(state_dim=state_size, action_dim=action_size, max_action=action_high)

def load(agent, dir, prefix):
    agent.actor.load_state_dict(torch.load(os.path.join(dir,'%s_actor.pth' % prefix)))
    agent.critic.load_state_dict(torch.load(os.path.join(dir,'%s_critic.pth' % prefix)))
    agent.actor_target.load_state_dict(torch.load(os.path.join(dir,'%s_actor_t.pth' % prefix)))
    agent.critic_target.load_state_dict(torch.load(os.path.join(dir,'%s_critic_t.pth' % prefix)))

def load_cpu(agent, dir, prefix):
    agent.actor.load_state_dict(torch.load(os.path.join(dir,'%s_actor.pth' % prefix), map_location='cpu'))
    agent.critic.load_state_dict(torch.load(os.path.join(dir,'%s_critic.pth' % prefix), map_location='cpu'))
    agent.actor_target.load_state_dict(torch.load(os.path.join(dir,'%s_actor_t.pth' % prefix), map_location='cpu'))
    agent.critic_target.load_state_dict(torch.load(os.path.join(dir,'%s_critic_t.pth' % prefix), map_location='cpu'))

from collections import deque
import os

std_noise = 0.1

def play(env, agent, n_episodes):
    state = env.reset()
    
    scores_deque = deque(maxlen=100)
    scores = []
    
    low = env.action_space.low
    high = env.action_space.high

    for i_episode in range(1, n_episodes+1):
        state = env.reset()
        score = 0
        
        time_start = time.time()
        done = False
        
        while True:
            action = agent.select_action(np.array(state))
            # env.render()
            # time.sleep(0.01)
            # print("action:",action,"state:",state)
            
            next_state, reward, done, _ = env.step(action)
            state = next_state
            score += reward
            if done:
                break 

        s = (int)(time.time() - time_start)
        
        scores_deque.append(score)
        scores.append(score)

        print('Episode {}\tAverage Score: {:.2f},\tScore: {:.2f} \tTime: {:02}:{:02}:{:02}'\
                  .format(i_episode, np.mean(scores_deque), score, s//3600, s%3600//60, s%60))  



load(agent, 'TD3_control/dir_Walker2D_002', file_prefix)
play(env, agent, n_episodes=100)

env.close()
