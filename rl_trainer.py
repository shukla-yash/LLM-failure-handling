from rl_env import PickPlaceRLEnv
from env import PickPlaceEnv
from wrapper import ObjectGraspFailureWrapper
import numpy as np
import pybullet
from PPO import PPO
import yaml
import os
from utils import logger
from pathlib import Path


class Trainer:
    '''
    A class that trains the agent using RL.
    '''
    def __init__(self, env, agent, params):
        self._env = env
        self._agent = agent
        self._params = params

        self.max_ep_len = params['env']['max_ep_len']                   # max timesteps in one episode
        self.max_training_timesteps = params['env']['max_training_timesteps']   # break training loop if timeteps > max_training_timesteps
        self.log_freq = params['env']['log_freq']           # log avg reward in the interval (in num timesteps)
        self.save_model_freq = params['env']['save_model_freq']          # save model frequency (in num timesteps)
        self.total_number_of_episodes = params['training']['total_number_of_episodes']
        self.update_episode = params['ppo']['update_episode']

    def train(self):
        '''
        Train the agent using RL.
        '''
        proj_path = os.path.dirname(os.path.abspath(__file__))
        log_path = proj_path + "/logs"
        log_path = Path(log_path)
        model_path = proj_path + "/models"
        my_logger = logger.Logger(path=log_path, run_name="test-1", step=0)
        total_episodes = 0
        max_epochs = 100
        done_arr = []
        current_episode_number = 0
        timesteps_in_current_iter = 0
        reward_arr = []

        for i in range(self.total_number_of_episodes):
            # one iteration of training loop
            state = self._env.reset()
            current_ep_reward = 0
            print_avg_reward = 0

            for t in range(1, self.max_ep_len+1):
                action = self._agent.select_action(state)
                state, reward, done, info = self._env.step(action)

                current_ep_reward += reward
                timesteps_in_current_iter +=1
                # saving reward and is_terminals
                self._agent.buffer.rewards.append(reward)
                if done or t == self.max_ep_len:
                    self._agent.buffer.is_terminals.append(True)
                    if done:
                        done_arr.append(1)
                    else:
                        done_arr.append(0)
                    reward_arr.append(current_ep_reward) 
                    my_logger.scalar("reward", current_ep_reward)               
                    print("Episode : {} \t\t Timestep : {} \t\t Average Reward : {}".format(i, t, np.mean(reward_arr[-50:])))
                else:
                    self._agent.buffer.is_terminals.append(False)

                # save model weights
                if timesteps_in_current_iter % self.save_model_freq == 0:
                    print("--------------------------------------------------------------------------------------------")
                    print("saving model at : " + "path")
                    self._agent.save(model_path)
                    print("model saved")
                    print("--------------------------------------------------------------------------------------------")

                if done or t == self.max_ep_len:
                    current_episode_number += 1
                    total_episodes += 1
                    # log if log_freq is reached
                    if i % self.log_freq == 0:
                        my_logger.write(step=total_episodes)
                    # update the agent
                    if total_episodes % self.update_episode == 0:
                        self._agent.update()
                    break
