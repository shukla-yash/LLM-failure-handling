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
from rl_trainer import Trainer
from utils.file_utils import read_one_block_of_yaml_data

class PlanningAgent:
    '''
    A agent that executes a hard-coded plan and launch RL if task is not successful.
    '''

    def __init__(self, env,  plan: list, params: dict):
        self._env = env
        self._plan = plan
        self._current_plan_idx = 0
        self._params = params
        self.env_state = None


    def execute_plan_step(self):
        opertation = self._plan[self._current_plan_idx]

        # parse the operation
        if isinstance(opertation, str):
            opertation, arguments = opertation.split('(')
            arguments = arguments[:-1]
            arguments = [arg.strip() for arg in arguments.split(',')]
        else:
            raise NotImplementedError
        
        # execute the operation
        operation_name = opertation
        opertation = getattr(self._env, opertation)
        
        print("Executing operation: ", operation_name)
        print("Arguments: ", arguments)
        if opertation(*arguments):
            action_success = opertation(*arguments)
            self._current_plan_idx += 1
            if action_success:
                return True
            else:
                return False
        else:
            return False
    
    def execute_plan(self):
        while self._current_plan_idx < len(self._plan):
            # get the env state before executing a step
            self.env_state = self._env.get_state()
            if not self.execute_plan_step():
                print("Failed to execute plan on step: ", self._plan[self._current_plan_idx])
                return False
        return True
    
    def initialize_rl(self):
        print("Initializing RL")
        self._rl_env = PickPlaceRLEnv(env=self._env, reset_to_state=True, state=self.env_state,
                                        type_of_failure="Object not pickup-able",
                                        target_object="blue block",
                                        object_list=obj_list)
        
        params = self._params


        # get the parameters for the RL agent
        action_std = params['env']['action_std']                    # starting std for action distribution (Multivariate Normal)
        action_std_decay_rate = params['env']['action_std_decay_rate']        # linearly decay action_std (action_std = action_std - action_std_decay_rate)
        ################ PPO hyperparameters ################
        update_timestep = params['ppo']['update_timestep']      # update policy every n timesteps
        update_episode = params['ppo']['update_episode']       # update policy every n episodes
        has_continuous_action_space = params['env']['has_continuous_action_space']  # continuous action space; else discrete
        K_epochs = params['ppo']['K_epochs']          # update policy for K epochs in one PPO update
        eps_clip = params['ppo']['eps_clip']       # clip parameter for PPO
        gamma = params['ppo']['gamma']          # discount factor
        lr_actor = params['ppo']['lr_actor']       # learning rate for actor network
        lr_critic = params['ppo']['lr_critic']      # learning rate for critic network


        state_space_dim = self._rl_env.state_space_dim
        total_number_of_actions_for_RL_env = self._rl_env.action_dim
        ppo_agent = PPO(state_space_dim, total_number_of_actions_for_RL_env, lr_actor, lr_critic, gamma, K_epochs, eps_clip, has_continuous_action_space, action_std)
        
        trainer = Trainer(self._rl_env, ppo_agent, params)
        trainer.train()

if __name__ == "__main__":
    num_blocks = 4
    num_bowls = 4
    high_resolution = False
    high_frame_rate = False

    ALL_BLOCKS = ['blue block', 'red block', 'green block','yellow block', 'orange block']
    ALL_BOWLS = ['blue bowl', 'yellow bowl', 'brown bowl', 'gray bowl']

    block_list = ['blue block', 'red block', 'green block','yellow block', 'orange block']
    bowl_list = np.random.choice(ALL_BOWLS, size=num_bowls, replace=False).tolist()
    bowl_list = []

    obj_list = block_list + bowl_list

    env = PickPlaceEnv(render=False, high_res=False, high_frame_rate=False)

    wrapped_env = ObjectGraspFailureWrapper(env)
    wrapped_env.reset(object_list=obj_list, 
              obj_which_fails='blue block')

    # load the plan from plan.txt
    plan = []
    with open("plan.txt", "r") as f:
        for line in f:
            plan.append(line.strip())

    # get the parameters for the RL agent
    params = read_one_block_of_yaml_data("params")

    agent = PlanningAgent(wrapped_env, plan, params)
    success =agent.execute_plan()
    if not success:
        agent.initialize_rl()
