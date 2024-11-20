from typing import Any, SupportsFloat
from functools import partial
from wrapper import Wrapper
from gymnasium import Env
from env import PickPlaceEnv
import os
import pybullet
from typing import Optional
import numpy as np

MAX_REWARD = 100
MAX_TIMESTEPS = 100

class PickPlaceRLEnv(Env, Wrapper):
    '''
    The RL environment
    '''

    def __init__(self, 
                 env: PickPlaceEnv, 
                 reset_to_state: bool = False, 
                 state: Optional[dict] = None,
                 type_of_failure: str = None,
                 target_object:str = None, 
                 object_list: Optional[list] = None) -> None:
        
        super().__init__(env=env)
        self.reset_to_state = reset_to_state
        self.env = env
        assert type_of_failure is not None, "Need to provide which operator has failed"
        self.type_of_failure = type_of_failure

        assert target_object is not None, "Need to provide which object has failed"
        self.target_object = target_object

        if self.reset_to_state:
            assert state is not None, "State must be provided if reset_to_state is True"
            self.state = state
            self.object_list = list(self.state.keys())
        else:
            assert object_list is not None, "Object list must be provided if reset_to_state is False"
            self.object_list = object_list
        
        total_number_of_objects = len(self.object_list)

        self.action_mapping = {}
        for i in range(total_number_of_objects):
            self.action_mapping[i] = partial(self.env.pick, self.object_list[i])

            # self.action_mapping[i+total_number_of_objects] = self.env.putdown(total_number_of_objects[i])
            # self.action_mapping[i+total_number_of_objects] = partial(self.env.putdown, self.object_list[i])
        self.action_mapping[i+1] = partial(self.env.putdown, "all objects")

        self.total_number_of_actions = len(self.action_mapping.keys())

    def reset(self) -> dict:
        '''
        Reset the environment using the state_id
        '''
        # print("environment resetting")
        if self.reset_to_state:
            object_list = list(self.state.keys())
            # reset the environment using object_list
            
            # TODO: add reset for other types of failures
            if self.type_of_failure == 'Object not pickup-able':
                self.env.set_skip_reset(True)
                self.env.reset(object_list=object_list, obj_which_fails=self.target_object)
            else:
                self.env.reset(object_list=object_list)

            # set the position and orientation of the objects
            for obj_name in object_list:
                obj_id = self.env.get_obj_id(obj_name)
                obj_pos = self.state[obj_name]['position']
                obj_orn = self.state[obj_name]['orientation']
                pybullet.resetBasePositionAndOrientation(obj_id, obj_pos, obj_orn)
        
        else:
            object_list = self.object_list
            if self.type_of_failure == 'Object not pickup-able':
                self.env.reset(object_list=object_list, obj_which_fails=self.target_object)
            else:
                self.env.reset(object_list=object_list)

        self.get_observation()
        obs = self.get_observation()
        self.episodic_timesteps = 0

        return obs

    def step(self, action: Any):

        # take the action
        action_name = self.action_mapping[action]
        function_name = action_name.func.__name__
        object_name = action_name.args[0]
        # print("ACTION IS: ", function_name, " ",  object_name)
        self.action_mapping[action]()
        
        # step the simulation
        self.env.step_sim_and_render()

        # get the observation
        obs = self.get_observation()
        reward, done = self.reward()

        self.episodic_timesteps += 1
        if self.episodic_timesteps > MAX_TIMESTEPS:
            done = True

        # done = self.done()
        info = {}
        return obs, reward, done, info

    def reward(self):
        '''
        Get the reward
        '''
        if self.type_of_failure == 'Object not pickup-able':
            return self.get_pickupable_reward()
        elif self.type_of_failure == 'Object not reachable':
            return self.get_reachable_reward()        
        elif self.type_of_failure == 'Object not visible':
            return self.get_visible_reward()        

    def get_pickupable_reward(self):
        if not self.env.hand_empty():
            robot_ee_pos = self.env.get_ee_pos()
            object_ee_pos = self.env.get_obj_pos(self.target_object)
            # xy_dist = np.linalg.norm(robot_ee_pos[:2] - object_ee_pos[:2])
            # TODO: modify this to make it more robust
            dist = np.linalg.norm(robot_ee_pos - object_ee_pos)
            if dist < 0.1:
                return MAX_REWARD, True
        return -1, False

    def get_reachable_reward(self):
        object_ee_pos = self.env.get_obj_pos(self.target_object)
        xy_dist = np.linalg.norm([0,0] - object_ee_pos[:2])
        if xy_dist < 0.5:
            return MAX_REWARD, True
        return -1, False
    
    def get_visible_reward(self): # TODO: Needs more checks
        if self.env.locate(self.target_object):
            return MAX_REWARD, True
        return -1, False        

    def get_observation(self):
        obj_pos = self.env.get_object_positions()
        obj_pos = np.array(obj_pos)[:, :2].flatten()   ## Ignore the 'z' axis
        # obj_pos = np.array(obj_pos).flatten()   ## Do not ignore the 'z' axis

        if self.env.hand_empty():
            obj_pos = np.append(obj_pos, 0)
            # obj_pos.append(0)
        else:
            obj_pos = np.append(obj_pos, 1)
            # obj_pos.append(1)
        return obj_pos