from typing import Any, SupportsFloat
from wrapper import Wrapper
from gymnasium import Env
from env import PickPlaceEnv
import os
import pybullet
from typing import Optional


class PickPlaceRLEnv(Env, Wrapper):
    '''
    The RL environment
    '''

    def __init__(self, 
                 env: PickPlaceEnv, 
                 reset_to_state: bool = False, 
                 state: Optional[dict] = None, 
                 object_list: Optional[list] = None) -> None:
        
        super().__init__(env=env)
        self.reset_to_state = reset_to_state

        if self.reset_to_state:
            assert state is not None, "State must be provided if reset_to_state is True"
            self.state = state
        else:
            assert object_list is not None, "Object list must be provided if reset_to_state is False"
            self.object_list = object_list
        
    def reset(self) -> dict:
        '''
        Reset the environment using the state_id
        '''

        if self.reset_to_state:
            object_list = list(self.state.keys())
            # reset the environment using object_list
            self.env.reset(object_list=object_list)

            # set the position and orientation of the objects
            for obj_name in object_list:
                obj_id = self.env.get_obj_id(obj_name)
                obj_pos = self.state[obj_name]['position']
                obj_orn = self.state[obj_name]['orientation']
                pybullet.resetBasePositionAndOrientation(obj_id, obj_pos, obj_orn)
        
        else:
            object_list = self.object_list
            self.env.reset(object_list=object_list)

        obs = self.get_observation()
        return obs

    def step(self, action: Any):

        # take the action
        self.env.movep(action)
        
        # step the simulation
        self.env.step_sim_and_render()

        # get the observation
        obs = self.get_observation()
        reward = self.reward()
        done = self.done()
        info = {}
        return obs, reward, done, info

    def reward(self):
        '''
        Get the reward
        '''
        return 0
    
    def done(self):
        '''
        Check if the episode is done
        '''
        return False
