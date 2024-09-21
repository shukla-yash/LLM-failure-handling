import pybullet
from typing import Optional, List, Union
import numpy as np

class Wrapper:
    """
    Base class for all wrappers, adapted from Robosuite.

    Args:
        env: The environment to wrap.
    """

    def __init__(self, env):
        self.env = env

    @classmethod
    def class_name(cls):
        return cls.__name__

    def _warn_double_wrap(self):
        """
        Utility function that checks if we're accidentally trying to double wrap an env

        Raises:
            Exception: [Double wrapping env]
        """
        env = self.env
        while True:
            if isinstance(env, Wrapper):
                if env.class_name() == self.class_name():
                    raise Exception("Attempted to double wrap with Wrapper: {}".format(self.__class__.__name__))
                env = env.env
            else:
                break

    def reset(self, **kwargs):
        """reset the environment"""
        return self.env.reset(**kwargs)
    
    def movep(self, position):
        """Move to target end effector position."""
        return self.env.movep(position)
    
    def servoj(self, joints):
        """Move to target joint positions with position control."""
        return self.env.servoj(joints)

    def step_sim_and_render(self):
        """
        Step the simulation and render.
        """
        return self.env.step_sim_and_render()

    @property
    def unwrapped(self):
        """
        Grabs unwrapped environment

        Returns:
            env (MujocoEnv): Unwrapped environment
        """
        if hasattr(self.env, "unwrapped"):
            return self.env.unwrapped
        else:
            return self.env

    # this method is a fallback option on any methods the original env might support
    def __getattr__(self, attr):
        # using getattr ensures that both __getattribute__ and __getattr__ (fallback) get called
        # (see https://stackoverflow.com/questions/3278077/difference-between-getattr-vs-getattribute)
        orig_attr = getattr(self.env, attr)
        if callable(orig_attr):

            def hooked(*args, **kwargs):
                result = orig_attr(*args, **kwargs)
                # prevent wrapped_class from becoming unwrapped
                # NOTE: had to use "is" to prevent errors when returning numpy arrays from a wrapped method
                if result is self.env:
                    return self
                return result

            return hooked
        else:
            return orig_attr


class PickFailureWrapper(Wrapper):
    
    def __init__(self, env, 
                 ) -> None:
        super().__init__(env)
        self.halfExtents = [0.02, 0.02, 0.02]
    
    def reset(self, 
              object_list: List[str], 
              obj_which_fails: str, 
              obstructing_object:Optional[str] = None) -> None:
        
        '''
        Custom reset function to simulate the failure scenarios in the environment.
        args:
            object_list: list of objects to be placed in the environment
            obj_which_fails: object which fails to be picked or placed
            obstructing_object: object which obstructs the obj_which_fails
        '''

        if not obj_which_fails:
            print("obj_which_fails not provided, resetting the environment without any failures")
            return self.env.reset(object_list)

        self.env.reset(object_list)
        # move the obj_which_fails under another object or object obstructed
        fail_obj_id = self.env.obj_name_to_id[obj_which_fails]

        # check whether thr obj_which_fails and obstructing_object are present in the scene
        assert obj_which_fails in object_list, f"{obj_which_fails} not in object_list"
        if obstructing_object is not None:
            assert obstructing_object in object_list, f"{obstructing_object} not in object_list"
        
        object_type = obj_which_fails.split(' ')[1]

        if not obstructing_object:
            # randomly select an object to obstruct the obj_which_fails
            obstructing_object = np.random.choice([obj for obj in object_list if obj != obj_which_fails])
        
        # get the object id of the obstructing object
        obstructing_obj_id = self.env.obj_name_to_id[obstructing_object]

        # get the position of the obstructing object
        obstructing_obj_pos, obstructing_obj_orn = pybullet.getBasePositionAndOrientation(obstructing_obj_id)

        # get the initial orientation of the obj_which_fails
        fail_obj_pos, fail_obj_orn = pybullet.getBasePositionAndOrientation(fail_obj_id)

        # move the obstructing object up by its height
        obstructing_obj_pos_move_up = np.array(obstructing_obj_pos)
        obstructing_obj_pos_move_up[2] += self.halfExtents[2] * 2
        pybullet.resetBasePositionAndOrientation(obstructing_obj_id, obstructing_obj_pos_move_up, obstructing_obj_orn)

        # put the obj_which_fails under the obstructing object
        pybullet.resetBasePositionAndOrientation(fail_obj_id, obstructing_obj_pos, fail_obj_orn)

        # step the simulation
        for _ in range(10):
            pybullet.stepSimulation()
        
        return self.env.get_observation()

class PerceptionFailureWrapper(Wrapper):
    
    def __init__(self, env, 
                 ) -> None:
        super().__init__(env)

    def reset(self,
              perceptual_failure_objects: Union[List[str], str],
              object_list: List[str], 
              failure_type: str = "not visible") -> None: 

        if not perceptual_failure_objects:
            print("perceptual_failure_objects not provided, resetting the environment without any failures")
            return self.env.reset(object_list)

        self.env.reset(object_list)
        # hide the perceptual_failure_objects
        # check whether the perceptual_failure_objects are present in the scene
        if isinstance(perceptual_failure_objects, str):
            perceptual_failure_objects = [perceptual_failure_objects]

        for obj in perceptual_failure_objects:
            assert obj in object_list, f"{obj} not in object_list" 

        if failure_type == "not visible":
            for obj in perceptual_failure_objects:
                obj_id = self.env.obj_name_to_id[obj]
                pybullet.changeVisualShape(obj_id, -1, rgbaColor=[0, 0, 0, 0]) 
        elif failure_type == "not present":
            for obj in perceptual_failure_objects:
                obj_id = self.env.obj_name_to_id[obj]
                pybullet.removeBody(obj_id)
        else:
            raise ValueError(f"failure_type {failure_type} not supported, choose from ['not visible', 'not present']")
        
        # step the simulation
        for _ in range(10):
            pybullet.stepSimulation()

        return self.env.get_observation()


def place_failure(obj_which_fails):
    ## obj_which_fails is not clear    
    pass
