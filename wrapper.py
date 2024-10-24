import pybullet
from typing import Optional, List, Union
import numpy as np
from scipy.spatial.transform import Rotation as R

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
        '''
        Custom reset function to simulate the failure scenarios in the environment.
        args:
            perceptual_failure_objects: object which fails to be perceived
            object_list: list of objects to be placed in the environment
            failure_type: type of failure, choose from ['not visible', 'not present']
        '''
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
                # remove the object from the object_list
                self.env.object_list.remove(obj)
                # remove from obj_name_to_id
                self.env.obj_name_to_id.pop(obj)
        
        else:
            raise ValueError(f"failure_type {failure_type} not supported, choose from ['not visible', 'not present']")
        print("env obj list:", self.env.object_list)
        # step the simulation
        for _ in range(10):
            pybullet.stepSimulation()

        return self.env.get_observation()

    def get_obj_pos(self, obj_name):
        from env import CORNER_POS
        obj_name = obj_name.replace('the', '').replace('_', ' ').strip()
        if obj_name in CORNER_POS:
            position = np.float32(np.array(CORNER_POS[obj_name]))
        elif obj_name in self.env.obj_name_to_id.keys():
            pick_id = self.get_obj_id(obj_name)
            pose = pybullet.getBasePositionAndOrientation(pick_id)
            position = np.float32(pose[0])
        else:
            print(f"get_obj_pos: {obj_name} not in the scene")
            position = None
        return position

class PlaceFailureWrapper(Wrapper):
    
    def __init__(self, env, 
                 ) -> None:
        super
    
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

class ObjectGraspFailureWrapper(Wrapper):

    '''
    Object blocked by other blocks in all four sides
    '''
    
    def __init__(self, env, 
                 ) -> None:
        super().__init__(env)
        self.halfExtents_cube = [0.02, 0.02, 0.02]
        self.halfExtents_bowl = [0.04, 0.04, 0.04]
    
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

        # if obstructing_object is not provided, randomly select 4 objects to obstruct the obj_which_fails
        if len(obstructing_object) != 4:
            print("not enough obstructing objects provided, randomly selecting 4 objects")
            obstructing_object = np.random.choice([obj for obj in object_list if obj != obj_which_fails], 4)

        # check whether thr obj_which_fails and obstructing_object are present in the scene
        assert obj_which_fails in object_list, f"{obj_which_fails} not in object_list"
        for obj in obstructing_object:
            assert obj in object_list, f"{obj} not in object_list"
        
        # get the object id of the obstructing object
        obstructing_obj_ids = [self.env.obj_name_to_id[obj] for obj in obstructing_object]

        # get the position and orientation of the obj_which_fails
        fail_obj_id = self.env.obj_name_to_id[obj_which_fails]
        fail_obj_pos, fail_obj_orn = pybullet.getBasePositionAndOrientation(fail_obj_id)
        fail_obj_pos = np.array(fail_obj_pos)


        # calculate the positions of the obstructing objects
        fail_obj_rot = R.from_quat(fail_obj_orn)
        fail_obj_rot = fail_obj_rot.as_matrix()

        # calculate the positions of the obstructing objects
        displacement_object_frame = np.array([[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0]], dtype=float)
        # displacement_object_frame *= self.halfExtents[0] * 2
        for idx in range(4):
            if obstructing_object[idx].split(' ')[1] == 'block':
                displacement_object_frame[idx] *= self.halfExtents_cube[0] * 2
            elif obstructing_object[idx].split(' ')[1] == 'bowl':
                displacement_object_frame[idx] *= self.halfExtents_bowl[0] * 2
            else:
                raise ValueError(f"object type {obstructing_object[idx].split(' ')[1]} not supported")
        # calculate the positions of the obstructing objects
        obstructing_obj_pos = [fail_obj_pos + np.dot(fail_obj_rot, displacement) for displacement in displacement_object_frame]
        print("obstructing_obj_pos:", obstructing_obj_pos)
        # move the obstructing objects to the calculated positions
        for obj_id, pos in zip(obstructing_obj_ids, obstructing_obj_pos):
            pybullet.resetBasePositionAndOrientation(obj_id, pos, fail_obj_orn)

        # step the simulation
        for _ in range(10):
            pybullet.stepSimulation()
        return self.env.get_observation()

class ObjectNotReachableWrapper(Wrapper):

    '''
    Object placed at unreachable position
    '''
    
    def __init__(self, env, 
                 ) -> None:
        super().__init__(env)
        self.robot_working_distance = 0.75
        self.object_robot_distance_upper_limit = 1.0

    def reset(self,
              object_list: List[str],
              obj_which_fails: str):
        '''
        Custom reset function to simulate the failure scenarios in the environment.
        args:
            obj_which_fails: object which is placed at unreachable position
        '''
        if not obj_which_fails:
            print("obj_which_fails not provided, resetting the environment without any failures")
            return self.env.reset(object_list)

        self.env.reset(object_list)

        assert obj_which_fails in self.env.object_list, f"{obj_which_fails} not in object_list"

        # get the object id of the obj_which_fails
        fail_obj_id = self.env.obj_name_to_id[obj_which_fails]

        # put the obj_which_fails at an unreachable position
        radius = np.random.uniform(self.robot_working_distance, self.object_robot_distance_upper_limit)
        angle = np.random.uniform(np.pi, 2*np.pi)
        x = radius * np.cos(angle)
        y = radius * np.sin(angle)

        # get the initial position and orinetation of the obj_which_fails
        fail_obj_pos, fail_obj_orn = pybullet.getBasePositionAndOrientation(fail_obj_id)

        # move the obj_which_fails to the unreachable position
        unreachable_pos = np.array([x, y, fail_obj_pos[2]])
        pybullet.resetBasePositionAndOrientation(fail_obj_id, unreachable_pos, fail_obj_orn)

        # step the simulation
        for _ in range(10):
            pybullet.stepSimulation()
        
        return self.env.get_observation()

