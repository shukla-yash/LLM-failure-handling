from rl_env import PickPlaceRLEnv
from env import PickPlaceEnv
from wrapper import ObjectGraspFailureWrapper
import numpy as np
import pybullet

num_blocks = 4
num_bowls = 4
high_resolution = False
high_frame_rate = False

ALL_BLOCKS = ['blue block', 'red block', 'green block','yellow block']
ALL_BOWLS = ['blue bowl', 'yellow bowl', 'brown bowl', 'gray bowl']

block_list = ['blue block', 'red block', 'green block','yellow block']
bowl_list = np.random.choice(ALL_BOWLS, size=num_bowls, replace=False).tolist()

obj_list = block_list + bowl_list

env = PickPlaceEnv(render=False, high_res=high_resolution, high_frame_rate=high_frame_rate)
wrapped_env = ObjectGraspFailureWrapper(env)

wrapped_env.reset(object_list=obj_list, 
              obj_which_fails='blue block')

states = wrapped_env.get_state()
print("get obj pose:", wrapped_env.get_obj_pos(block_list[0]))
print("get obj pose:", wrapped_env.get_obj_pos(bowl_list[0]))

block_list = ['blue block', 'red block', 'green block','yellow block']

wrapped_env.pick(block_list[0])
pybullet.disconnect()
env2 = PickPlaceEnv(render=False, high_res=high_resolution, high_frame_rate=high_frame_rate)
rl_env = PickPlaceRLEnv(env=env2, reset_to_state=True, state=states)
rl_env.reset()
print("get obj pose:", rl_env.get_obj_pos(block_list[0]))
print("get obj pose:", rl_env.get_obj_pos(bowl_list[0]))

ee_pose = rl_env.get_ee_pos()
new_ee_pose = np.array(ee_pose) + np.array([0.1, 0.1, 0.1])


total_number_of_actions_for_RL_env = rl_env.total_number_of_actions
state_space_dim = 3*len(obj_list)


for i in range(100):
    print("Step: ", i)
    rl_env.step(new_ee_pose)





