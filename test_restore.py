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


env = PickPlaceEnv(render=False, high_res=high_resolution, high_frame_rate=high_frame_rate)

filename = 'test_state.bullet'

rl_env = PickPlaceRLEnv(env=env, filename=filename)
ee_pose = rl_env.get_ee_pos()
new_ee_pose = np.array(ee_pose) + np.array([0.1, 0.1, 0.1])

rl_env.reset()
for i in range(100):
    print("Step: ", i)
    rl_env.step(new_ee_pose)
