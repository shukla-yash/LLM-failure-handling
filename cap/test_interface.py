import numpy as np

from cap.interface import LMP_interface
from pick_env import PickPlaceEnv
from constants import *


num_blocks = 1
num_bowls = 1
high_resolution = False
high_frame_rate = False


env = PickPlaceEnv(render=True, high_res=high_resolution, high_frame_rate=high_frame_rate)
block_list = np.random.choice(ALL_BLOCKS, size=num_blocks, replace=False).tolist()
bowl_list = np.random.choice(ALL_BOWLS, size=num_bowls, replace=False).tolist()
obj_list = block_list + bowl_list
_ = env.reset(obj_list)

interface = LMP_interface(env, None)
