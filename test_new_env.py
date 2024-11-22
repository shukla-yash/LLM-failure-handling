from rl_env import PickPlaceRLEnv
from env import PickPlaceEnv
from wrapper import ObjectNotReachableWrapper
import numpy as np
import pybullet
from PPO import PPO
import yaml
import os
from utils import logger
from pathlib import Path
import pybullet as p
import time

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


env = PickPlaceEnv(render=True, high_res=False, high_frame_rate=False)
wrapped_env = ObjectNotReachableWrapper(env)


wrapped_env.reset(object_list=obj_list, 
              obj_which_fails='blue block')

time.sleep(1)

wrapped_env.bring_closer()

while True:
    keys = p.getKeyboardEvents()
#   print(keys)
    time.sleep(0.01)