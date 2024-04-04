import cv2
import argparse

import numpy as np
import matplotlib.pyplot as plt 
import imageio

from pick_env import PickPlaceEnv
from push_env import PushEnv
from general_env import GeneralEnv
from constants import *


#@title Initialize Env { vertical-output: true }
num_blocks = 1 #@param {type:"slider", min:0, max:4, step:1}
num_bowls = 1 #@param {type:"slider", min:0, max:4, step:1}
high_resolution = False #@param {type:"boolean"}
high_frame_rate = False #@param {type:"boolean"}

parser = argparse.ArgumentParser()
parser.add_argument('--environment', '-e', default='push')
args = parser.parse_args()

# # setup env and LMP
if args.environment == 'push':
  env = PushEnv(render=True, high_res=high_resolution, high_frame_rate=high_frame_rate, test=True)
elif args.environment == 'general':
  env = GeneralEnv(render=True, high_res=high_resolution, high_frame_rate=high_frame_rate)
else:
  env = PickPlaceEnv(render=True, high_res=high_resolution, high_frame_rate=high_frame_rate)
block_list = np.random.choice(ALL_BLOCKS, size=num_blocks, replace=False).tolist()
bowl_list = np.random.choice(ALL_BOWLS, size=num_bowls, replace=False).tolist()
obj_list = block_list + bowl_list
_ = env.reset(obj_list)
# lmp_tabletop_ui = setup_LMP(env, cfg_tabletop)

print('-----------------------------------------------------')
print('Block list: ')
print(block_list)
print('=====================================================')
print('Bowl list:')
print(bowl_list)
print('-----------------------------------------------------')
rng = np.random.default_rng()

print(f'Pushing {block_list[0]} to {bowl_list[0]}')

print("get obj pose:", env.get_obj_pos(block_list[0]))
print("get obj pose:", env.get_obj_pos(bowl_list[0]))

for i in range(1):
  if args.environment != 'push':
    act = {'pick': env.get_obj_pos(block_list[0]), 'place': env.get_obj_pos(bowl_list[0])}
  else:
    act = {'obj': env.get_obj_pos(block_list[0]), 'destination': env.get_obj_pos(bowl_list[0])}
  env.step(act)

if args.environment == 'push':
  env.save_video('videos/push.gif')
else:
  env.save_video('videos/pick.gif')

# display env
img = cv2.cvtColor(env.get_camera_image(), cv2.COLOR_BGR2RGB)


plt.title('ViLD Input Image')
plt.imshow(img)
plt.show()
imageio.imwrite('tmp.jpg', img)
print("Printed image")