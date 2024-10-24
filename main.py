from env import PickPlaceEnv
import numpy as np
from llm_agent import LLMAgent

num_blocks = 4
num_bowls = 4
high_resolution = False
high_frame_rate = False

block_list = ['blue block', 'red block', 'green block','yellow block']

env = PickPlaceEnv(render=True, high_res=high_resolution, high_frame_rate=high_frame_rate)

obj_list = block_list
_ = env.reset(obj_list)

env.pick(block_list[2])
env.place(block_list[2], block_list[0])
env.pick(block_list[3])
env.place(block_list[3], block_list[2])

llmagent = LLMAgent(env = env, task= 'Goal: red block on top of blue block', api_token='')
llmagent.update_plan()
plan_length = len(llmagent)
for i in range(plan_length):
  llmagent.step()