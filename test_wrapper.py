from env import PickPlaceEnv
import numpy as np
from wrapper import PickFailureWrapper, PerceptionFailureWrapper, ObjectGraspFailureWrapper, ObjectNotReachableWrapper

num_blocks = 4
num_bowls = 4
high_resolution = False
high_frame_rate = False

ALL_BLOCKS = ['blue block', 'red block', 'green block','yellow block']
ALL_BOWLS = ['blue bowl', 'yellow bowl', 'brown bowl', 'gray bowl']

env = PickPlaceEnv(render=True, high_res=high_resolution, high_frame_rate=high_frame_rate)
env = ObjectNotReachableWrapper(env)
# env = PickFailureWrapper(env)

# block_list = np.random.choice(ALL_BLOCKS, size=num_blocks, replace=False).tolist()
block_list = ['blue block', 'red block', 'green block','yellow block']
bowl_list = np.random.choice(ALL_BOWLS, size=num_bowls, replace=False).tolist()
obj_list = block_list + bowl_list
_ = env.reset(object_list=obj_list, 
              obj_which_fails='blue block')

# _ = env.reset(obj_list,
#               obj_which_fails = 'blue block',
#               obstructing_object = 'red block')

# Example to get pose:
print("get obj pose:", env.get_obj_pos(block_list[0]))
print("get obj pose:", env.get_obj_pos(bowl_list[0]))

obj_predicates = {obj_list[i]: {} for i in range(len(obj_list))}

# Get predicate values:
for count, obj in enumerate(obj_list):
  obj_predicates[obj]['is_on_table'] = env.on_table(obj)
  obj_predicates[obj]['is_clear'] = env.clear(obj)

# Check if gripper is empty:
print("is gripper empty: ", env.hand_empty())

print("obj predicates:", obj_predicates)

# Example to pick up object: (Argument is which object to pick)
success = env.pick(block_list[0])  
print("pick success: ", success)

# Example to place object: (Argument is where to place. No notion of which object to place because we assume the object is already in hand)
success = env.place(block_list[1])  
print("place success: ", success)

print("is clear block:", env.clear(block_list[1]))

# Example to place block om table: (No arguments. No notion of which object to place because we assume the object is already in hand)
success = env.pick(block_list[0])  
print("pick success: ", success)
env.putdown()  

success = env.pick('orange block')  
print("pick success: ", success)

print(env.locate('orange block'))