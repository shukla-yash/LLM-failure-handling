from rl_env import PickPlaceRLEnv
from env import PickPlaceEnv
from wrapper import ObjectGraspFailureWrapper
import numpy as np
import pybullet
from PPO import PPO
import yaml
import os
from utils import logger
from pathlib import Path

def read_one_block_of_yaml_data(filename):
    project_path = os.path.dirname(os.path.abspath(__file__))
    with open(f'{project_path}/{filename}.yaml','r') as f:
        output = yaml.safe_load(f)
    return output 
    
params = read_one_block_of_yaml_data('params')

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

env = PickPlaceEnv(render=False, high_res=high_resolution, high_frame_rate=high_frame_rate)
wrapped_env = ObjectGraspFailureWrapper(env)

wrapped_env.reset(object_list=obj_list, 
              obj_which_fails='blue block', 
              obstructing_object = ['orange block', 'red block', 'green block','yellow block'])

states = wrapped_env.get_state()
print("get obj pose:", wrapped_env.get_obj_pos(block_list[0]))
# print("get obj pose:", wrapped_env.get_obj_pos(bowl_list[0]))


wrapped_env.pick(block_list[0])
# pybullet.disconnect()
# env2 = PickPlaceEnv(render=False, high_res=high_resolution, high_frame_rate=high_frame_rate)
rl_env = PickPlaceRLEnv(env=wrapped_env, reset_to_state=True, state=states, 
                        type_of_failure="Object not pickup-able", 
                        target_object="blue block", 
                        object_list=obj_list)
rl_env.reset()
print("get obj pose:", rl_env.get_obj_pos(block_list[0]))
# print("get obj pose:", rl_env.get_obj_pos(bowl_list[0]))

# for debugging
rl_env.pick(block_list[0])
rl_env.putdown(block_list[0])
rl_env.pick(block_list[1])
rl_env.putdown(block_list[1])
rl_env.reset()
rl_env.pick(block_list[2])
rl_env.putdown(block_list[2])
rl_env.pick(block_list[3])
rl_env.putdown(block_list[3])
rl_env.pick(block_list[4])
rl_env.putdown(block_list[4])
rl_env.pick(block_list[0])
rl_env.putdown(block_list[0])

ee_pose = rl_env.get_ee_pos()
new_ee_pose = np.array(ee_pose) + np.array([0.1, 0.1, 0.1])


total_number_of_actions_for_RL_env = rl_env.total_number_of_actions
state_space_dim = 2*len(obj_list) + 1

max_ep_len = params['env']['max_ep_len']                   # max timesteps in one episode
max_training_timesteps = params['env']['max_training_timesteps']   # break training loop if timeteps > max_training_timesteps
print_freq = params['env']['print_freq']        # print avg reward in the interval (in num timesteps)
log_freq = params['env']['log_freq']           # log avg reward in the interval (in num timesteps)
save_model_freq = params['env']['save_model_freq']          # save model frequency (in num timesteps)
action_std = params['env']['action_std']                    # starting std for action distribution (Multivariate Normal)
action_std_decay_rate = params['env']['action_std_decay_rate']        # linearly decay action_std (action_std = action_std - action_std_decay_rate)
min_action_std = params['env']['min_action_std']                # minimum action_std (stop decay after action_std <= min_action_std)
action_std_decay_freq = params['env']['action_std_decay_freq']  # action_std decay frequency (in num timesteps)
total_number_of_episodes = params['training']['total_number_of_episodes']
################ PPO hyperparameters ################
update_timestep = params['ppo']['update_timestep']      # update policy every n timesteps
update_episode = params['ppo']['update_episode']       # update policy every n episodes
has_continuous_action_space = params['env']['has_continuous_action_space']  # continuous action space; else discrete
K_epochs = params['ppo']['K_epochs']          # update policy for K epochs in one PPO update
eps_clip = params['ppo']['eps_clip']       # clip parameter for PPO
gamma = params['ppo']['gamma']          # discount factor
lr_actor = params['ppo']['lr_actor']       # learning rate for actor network
lr_critic = params['ppo']['lr_critic']      # learning rate for critic network

ppo_agent = PPO(state_space_dim, total_number_of_actions_for_RL_env, lr_actor, lr_critic, gamma, K_epochs, eps_clip, has_continuous_action_space, action_std)

# for i in range(100):
#     print("Step: ", i)
#     rl_env.step(new_ee_pose)


proj_path = os.path.dirname(os.path.abspath(__file__))
log_path = proj_path + "/logs"
log_path = Path(log_path)
model_path = proj_path + "/models"
my_logger = logger.Logger(path=log_path, run_name="test-1", step=0)
total_episodes = 0
max_epochs = 100

# for _ in range(max_epochs):
current_episode_number = 0
timesteps_in_current_iter = 0
done_arr = []
reward_arr = []

# training loop
while current_episode_number <= total_number_of_episodes:

    state = rl_env.reset()
    current_ep_reward = 0
    print_avg_reward = 0

    for t in range(1, max_ep_len+1):

        action = ppo_agent.select_action(state)
        # print("action: ", action)
        state, reward, done, info = rl_env.step(action)

        current_ep_reward += reward
        timesteps_in_current_iter +=1

        # saving reward and is_terminals
        ppo_agent.buffer.rewards.append(reward)
        if done or t == max_ep_len:
            ppo_agent.buffer.is_terminals.append(True)
            if done:
                done_arr.append(1)
            else:
                done_arr.append(0)
            reward_arr.append(current_ep_reward) 
            my_logger.scalar("reward", current_ep_reward)               
            print("Episode : {} \t\t Timestep : {} \t\t Average Reward : {}".format(current_episode_number, timesteps_in_current_iter, np.mean(reward_arr[-50:])))
        else:
            ppo_agent.buffer.is_terminals.append(False)

        # save model weights
        if timesteps_in_current_iter % save_model_freq == 0:
            print("--------------------------------------------------------------------------------------------")
            print("saving model at : " + "path")
            ppo_agent.save(model_path)
            print("model saved")
            print("--------------------------------------------------------------------------------------------")


        # break; if the episode is over
        if done or t == max_ep_len:
            current_episode_number += 1
            total_episodes += 1
            # log if log_freq is reached
            if total_episodes % log_freq == 0:
                my_logger.write(step=total_episodes)
                        # update PPO agent
            if total_episodes % update_episode == 0:
                print("updating PPO agent")
                ppo_agent.update()
            break

    # check for convergence
    if len(reward_arr) > 100 and np.mean(done_arr[-100:]) > 0.85 and np.mean(reward_arr[-100:]) > 70 :
        print("saving converged model at : " + "path")
        ppo_agent.save("path")
        is_task_leared =True
        break