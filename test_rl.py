from rl_env import PickPlaceRLEnv
from env import PickPlaceEnv
from wrapper import ObjectGraspFailureWrapper
import numpy as np
import pybullet
from PPO import PPO
import yaml
import os

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
rl_env = PickPlaceRLEnv(env=env2, reset_to_state=True, state=states, 
                        type_of_failure="Object not pickup-able", 
                        target_object="blue block", 
                        object_list=obj_list)
rl_env.reset()
print("get obj pose:", rl_env.get_obj_pos(block_list[0]))
print("get obj pose:", rl_env.get_obj_pos(bowl_list[0]))

ee_pose = rl_env.get_ee_pos()
new_ee_pose = np.array(ee_pose) + np.array([0.1, 0.1, 0.1])


total_number_of_actions_for_RL_env = rl_env.total_number_of_actions
state_space_dim = 3*len(obj_list)

max_ep_len = params['env']['max_ep_len']                   # max timesteps in one episode
max_training_timesteps = params['env']['max_training_timesteps']   # break training loop if timeteps > max_training_timesteps
print_freq = params['env']['print_freq']        # print avg reward in the interval (in num timesteps)
log_freq = params['env']['log_freq']           # log avg reward in the interval (in num timesteps)
save_model_freq = params['env']['save_model_freq']          # save model frequency (in num timesteps)
action_std = params['env']['action_std']                    # starting std for action distribution (Multivariate Normal)
action_std_decay_rate = params['env']['action_std_decay_rate']        # linearly decay action_std (action_std = action_std - action_std_decay_rate)
min_action_std = params['env']['min_action_std']                # minimum action_std (stop decay after action_std <= min_action_std)
action_std_decay_freq = params['env']['action_std_decay_freq']  # action_std decay frequency (in num timesteps)
episodes_in_each_iter = params['training']['episodes_in_each_iter']
################ PPO hyperparameters ################
update_timestep = params['ppo']['update_timestep']      # update policy every n timesteps
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





while True:
    episodes_in_current_iter = 0
    timesteps_in_current_iter = 0
    done_arr = []
    reward_arr = []

    # training loop
    while episodes_in_current_iter <= episodes_in_each_iter:

        state = rl_env.reset()
        current_ep_reward = 0
        print_avg_reward = 0

        for t in range(1, max_ep_len+1):

            action = ppo_agent.select_action(state)
            # print("action: ", action)
            state, reward, done, info = rl_env.step(action)

            # saving reward and is_terminals
            ppo_agent.buffer.rewards.append(reward)
            if done:
                ppo_agent.buffer.is_terminals.append(True)
                done_arr.append(1)
                reward_arr.append(current_ep_reward)                
            else:
                ppo_agent.buffer.is_terminals.append(False)
                done_arr.append(0)
                reward_arr.append(current_ep_reward)

            current_ep_reward += reward
            timesteps_in_current_iter +=1

            # update PPO agent
            if timesteps_in_current_iter % update_timestep == 0:
                print("updating PPO agent")
                ppo_agent.update()

            # printing average reward
            if timesteps_in_current_iter % print_freq == 0:
                print("Episode : {} \t\t Timestep : {} \t\t Average Reward : {}".format(episodes_in_current_iter, timesteps_in_current_iter, np.mean(reward_arr[-50:])))

            # save model weights
            if timesteps_in_current_iter % save_model_freq == 0:
                print("--------------------------------------------------------------------------------------------")
                print("saving model at : " + "path")
                ppo_agent.save("path")
                print("model saved")
                print("--------------------------------------------------------------------------------------------")

            # break; if the episode is over
            if done:
                episodes_in_current_iter += 1
                break
        print("episode : ", episodes_in_current_iter, "reward : ", current_ep_reward)
        if len(done_arr) > 100 and np.mean(done_arr[-100:]) > 0.8:
            print("saving converged model at : " + "path")
            ppo_agent.save("path")
            is_task_leared =True
            break