from general_env import GeneralEnv
from interface import LMP_interface
from utils import * 


openai.api_key = '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'
num_blocks = 2
num_bowls = 1 
high_resolution = False
high_frame_rate = False


env = GeneralEnv(render=True, high_res=high_resolution, high_frame_rate=high_frame_rate, test=False)

block_list = ['blue block', 'red block']
bowl_list = ['blue bowl']

pickable_cfg = {
    'blue block': False,
    'red block': True,
    'blue bowl': True
}

obj_list = block_list + bowl_list
_ = env.reset(obj_list, pickable_cfg)
rng = np.random.default_rng()


user_input = 'get the blue block into the blue bowl' #

env.cache_video = []

keys = ['fgen', 'parse_obj_name', 'parse_position', 'parse_question', 'table_ui', 'transform_shape_pts']
prompts = keys
cfg_tabletop = build_cfg(keys, prompts)
lmp_tabletop_ui = setup_LMP(env, cfg_tabletop)
lmp_tabletop_ui(user_input, f'objects = {env.object_list}')

env.save_video('videos/test_lmp_push.gif') 
