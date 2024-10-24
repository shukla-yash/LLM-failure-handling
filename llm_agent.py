'''
LLM Agent serving as the interface between LLM and the environment. It builds and loads system prompt files from the environment, 
parses the plan, and creates RL subtasks when needed.
'''

from env import PickPlaceEnv
from wrapper import *
from apis.gpt4_apis import get_language_response, get_vision_response

class LLMAgent:
    # def __init__(self, env: PickPlaceEnv | Wrapper, task: str, api_token: str):
    def __init__(self, env, task, api_token):

        self._env = env 
        self._plan = [] 
        self._task = task
        self._current_plan_idx = 0
        self._system_prompts = ['initial_system_prompt.txt', 'environment_belief.txt']
        self._user_prompts = ['user_tmp.txt']
        self._api_token = api_token
        
    def update_plan(self): 
        plan = get_language_response(
            system_prompts=self._system_prompts,
            user_prompts=self._user_prompts, 
            api_token=self._api_token
        )
        print("plan: ", plan)
        self._plan = eval(plan) # TODO: Replace this with primitives and structured responses 
        # self._plan = plan # TODO: Replace this with primitives and structured responses 

    def __len__(self):
        return len(self._plan)

    def step(self):
        operator = self._plan[self._current_plan_idx]
        if isinstance(operator, str):
            operator, arguments = operator.split('(')
            arguments = arguments[:-1]
            arguments = [arg.strip() for arg in arguments.split(',')]
        else:
            raise NotImplementedError
        operator = getattr(self._env, operator)
        print(operator)
        if operator(*arguments):
            operator(*arguments)
            self._current_plan_idx += 1
            return True 
        else:
            return False

