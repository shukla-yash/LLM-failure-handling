'''
LLM Agent serving as the interface between LLM and the environment. It builds and loads system prompt files from the environment, 
parses the plan, and creates RL subtasks when needed.
'''

from env import PickPlaceEnv
from wrapper import *
from apis.gpt4_apis import get_language_response, get_vision_response

class LLMAgent:
    def __init__(self, env: PickPlaceEnv | Wrapper, task: str, api_token: str):
        self._env = env 
        self._plan = [] 
        self._task = task
        self._current_plan_idx = 0
        self._system_prompts = ['initial_system_prompt.txt']
        self._user_prompts = []
        self._api_token = api_token
        
    def update_plan(self): 
        plan = get_language_response(
            system_prompts=self._system_prompts,
            user_prompts=self._user_prompts, 
            api_token=self._api_token
        )
        self.plan = eval(plan) # TODO: Replace this with primitives and structured responses 

    def step(self):
        operator = self._plan[self._current_plan_idx]
        if isinstance(operator, str):
            operator, arguments = operator.split('(')
            arguments = arguments[:-1]
            arguments = [arg.strip() for arg in arguments.split(',')]
        else:
            raise NotImplementedError
        operator = getattr(self._env, operator)
        if operator(*arguments):
            self._current_plan_idx += 1
            return True 
        else:
            return False

