import base64
import requests
import os

from openai import OpenAI


# Create a local Qdrant vector store

# Create the MultiModal index

def build_prompts(prompts):
    path = os.path.dirname(os.path.realpath(__file__))
    # print(path)
    prompt = ''
    for prompt_file in prompts:
        with open(path+'/prompts/' + prompt_file, 'r') as f:
            prompt += f.read() 
            prompt += '\n\n\n' 

    return prompt

def build_system_prompts(prompts):
    path = os.path.dirname(os.path.realpath(__file__))
    # print(path)
    prompt = ''
    for prompt_file in prompts:
        with open(path+'/prompts/system_prompt/' + prompt_file, 'r') as f:
            prompt += f.read() 
            prompt += '\n\n\n' 

    return prompt

def build_user_prompts(prompts):
    path = os.path.dirname(os.path.realpath(__file__))
    # print(path)
    prompt = ''
    for prompt_file in prompts:
        with open(path+'/prompts/user_prompts/' + prompt_file, 'r') as f:
            prompt += f.read() 
            prompt += '\n\n\n' 

    return prompt


def encode_image(image_path):
    path = os.path.dirname(os.path.realpath(__file__))
    with open(path+'/'+image_path, "rb") as image_file:
      return base64.b64encode(image_file.read()).decode('utf-8')
    

def get_vision_response(system_prompts, user_prompts: list, images: str, api_token: str):
    user_prompt = build_prompts(user_prompts) 
    # print('user prompt: ', user_prompt)
    system_prompt = build_system_prompts(system_prompts) 
    # print('system prompt: ', system_prompt)
    # client = OpenAI() 

    base64_image = encode_image('images/' + images) 
    headers = {
      "Content-Type": "application/json",
      "Authorization": f"Bearer {api_token}"
    }

    payload = {
      'model': "gpt-4o",
      'messages': [
        {
          "role": "user",
          "content": [
            {
                "type": "text", 
                "text": user_prompt
            },
            {
              "type": "image_url",
              "image_url": {
                "url": f"data:image/jpeg;base64,{base64_image}"
                }
            },
          ],
        }, 
        {
            'role': 'system', 
            'content': [
                {
                    'type': 'text', 
                    'text': system_prompt
                }
          ]
        }
      ],
      'max_tokens':300,
    }

    response = requests.post("https://api.openai.com/v1/chat/completions", headers=headers, json=payload)

    return response.json()['choices'][0]['message']['content']

def get_language_response(system_prompts, user_prompts: list, api_token: str):
    user_prompt = build_prompts(user_prompts) 
    # print('user prompt: ', user_prompt)
    system_prompt = build_system_prompts(system_prompts) 

    print('system prompt: ', system_prompt)
    # client = OpenAI() 

    # base64_image = encode_image('images/' + images) 
    headers = {
      "Content-Type": "application/json",
      "Authorization": f"Bearer {api_token}"
    }

    payload = {
      'model': "gpt-4o",
      'messages': [
        {
          "role": "user",
          "content": [
            {
                "type": "text", 
                "text": user_prompt
            },
          ],
        }, 
        {
            'role': 'system', 
            'content': [
                {
                    'type': 'text', 
                    'text': system_prompt
                }
          ]
        }
      ],
      'max_tokens':300,
    }

    response = requests.post("https://api.openai.com/v1/chat/completions", headers=headers, json=payload)
    # print(response.json())
    return response.json()['choices'][0]['message']['content']

if __name__ == '__main__':
    system_prompts = ['failure_system_prompt.txt', 'environment_belief.txt']
    user_prompts = ['user_tmp.txt'] 
    images = 'test.jpg' 

    print(get_language_response(system_prompts, user_prompts)
    
    ## right now lets just assume that we only need LLM (no images)

    """
    List of prompts:
    1. Initial prompt:
        Goal in NL (pick up blue block and stack it on red block) - `goal_nl'
        List of Operator APIs available to the robot agent (reach; locate; pick up; place; stack)
        Initial belief of the world (query from the env to get the probabilistic predicates)
        (blue_block on table - 80%)
        (red block clear - 50 %) ..... 
      only output the plan in bullet points. do not write anything else
    Return: Plan (sequence of APIs) - NL (og_plan)
    Parse that plan and get a list of API(parameter calls)

    2. Failure prompt (just one):
    when an API fails, it returns some information -> x (we are doing multiple experiments; in one exp; it just returns true/false; in another it returns the reason it failed)
    you take that returned info and ask the LLM to replan
    replanning prompt:
    x

    The above operator has failed. The original plan was `og_plan`. It failed at the 'third operator'
    Now replan from the current failed state to achieve the same goal of `goal_nl' 

    Return: Plan (sequence of APIs) from failed state - 1 - NL (og_plan)
    Parse that plan and get a list of API(parameter calls)

    """


  """
  1. getting probabilistic beliefs from the env and writing them in environment_belief.txt
  2. parsing the initial plan from the LLM and generating sequence of API Calls
  3. write a for loop that executes the API calls
  4. If any API call fails; update the failure_system_prompt and the environment_belief.txt files
  5. parsing the new plan and generating sequence of API calls and execute them
  6. loop (if another failure go to step 4)
  """

  """
  For each API call; maintain a belief over preconditions and effects and keep updating it 
  """