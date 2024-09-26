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
    

def get_vision_response(system_prompts, user_prompts: list, images: str, api_key: str):
    user_prompt = build_prompts(user_prompts) 
    # print('user prompt: ', user_prompt)
    system_prompt = build_system_prompts(system_prompts) 
    # print('system prompt: ', system_prompt)
    # client = OpenAI() 

    base64_image = encode_image('images/' + images) 
    headers = {
      "Content-Type": "application/json",
      "Authorization": f"Bearer {api_key}"
    }

    payload = {
      'model': "gpt-4-turbo",
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

def get_language_response(system_prompts, user_prompts: list, api_key: str):
    user_prompt = build_prompts(user_prompts) 
    # print('user prompt: ', user_prompt)
    system_prompt = build_system_prompts(system_prompts) 
    # print('system prompt: ', system_prompt)
    # client = OpenAI() 

    base64_image = encode_image('images/' + images) 
    headers = {
      "Content-Type": "application/json",
      "Authorization": f"Bearer {api_key}"
    }

    payload = {
      'model': "gpt-4-turbo",
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

    return response.json()['choices'][0]['message']['content']


if __name__ == '__main__':
    system_prompts = ['sys_test.txt']
    user_prompts = ['test.txt'] 
    images = 'test.jpg' 

    print(get_vision_response(system_prompts, user_prompts, images)) 
    