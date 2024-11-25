import os
import yaml

def read_one_block_of_yaml_data(filename):
    project_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    with open(f'{project_path}/{filename}.yaml','r') as f:
        output = yaml.safe_load(f)
    return output 
    