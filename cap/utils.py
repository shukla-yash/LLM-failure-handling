import os
import pybullet
import pybullet_data
import numpy as np
import threading
import copy
import openai
import cv2
from moviepy.editor import ImageSequenceClip

# imports for LMPs
# import shapely
import ast
import astunparse
from time import sleep
from constants import *
from wrapper import LMP_wrapper
from lmp import *
from shapely.geometry import *
from shapely.affinity import *
# from openai.error import RateLimitError, APIConnectionError
from pygments import highlight
from pygments.lexers import PythonLexer
from pygments.formatters import TerminalFormatter


def setup_LMP(env, cfg_tabletop):
  # LMP env wrapper
  cfg_tabletop = copy.deepcopy(cfg_tabletop)
  cfg_tabletop['env'] = dict()
  cfg_tabletop['env']['init_objs'] = list(env.obj_name_to_id.keys())
  cfg_tabletop['env']['coords'] = lmp_tabletop_coords
  LMP_env = LMP_wrapper(env, cfg_tabletop)
  # creating APIs that the LMPs can interact with
  fixed_vars = {
      'np': np
  }
  fixed_vars.update({
      name: eval(name)
      for name in shapely.geometry.__all__ + shapely.affinity.__all__
  })
  variable_vars = {
      k: getattr(LMP_env, k)
      for k in [
          'get_bbox', 'get_obj_pos', 'get_color', 'is_obj_visible', 'denormalize_xy',
          'put_first_on_second', 'get_obj_names',
          'get_corner_name', 'get_side_name',
      ]
  }
  variable_vars['say'] = lambda msg: print(f'robot says: {msg}')

  # creating the function-generating LMP
  lmp_fgen = LMPFGen(cfg_tabletop['lmps']['fgen'], fixed_vars, variable_vars)

  # creating other low-level LMPs
  variable_vars.update({
      k: LMP(k, cfg_tabletop['lmps'][k], lmp_fgen, fixed_vars, variable_vars)
      for k in ['parse_obj_name', 'parse_position', 'parse_question', 'transform_shape_pts']
  })

  # creating the LMP that deals w/ high-level language commands
  lmp_tabletop_ui = LMP(
      'tabletop_ui', cfg_tabletop['lmps']['tabletop_ui'], lmp_fgen, fixed_vars, variable_vars
  )

  return lmp_tabletop_ui