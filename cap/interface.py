from lmp import LMP, LMPFGen
import numpy as np 
import pybullet
from wrapper import LMP_wrapper
from constants import *


class LMP_interface(LMP_wrapper):

    def __init__(self, env, lmp_config, render=False):
        super().__init__(env, lmp_config, render=render)
        # self._cfg = lmp_config 

    def get_ee_pos(self):
        return self.env.env.get_ee_pos()
    
    def detect(self, obj_name, threshold=50):
        if not self.is_obj_visible(obj_name):
            return False
        obj_id = self.env.get_obj_id(obj_name)
        seg_image = self.env.env.get_seg_image()
        # print(seg_image.shape)
        height, width = seg_image.shape
        obj_pixels = 0
        for i in range(height):
            for j in range(width):
                # obj_id = seg_image[i][j]
                if obj_id == seg_image[i][j]:
                    obj_pixels += 1

        return obj_pixels >= threshold
    
    def reach(self, obj_name):
        # if not self.detect(obj_name, threshold=50):
        #     return False
        target_pos = self.get_obj_pos(obj_name)
        self.wrapper.goto_pos(target_pos)

        return self.env.is_close_to(obj_name, obj_name)
    
    def pickup(self, obj_name):
        obj_pos = self.env.get_obj_pos_np(obj_name)
        action = {
            'pick': obj_pos,
            'height': 0.2
            }
        self.env.step(action)

        gripper_id = self.env.tip_link_id
        obj_id = self.env.get_obj_id(obj_name)
        is_holding = False
        gripper_link_ids = self.env.gripper.body
        contacts = pybullet.getContactPoints(gripper_id, obj_id)
        for contact in contacts:
            if contact[3] in gripper_link_ids:
                return True 
        return False

    def place(self, obj_name, dest_name):
        if not self.is_obj_visible(obj_name):
            return False # Unknown object
        if not dest_name in CORNER_POS and not self.is_obj_visible(dest_name):
            return False # Unknown destination
        
        # Check whether obj is already being held
        obj_id = self.get_obj_id(obj_name)
        gripper_id = self.env.tip_link_id
        gripper_link_ids = self.env.gripper.body 
        contacts = pybullet.getContactPoints(gripper_id, obj_id)
        is_holding = False
        for c in contacts:
            if c[3] in gripper_link_ids:
                is_holding = True
        if not is_holding:
            return False # Pick up the object first

        if dest_name in CORNER_POS.keys():
            to_pos = CORNER_POS[dest_name]
        else:
            to_pos = self.get_obj_pos_np(dest_name)

        action = {
            'place': obj_pos,
        }
        self.env.step(action)

        if not dest_name in CORNER_POS.keys():
            return self.is_close_to(obj_name, dest_name)
        else:
            obj_pos = self.get_obj_pos_np(obj_name)
            return np.linalg.norm(obj_pos-to_pos) < 0.1

    def pick_and_place(self, obj_name, pos_name):
        raise NotImplementedError # Use pick and place seperately

    def push(self, obj_name, dest_name):

        if not self.is_obj_visible(obj_name):
            return False # Unknown object
        
        if not dest_name in CORNER_POS and not self.is_obj_visible(dest_name):
            return False # Unknown destination
        
        push_pos = self.get_obj_pos_np(obj_name)
        if dest_name in CORNER_POS.keys():
            to_pos = CORNER_POS[dest_name]
        else:
            to_pos = self.get_obj_pos_np(dest_name)
        action = {
            'push_pos': push_pos,
            'to_pos': to_pos
        }

        self.env.step(action)
        
        if not dest_name in CORNER_POS.keys():
            return self.is_close_to(obj_name, dest_name)
        else:
            obj_pos = self.get_obj_pos_np(obj_name)
            return np.linalg.norm(obj_pos-to_pos) < 0.1
