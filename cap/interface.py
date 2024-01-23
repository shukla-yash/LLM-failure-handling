from lmp import LMP, LMPFGen
import numpy as np 


class LMP_interface:

    def __init__(self, env, lmp_config):
        self._env = env 
        self._cfg = lmp_config 

    def get_ee_pos(self):
        return self._env.get_ee_pos()
    
    def detect(self, obj_name, threshold=50):
        obj_id = self.env.obj_name_to_id[obj_name]
        seg_image = self._env.get_seg_image()
        # print(seg_image)
        height, width = seg_image.shape
        obj_pixels = 0
        for i in range(height):
            for j in range(width):
                # obj_id = seg_image[i][j]
                if obj_id == seg_image[i][j]:
                    obj_pixels += 1
        return obj_pixels >= threshold
    
    def pick(self, obj_name):
        pass 

    def place(self, pos_name):
        pass 

    def pick_and_place(self, obj_name, pos_name):
        # DO SOMETHING HERE
        return self._env.on_top_of(obj_name, pos_name)

    def push(self, obj_name, pos_name):
        # DO SOMETHING HERE
        return self._env.close_to(obj_name, pos_name)
