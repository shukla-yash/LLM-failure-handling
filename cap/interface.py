from lmp import LMP, LMPFGen
import numpy as np 


class LMP_interface:

    def __init__(self, env, lmp_config):
        self._env = env 
        self._cfg = lmp_config 

    def get_ee_pos(self):
        return self._env.get_ee_pos()
    
    def detect(self, obj_name, threshold=50):
        seg_image = self._env.get_seg_image()
        print(seg_image)
        height, width = seg_image.shape
        obj_pixels = 0
        for i in range(height):
            for j in range(width):
                obj_id = seg_image[i][j]
                if obj_id != -1:
                    obj_pixels += 1
        return obj_pixels >= threshold
    
    def pick
