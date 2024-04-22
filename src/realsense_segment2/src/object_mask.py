#!/home/unicorn/miniconda3/envs/kinovaconda/bin/python
import os
import groundingdino.datasets.transforms as T
import numpy as np
import torch
from groundingdino.models import build_model
from groundingdino.util import box_ops
from groundingdino.util.inference import predict
from groundingdino.util.slconfig import SLConfig
from groundingdino.util.utils import clean_state_dict
from segment_anything import sam_model_registry
from segment_anything import SamPredictor
from torchvision import transforms

from groundingdino.util.inference import load_model, load_image, predict, annotate
import cv2
from PIL import Image, ImageFile
import torch
import numpy as np
import math
from natsort import natsorted
import torchvision.transforms as transforms
import torchvision.models as models
import glob
import PIL

def transform_image(image) -> torch.Tensor:
    transform = T.Compose([
        # transforms.Resize((224, 224)),
        # T.Resize((800, 1200)),
        T.ToTensor(),
        T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
    ])

    image_transformed, _ = transform(image, None)
    return image_transformed


class LangSAM():

    def __init__(self, sam_type="vit_h", ckpt_path=None):
        self.sam_type = sam_type
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.build_groundingdino()
        self.build_sam(ckpt_path)
        ImageFile.LOAD_TRUNCATED_IMAGES = True
    def build_sam(self, ckpt_path):
        ckpt_path = '/home/unicorn/Work2/GroundingDINO/SAM/sam_vit_h_4b8939.pth'
        if self.sam_type is None or ckpt_path is None:
            if self.sam_type is None:
                print("No sam type indicated. Using vit_h by default.")
                self.sam_type = "vit_h"
            checkpoint_url = SAM_MODELS[self.sam_type]
            try:
                sam = sam_model_registry[self.sam_type]()
                state_dict = torch.hub.load_state_dict_from_url(checkpoint_url)
                sam.load_state_dict(state_dict, strict=True)
            except:
                raise ValueError(f"Problem loading SAM please make sure you have the right model type: {self.sam_type} \
                    and a working checkpoint: {checkpoint_url}. Recommend deleting the checkpoint and \
                    re-downloading it.")
            sam.to(device=self.device)
            self.sam = SamPredictor(sam)
        else:
            try:
                sam = sam_model_registry[self.sam_type](ckpt_path)
            except:
                raise ValueError(f"Problem loading SAM. Your model type: {self.sam_type} \
                should match your checkpoint path: {ckpt_path}. Recommend calling LangSAM \
                using matching model type AND checkpoint path")
            sam.to(device=self.device)
            self.sam = SamPredictor(sam)

    def build_groundingdino(self):
        self.groundingdino = load_model("/home/unicorn/Work2/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py", "/home/unicorn/Work2/GroundingDINO/weights/groundingdino_swint_ogc.pth")
        # self.groundingdino = load_model("groundingdino/config/GroundingDINO_SwinT_OGC.py", "weights/groundingdino_swint_ogc.pth")

    def predict_dino(self, image_path, object_string, box_threshold=None, text_threshold=None):
        image_source, image = load_image(image_path)
        boxes, logits, phrases = predict(
            model=self.groundingdino,
            image=image,
            caption=object_string,
            box_threshold=0.25,
            text_threshold=0.25)

        # W, H = Image.open(image_path).size
        W, H = image_path.size


        boxes = box_ops.box_cxcywh_to_xyxy(boxes) * torch.Tensor([W, H, W, H])

        return boxes, logits, phrases

    def predict_sam(self, rgb_image, boxes):
        # image_pil = Image.open(rgb_image).convert('RGB')
        image_pil = rgb_image
        image_array = np.asarray(image_pil)
        self.sam.set_image(image_array)
        transformed_boxes = self.sam.transform.apply_boxes_torch(boxes, image_array.shape[:2])
        masks, _, _ = self.sam.predict_torch(
            point_coords=None,
            point_labels=None,
            boxes=transformed_boxes.to(self.sam.device),
            multimask_output=False,
        )
        return masks.cpu()

    def select_top_elements(self, boxes, logits, phrases, num_elements):
        # Convert logits to a PyTorch tensor if not already
        # Get indices of the top 7 elements
        top_indices = torch.argsort(logits)[-num_elements:]
        # phrases = torch.tensor(phrases)
        # Select corresponding rows from boxes and phrases
        selected_boxes = boxes[top_indices]
        selected_logits = logits[top_indices]  # Convert back to NumPy if needed
        # selected_phrases = phrases[top_indices]
        selected_phrases = phrases[-1:]

        return selected_boxes, selected_logits, selected_phrases

    def predict(self, rgb_image_path, object_string, box_threshold=0.3, text_threshold=0.25):
        # print("here3")

        boxes, logits, phrases = self.predict_dino(rgb_image_path, object_string, box_threshold, text_threshold)
        masks = torch.tensor([])
        if len(boxes) > 0:
            boxes, logits, phrases = self.select_top_elements(boxes, logits, phrases, 1)
            masks = self.predict_sam(rgb_image_path, boxes)
            masks = masks.squeeze(1)
        return masks, boxes, phrases, logits

    def get_mask(self,rgb_image, depth_image, object_string):

        # image_pil = Image.open(rgb_image).convert("RGB")
        image_pil = rgb_image
        depth_image_pil = depth_image

        masks, boxes, labels, logits = self.predict(rgb_image, object_string)

        tf_image = transform_image(image_pil)
        tf_image = (tf_image * 255).byte()
        image_array = np.asarray(image_pil)
        depth_image_array = np.asarray(depth_image_pil)

        image = draw_image(image_array, masks, boxes, labels)
        # image = draw_image(tf_image, masks, boxes, labels)
        image = Image.fromarray(image)

        image.save("masked_image.png")

        masks = masks[0]
        masked_image = image_array * np.expand_dims(masks.numpy(), axis = -1)

        nonzero_indices = np.argwhere(masks.numpy())

        # Get the bounding box coordinates
        y_min, x_min = np.min(nonzero_indices, axis=0)
        y_max, x_max = np.max(nonzero_indices, axis=0)
        bbox_height = y_max - y_min
        bbox_width = x_max - x_min

        bbox = (x_min, y_min, x_max, y_max)

        # Crop the image using the bounding box
        cropped_image = masked_image[y_min:y_max, x_min:x_max]
        resized_image_pil = Image.fromarray(cropped_image)
        resized_image_pil.save("2.png")

        masked_image = depth_image_array * masks.numpy()
        return bbox, masked_image

        # except:
        #     print("couldnt do it")
    

import cv2
import numpy as np
import torch
from PIL import Image
from torchvision.utils import draw_bounding_boxes
from torchvision.utils import draw_segmentation_masks

MIN_AREA = 100


# def load_image(image_path: str):
#     return Image.open(image_path).convert("RGB")


def draw_image(image, masks, boxes, labels, alpha=0.4):
    image = torch.from_numpy(image).permute(2, 0, 1)
    if len(boxes) > 0:
        image = draw_bounding_boxes(image, boxes, colors=['red'] * len(boxes), labels=labels, width=2)
    if len(masks) > 0:
        image = draw_segmentation_masks(image, masks=masks, colors=['cyan'] * len(masks), alpha=alpha)
    return image.numpy().transpose(1, 2, 0)
