#!/usr/bin/python3.8

import rospy
from sensor_msgs.msg import Image
from segmenter_ros2.msg import SegResult, Mask, Pixel, MappedImages
from output import fastSamVisualizer
from cv_bridge import CvBridge, CvBridgeError
from utils.helpers import cleanMemory, monitorParams
from temp import init_params
from torch import Tensor
import numpy as np
import time

from detectron2.config import get_cfg
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.utils.visualizer import Visualizer
from detectron2.data import MetadataCatalog


class Segmenter:
    def __init__(self) -> None:
        init_params()
        
        # Initial checks
        monitorParams()
        cleanMemory()

        # Get parameters
        print('Loading configuration parameters ...\n')
        params = rospy.get_param('~params')
        self.iou = params['model_params']['iou']
        self.conf = params['model_params']['conf']
        modelName = params['model_params']['model_name']
        modelPath = params['model_params']['model_path']
        self.imageSize = params['image_params']['image_size']
        mappedImageTopic = params['ros_topics']['mapped_images_topic']
        segImageTopic = params['ros_topics']['segmented_image_topic']
        self.pointPrompt = params['model_params']['point_prompt']
        self.boxPrompt = params['model_params']['box_prompt']
        self.pointLabel = params['model_params']['point_label']
        self.counters = params['model_params']['contour']

        # Subscribers
        rospy.Subscriber(mappedImageTopic, MappedImages, self.segmentation, queue_size=1, buff_size=2**32)

        # Publishers
        self.publisherSeg = rospy.Publisher(
            segImageTopic, SegResult, queue_size=1
        )

        self.test_acc = rospy.Publisher(
            "/test_acc", Image, queue_size=1
        )

        # ROS Bridge
        self.bridge = CvBridge()

        self.init_model()

        rospy.loginfo("segmenter node ready")

    def init_model(self) -> None:
        cfg = get_cfg()
        cfg.merge_from_file(model_zoo.get_config_file("COCO-PanopticSegmentation/panoptic_fpn_R_50_3x.yaml"))
        cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url("COCO-PanopticSegmentation/panoptic_fpn_R_50_3x")
        cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5  
        cfg.MODEL.DEVICE = 'cpu'
        
        # cfg.merge_from_file("src/scene_segment_ros2/config/PanopticFCN-R50-400-3x-FAST.yaml")
        # cfg.MODEL.WEIGHTS = "https://drive.google.com/file/d/1csiLLsZT8dUQj7XETo9vsjXfAh5r7KZU/view?usp=sharing"
        # cfg.MODEL.IGNORE_VALUE = -1

        self.predictor = DefaultPredictor(cfg)

        self.metadata = MetadataCatalog.get("coco_2017_val_panoptic")
        
    def segmentation(self, msg) -> None:
        try:
            rospy.loginfo("tick")
            # Convert the ROS Image message to a CV2 image
            rgb_image = self.bridge.imgmsg_to_cv2(msg.RGBImage)
            depth_image = self.bridge.imgmsg_to_cv2(msg.DepthImage)

            # Processing
            t = time.time()
            output = self.predictor(rgb_image)
            rospy.loginfo(f"processing: {time.time()-t}")

            # Get the masks of the accepted objects
            t = time.time()
            masks = self._get_masks(output["panoptic_seg"], [0, 56, 60])
            rospy.loginfo(f"sorting: {time.time()-t}")

            # Generate the message
            t = time.time()
            output_msg = self._generate_message(masks, depth_image)
            rospy.loginfo(f"generating message: {time.time()-t} \n\n")

            self.publisherSeg.publish(output_msg)
            visu = Visualizer(rgb_image, self.metadata).draw_panoptic_seg(output["panoptic_seg"][0], output["panoptic_seg"][1])
            self.test_acc.publish(self.bridge.cv2_to_imgmsg(visu.get_image(), "rgb8"))

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))


    def show_mask(self, mask, img):
        test = img.copy()
        for co in mask:
            test[co.y][co.x] = 10000
        
        msg = self.bridge.cv2_to_imgmsg(test)
        self.test_acc.publish(msg)
           

    def _get_masks(self, output: Tensor, accepted_ids: list) -> dict:
        mask = output[0].tolist()
        anns = output[1]

        masks = {id: {} for id in accepted_ids}

        for y in range(len(mask)):

            for x in range(len(mask[y])):
                id = mask[y][x]

                if (
                    id and
                    anns[id-1]["isthing"] and
                    anns[id-1]["category_id"] in accepted_ids
                ):
                    instance_id = anns[id-1]["instance_id"]
                    masks_cat = masks[anns[id-1]["category_id"]]

                    if instance_id not in masks_cat.keys():
                        masks_cat[instance_id] = []

                    masks_cat[instance_id].append((x, y))

        return masks
    
    def _generate_message(self, masks: dict, img: Image) -> SegResult:
        msg = SegResult()
        msg.Image = self.bridge.cv2_to_imgmsg(img)
        msg.masks = []

        for cat in masks.keys():
            
            for inst in masks[cat].keys():

                mask = Mask()
                mask.category = cat
                mask.instance = inst
                mask.mask = []

                for p in masks[cat][inst]:
                    pixel = Pixel()
                    pixel.x, pixel.y = p
                    mask.mask.append(pixel)

                msg.masks.append(mask)

        return msg
            


# Run the program
if __name__ == '__main__':
    # Initialization
    rospy.init_node('segmenter', anonymous=False)
    segmenter = Segmenter()
    rospy.spin()