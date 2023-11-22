#!/usr/bin/python3.8

import rospy
from std_msgs.msg import String
from segmenter_ros2.msg import SegResult, ObjectCoordinates
from utils.position_estimator import estimate_position
from cv_bridge import CvBridge
from utils.detected_object import DetectedObject

from temp import init_params
import time

class Interpreter:
    def __init__(self) -> None:
        init_params()
        # Get parameters
        print('Loading configuration parameters ...\n')
        params = rospy.get_param('~params')
        segImageTopic = params['ros_topics']['segmented_image_topic']
        self.acceptedObjects = params['interpretor_params']['accepted_objects']
        positionTopic = params['ros_topics']['position_topic']
        objectInfoTopic = params['ros_topics']['object_info_topic']

        # Subscribers
        rospy.Subscriber(
          segImageTopic, SegResult, self.segmentation_callback, queue_size=10
        )

        rospy.Subscriber(
          positionTopic, String, self._update_position, queue_size=1
        )

        # Publishers
        self.obj_info_publisher = rospy.Publisher(
            objectInfoTopic, ObjectCoordinates, queue_size=1 
        )

        # Ros bridge
        self.bridge = CvBridge()

        self.position = (0, 0, 0, 0) # x, y, z, angle

        self.objects = {}

        rospy.loginfo("interpreter node ready")

    def _update_position(self, msg):
        self.position = msg

    def segmentation_callback(self, msg: SegResult) -> None:

        image = self.bridge.imgmsg_to_cv2(msg.Image)
        t = time.time()

        for mask in msg.masks:
            obj_msg = ObjectCoordinates()
            obj_msg.name = "test"
            obj_msg.points = estimate_position(mask.mask, image, self.position)
            self.obj_info_publisher.publish(obj_msg)
    
        print(time.time() - t)
        

if __name__ == "__main__":
    # Initialization
    rospy.init_node('interpreter', anonymous=False)
    segmenter = Interpreter()
    rospy.spin()