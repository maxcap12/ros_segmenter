#!/usr/bin/python3.8

import rospy
from geometry_msgs.msg import PoseStamped
from segmenter_ros2.msg import SegResult, ObjectCoordinates
from utils.position_estimator import estimate_position
from utils.postprocess import postprocess1
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
        objectInfoTopic = params['ros_topics']['object_info_topic']

        # Subscribers
        rospy.Subscriber(
          segImageTopic, SegResult, self.segmentation_callback, queue_size=10
        )

        # Publishers
        self.obj_info_publisher = rospy.Publisher(
            objectInfoTopic, ObjectCoordinates, queue_size=1 
        )

        # Ros bridge
        self.bridge = CvBridge()

        self.position = (0., 0., 0.) # x, y, z
        self.angle = (0., 0., 0.)

        self.objects = {}

        rospy.loginfo("interpreter node ready")

    def segmentation_callback(self, msg: SegResult) -> None:

        image = self.bridge.imgmsg_to_cv2(msg.Image)
        t = time.time()

        self.position = (
            msg.pos.x,
            msg.pos.y,
            msg.pos.z
        )
        self.angle = (
            msg.angle.x,
            msg.angle.y,
            msg.angle.z
        )

        print(f"pos: {self.position}")
        print(f"angle: {self.angle}")

        for mask in msg.masks:
            obj_msg = ObjectCoordinates()
            obj_msg.name = str(mask.categroy)
            positions = estimate_position([mask.mask[i] for i in range(0, len(mask.mask), 5)], image, self.position, self.angle)
            obj_msg.points = postprocess1(positions, 3)
            print(len(obj_msg.points))
            self.obj_info_publisher.publish(obj_msg)
    
        rospy.loginfo(f"computing time: {time.time() - t}")
        

if __name__ == "__main__":
    # Initialization
    rospy.init_node('interpreter', anonymous=False)
    segmenter = Interpreter()
    rospy.spin()
