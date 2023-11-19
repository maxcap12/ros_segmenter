#!/usr/bin/python3.8

import rospy
from std_msgs.msg import String
from segmenter_ros2.msg import SegResult
from utils.position_estimator import estimate_position
from cv_bridge import CvBridge
from utils.detected_object import DetectedObject

from temp import init_params

class Interpreter(object):
    def __init__(self) -> None:
        init_params()
        # Get parameters
        print('Loading configuration parameters ...\n')
        params = rospy.get_param('~params')
        segImageTopic = params['ros_topics']['segmented_image_topic']
        self.acceptedObjects = params['interpretor_params']['accepted_objects']
        positionTopic = params['ros_topics']['position_topic']

        # Subscribers
        rospy.Subscriber(
          segImageTopic, SegResult, self.segmentation_callback, queue_size=10
        )

        rospy.Subscriber(
          positionTopic, String, self._update_position, queue_size=1
        )

        # Ros bridge
        self.bridge = CvBridge()

        self.position = (0, 0, 0, 0) # x, y, z, angle

        self.objects = {}

        rospy.loginfo("interpreter node ready")

    def _update_position(self, msg):
        self.position = msg

    def segmentation_callback(self, msg: SegResult) -> None:
        print("tick")
        image = self.bridge.imgmsg_to_cv2(msg.Image)

        for mask in msg.masks:
            (x, y, z, w, h) = estimate_position(mask.mask, image)
            obj = DetectedObject(x, y, z, w, h, self.position[3])

            if mask.category not in self.objects.keys(): # first detected object of this category
                self.objects[mask.category] = [obj]

            else:
                for i in range(len(self.objects[mask.category])):
                    if self.objects[mask.category][i] == obj: # the object has been detected previously
                        self.objects[mask.category][i] += obj # merge the previous and new info about the object
                        break
                    if i == len(self.objects[mask.category]): # the object has not been detected yet
                        self.objects[mask.category][i].append(obj)
          
        print(f"{sum([len(items) for items in self.objects.values()])} objects detected")
  

if __name__ == "__main__":
    # Initialization
    rospy.init_node('interpreter', anonymous=False)
    segmenter = Interpreter()
    rospy.spin()