#!/usr/bin/python3.8

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from segmenter_ros2.msg import MappedImages
from time import sleep

from temp import init_params
from cv_bridge import CvBridge

class imageMapper:
    def __init__(self) -> None:
        init_params()
        
        self.rgb_image = None
        self.depth_image = None
        self.pos = None
        self.timer = rospy.Rate(2)

        # Get parameters
        print('Loading configuration parameters ...\n')
        params = rospy.get_param('~params')
        rawImageTopic = params['ros_topics']['raw_image_topic']
        depthImageTopic = params['ros_topics']['depth_image_topic']
        mappedImageTopic = params['ros_topics']['mapped_images_topic']
        positionTopic = params['ros_topics']['position_topic']

        # Subscribers
        rospy.Subscriber(rawImageTopic, Image, self._color_image_callback, queue_size=1, buff_size=2**32)
        rospy.Subscriber(depthImageTopic, Image, self._depth_image_callback, queue_size=1, buff_size=2**32)
        rospy.Subscriber(positionTopic, PoseStamped, self._position_callback, queue_size=1)

        # Publishers
        self.mapped_images_publisher = rospy.Publisher(
            mappedImageTopic, MappedImages, queue_size=1
        )

        rospy.loginfo("mapper ready")

        while not rospy.is_shutdown():
            self._send_msg()
            self.timer.sleep()

    def _color_image_callback(self, img: Image):
        if self.rgb_image is None and self.pos is not None:
            self.rgb_image = img

    def _depth_image_callback(self, img: Image):
        if self.depth_image is None and self.pos is not None:
            self.depth_image = img

    def _position_callback(self, msg: PoseStamped):
        if self.pos is None:
            self.pos = msg

    def _send_msg(self):
        try:
            if (
                self.rgb_image is not None and
                self.depth_image is not None and
                self.pos is not None
            ):
                rospy.loginfo("tick")

                msg = MappedImages()
                msg.RGBImage = self.rgb_image
                msg.DepthImage = self.depth_image
                msg.pos.x = self.pos.pose.position.x
                msg.pos.y = self.pos.pose.position.y
                msg.pos.z = self.pos.pose.position.z
                msg.angle.x = self.pos.pose.orientation.x 
                msg.angle.y = self.pos.pose.orientation.y 
                msg.angle.z = self.pos.pose.orientation.z

                self.mapped_images_publisher.publish(msg)
                self.depth_image = None
                self.rgb_image = None
                self.pos = None
            else:
                rospy.loginfo("miss")
        except TypeError as e:
            rospy.logerr(e)


# Run the program
if __name__ == '__main__':
    # Initialization
    rospy.init_node('image_mapper', anonymous=False)
    segmenter = imageMapper()
    rospy.spin()