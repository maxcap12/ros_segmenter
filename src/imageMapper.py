#!/usr/bin/python3.8

import rospy
from sensor_msgs.msg import Image
from segmenter_ros2.msg import MappedImages
from time import sleep

from temp import init_params
from cv_bridge import CvBridge

class imageMapper:
    def __init__(self) -> None:
        init_params()
        
        self.rgb_image = None
        self.depth_image = None
        self.timer = rospy.Rate(2)

        # Get parameters
        print('Loading configuration parameters ...\n')
        params = rospy.get_param('~params')
        rawImageTopic = params['ros_topics']['raw_image_topic']
        depthImageTopic = params['ros_topics']['depth_image_topic']
        mappedImageTopic = params['ros_topics']['mapped_images_topic']

        # Subscribers
        rospy.Subscriber(rawImageTopic, Image, self._color_image_callback, queue_size=1, buff_size=2**32)
        rospy.Subscriber(depthImageTopic, Image, self._depth_image_callback, queue_size=1, buff_size=2**32)

        # Publishers
        self.mapped_images_publisher = rospy.Publisher(
            mappedImageTopic, MappedImages, queue_size=1
        )

        rospy.loginfo("mapper ready")

        while not rospy.is_shutdown():
            self._send_images()
            self.timer.sleep()

    def _color_image_callback(self, img: Image):
        if self.rgb_image is None:
            self.rgb_image = img

    def _depth_image_callback(self, img: Image):
        if self.depth_image is None:
            self.depth_image = img

    def _send_images(self):
        try:
            if (
                self.rgb_image is not None and
                self.depth_image is not None
            ):
                rospy.loginfo("tick")
                msg = MappedImages()
                msg.RGBImage = self.rgb_image
                msg.DepthImage = self.depth_image
                self.mapped_images_publisher.publish(msg)
                self.depth_image = None
                self.rgb_image = None
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