#!/usr/bin/python3.8

import rospy
from segmenter_ros2.msg import ObjectCoordinates
from visualization_msgs.msg import Marker, MarkerArray
from temp import init_params

import math
import time

class Visialization:
    def __init__(self) -> None:
        rospy.init_node("visualizatio_node", anonymous=False)
        self.stop = False
        init_params()

        # Get parameters
        print('Loading configuration parameters ...\n')
        params = rospy.get_param('~params')
        visualizationTopic = params['ros_topics']['visu_topic']
        objectInfoTopic = params['ros_topics']['object_info_topic']

        # Subscribers
        rospy.Subscriber(objectInfoTopic, ObjectCoordinates, self.generate_vis, queue_size=1)

        # Publishers
        self.visuPublisher = rospy.Publisher(visualizationTopic, MarkerArray, queue_size=100)

        rospy.loginfo("visualization ready")

    def reset_visu(self):
        marker = Marker()
        marker.action = marker.DELETEALL
        markerArray = MarkerArray()
        markerArray.markers.append(marker)
        self.visuPublisher.publish(markerArray)

    def generate_vis(self, msg):
        if self.stop: return

        markerArray = MarkerArray()

        print(len(msg.points))

        for i in range(len(msg.points)):
            point = msg.points[i]
            marker = Marker()
            marker.header.frame_id = "my_frame"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.
            marker.color.r = 1.
            marker.color.g = 0.
            marker.color.b = 0.
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = point.x / 20
            marker.pose.position.y = point.z / 20
            marker.pose.position.z = point.y / 20
            marker.id = len(markerArray.markers)

            markerArray.markers.append(marker)

            if not i%100:
                print(i//100)
                self.visuPublisher.publish(markerArray)
                time.sleep(0.1)

        self.stop = True
        
        rospy.loginfo("object visualization finished")


if __name__ == "__main__":
    visu = Visialization()
    rospy.spin()