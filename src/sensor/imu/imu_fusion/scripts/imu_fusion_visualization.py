#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Imu

class ImuFusionVisualization():
    def __init__(self):
        self._pub_marker = rospy.Publisher('imu_fusion_vis', Marker, queue_size=1)

        rospy.Subscriber('/sensor/imu/fusion', Imu, self.imu_fusion_callback)

    def imu_fusion_callback(self, data):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "ego_car"
        marker.ns = 'imu'
        marker.id = 0
        marker.type = Cu

if __name__ == "__main__":
    rospy.init_node('imu_fusion_vis_node')
    app = ImuFusionVisualization()
    rospy.spin()
