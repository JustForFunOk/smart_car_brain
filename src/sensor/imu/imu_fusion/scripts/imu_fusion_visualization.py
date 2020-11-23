#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from sensor_msgs.msg import Imu

class ImuFusionVisualization():
    def __init__(self):
        rospy.Subscriber('/sensor/imu/imu_fusion/fusion', Imu, self.imu_fusion_callback)

    def imu_fusion_callback(self, msg):
        br = tf.TransformBroadcaster()
        br.sendTransform((0, 0, 0),
                              (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
                              rospy.Time.now(),
                              "ego_car",
                              "map")

if __name__ == "__main__":
    rospy.init_node('imu_fusion_vis_node')
    app = ImuFusionVisualization()
    rospy.spin()
