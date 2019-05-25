#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
import rospy
import numpy as np
from visualization_msgs.msg import Marker

from sensor_msgs.msg import LaserScan
class feature_extract():
    def __init__(self):
        rospy.init_node("feature_extract")
        self.scan_sub = rospy.Subscriber("/myscan", LaserScan, self.scan_cb)
        print "init"
        self.if_first = True
        self.angle_min = None
        self.angle_max = None
        self.angle_increment = None
        self.time_increment = None
        self.scan_time = None
        self.range_min = None
        self.range_max = None
        rospy.spin()


    def scan_cb(self, msg):
        print msg.range_max
        # if self.if_first:
        #     self.angle_min = msg.angle_min
        #     self.angle_max = msg.angle_max
        #     self.angle_increment = msg.angle_increment
        #     self.time_increment = msg.time_increment
        #     self.scan_time = msg.scan_time
        #     self.range_min = msg.range_min
        #     self.range_max = msg.range_max
        # self.feature_extract(msg.ranges)
        self.scan_sub.unregister()
        rng = np.array(msg.ranges)
        ang = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        points = np.vstack((rng * np.cos(ang),
                            rng * np.sin(ang)))
        msg.range_max = 3
        points = points[:, rng < msg.range_max]
        lines = self.split_and_merge(points)

    def split_and_merge(self, points):
        print type(scan)


if __name__ == '__main__':

    feature_extract()