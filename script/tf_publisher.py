#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
import rospy
import numpy as np
import tf

if __name__ == '__main__':
    rospy.init_node("tf_publisher")
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((.0, .0, .0),
                         (.0, .0, .0, 1.0),
                         rospy.Time.now(),
                         "map",
                         "odom")
        rate.sleep()
