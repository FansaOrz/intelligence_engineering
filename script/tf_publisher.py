#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
import rospy
import numpy as np
import tf
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

class tf_pub():
    def __init__(self):
        rospy.init_node("tf_publisher")
        self.if_get_odom = False
        self.br = tf.TransformBroadcaster()
        self.init_pose_sub = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.init_pose_callback)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.init_pose = PoseWithCovarianceStamped()
        self.init_odom = Odometry()
        # self.init_odom.pose.pose.orientation.x

    def init_pose_callback(self, msg):
        self.init_pose = msg
        self.if_get_odom = True

    def odom_callback(self, msg):
        self.odom_pose = msg
        self.br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                              (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                              rospy.Time.now(),
                              "odom",
                              "base_link")

    def pub_tf(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            if self.if_get_odom:
                self.br.sendTransform((self.init_pose.pose.pose.position.x, self.init_pose.pose.pose.position.y,
                                       self.init_pose.pose.pose.position.z),
                                      (self.init_pose.pose.pose.orientation.x, self.init_pose.pose.pose.orientation.y,
                                       self.init_pose.pose.pose.orientation.z, self.init_pose.pose.pose.orientation.w),
                                      rospy.Time.now(),
                                      "odom",
                                      "map")
            rate.sleep()

if __name__ == '__main__':
    tf_cl = tf_pub()
    tf_cl.pub_tf()

