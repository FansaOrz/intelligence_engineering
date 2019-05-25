#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
import rospy
from geometry_msgs.msg import Twist
import math


class path_pub():
    def __init__(self):
        rospy.init_node("pub_cmd")
        self.if_pub_angle = True
        self.cmd_sub = rospy.Subscriber("/cmd_vel_1", Twist, self.cmd_callback)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=15)
        self.Twist = Twist()
        while not rospy.is_shutdown():
            rospy.sleep(.1)

            if not self.if_pub_angle and self.Twist.linear.x != 0.0:
                self.Twist.angular.z += (0.0 - self.Twist.angular.z) / 8
                if abs(self.Twist.angular.z) < 0.04:
                    self.Twist.angular.z = 0.0
            self.cmd_pub.publish(self.Twist)


    def cmd_callback(self, msg):
        self.Twist = msg
        self.Twist.linear.x = self.Twist.linear.x * 1
        self.if_pub_angle = False

if __name__ == '__main__':
    path_pub()
