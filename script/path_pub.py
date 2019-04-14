#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
import rospy
import numpy as np
import thread
from math import floor
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovarianceStamped
import tf


class path_pub():
    def __init__(self):
        rospy.init_node("path_pub")
        self.path_pub = rospy.Publisher("/path", Path, queue_size=15)
        self.map_test_pub = rospy.Publisher("/map_test", OccupancyGrid, queue_size=15)
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.current_path = Path()
        rospy.sleep(1)
        self.if_start_find_path = False
        self.goal_pose = PoseStamped()
        self.init_pose = PoseWithCovarianceStamped()
        self.init_pose_sub = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.init_pose_callback)
        self.goal_pose_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_pose_callback)
        self.last_time = rospy.get_rostime()
        self.start_find_path()
        rospy.Rate(1)
        rospy.spin()

    def start_find_path(self):
        arg = tuple([1])
        thread.start_new_thread(self.find_path, arg)

    # 回调函数系列
    def goal_pose_callback(self, msg):
        self.goal_pose = msg
        self.if_start_find_path = True
        print "goalgoalgoal"

    def MAP_GXWX(self, map, x):
        return (floor((x - map.origin_x) / map.scale + .5) + map.size_x / 2)

    def MAP_GYWY(self, map, y):
        return (floor((y - map.origin_y) / map.scale + .5) + map.size_y / 2)

    def MAP_VALID(self, map, i, j):
        return ((i >= 0) and (i < map.size_x) and (j >= 0) and (j < map.size_y))



    def convert_map(self, map):


    def map_callback(self, msg):
        self.convert_map(msg)
        print msg.header
        print "------"
        print msg.info
        print "------"
        print len(msg.data)
        self.map_msg = msg
        raw = np.array(msg.data, dtype=np.int8)
        # print raw.shape
        raw_new = raw.reshape((1248, 1152))
        # print type(raw_new)
        self.map_data_pub = raw_new.flatten()
        print len(list(self.map_data_pub))
        self.map_data_pub = list(self.map_data_pub)
        # print self.map_data_pub
        rospy.sleep(1)
        self.map_msg.data = self.map_data_pub
        self.map_test_pub.publish(self.map_msg)
        self.map_sub.unregister()

    def init_pose_callback(self, msg):
        print "yyuyyyyyyyy"
        self.init_pose = msg
        print msg

    def find_path(self, arg):
        if self.if_start_find_path:
            print ('\033[0;32m [Information] Start find path with A* \033[0m')
        else:
            rospy.sleep(1)
            print ('\033[0;33m [Warning] Please set goal pose\033[0m')
            return

    def publisher(self):
        time = 1
        while True:
            current_time = rospy.get_rostime()
            # print
            print "current_time", current_time

            current_pose = PoseStamped()
            current_pose.pose.position.x = (current_time - self.last_time).to_sec() * .1 * time
            time += 1
            self.current_path.header.stamp = current_time
            self.current_path.header.frame_id = "odom"
            self.current_path.poses.append(current_pose)
            self.path_pub.publish(self.current_path)
            self.last_time = current_time
            rospy.sleep(1)




if __name__ == '__main__':
    path_pub()
