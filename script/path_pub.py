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

    def MAP_INDEX(self, map, i, j):
        return (i + (j * map.size_x))

    def convert_map(self, map):
        map_new = map_t()
        #查找
        # ROS_ASSERT(MAP_NEW)
        map_new.size_x = map.info.width
        map_new.size_y = map.info.height
        map_new.scale = map.info.resolution
        map_new.origin_x = map.info.origin.position.x + (map_new.size_x / 2) * map_new.scale
        map_new.origin_y = map.info.origin.position.y + (map_new.size_y / 2) * map_new.scale
        for i in range(map_new.size_x * map_new.size_y):
            if map.data[i] == 0:
                map_new.cells[i].occ_state = -1
            elif map.data[i] == 100:
                map_new.cells[i].occ_state = 1
            else:
                map_new.cells[i].occ_state = 0


    def map_get_cell(self, ox, oy, oa):
        i = self.MAP_GXWX(map, ox)
        j = self.MAP_GYWY(map, oy)
        if not self.MAP_VALID(map, i, j):
            return None
        cell = map.cells + self.MAP_INDEX(map, i, j)
        return cell

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

class map_t():
    def __init__(self):
        self.size_x = 0
        self.size_y = 0
        self.origin_x = 0
        self.origin_y = 0
        self.scale = 0
        self.cells = list(map_cell())
        self.max_occ_dist = 0


class map_cell_t():
    def __init__(self):
        #Occupancy state (-1 = free, 0 = unknown, +1 = occ)
        self.occ_state = 0
        #Distance to the nearest occupied cell
        self.occ_dist = 0


if __name__ == '__main__':
    path_pub()
