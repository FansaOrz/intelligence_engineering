#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
import rospy
import numpy as np
import A_star_path_finding as A_star
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovarianceStamped


class path_pub():
    def __init__(self):
        rospy.init_node("path_pub")
        self.path_pub = rospy.Publisher("/path_my_A_star", Path, queue_size=15)

        # 关于地图的一些变量
        self.origin_x = 0
        self.origin_y = 0
        self.resolution = 0
        self.width = 0
        self.height = 0
        # self.map_test_pub = rospy.Publisher("/map_test", OccupancyGrid, queue_size=15)
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.current_path = Path()
        rospy.sleep(1)
        # 起始点和目标点
        self.start_map_point = []
        self.goal_map_point = []
        # 地图上的路径
        self.path_map = []

        # 是否要寻找路径的开关
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
        if self.if_start_find_path:
            print ('\033[0;32m[I] : Start find path with A* \033[0m')
            temp = A_star.find_path(self.map, self.start_map_point, self.goal_map_point)
            self.path_map = temp.start_find()
            print self.path_map
            self.publisher_path()
        else:
            rospy.sleep(1)
            print ('\033[0;33m[W] : Please set goal pose\033[0m')
            return

    # 回调函数系列
    def goal_pose_callback(self, msg):
        self.path_map = []
        self.goal_pose = msg
        self.if_start_find_path = True
        # print msg
        self.goal_map_point =  self.WorldTomap(msg.pose.position.x, msg.pose.position.y)
        print "-----------------goal point---------------",self.goal_map_point
        if self.goal_map_point == [-1, -1]:
            print "\033[0;30m[Kamerider E] : Please set the valid goal point\033[0m"
            return
        else:
            self.start_find_path()

    def mapToWorld(self, mx, my):
        # print "mapToWorld"
        wx = self.origin_x + mx * self.resolution
        wy = self.origin_y + my * self.resolution
        return [wx, wy]

    def WorldTomap(self, wx, wy):
        # 返回-1，-1就是有问题
        # print wx, wy
        # print self.origin_x, self.origin_y
        if wx < self.origin_x or wy < self.origin_y:
            # print "<<<<<<<"
            return [-1, -1]
        mx = (int)((wx - self.origin_x) / self.resolution)
        my = (int)((wy - self.origin_y) / self.resolution)
        if mx < self.width and my < self.height:
            # print ">>>>>>>>>>>"
            return [my, mx]
        return [-1, -1]

    def map_callback(self, msg):
        # self.convert_map(msg)
        print msg.header
        print "------"
        print msg.info
        print "------"
        print len(msg.data)
        # 初始化map里的参数值
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.resolution = msg.info.resolution
        self.width = msg.info.width
        self.height = msg.info.height
        print "-------",self.width
        # # 把map里的消息存下来
        # self.map_msg = msg
        raw = np.array(msg.data, dtype=np.int8)
        # print raw.shape
        self.map = raw.reshape((self.height, self.width))
        print type(self.map)
        # self.map_data_pub = raw_new.flatten()
        # print len(list(self.map_data_pub))
        # self.map_data_pub = list(self.map_data_pub)
        # # print self.map_data_pub
        # rospy.sleep(1)
        # self.map_msg.data = self.map_data_pub
        # self.map_test_pub.publish(self.map_msg)
        self.map_sub.unregister()

    def init_pose_callback(self, msg):
        # print "===========get initial pose================"
        self.init_pose = msg
        # print msg
        # print "----------------worldtomap------------------"
        self.start_map_point =  self.WorldTomap(msg.pose.pose.position.x, msg.pose.pose.position.y)
        print "----------------start point----------------",self.start_map_point
        print "value = ", self.map[self.start_map_point[0]][self.start_map_point[1]]
        if self.start_map_point == [-1, -1]:
            print "\033[0;31m[E] : Please set the valid goal point\033[0m"

    def publisher_path(self):
        time = 1
        # rospy.sleep(10)
        for i in range(len(self.path_map)):
            current_time = rospy.get_rostime()
            # print
            # print "current_time", current_time
            rospy.sleep(.1)
            current_pose = PoseStamped()
            current_pose.pose.position.x, current_pose.pose.position.y= self.mapToWorld(self.path_map[i][1], self.path_map[i][0])
            current_pose.pose.position.z = 0.0
            current_pose.pose.orientation.x = 0.0
            current_pose.pose.orientation.y = 0.0
            current_pose.pose.orientation.z = 0.0
            current_pose.pose.orientation.w = 1.0
            time += 1
            self.current_path.header.stamp = current_time
            self.current_path.header.frame_id = "odom"
            self.current_path.poses.append(current_pose)
            self.path_pub.publish(self.current_path)
            self.last_time = current_time
            # rospy.sleep(1)




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
