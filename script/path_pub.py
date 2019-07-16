#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
import rospy
import time
import numpy as np
from scipy import interpolate
import A_star_path_finding_improved as A_star
import Dijkstra_path_finding as Dij
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


class path_pub():
    def __init__(self):
        rospy.init_node("path_pub")
        self.start_time = self.end_time = 0
        self.path_pub = rospy.Publisher("/path_my_A_star", Path, queue_size=15)
        self.path_pub_changed = rospy.Publisher("/path_my_A_star_changed", Path, queue_size=15)
        self.robot_radius = 0.3

        # 关于地图的一些变量
        self.origin_x = 0
        self.origin_y = 0
        self.resolution = 0
        self.width = 0
        self.height = 0
        self.map_test_pub = rospy.Publisher("/map_test", OccupancyGrid, queue_size=15)
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.current_path = Path()
        self.current_path_changed = Path()
        rospy.sleep(1)
        # 起始点和目标点
        self.start_map_point = []
        self.goal_map_point = []
        # 地图上的路径
        self.path_map = []
        self.path_world = []
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

    def map_inflation(self):
        print "inflation"

    def start_find_path(self):
        if self.if_start_find_path:

            print ('\033[32m[I] : Start find path with A* \033[0m')
            self.path_map, open_list_index = self.Dij_find_path.start_find(self.start_map_point, self.goal_map_point)
            self.update_map(open_list_index)
            self.map_test_pub.publish(self.map_msg)

            # print self.path_map
            self.publisher_path()
        else:
            rospy.sleep(1)
            print ('\033[33m[W] : Please set goal pose\033[0m')
            return

    def update_map(self, index):
        temp = list(self.map_msg.data)
        for i in range(len(index)):
            temp_x = index[i][0][0] - 1
            temp_y = index[i][1][0] - 1
            temp[temp_x * self.width + temp_y] = 50
        self.map_msg.data = tuple(temp)
        time.sleep(3)
        self.map_test_pub.publish(self.map_msg)
        print "-----show_changed_map-----"

    # 回调函数系列
    def goal_pose_callback(self, msg):
        self.path_map = []
        self.goal_pose = msg
        self.if_start_find_path = True
        # print msg
        self.goal_map_point =  self.WorldTomap(msg.pose.position.x, msg.pose.position.y)
        print "-----------------goal point---------------",self.goal_map_point
        if self.goal_map_point == [-1, -1]:
            print "\033[30m[E] : Please set the valid goal point\033[0m"
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
            return [my, mx]
        return [-1, -1]

    def map_callback(self, msg):
        print msg.header
        print "------"
        print msg.info
        print "------"

        # 初始化map里的参数值
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.resolution = msg.info.resolution
        self.width = msg.info.width
        self.height = msg.info.height
        print "-----width------",self.width
        # # 把map里的消息存下来
        self.map_msg = msg
        raw = np.array(msg.data, dtype=np.int8)
        self.map = raw.reshape((self.height, self.width))
        self.Dij_find_path = A_star.find_path(self.map, robot_size=8, inflation_size=2)
        self.map_sub.unregister()

    def init_pose_callback(self, msg):
        self.init_pose = msg
        self.start_map_point =  self.WorldTomap(msg.pose.pose.position.x, msg.pose.pose.position.y)
        print "----------------start point----------------",self.start_map_point
        print "value = ", self.map[self.start_map_point[0]][self.start_map_point[1]]
        if self.start_map_point == [-1, -1]:
            print "\033[0;31m[E] : Please set the valid goal point\033[0m"

    def publisher_path(self):
        time = 1
        y1 = []
        y2 = []
        # rospy.sleep(10)
        for i in range(len(self.path_map)):
            current_time = rospy.get_rostime()
            # print
            # print "current_time", current_time
            # rospy.sleep(.05)
            current_pose = PoseStamped()
            current_pose.pose.position.x, current_pose.pose.position.y= self.mapToWorld(self.path_map[i][1], self.path_map[i][0])
            # self.path_world.append(self.mapToWorld(self.path_map[i][1], self.path_map[i][0]))
            y1.append(self.mapToWorld(self.path_map[i][1], self.path_map[i][0])[0])
            y2.append(self.mapToWorld(self.path_map[i][1], self.path_map[i][0])[1])
            current_pose.pose.position.z = 0.0
            current_pose.pose.orientation.x = 0.0
            current_pose.pose.orientation.y = 0.0
            current_pose.pose.orientation.z = 0.0
            current_pose.pose.orientation.w = 1.0
            time += 1
            self.current_path.header.stamp = current_time
            self.current_path.header.frame_id = "map"
            self.current_path.poses.append(current_pose)
            self.path_pub.publish(self.current_path)
            self.last_time = current_time
            # rospy.sleep(1)


        # 通过差值做平滑处理
        length = len(self.path_map)
        x = np.array([num for num in range(length)])
        # print "-------------x:",x
        # print "-------------y1:",y1
        xnew = np.arange(0,length - 1, 0.1)
        # print "-------------xnew:",xnew
        func1 = interpolate.interp1d(x, y1, kind='cubic')
        func2 = interpolate.interp1d(x, y2, kind='cubic')
        ynew1 = func1(xnew)
        ynew2 = func2(xnew)
        for i in range(len(ynew1)):
            current_time = rospy.get_rostime()
            # print
            # print "current_time", current_time
            # rospy.sleep(.01)
            current_pose = PoseStamped()
            current_pose.pose.position.x = ynew1[i]
            current_pose.pose.position.y = ynew2[i]
            current_pose.pose.position.z = 0.0
            current_pose.pose.orientation.x = 0.0
            current_pose.pose.orientation.y = 0.0
            current_pose.pose.orientation.z = 0.0
            current_pose.pose.orientation.w = 1.0
            time += 1
            self.current_path_changed.header.stamp = current_time
            self.current_path_changed.header.frame_id = "map"
            self.current_path_changed.poses.append(current_pose)
            self.path_pub_changed.publish(self.current_path_changed)
            self.last_time = current_time
            rospy.sleep(.01)


if __name__ == '__main__':
    path_pub()
