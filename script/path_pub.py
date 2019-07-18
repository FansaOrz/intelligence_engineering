#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
import rospy
import time
import numpy as np
import math
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
        # 用于路径规划的分割合并
        self.line_database = []
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

    def smooth(self, pts):
        new_pts = []
        n_spline = 15

        idx = 1
        while idx + 1 < len(pts):
            m1 = (pts[idx - 1] + pts[idx]) / 2.
            m2 = (pts[idx] + pts[idx + 1]) / 2.
            k = float(np.sum(np.square(pts[idx - 1] - pts[idx]))) / np.sum(np.square(pts[idx] - pts[idx + 1]))
            m = (m1 + k * m2) / (1 + k)
            m1 += pts[idx] - m
            m2 += pts[idx] - m

            if idx == 1:
                for i in range(n_spline):
                    t = 1. * i / n_spline
                    p = ((1 - t) ** 2) * pts[0] + 2 * t * (1 - t) * m1 + (t ** 2) * pts[1]
                    new_pts.append(p)
            else:
                for i in range(n_spline):
                    t = 1. * i / n_spline
                    p = ((1 - t) ** 3) * pts[idx - 1] + 3 * t * ((1 - t) ** 2) * last_m2 + 3 * (t ** 2) * (
                                1 - t) * m1 + (t ** 3) * pts[idx]
                    new_pts.append(p)

            if idx == len(pts) - 2:
                for i in range(n_spline + 1):
                    t = 1. * i / n_spline
                    p = ((1 - t) ** 2) * pts[idx] + 2 * t * (1 - t) * m2 + (t ** 2) * pts[idx + 1]
                    new_pts.append(p)

            last_m2 = m2
            idx += 1

        return np.array(new_pts)

    def start_find_path(self):
        if self.if_start_find_path:

            print ('\033[32m[I] : Start find path with A* \033[0m')
            # 获取规划的路径
            self.path_map, open_list_index = self.Dij_find_path.start_find(self.start_map_point, self.goal_map_point)
            # 把遍历的open表节点更新到地图中
            self.path_map_be = self.path_map
            self.update_map(open_list_index)
            self.map_test_pub.publish(self.map_msg)
            # 分割合并算法，提取关键拐点
            self.split(self.path_map_be, 0.3)
            self.line_database = self.merge(self.line_database, 2.4)
            # 把关键拐点添加到新的path中
            self.path_map_be = []
            # 提取关键拐点
            for i in range(len(self.line_database)):
                self.path_map_be.append(np.array(self.line_database[i][0]))
            end_length = len(self.line_database[len(self.line_database) - 1]) - 1
            self.path_map_be.append(np.array(self.line_database[len(self.line_database) - 1][end_length]))

            print "-----------------------------------------"
            print len(self.path_map_be)
            print len(self.path_map_be[0])
            print self.path_map_be
            # 贝塞尔插值之后的路径
            self.path_map_be = self.smooth(self.path_map_be)
            print self.path_map_be
            # print self.path_map
            # 发布规划的路径
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

    def merge(self, line_dataset, threshold):
        new_line_dataset = [line_dataset[0]]
        # 遍历第二条直线到最后一条直线
        for i in range(1, len(line_dataset)):
            # 将相邻的两条直线拟合成一条,前一条直线的第一个点，后一条直线的最后一个点
            first_point = new_line_dataset[len(new_line_dataset) - 1][0]
            end_point = line_dataset[i][len(line_dataset[i]) - 1]
            # 计算直线方程
            x1 = first_point[0]
            y1 = first_point[1]
            x2 = end_point[0]
            y2 = end_point[1]
            a = y2 - y1
            b = x1 - x2
            c = y1 * x2 - y2 * x1
            # 声明最大距离变量
            max_dis = 0
            # 遍历第一条直线
            # print "22222"
            for j1 in range(len(new_line_dataset[len(new_line_dataset) - 1]) - 1):
                # print "111111"
                temp_point = new_line_dataset[len(new_line_dataset) - 1][j1]
                x = temp_point[0]
                y = temp_point[1]
                # 计算距离直线的距离
                current_dis = abs(a * x + b * y + c) / math.sqrt(math.pow(a, 2) + math.pow(b, 2))
                if current_dis > max_dis:
                    max_dis = current_dis
            # 遍历第二条直线
            for j2 in range(len(line_dataset[i]) - 1):
                temp_point = line_dataset[i][j2]
                x = temp_point[0]
                y = temp_point[1]
                # 计算距离直线的距离
                current_dis = abs(a * x + b * y + c) / math.sqrt(math.pow(a, 2) + math.pow(b, 2))
                if current_dis > max_dis:
                    max_dis = current_dis
            # 小于阈值就可以合并
            if max_dis < threshold:
                # print "1111111111", len(new_line_dataset)
                # 只需记录起点和终点
                temp_line = [first_point, end_point]
                # 如果合并了，就把最后一条直线删掉
                new_line_dataset[len(new_line_dataset) - 1] = temp_line
                # 把新的合并完的直线加到new里面
                # new_line_dataset.append(temp_line)
            else:
                new_line_dataset.append(line_dataset[i])
        return new_line_dataset

    def split(self, points, threshold):
        # print "split"
        points_length = len(points)
        # 假设直线方程为a*x+b*y+c=0,计算直线方程
        x1 = points[0][0]
        y1 = points[0][1]
        x2 = points[points_length - 1][0]
        y2 = points[points_length - 1][1]
        a = y2 - y1
        b = x1 - x2
        c = y1 * x2 - y2 * x1
        # 声明最大距离变量
        max_dis = 0
        max_dis_index = 0
        # 遍历当前路径中的所有节点，寻找最远距离的点
        for temp_point_index in range(points_length):
            x = points[temp_point_index][0]
            y = points[temp_point_index][1]
            current_dis = abs(a * x + b * y + c) / math.sqrt(math.pow(a, 2) + math.pow(b, 2))
            # 找到一个更大的，就更新最大值和下标
            if current_dis > max_dis:
                max_dis = current_dis
                max_dis_index = temp_point_index
        if max_dis > threshold:
            # print "more split"
            if max_dis_index - 0 == 1:
                self.line_database.append(points)
            else:
                self.split(points[0:max_dis_index + 1], threshold)
            if points_length - 1 - max_dis_index == 1:
                self.line_database.append(points)
            else:
                self.split(points[max_dis_index:points_length], threshold)
        else:
            # 如果不用分割了，就把当前这一条直线存到直线库中
            self.line_database.append(points)
            return
    def publisher_path(self):
        print "total path_length===============", len(self.path_map)
        time = 1
        y1 = []
        y2 = []
        for i in range(len(self.path_map)):
            current_time = rospy.get_rostime()

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

        print "total path_length_be===============", len(self.path_map_be)
        time = 1
        y1 = []
        y2 = []
        for i in range(len(self.path_map_be) - 1, -1, -1):
            current_time = rospy.get_rostime()

            current_pose = PoseStamped()
            current_pose.pose.position.x, current_pose.pose.position.y= self.mapToWorld(self.path_map_be[i][1], self.path_map_be[i][0])
            # self.path_world.append(self.mapToWorld(self.path_map[i][1], self.path_map[i][0]))
            y1.append(self.mapToWorld(self.path_map_be[i][1], self.path_map_be[i][0])[0])
            y2.append(self.mapToWorld(self.path_map_be[i][1], self.path_map_be[i][0])[1])
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
            # rospy.sleep(1)
            # rospy.sleep(1)






        # # 通过差值做平滑处理
        # length = len(self.path_map)
        # x = np.array([num for num in range(length)])
        # xnew = np.arange(0,length - 1, 0.1)
        # # print "-------------xnew:",xnew
        # func1 = interpolate.interp1d(x, y1, kind='cubic')
        # func2 = interpolate.interp1d(x, y2, kind='cubic')
        # ynew1 = func1(xnew)
        # ynew2 = func2(xnew)
        # for i in range(len(self.path_map_be)):
        #     current_time = rospy.get_rostime()
        #     # print
        #     # print "current_time", current_time
        #     # rospy.sleep(.01)
        #     current_pose = PoseStamped()
        #     current_pose.pose.position.x = ynew1[i]
        #     current_pose.pose.position.y = ynew2[i]
        #     current_pose.pose.position.z = 0.0
        #     current_pose.pose.orientation.x = 0.0
        #     current_pose.pose.orientation.y = 0.0
        #     current_pose.pose.orientation.z = 0.0
        #     current_pose.pose.orientation.w = 1.0
        #     time += 1
        #     self.current_path_changed.header.stamp = current_time
        #     self.current_path_changed.header.frame_id = "map"
        #     self.current_path_changed.poses.append(current_pose)
        #     self.path_pub_changed.publish(self.current_path_changed)
        #     self.last_time = current_time
        #     rospy.sleep(.01)


if __name__ == '__main__':
    path_pub()
