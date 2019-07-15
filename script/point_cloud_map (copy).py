#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
import cv2
import math
import numpy as np
from numpy.linalg import norm
import rospy
import tf
from copy import deepcopy
from tf import transformations
from math import pi
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData
import thread

class Scan_Reg():

    def __init__(self):

        rospy.init_node("scan_reg")
        self.map_pub = rospy.Publisher("/my_map", OccupancyGrid, queue_size=15)
        self.map_metadata_pub = rospy.Publisher("/map_metadata", MapMetaData, queue_size=15)
        self.map = OccupancyGrid()
        self.br = tf.TransformBroadcaster()
        self.map.header.frame_id = "map"

        # 自己定义地图的信息
        self.resolution = 0.05
        self.width = self.height = 500
        self.origin_x = -5
        self.origin_y = -12.5
        self.map.info.resolution = self.resolution
        self.map.info.origin.position.x = self.origin_x
        self.map.info.origin.position.y = self.origin_y
        self.map.info.origin.orientation.w = 1
        self.map.info.width = self.width
        self.map.info.height = self.height
        self.map = self.init_map(self.width, self.height, self.map)
        # 定义两个scan信息
        self.scan_old = []
        self.scan_new = []
        self.kd_new = None
        # 回调函数
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.Scan_cb)
        self.if_first = True
        # 当前laser的位置
        self.current_pose = Pose()
        self.current_pose.orientation.w = 1
        # 初始旋转矩阵
        self.init_R = [[1,0,0],[0,1,0],[0,0,1]]
        # 降采样参数
        self.down_sample = 1
        # scan采样步长
        self.scan_step = None
        # 计数、接受scan多少次了
        self.scan_num = 0
        # 开启三个线程
        self.start_tf_pub()
        self.start_map_pub()
        # self.start_cal_map()
        print self.current_pose
        rospy.spin()

    # 根据高度和宽度初始化地图，使每个点都是-1
    def init_map(self, width, height, map):
        for i in range(width * height):
            map.data.append(0)
        return map

    # 开启一个线程发布tf变换
    def start_tf_pub(self):
        arg = tuple([1])
        thread.start_new_thread(self.tf_pub, arg)

    # 开启一个线程发布map
    def start_map_pub(self):
        arg = tuple([1])
        thread.start_new_thread(self.mymap_pub, arg)

    # 开启一个线程计算map
    def start_cal_map(self):
        arg = tuple([1])
        thread.start_new_thread(self.cal_map, arg)

    # tf发布的线程
    def tf_pub(self, arg):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            # print "-------"
            self.br.sendTransform((self.current_pose.position.x, self.current_pose.position.y, self.current_pose.position.z),
                              (self.current_pose.orientation.x, self.current_pose.orientation.y, self.current_pose.orientation.z, self.current_pose.orientation.w),
                              rospy.Time.now(),
                              "camera_link",
                              "map")
            rate.sleep()

    # map发布的线程
    def mymap_pub(self, arg):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            self.map.header.seq += 1
            self.map.header.stamp = rospy.Time.now()
            self.map.info.map_load_time = rospy.Time.now()
            self.map_pub.publish(self.map)
            self.map_metadata_pub.publish(self.map.info)
            rate.sleep()

    # 更新地图
    def cal_map(self, msg):
        for i in range(0, len(self.Points), self.down_sample):
            # 小于min或者大于max
            if self.Points[i] < self.range_min or self.Points[i] > self.range_max or math.isnan(self.Points[i]):
                continue
            if i >= len(self.Points) / 2:
                theta_i = ((i + 1) * self.angle_increment + self.angle_min)
                y_i = math.sin(theta_i) * self.Points[i]
                x_i = math.cos(theta_i) * self.Points[i]
                # theta_i -= self.angle_max
                mix, miy = self.WorldTomap(self.current_pose.position.x + x_i, self.current_pose.position.y + y_i)
                # print "-------------"
            else:
                theta_i = self.angle_max - ((i + 1) * self.angle_increment)
                y_i = math.sin(theta_i) * self.Points[i]
                x_i = math.cos(theta_i) * self.Points[i]
                mix, miy = self.WorldTomap( self.current_pose.position.x + x_i, self.current_pose.position.y - y_i)
                # print "+++++++"
            # print self.current_pose.position.x + x_i, self.current_pose.position.y + y_i
            # print theta_i
            self.map.data[mix * 500 + miy] = 100

            # print i
        print "add to map"

    # 世界坐标系转地图坐标系
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

    def register_scan(self, points, step):
        for i in range(0, len(points), step):
            # print i
            # print step
            if points[i] < self.range_min or points[i] > self.range_max or math.isnan(points[i]):
                continue
            if i >= len(points) / 2:
                theta_i = ((i + 1) * self.angle_increment + self.angle_min)
            else:
                theta_i = self.angle_max - ((i + 1) * self.angle_increment)
            y_i = math.sin(theta_i) * points[i]
            x_i = math.cos(theta_i) * points[i]
            self.scan_new.append([x_i, y_i])
            # print "------len", len(self.scan_new)
        # print self.scan_new
        self.kd_new = KDTree(self.scan_new)

        # 初始化两个scan
        # self.scan_old = self.scan_new

    # SVD函数
    def SVD_cal_R_t(self):
        # print "SVD"
        ave_p_x = 0
        ave_p_y = 0
        ave_q_x = 0
        ave_q_y = 0
        for i in range(len(self.scan_old)):
            ave_p_x += self.scan_old[i][0]
            ave_p_y += self.scan_old[i][1]
            ave_q_x += self.scan_new[i][0]
            ave_q_y += self.scan_new[i][1]
        ave_p = [[ave_p_x / len(self.scan_old)], [ave_p_y / len(self.scan_old)], [0]]
        ave_q = [[ave_q_x / len(self.scan_old)], [ave_q_y / len(self.scan_old)], [0]]
        Y = X = []
        for i in range(len(self.scan_old)):
            yi = [self.scan_new[i][0] - ave_q[0][0], self.scan_new[i][1] - ave_q[1][0], 0]
            xi = [self.scan_old[i][0] - ave_p[0][0], self.scan_old[i][1] - ave_p[1][0], 0]
            Y.append(yi)
            X.append(xi)
        # Y = np.array(Y)
        Y = np.mat(Y)
        # X = np.array(X)
        X = np.mat(X)
        # print "qqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqq"
        # print X
        X = np.transpose(X)
        # print X
        # print "-------------"
        # print X
        # print "-------------"
        # print Y
        # print "-------------"
        # print X*Y
        print X*Y
        u, s, v = np.linalg.svd(X*Y)
        print u
        print s
        print v
        # print s*v
        # print len(ave_p)
        # print len(ave_q)
        R = np.transpose(v)*np.transpose(u)
        # print R.shape
        t = ave_q - R*ave_p
        print "=====================R======================="
        print R
        print "=====================t======================="
        print t
        return R, t

    def Rotation_to_quaternion(self, R):
        # print R[0,0]
        w = math.sqrt(1+R[0, 0]+R[1, 1]+R[2, 2])/2
        x = (R[2, 1]-R[1, 2])/(4*w)
        y = (R[0, 2]-R[2, 0])/(4*w)
        z = (R[1, 0]-R[0, 1])/(4*w)
        return x,y,z,w

    # 订阅激光雷达的消息
    def Scan_cb(self, msg):
        # 第一次的话就存下来各个数据， 然后更新第一次地图信息
        if self.if_first:
            self.angle_min = msg.angle_min
            self.angle_max = msg.angle_max
            self.angle_increment = msg.angle_increment
            self.range_min = msg.range_min
            self.range_max = msg.range_max
            self.if_first = False
            self.Points = msg.ranges
            # 每条scan取10个点
            self.scan_step = len(msg.ranges) / 10
            # 记录这10个点的世界坐标
            self.register_scan(msg.ranges, self.scan_step)
            self.scan_old = self.scan_new
            self.start_cal_map()
            return
        # 计数一次
        self.scan_num += 1
        # 记录这10个点的世界坐标
        self.scan_new = []
        self.register_scan(msg.ranges, self.scan_step)
        self.scan_new = []

        # print "---------------len", len(self.scan_new)
        # print "---------------len", len(self.scan_old)
        # print "-------"
        # print self.scan_old
        # print "-------"
        # print self.scan_new
        # print "-------"
        for i in range(5):
            self.scan_new = []
            # TODO 检查old和new顺序是不是一样的
            for i in range(len(self.scan_old)):
                self.scan_new.append(self.kd_new.search_nn(self.scan_old[i])[0].data)
            # 清空kd树

            # 利用SVD计算R和t
            R, t = self.SVD_cal_R_t()
            print "num===========================",i
            for i in range(len(self.scan_old)):
                # X = R[0:2, 0:2] * [[self.scan_old[i][0]], [self.scan_old[i][1]]] + [[t[0, 0]],[t[1, 0]]]
                # print X
                temp = [[self.scan_old[i][0]], [self.scan_old[i][0]], [0]]
                X_ = R*temp + t
                # print type(X_)
                self.scan_old[i][0] = X_[0,0]
                self.scan_old[i][1] = X_[1,0]

                # print self.scan_old[i][0] - temp[0][0]
                # print self.scan_old[i][1] - temp[1][0]
                # self.scan_old[i] = [X[0],X[1]]

            # print "=========================="
            # for i in range(len(self.scan_old)):
            #     print self.scan_new[i][0] - self.scan_old[i][0],self.scan_new[i][1] - self.scan_old[i][1]
            # print "=========================="
        self.kd_new = None
        # 更新currentpose
        self.current_pose.position.x += t[0][0]
        self.current_pose.position.y += t[1][0]
        # self.current_pose.position.x += t[0][0]
        # print transformations.translation_from_matrix([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        # transformations.quaternion_from_euler()
        self.init_R = R * self.init_R
        print self.init_R.shape
        x,y,z,w = self.Rotation_to_quaternion(self.init_R)
        self.current_pose.orientation.x = x
        self.current_pose.orientation.y = y
        self.current_pose.orientation.z = z
        self.current_pose.orientation.w = w
        if self.scan_num == 10:
            self.scan_num = 0
            self.cal_map(msg.ranges)

    # def split_and_merge(self, msg):

def max_heapreplace(heap, new_node, key=lambda x: x[1]):
    heap[0] = new_node
    root, child = 0, 1
    end = len(heap) - 1
    while child <= end:
        if child < end and key(heap[child]) < key(heap[child + 1]):
            child += 1
        if key(heap[child]) <= key(new_node):
            break
        heap[root] = heap[child]
        root, child = child, 2 * child + 1
    heap[root] = new_node


def max_heappush(heap, new_node, key=lambda x: x[1]):
    heap.append(new_node)
    pos = len(heap) - 1
    while 0 < pos:
        parent_pos = pos - 1 >> 1
        if key(new_node) <= key(heap[parent_pos]):
            break
        heap[pos] = heap[parent_pos]
        pos = parent_pos
    heap[pos] = new_node

def partition_sort(arr, k, key=lambda x: x):
    start, end = 0, len(arr) - 1
    assert 0 <= k <= end
    while True:
        i, j, pivot = start, end, deepcopy(arr[start])
        while i < j:
            # 从右向左查找较小元素
            while i < j and key(pivot) <= key(arr[j]):
                j -= 1
            if i == j: break
            arr[i] = arr[j]
            i += 1
            # 从左向右查找较大元素
            while i < j and key(arr[i]) <= key(pivot):
                i += 1
            if i == j: break
            arr[j] = arr[i]
            j -= 1
        arr[i] = pivot

        if i == k:
            return
        elif i < k:
            start = i + 1
        else:
            end = i - 1

class KDNode(object):
    def __init__(self, data=None, label=None, left=None, right=None, axis=None, parent=None):
        self.data = data
        self.label = label
        self.left = left
        self.right = right
        self.axis = axis
        self.parent = parent


class KDTree(object):

    def __init__(self, X, y=None):
        self.root = None
        self.y_valid = False if y is None else True
        self.create(X, y)

    def create(self, X, y=None):
        def create_(X, axis, parent=None):
            n_samples = np.shape(X)[0]
            if n_samples == 0:
                return None
            mid = n_samples >> 1
            partition_sort(X, mid, key=lambda x: x[axis])

            if self.y_valid:
                kd_node = KDNode(X[mid][:-1], X[mid][-1], axis=axis, parent=parent)
            else:
                kd_node = KDNode(X[mid], axis=axis, parent=parent)

            next_axis = (axis + 1) % k_dimensions
            kd_node.left = create_(X[:mid], next_axis, kd_node)
            kd_node.right = create_(X[mid + 1:], next_axis, kd_node)
            return kd_node

        print('building kd-tree...')
        k_dimensions = np.shape(X)[1]
        if y is not None:
            X = np.hstack((np.array(X), np.array([y]).T)).tolist()
        self.root = create_(X, 0)

    def search_knn(self, point, k, dist=None):
        def search_knn_(kd_node):
            if kd_node is None:
                return
            data = kd_node.data
            distance = p_dist(data)
            if len(heap) < k:
                # 向大根堆中插入新元素
                max_heappush(heap, (kd_node, distance))
            elif distance < heap[0][1]:
                # 替换大根堆堆顶元素
                max_heapreplace(heap, (kd_node, distance))

            axis = kd_node.axis
            if abs(point[axis] - data[axis]) < heap[0][1] or len(heap) < k:
                # 当前最小超球体与分割超平面相交或堆中元素少于k个
                search_knn_(kd_node.left)
                search_knn_(kd_node.right)
            elif point[axis] < data[axis]:
                search_knn_(kd_node.left)
            else:
                search_knn_(kd_node.right)

        if self.root is None:
            raise Exception('kd-tree must be not null.')
        if k < 1:
            raise ValueError("k must be greater than 0.")

        # 默认使用2范数度量距离
        if dist is None:
            p_dist = lambda x: norm(np.array(x) - np.array(point))
        else:
            p_dist = lambda x: dist(x, point)

        heap = []
        search_knn_(self.root)
        return sorted(heap, key=lambda x: x[1])

    def search_nn(self, point, dist=None):
        return self.search_knn(point, 1, dist)[0]

    def pre_order(self, root=KDNode()):
        """先序遍历"""
        if root is None:
            return
        elif root.data is None:
            root = self.root

        yield root
        for x in self.pre_order(root.left):
            yield x
        for x in self.pre_order(root.right):
            yield x

    def lev_order(self, root=KDNode(), queue=None):
        """层次遍历"""
        if root is None:
            return
        elif root.data is None:
            root = self.root
        if queue is None:
            queue = []
        yield root
        if root.left:
            queue.append(root.left)
        if root.right:
            queue.append(root.right)
        if queue:
            for x in self.lev_order(queue.pop(0), queue):
                yield x

    @classmethod
    def height(cls, root):
        """kd-tree深度"""
        if root is None:
            return 0
        else:
            return max(cls.height(root.left), cls.height(root.right)) + 1

if __name__ == '__main__':
    Scan_Reg()