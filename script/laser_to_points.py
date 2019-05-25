#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

import numpy as np
from numpy.linalg import norm
from copy import deepcopy
import random
from time import time


def max_heapreplace(heap, new_node, key=lambda x: x[1]):
    """
    大根堆替换堆顶元素

    :param heap: 大根堆/列表
    :param new_node: 新节点
    :return: None
    """
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
    """
    大根堆插入元素

    :param heap: 大根堆/列表
    :param new_node: 新节点
    :return: None
    """
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

    """
    以枢纽(位置k)为中心将数组划分为两部分, 枢纽左侧的元素不大于枢纽右侧的元素

    :param arr: 待划分数组
    :param p: 枢纽前部元素个数
    :param key: 比较方式
    :return: None
    """
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
    """kd树节点"""

    def __init__(self, data=None, label=None, left=None, right=None, axis=None, parent=None):
        """
        :param data: 数据
        :param label: 数据标签
        :param left: 左孩子节点
        :param right: 右孩子节点
        :param axis: 分割轴
        :param parent: 父节点
        """
        self.data = data
        self.label = label
        self.left = left
        self.right = right
        self.axis = axis
        self.parent = parent


class KDTree(object):
    """kd树"""

    def __init__(self, X, y=None):
        """
        构造函数

        :param X: 输入特征集, n_samples*n_features
        :param y: 输入标签集, 1*n_samples
        """
        self.root = None
        self.y_valid = False if y is None else True
        self.create(X, y)

    def create(self, X, y=None):
        """
        构建kd树

        :param X: 输入特征集, n_samples*n_features
        :param y: 输入标签集, 1*n_samples
        :return: KDNode
        """

        def create_(X, axis, parent=None):
            """
            递归生成kd树

            :param X: 合并标签后输入集
            :param axis: 切分轴
            :param parent: 父节点
            :return: KDNode
            """
            n_samples = np.shape(X)[0]
            if n_samples == 0:
                return None
            # 右移一位
            mid = n_samples >> 1
            # print "mid", mid
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
        print "-----------------------------"
        """
        kd树中搜索k个最近邻样本

        :param point: 样本点
        :param k: 近邻数
        :param dist: 度量方式
        :return:
        """

        def search_knn_(kd_node):
            print "+++++++++++++++++++"
            """
            搜索k近邻节点

            :param kd_node: KDNode
            :return: None
            """
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
        """
        搜索point在样本集中的最近邻

        :param point:
        :param dist:
        :return:
        """
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
    N = 6
    X = [[2,3], [5,4], [9,6], [4, 7], [8, 1], [7, 2]]
    print X
    kd_tree = KDTree(X)

    # for x in X[:10]:
    #     res1 = ([list(node[0].data) for node in kd_tree.search_knn(x, 20)])
    #     distances = norm(np.array(X) - np.array(x), axis=1)
    #     res2 = ([list(X[i]) for _, i in sorted(zip(distances, range(N)))[:20]])
    #     if all(x in res2 for x in res1):
    #         print('correct ^_^ ^_^')
    #     else:
    #         print('error >_< >_<')
    # print('\n')

    """10万个样本集中查找10个实例的最近邻"""
    n = 3
    indices = random.sample(range(N), n)
    print indices
    # 1、kd-tree搜索, 0.19251227378845215s
    tm = time()
    for i, index in enumerate(indices):
        print  kd_tree.search_nn([2, 4])[0].data
        # print
    print('kd-tree search: {}s'.format(time() - tm))
