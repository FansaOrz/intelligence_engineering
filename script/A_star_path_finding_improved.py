#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
"""
本代码中的A星算法采用了八临近的方法：左右上下代价为10，斜对角线代价为14
F = G + H
G代表：从起点移动到指定方格的移动代价
H代表：从指定的方格移动到终点的估算成本（采用街区距离，只能上下左右走）
"""
import datetime
import numpy as np
import math

class find_path():
    def __init__(self, map, robot_size):
        # map是一个二维地图， start是起点坐标[]，goal是终点坐标[]
        self.map = self.extend_map(map)
        # 地图的宽度和长度
        self.width = len(self.map)
        self.height = len(self.map[0])
        # 每个节点有一个状态 2代表在open表中 3代表在close表中
        self.state_map = np.zeros([len(map) + 2, len(map[0]) + 2])
        # 记录一共遍历了多少个节点
        self.num_total = 0
        self.start = self.goal = []
        # 保存open和close表节点
        self.open_list = []
        self.cloase_list = []
        # 生成的路径
        self.path = []
        # 机器人的物理大小,此处的物理大小已经变换成栅格的数量
        self.robot_size = robot_size
        # 扩展的所有的节点，最后会在地图中显示出来
        self.open_list_index = []
        # 判断是否到达了目标点
        self.if_reach = False
        self.map_inflation()

    def map_inflation(self):
        print "inflation"
        # 遍历地图中所有的黑色的障碍物的节点
        for i in range(self.width):
            for j in range(self.height):
                if self.map[i][j] == 100:
                    # 是障碍物的话就把周围的白色变灰
                    for mi in range(2 * self.robot_size + 1):
                        for mj in range(2 * self.robot_size + 1):
                            if self.map[i - (self.robot_size - mi)][j - (self.robot_size - mj)] != 0:
                                continue
                            dis = math.sqrt(math.pow(mi - self.robot_size, 2) + math.pow(mj - self.robot_size, 2))
                            # print dis
                            # print math.pow(i - self.robot_size, 2)
                            # print mi - self.robot_size
                            # 大于机器人的半径，就不变灰
                            if dis > self.robot_size:
                                continue
                            # 否则就变灰
                            self.map[i - (self.robot_size - mi)][j - (self.robot_size - mj)] = 30
        print "done"

    # 在地图外围扩展一圈
    def extend_map(self, map):
        new_row = np.ones(len(map[0]))
        new_col = np.ones(len(map) + 2)
        x = np.insert(map, 0, new_row, axis=0)
        x = np.insert(x, len(map) + 1, new_row, axis=0)
        x = np.insert(x, 0 , new_col, axis=1)
        x = np.insert(x, len(map[0]) + 1 , new_col, axis=1)
        return x

    def start_find(self, start, goal):
        # 起始点（因为地图外围扩展了一圈，所以要加一）
        self.start = start
        self.start[0] += 1
        self.start[1] += 1
        # 目标点（因为地图外围扩展了一圈，所以要加一）
        self.goal = goal
        self.goal[0] += 1
        self.goal[1] += 1
        # 开始找路径时记录一下时间
        start_time = datetime.datetime.now()
        # 第一次操作
        if self.map[self.start[0]][self.start[1]] != 0:
            print "\033[0;31m[E] : Please set the valid start point\033[0m"
            print "value = ", self.map[self.start[0]][self.start[1]]
            return "None"
        if self.map[self.goal[0]][self.goal[1]] != 0:
            print "\033[0;31m[E] : Please set the valid goal point\033[0m"
            return "None"
        # 把起点的周围的点指向起点，起点和周围的点加到open list,
        self.append_around_open(self.start, cost_g=0)
        # 把起始节点加到close_list
        temp = map_node()
        temp.x = self.start[0]
        temp.y = self.start[1]
        self.append_close(temp)
        while True:
            # 寻找当前F值最小的节点，返回它在open表中的下标
            min_cost, index_min_cost = self.find_min_cost_f()
            current_node = self.open_list[index_min_cost]
            # 判断是否到达目标节点
            if current_node.x == self.goal[0] and current_node.y == self.goal[1]:
                self.append_path(current_node)
                break
            # 没有到达目标节点的话就把周围的节点加到open表中
            self.append_around_open([current_node.x, current_node.y], cost_g=current_node.cost_g)
            # 当前的节点加到close list
            self.append_close(current_node)
            self.open_list.remove(current_node)
        # 路径寻找完了，记录一下时间。最后比较一下程序运行的时间
        end_time = datetime.datetime.now()
        print "total_time====================", (end_time - start_time)
        # 一共有多少个节点加到了open表中
        print "total_num=====================", self.num_total
        return self.path, self.open_list_index

    # 最后找到终点，把最短路径append到path里
    def append_path(self, node):
        while True:
            self.path.append([node.x - 1, node.y - 1])
            if node.x == self.start[0] and node.y == self.start[1]:
                break
            current_index = self.find_close_index(node.parent[0], node.parent[1])
            node = self.cloase_list[current_index]

    # 寻找open表中的最小代价节点和index
    def find_min_cost_f(self):
        # 记录最小花费和其在openlist中的下标
        min_cost = 100000
        index_min_cost = 0
        for i in range(len(self.open_list)):
            if self.open_list[i].cost_f < min_cost:
                min_cost = self.open_list[i].cost_f
                index_min_cost = i
        return min_cost, index_min_cost

    # Python中缺少指针这个概念，所以每次找父节点要遍历一下
    def find_close_index(self, x, y):
        for i in range(len(self.cloase_list)):
            if self.cloase_list[i].x == x and self.cloase_list[i].y == y:
                return i

    def find_index(self, x, y):
        for i in range(len(self.open_list)):
            if self.open_list[i].x == x and self.open_list[i].y == y:
                return i

    def append_around_open(self, coordinate, cost_g):
        # 周围8个点
        # 左:如果是可以走的点 并且不在closelist中
        if self.map[coordinate[0]][coordinate[1] - 1] == 0 \
                and self.state_map[coordinate[0]][coordinate[1] - 1] != 3:
            self.open_list_index.append(([coordinate[0]],[coordinate[1] - 1]))
            # 记录遍历的节点个数
            self.num_total += 1
            temp = map_node()
            # 计算G和H代价
            temp.cost_g = 10 + cost_g
            temp.cost_h = (abs(self.goal[0] - coordinate[0]) + abs(self.goal[1] - (coordinate[1] - 1))) * 10
            temp.cost_f = temp.cost_g + temp.cost_h
            temp.x = coordinate[0]
            temp.y = coordinate[1] - 1
            # 链接父节点
            temp.parent[0] = coordinate[0]
            temp.parent[1] = coordinate[1]
            # print "temp", temp
            if self.state_map[coordinate[0]][coordinate[1] - 1] == 2:
                current_index = self.find_index(coordinate[0], coordinate[1] - 1)
                # 如果之前的cost比现在的cost大，就替换父节点和cost
                if self.open_list[current_index].cost_f > temp.cost_f:
                    self.open_list[current_index] = temp
            else:
                self.state_map[coordinate[0]][coordinate[1] - 1] = 2
                # 加入open list
                self.open_list.append(temp)

        # 左上:如果是可以走的点 并且不在openlist和closelist中
        if self.map[coordinate[0] - 1][coordinate[1] - 1] == 0 \
                and self.state_map[coordinate[0] - 1][coordinate[1] - 1] != 3:
            self.open_list_index.append(([coordinate[0] - 1],[coordinate[1] - 1]))
            # 记录遍历的节点个数
            self.num_total += 1
            temp = map_node()
            # 计算G和H代价
            temp.cost_g = 14 + cost_g
            temp.cost_h = (abs(self.goal[0] - (coordinate[0] - 1)) + abs(self.goal[1] - (coordinate[1] - 1))) * 10
            temp.cost_f = temp.cost_g + temp.cost_h
            temp.x = coordinate[0] - 1
            temp.y = coordinate[1] - 1
            # 链接父节点
            temp.parent[0] = coordinate[0]
            temp.parent[1] = coordinate[1]
            # print "temp", temp
            if self.state_map[coordinate[0] - 1][coordinate[1] - 1] == 2:
                current_index = self.find_index(coordinate[0] - 1, coordinate[1] - 1)
                # 如果之前的cost比现在的cost大，就替换父节点和cost
                if self.open_list[current_index].cost_f > temp.cost_f:
                    self.open_list[current_index] = temp
            else:
                self.state_map[coordinate[0] - 1][coordinate[1] - 1] = 2

                # 加入open list
                self.open_list.append(temp)
        # 上:如果是可以走的点 并且不在openlist和closelist中
        if self.map[coordinate[0] - 1][coordinate[1]] == 0 \
                and self.state_map[coordinate[0] - 1][coordinate[1]] != 3:
            self.open_list_index.append(([coordinate[0] - 1],[coordinate[1]]))
            # 记录遍历的节点个数
            self.num_total += 1
            temp = map_node()
            # 计算G和H代价
            temp.cost_g = 10 + cost_g
            temp.cost_h = (abs(self.goal[0] - (coordinate[0] - 1)) + abs(self.goal[1] - (coordinate[1]))) * 10
            temp.cost_f = temp.cost_g + temp.cost_h
            temp.x = coordinate[0] - 1
            temp.y = coordinate[1]
            # 链接父节点
            temp.parent[0] = coordinate[0]
            temp.parent[1] = coordinate[1]
            if self.state_map[coordinate[0] - 1][coordinate[1]] == 2:
                current_index = self.find_index(coordinate[0] - 1, coordinate[1])
                # 如果之前的cost比现在的cost大，就替换父节点和cost
                if self.open_list[current_index].cost_f > temp.cost_f:
                    self.open_list[current_index] = temp
            else:
                self.state_map[coordinate[0] - 1][coordinate[1]] = 2
                # 加入open list
                self.open_list.append(temp)
        # 右上:如果是可以走的点 并且不在openlist和closelist中
        if self.map[coordinate[0] - 1][coordinate[1] + 1] == 0 \
                and self.state_map[coordinate[0] - 1][coordinate[1] + 1] != 3:
            self.open_list_index.append(([coordinate[0] - 1],[coordinate[1] + 1]))
            # 记录遍历的节点个数
            self.num_total += 1
            temp = map_node()
            # 计算G和H代价
            temp.cost_g = 14 + cost_g
            temp.cost_h = (abs(self.goal[0] - (coordinate[0] - 1)) + abs(self.goal[1] - (coordinate[1] + 1))) * 10
            temp.cost_f = temp.cost_g + temp.cost_h
            temp.x = coordinate[0] - 1
            temp.y = coordinate[1] + 1
            # 链接父节点
            temp.parent[0] = coordinate[0]
            temp.parent[1] = coordinate[1]
            if self.state_map[coordinate[0] - 1][coordinate[1] + 1] == 2:
                current_index = self.find_index(coordinate[0] - 1, coordinate[1] + 1)
                # 如果之前的cost比现在的cost大，就替换父节点和cost
                if self.open_list[current_index].cost_f > temp.cost_f:
                    self.open_list[current_index] = temp
            else:
                self.state_map[coordinate[0] - 1][coordinate[1] + 1] = 2

                # 加入open list
                self.open_list.append(temp)
        # 右:如果是可以走的点 并且不在openlist和closelist中
        if self.map[coordinate[0]][coordinate[1] + 1] == 0 \
                and self.state_map[coordinate[0]][coordinate[1] + 1] != 3:
            self.open_list_index.append(([coordinate[0]],[coordinate[1] + 1]))
            # 记录遍历的节点个数
            self.num_total += 1
            temp = map_node()
            # 计算G和H代价
            temp.cost_g = 10 + cost_g
            temp.cost_h = (abs(self.goal[0] - (coordinate[0])) + abs(self.goal[1] - (coordinate[1] + 1))) * 10
            temp.cost_f = temp.cost_g + temp.cost_h
            temp.x = coordinate[0]
            temp.y = coordinate[1] + 1
            # 链接父节点
            temp.parent[0] = coordinate[0]
            temp.parent[1] = coordinate[1]
            if self.state_map[coordinate[0]][coordinate[1] + 1] == 2:
                current_index = self.find_index(coordinate[0], coordinate[1] + 1)
                # 如果之前的cost比现在的cost大，就替换父节点和cost
                if self.open_list[current_index].cost_f > temp.cost_f:
                    self.open_list[current_index] = temp
            else:
                self.state_map[coordinate[0]][coordinate[1] + 1] = 2

                # 加入open list
                self.open_list.append(temp)
        # 右下:如果是可以走的点 并且不在openlist和closelist中
        if self.map[coordinate[0] + 1][coordinate[1] + 1] == 0 \
                and self.state_map[coordinate[0] + 1][coordinate[1] + 1] != 3:
            self.open_list_index.append(([coordinate[0] + 1],[coordinate[1] + 1]))
            # 记录遍历的节点个数
            self.num_total += 1
            temp = map_node()
            # 计算G和H代价
            temp.cost_g = 14 + cost_g
            temp.cost_h = (abs(self.goal[0] - (coordinate[0] + 1)) + abs(self.goal[1] - (coordinate[1] + 1))) * 10
            temp.cost_f = temp.cost_g + temp.cost_h
            temp.x = coordinate[0] + 1
            temp.y = coordinate[1] + 1
            # 链接父节点
            temp.parent[0] = coordinate[0]
            temp.parent[1] = coordinate[1]
            if self.state_map[coordinate[0] + 1][coordinate[1] + 1] == 2:
                current_index = self.find_index(coordinate[0] + 1, coordinate[1] + 1)
                # 如果之前的cost比现在的cost大，就替换父节点和cost
                if self.open_list[current_index].cost_f > temp.cost_f:
                    self.open_list[current_index] = temp
            else:
                self.state_map[coordinate[0] + 1][coordinate[1] + 1] = 2

                # 加入open list
                self.open_list.append(temp)
        # 下:如果是可以走的点 并且不在openlist和closelist中
        if self.map[coordinate[0] + 1][coordinate[1]] == 0 \
                and self.state_map[coordinate[0] + 1][coordinate[1]] != 3:
            self.open_list_index.append(([coordinate[0] + 1],[coordinate[1]]))
            # 记录遍历的节点个数
            self.num_total += 1
            temp = map_node()
            # 计算G和H代价
            temp.cost_g = 10 + cost_g
            temp.cost_h = (abs(self.goal[0] - (coordinate[0] + 1)) + abs(self.goal[1] - (coordinate[1]))) * 10
            temp.cost_f = temp.cost_g + temp.cost_h
            temp.x = coordinate[0] + 1
            temp.y = coordinate[1]
            # 链接父节点
            temp.parent[0] = coordinate[0]
            temp.parent[1] = coordinate[1]
            if self.state_map[coordinate[0] + 1][coordinate[1]] == 2:
                current_index = self.find_index(coordinate[0] + 1, coordinate[1])
                # 如果之前的cost比现在的cost大，就替换父节点和cost
                if self.open_list[current_index].cost_f > temp.cost_f:
                    self.open_list[current_index] = temp
            else:
                self.state_map[coordinate[0] + 1][coordinate[1]] = 2

                # 加入open list
                self.open_list.append(temp)
        # 左下:如果是可以走的点 并且不在openlist和closelist中
        if self.map[coordinate[0] + 1][coordinate[1] - 1] == 0 \
                and self.state_map[coordinate[0] + 1][coordinate[1] - 1] != 3:
            self.open_list_index.append(([coordinate[0] + 1],[coordinate[1] - 1]))
            # 记录遍历的节点个数
            self.num_total += 1
            temp = map_node()
            # 计算G和H代价
            temp.cost_g = 14 + cost_g
            temp.cost_h = (abs(self.goal[0] - (coordinate[0] + 1)) + abs(self.goal[1] - (coordinate[1] - 1))) * 10
            temp.cost_f = temp.cost_g + temp.cost_h
            temp.x = coordinate[0] + 1
            temp.y = coordinate[1] - 1
            # 链接父节点
            temp.parent[0] = coordinate[0]
            temp.parent[1] = coordinate[1]
            if self.state_map[coordinate[0] + 1][coordinate[1] - 1] == 2:
                current_index = self.find_index(coordinate[0] + 1, coordinate[1] - 1)
                # 如果之前的cost比现在的cost大，就替换父节点和cost
                if self.open_list[current_index].cost_f > temp.cost_f:
                    self.open_list[current_index] = temp
            else:
                self.state_map[coordinate[0] + 1][coordinate[1] - 1] = 2

                # 加入open list
                self.open_list.append(temp)

    def append_close(self, node):
        # 更改节点状态
        self.state_map[node.x][node.y] = 3
        self.cloase_list.append(node)

# 自定义地图节点，每个节点有父节点，G、H、F数值
class map_node():
    def __init__(self):
        self.x = 0
        self.y = 0
        self.cost_f = 0
        self.cost_g = 0
        self.cost_h = 0
        self.parent = [0,0]
        self.state = 0

if __name__ == '__main__':
    # map_ = [[1,0,0,1,0],[1,0,0,1,0],[1,0,1,1,0],[1,0,0,0,0]]
    # for i in range(4):
    # map_ = [[0,0,0,0,1,0,1,0,0,0,0],
    #         [0,1,0,0,1,0,1,0,0,0,0],
    #         [0,1,0,0,1,0,1,1,1,1,0],
    #         [0,1,1,1,1,0,1,0,0,0,0],
    #         [0,1,0,0,0,0,1,0,0,0,0],
    #         [0,1,0,1,0,0,1,0,1,0,0],
    #         [0,0,0,1,0,0,0,0,1,0,0],
    #         [0,0,0,1,1,1,1,1,1,0,0],
    #         [0,0,0,1,0,0,0,0,0,0,0]]
    map_ = [[0,0,0,0,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,0,0,0,0],
            [0,0,0,0,0,1,0,0,0,0,0],
            [0,0,0,0,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,0,0,0,0]]
    #     for j in range(4):
    #         print map_[i][j]
    temp = find_path(map_, [5,0], [5,9])
    # print temp.state_map
    # print temp.extend_map()
    print temp.start_find()