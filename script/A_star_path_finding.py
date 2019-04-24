#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
'''
本代码中的A星算法采用了八临近的方法：左右上下代价为10，斜对角线代价为14
F = G + H
G代表：从起点移动到指定方格的移动代价
H代表：从指定的方格移动到终点的估算成本（采用街区距离，只能上下左右走）

'''
import numpy as np

class find_path():
    def __init__(self, map, start, goal):
        # map是一个二维地图， start是起点坐标[]，goal是终点坐标[]
        self.map = map
        # 2代表在open表中 3代表在close表中
        self.state_map = map
        self.start = start
        self.goal = goal
        self.open_list = []
        self.cloase_list = []
        self.path = []
        self.if_reach = False

    def start_find(self):
        #第一次操作，把起点的周围的点指向起点，起点和周围的点加到open list
        self.append_around_open(self.start)
        # 把起始节点加到close_list
        temp = map_node()
        temp.x = self.start[0]
        temp.y = self.start[1]
        self.append_close(temp)
        while not self.if_reach:
            min_cost, index_min_cost = self.find_min_cost_f()
            current_node = self.open_list[index_min_cost]
            # 如果最小的节点正好等于终点
            if current_node.x == self.goal.x and current_node.y == self.goal.y:
                self.if_reach = True
                self.append_path(current_node)
            self.append_around_open([current_node.x, current_node.y])
        return self.path

    # 最后找到终点，把最短路径append到path里
    def append_path(self, node):
        while True:
            self.path.append([node.x, node.y])
            if node.x == self.start[0] and node.y == self.start[1]:
                break
            current_index = self.find_index(node.parent[0], node.parent[1])
            node = self.open_list[current_index]

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

    def find_index(self, x, y):
        for i in range(len(self.open_list)):
            if self.open_list[i].x == x and self.open_list[i].y == y:
                return i

    def append_around_open(self, coordinate):
        #周围8个点
        #左:如果是可以走的点 并且不在closelist中
        if self.map[coordinate[0]][coordinate[1] - 1] == 0 \
                and self.state_map[coordinate[0]][coordinate[1] - 1] != 3:
            temp = map_node()
            # 计算G和H代价
            temp.cost_h = 10
            temp.cost_g = (self.goal[0] - coordinate[0] + self.goal[1] - (coordinate[1] - 1)) * 10
            temp.cost_f = temp.cost_g + temp.cost_h
            temp.x = coordinate[0]
            temp.y = coordinate[1] - 1
            #链接父节点
            temp.parent[0] = coordinate[0]
            temp.parent[1] = coordinate[1]
            self.state_map[coordinate[0]][coordinate[1] - 1] = 2
            print "temp", temp
            if self.state_map[coordinate[0]][coordinate[1] - 1] == 2:
                current_index = self.find_index(coordinate[0], coordinate[1] - 1)
                # 如果之前的cost比现在的cost大，就替换父节点和cost
                if self.open_list[current_index].cost_f > temp.cost_f:
                    self.open_list[current_index] = temp
                else:
                    # 加入open list
                    self.open_list.append(temp)
            else:
                # 加入open list
                self.open_list.append(temp)







        #左上:如果是可以走的点 并且不在openlist和closelist中
        if self.map[coordinate[0] - 1][coordinate[1] - 1] == 0 \
                and self.state_map[coordinate[0] - 1][coordinate[1] - 1] != 3:
            temp = map_node()
            # 计算G和H代价
            temp.cost_h = 10
            temp.cost_g = (self.goal[0] - (coordinate[0] - 1) + self.goal[1] - (coordinate[1] - 1)) * 10
            temp.cost_f = temp.cost_g + temp.cost_h
            temp.x = coordinate[0] - 1
            temp.y = coordinate[1] - 1
            # 链接父节点
            temp.parent[0] = coordinate[0]
            temp.parent[1] = coordinate[1]
            self.state_map[coordinate[0] - 1][coordinate[1] - 1] = 2
            print "temp", temp
            if self.state_map[coordinate[0] - 1][coordinate[1] - 1] == 2:
                current_index = self.find_index(coordinate[0] - 1, coordinate[1] - 1)
                # 如果之前的cost比现在的cost大，就替换父节点和cost
                if self.open_list[current_index].cost_f > temp.cost_f:
                    self.open_list[current_index] = temp
                else:
                    # 加入open list
                    self.open_list.append(temp)
            else:
                # 加入open list
                self.open_list.append(temp)
        #上:如果是可以走的点 并且不在openlist和closelist中
        if self.map[coordinate[0] - 1][coordinate[1]] == 0 \
                and self.state_map[coordinate[0] - 1][coordinate[1]] != 3:
            temp = map_node()
            # 计算G和H代价
            temp.cost_h = 10
            temp.cost_g = (self.goal[0] - (coordinate[0] - 1) + self.goal[1] - (coordinate[1])) * 10
            temp.cost_f = temp.cost_g + temp.cost_h
            temp.x = coordinate[0] - 1
            temp.y = coordinate[1]
            # 链接父节点
            temp.parent[0] = coordinate[0]
            temp.parent[1] = coordinate[1]
            self.state_map[coordinate[0] - 1][coordinate[1]] = 2
            print "temp", temp
            if self.state_map[coordinate[0] - 1][coordinate[1] - 1] == 2:
                current_index = self.find_index(coordinate[0] - 1, coordinate[1])
                # 如果之前的cost比现在的cost大，就替换父节点和cost
                if self.open_list[current_index].cost_f > temp.cost_f:
                    self.open_list[current_index] = temp
                else:
                    # 加入open list
                    self.open_list.append(temp)
            else:
                # 加入open list
                self.open_list.append(temp)
        #右上:如果是可以走的点 并且不在openlist和closelist中
        if self.map[coordinate[0] - 1][coordinate[1] + 1] == 0 \
                and self.state_map[coordinate[0] - 1][coordinate[1] + 1] != 3:
            temp = map_node()
            # 计算G和H代价
            temp.cost_h = 10
            temp.cost_g = (self.goal[0] - (coordinate[0] - 1) + self.goal[1] - (coordinate[1] + 1)) * 10
            temp.cost_f = temp.cost_g + temp.cost_h
            temp.x = coordinate[0] - 1
            temp.y = coordinate[1] + 1
            # 链接父节点
            temp.parent[0] = coordinate[0]
            temp.parent[1] = coordinate[1]
            self.state_map[coordinate[0] - 1][coordinate[1] + 1] = 2
            print "temp", temp
            if self.state_map[coordinate[0] - 1][coordinate[1] + 1] == 2:
                current_index = self.find_index(coordinate[0] - 1, coordinate[1] + 1)
                # 如果之前的cost比现在的cost大，就替换父节点和cost
                if self.open_list[current_index].cost_f > temp.cost_f:
                    self.open_list[current_index] = temp
                else:
                    # 加入open list
                    self.open_list.append(temp)
            else:
                # 加入open list
                self.open_list.append(temp)
        #右:如果是可以走的点 并且不在openlist和closelist中
        if self.map[coordinate[0]][coordinate[1] + 1] == 0 \
                and self.state_map[coordinate[0]][coordinate[1] + 1] != 3:
            temp = map_node()
            # 计算G和H代价
            temp.cost_h = 10
            temp.cost_g = (self.goal[0] - (coordinate[0]) + self.goal[1] - (coordinate[1] + 1)) * 10
            temp.cost_f = temp.cost_g + temp.cost_h
            temp.x = coordinate[0] - 1
            temp.y = coordinate[1] - 1
            # 链接父节点
            temp.parent[0] = coordinate[0]
            temp.parent[1] = coordinate[1]
            self.state_map[coordinate[0]][coordinate[1] + 1] = 2
            print "temp", temp
            if self.state_map[coordinate[0]][coordinate[1] + 1] == 2:
                current_index = self.find_index(coordinate[0], coordinate[1] + 1)
                # 如果之前的cost比现在的cost大，就替换父节点和cost
                if self.open_list[current_index].cost_f > temp.cost_f:
                    self.open_list[current_index] = temp
                else:
                    # 加入open list
                    self.open_list.append(temp)
            else:
                # 加入open list
                self.open_list.append(temp)
        #右下:如果是可以走的点 并且不在openlist和closelist中
        if self.map[coordinate[0] + 1][coordinate[1] + 1] == 0 \
                and self.state_map[coordinate[0] + 1][coordinate[1] + 1] != 3:
            temp = map_node()
            # 计算G和H代价
            temp.cost_h = 10
            temp.cost_g = (self.goal[0] - (coordinate[0] + 1) + self.goal[1] - (coordinate[1] + 1)) * 10
            temp.cost_f = temp.cost_g + temp.cost_h
            temp.x = coordinate[0] + 1
            temp.y = coordinate[1] + 1
            # 链接父节点
            temp.parent[0] = coordinate[0]
            temp.parent[1] = coordinate[1]
            self.state_map[coordinate[0] + 1][coordinate[1] + 1] = 2
            print "temp", temp
            if self.state_map[coordinate[0] + 1][coordinate[1] + 1] == 2:
                current_index = self.find_index(coordinate[0] + 1, coordinate[1] + 1)
                # 如果之前的cost比现在的cost大，就替换父节点和cost
                if self.open_list[current_index].cost_f > temp.cost_f:
                    self.open_list[current_index] = temp
                else:
                    # 加入open list
                    self.open_list.append(temp)
            else:
                # 加入open list
                self.open_list.append(temp)
        #下:如果是可以走的点 并且不在openlist和closelist中
        if self.map[coordinate[0] + 1][coordinate[1]] == 0 \
                and self.state_map[coordinate[0] + 1][coordinate[1]] != 3:
            temp = map_node()
            # 计算G和H代价
            temp.cost_h = 10
            temp.cost_g = (self.goal[0] - (coordinate[0] + 1) + self.goal[1] - (coordinate[1])) * 10
            temp.cost_f = temp.cost_g + temp.cost_h
            temp.x = coordinate[0] + 1
            temp.y = coordinate[1]
            # 链接父节点
            temp.parent[0] = coordinate[0]
            temp.parent[1] = coordinate[1]
            self.state_map[coordinate[0] + 1][coordinate[1]] = 2
            print "temp", temp
            if self.state_map[coordinate[0] + 1][coordinate[1]] == 2:
                current_index = self.find_index(coordinate[0] + 1, coordinate[1])
                # 如果之前的cost比现在的cost大，就替换父节点和cost
                if self.open_list[current_index].cost_f > temp.cost_f:
                    self.open_list[current_index] = temp
                else:
                    # 加入open list
                    self.open_list.append(temp)
            else:
                # 加入open list
                self.open_list.append(temp)
        #左下:如果是可以走的点 并且不在openlist和closelist中
        if self.map[coordinate[0] + 1][coordinate[1] - 1] == 0 \
                and self.state_map[coordinate[0] + 1][coordinate[1] - 1] != 3:
            temp = map_node()
            # 计算G和H代价
            temp.cost_h = 10
            temp.cost_g = (self.goal[0] - (coordinate[0] + 1) + self.goal[1] - (coordinate[1] - 1)) * 10
            temp.cost_f = temp.cost_g + temp.cost_h
            temp.x = coordinate[0] + 1
            temp.y = coordinate[1] - 1
            # 链接父节点
            temp.parent[0] = coordinate[0]
            temp.parent[1] = coordinate[1]
            self.state_map[coordinate[0] + 1][coordinate[1] - 1] = 2
            print "temp", temp
            if self.state_map[coordinate[0] + 1][coordinate[1] - 1] == 2:
                current_index = self.find_index(coordinate[0] + 1, coordinate[1] - 1)
                # 如果之前的cost比现在的cost大，就替换父节点和cost
                if self.open_list[current_index].cost_f > temp.cost_f:
                    self.open_list[current_index] = temp
                else:
                    # 加入open list
                    self.open_list.append(temp)
            else:
                # 加入open list
                self.open_list.append(temp)

    def append_close(self, node):
        # 更改节点状态
        self.state_map[node.x][node.y] = 3
        self.cloase_list.append(node)


class map_node():
    def __init__(self):
        self.x = 0
        self.y = 0
        self.cost_f = 0
        self.cost_g = 0
        self.cost_h = 0
        self.parent = []
        # self.child = []
        # 0表示未遍历 1表示在open_list 2表示在close_list ===========================无用===============================
        self.state = 0
