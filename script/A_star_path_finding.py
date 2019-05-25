#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
'''
本代码中的A星算法采用了八临近的方法：左右上下代价为10，斜对角线代价为14
F = G + H
G代表：从起点移动到指定方格的移动代价
H代表：从指定的方格移动到终点的估算成本（采用街区距离，只能上下左右走）

'''
import numpy as np
import time


class find_path():
    def __init__(self, map, start, goal):
        # map是一个二维地图， start是起点坐标[]，goal是终点坐标[]
        self.map = self.extend_map(map)
        # 2代表在open表中 3代表在close表中
        self.state_map = np.zeros([len(map) + 2, len(map[0]) + 2])
        # print self.map

        self.start = start
        self.start[0] += 1
        self.start[1] += 1
        self.goal = goal
        self.goal[0] += 1
        self.goal[1] += 1
        self.open_list = []
        self.cloase_list = []
        self.path = []
        self.if_reach = False

    def extend_map(self, map):
        new_row = np.ones(len(map[0]))
        new_col = np.ones(len(map) + 2)
        x = np.insert(map, 0, new_row, axis=0)
        x = np.insert(x, len(map) + 1, new_row, axis=0)
        x = np.insert(x, 0 , new_col, axis=1)
        x = np.insert(x, len(map[0]) + 1 , new_col, axis=1)
        return x

    def start_find(self):
        #第一次操作，把起点的周围的点指向起点，起点和周围的点加到open list,
        # print "-----start point-----",self.start
        if self.map[self.start[0]][self.start[1]] != 0:
            print "\033[0;31m[E] : Please set the valid start point\033[0m"
            print "value = ", self.map[self.start[0]][self.start[1]]
            return "None"
        if self.map[self.goal[0]][self.goal[1]] != 0:
            print "\033[0;31m[E] : Please set the valid goal point\033[0m"
            return "None"
        self.append_around_open(self.start, cost_g=0)

        # 把起始节点加到close_list
        temp = map_node()
        temp.x = self.start[0]
        temp.y = self.start[1]
        self.append_close(temp)
        while True:
            # print "-----"
            min_cost, index_min_cost = self.find_min_cost_f()
            current_node = self.open_list[index_min_cost]
            # print current_node.x
            # 如果最小的节点正好等于终点
            # print current_node.x, current_node.y
            # print self.state_map
            # time.sleep(1)
            if current_node.x == self.goal[0] and current_node.y == self.goal[1]:
                self.append_path(current_node)
                break
            self.append_around_open([current_node.x, current_node.y], cost_g=current_node.cost_g)
            # 加到close list
            self.append_close(current_node)
            self.open_list.remove(current_node)
        return self.path

    # 最后找到终点，把最短路径append到path里
    def append_path(self, node):
        while True:
            self.path.append([node.x - 1, node.y - 1])
            if node.x == self.start[0] and node.y == self.start[1]:
                break
            current_index = self.find_close_index(node.parent[0], node.parent[1])
            # print "----------------", current_index
            # print self.cloase_list
            node = self.cloase_list[current_index]

    # 寻找open表中的最小代价节点和index
    def find_min_cost_f(self):
        # 记录最小花费和其在openlist中的下标
        # print "--------------------------------one time----------------"
        min_cost = 100000
        index_min_cost = 0
        for i in range(len(self.open_list)):
            # print self.open_list[i].cost_f, min_cost
            if self.open_list[i].cost_f < min_cost:
                min_cost = self.open_list[i].cost_f
                index_min_cost = i
        return min_cost, index_min_cost

    def find_close_index(self, x, y):
        for i in range(len(self.cloase_list)):
            if self.cloase_list[i].x == x and self.cloase_list[i].y == y:
                return i

    def find_index(self, x, y):
        for i in range(len(self.open_list)):
            if self.open_list[i].x == x and self.open_list[i].y == y:
                return i

    def append_around_open(self, coordinate, cost_g):
        #周围8个点
        #左:如果是可以走的点 并且不在closelist中
        # print "+++++", self.map
        if self.map[coordinate[0]][coordinate[1] - 1] == 0 \
                and self.state_map[coordinate[0]][coordinate[1] - 1] != 3:
            temp = map_node()
            # 计算G和H代价
            temp.cost_g = 10 + cost_g
            temp.cost_h = (abs(self.goal[0] - coordinate[0]) + abs(self.goal[1] - (coordinate[1] - 1))) * 10
            temp.cost_f = temp.cost_g + temp.cost_h
            temp.x = coordinate[0]
            temp.y = coordinate[1] - 1
            #链接父节点
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

        #左上:如果是可以走的点 并且不在openlist和closelist中
        if self.map[coordinate[0] - 1][coordinate[1] - 1] == 0 \
                and self.state_map[coordinate[0] - 1][coordinate[1] - 1] != 3:
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
        #上:如果是可以走的点 并且不在openlist和closelist中
        if self.map[coordinate[0] - 1][coordinate[1]] == 0 \
                and self.state_map[coordinate[0] - 1][coordinate[1]] != 3:
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
            # print "temp", temp
            # print self.state_map[coordinate[0] - 1][coordinate[1] - 1]

            if self.state_map[coordinate[0] - 1][coordinate[1]] == 2:
                current_index = self.find_index(coordinate[0] - 1, coordinate[1])
                # 如果之前的cost比现在的cost大，就替换父节点和cost
                if self.open_list[current_index].cost_f > temp.cost_f:
                    self.open_list[current_index] = temp
            else:
                self.state_map[coordinate[0] - 1][coordinate[1]] = 2

                # 加入open list
                self.open_list.append(temp)
        #右上:如果是可以走的点 并且不在openlist和closelist中
        if self.map[coordinate[0] - 1][coordinate[1] + 1] == 0 \
                and self.state_map[coordinate[0] - 1][coordinate[1] + 1] != 3:
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
            # print "temp", temp
            if self.state_map[coordinate[0] - 1][coordinate[1] + 1] == 2:
                current_index = self.find_index(coordinate[0] - 1, coordinate[1] + 1)
                # 如果之前的cost比现在的cost大，就替换父节点和cost
                if self.open_list[current_index].cost_f > temp.cost_f:
                    self.open_list[current_index] = temp
            else:
                self.state_map[coordinate[0] - 1][coordinate[1] + 1] = 2

                # 加入open list
                self.open_list.append(temp)
        #右:如果是可以走的点 并且不在openlist和closelist中
        if self.map[coordinate[0]][coordinate[1] + 1] == 0 \
                and self.state_map[coordinate[0]][coordinate[1] + 1] != 3:
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
            # print "temp", temp
            if self.state_map[coordinate[0]][coordinate[1] + 1] == 2:
                current_index = self.find_index(coordinate[0], coordinate[1] + 1)
                # 如果之前的cost比现在的cost大，就替换父节点和cost
                if self.open_list[current_index].cost_f > temp.cost_f:
                    self.open_list[current_index] = temp
            else:
                self.state_map[coordinate[0]][coordinate[1] + 1] = 2

                # 加入open list
                self.open_list.append(temp)
        #右下:如果是可以走的点 并且不在openlist和closelist中
        if self.map[coordinate[0] + 1][coordinate[1] + 1] == 0 \
                and self.state_map[coordinate[0] + 1][coordinate[1] + 1] != 3:
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
            # print "temp", temp
            if self.state_map[coordinate[0] + 1][coordinate[1] + 1] == 2:
                current_index = self.find_index(coordinate[0] + 1, coordinate[1] + 1)
                # 如果之前的cost比现在的cost大，就替换父节点和cost
                if self.open_list[current_index].cost_f > temp.cost_f:
                    self.open_list[current_index] = temp
            else:
                self.state_map[coordinate[0] + 1][coordinate[1] + 1] = 2

                # 加入open list
                self.open_list.append(temp)
        #下:如果是可以走的点 并且不在openlist和closelist中
        if self.map[coordinate[0] + 1][coordinate[1]] == 0 \
                and self.state_map[coordinate[0] + 1][coordinate[1]] != 3:
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
            # print "temp", temp
            if self.state_map[coordinate[0] + 1][coordinate[1]] == 2:
                current_index = self.find_index(coordinate[0] + 1, coordinate[1])
                # 如果之前的cost比现在的cost大，就替换父节点和cost
                if self.open_list[current_index].cost_f > temp.cost_f:
                    self.open_list[current_index] = temp
            else:
                self.state_map[coordinate[0] + 1][coordinate[1]] = 2

                # 加入open list
                self.open_list.append(temp)
        #左下:如果是可以走的点 并且不在openlist和closelist中
        if self.map[coordinate[0] + 1][coordinate[1] - 1] == 0 \
                and self.state_map[coordinate[0] + 1][coordinate[1] - 1] != 3:
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
            # print "temp", temp
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

class map_node():
    def __init__(self):
        self.x = 0
        self.y = 0
        self.cost_f = 0
        self.cost_g = 0
        self.cost_h = 0
        self.parent = [0,0]
        # self.child = []
        # 0表示未遍历 1表示在open_list 2表示在close_list ===========================无用===============================
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