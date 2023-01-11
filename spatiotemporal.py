# maze.py
# From Classic Computer Science Problems in Python Chapter 2
# Copyright 2018 David Kopec
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from enum import Enum
from typing import List, NamedTuple, Callable, Optional, Union
import random
from math import sqrt
from generic_search import dfs, bfs, node_to_path, astar, astar_png, astar_three, Node

import glob
import cv2
import numpy as np
import time
import argparse
import csv

class Cell(str, Enum):
    EMPTY = " "
    BLOCKED = "X"
    START = "S"
    GOAL = "G"
    PATH = "*"


class MazeLocation(NamedTuple):
    row: int
    column: int

class ThreeMazeLocation(NamedTuple):
    layer: int
    row: int
    column: int

class Maze:
    def __init__(self, rows: int = 10, columns: int = 10, sparseness: float = 0.2, \
        start: MazeLocation = MazeLocation(0, 0), goal: MazeLocation = MazeLocation(9, 9), cost_matrix = np.zeros((1,1))) -> None:
        # initialize basic instance variables
        self._rows: int = rows
        self._columns: int = columns
        self.start: MazeLocation = start
        self.goal: MazeLocation = goal
        # fill the grid with empty cells
        self._grid: List[List[Cell]] = [[Cell.EMPTY for c in range(columns)] for r in range(rows)]
        # cost_matrix 代入
        self.cost_matrix = cost_matrix
        # random か png迷路か
        if rows == 10:
            # populate the grid with blocked cells
            self._randomly_fill(rows, columns, sparseness)
        else:
            self._png_fill(rows, columns)
        # fill the start and goal locations in
        self._grid[start.row][start.column] = Cell.START
        self._grid[goal.row][goal.column] = Cell.GOAL

    def _randomly_fill(self, rows: int, columns: int, sparseness: float):
        for row in range(rows):
            for column in range(columns):
                if random.uniform(0, 1.0) < sparseness:
                    self._grid[row][column] = Cell.BLOCKED

    def _png_fill(self, rows: int, columns: int):
        for row in range(rows):
            for column in range(columns):
                cost = self.cost_matrix[row][column]
                if cost == 0 | cost == 100:
                    self._grid[row][column] = Cell.BLOCKED

    # return a nicely formatted version of the maze for printing
    def __str__(self) -> str:
        output: str = ""
        for row in self._grid:
            output += "".join([c.value for c in row]) + "\n"
        return output

    def goal_test(self, ml: MazeLocation) -> bool:
        return ml == self.goal

    def successors(self, ml: MazeLocation) -> List[MazeLocation]:
        locations: List[MazeLocation] = []
        if ml.row + 1 < self._rows and self._grid[ml.row + 1][ml.column] != Cell.BLOCKED:
            locations.append(MazeLocation(ml.row + 1, ml.column))
        if ml.row - 1 >= 0 and self._grid[ml.row - 1][ml.column] != Cell.BLOCKED:
            locations.append(MazeLocation(ml.row - 1, ml.column))
        if ml.column + 1 < self._columns and self._grid[ml.row][ml.column + 1] != Cell.BLOCKED:
            locations.append(MazeLocation(ml.row, ml.column + 1))
        if ml.column - 1 >= 0 and self._grid[ml.row][ml.column - 1] != Cell.BLOCKED:
            locations.append(MazeLocation(ml.row, ml.column - 1))
        return locations

    def euc_successors(self, ml: MazeLocation) -> List[Union[MazeLocation, bool]]:
        locations: List[Union[MazeLocation, bool]] = []
        if ml.row + 1 < self._rows and self._grid[ml.row + 1][ml.column] != Cell.BLOCKED:
            locations.append([MazeLocation(ml.row + 1, ml.column), False])
        if ml.row - 1 >= 0 and self._grid[ml.row - 1][ml.column] != Cell.BLOCKED:
            locations.append([MazeLocation(ml.row - 1, ml.column), False])
        if ml.column + 1 < self._columns and self._grid[ml.row][ml.column + 1] != Cell.BLOCKED:
            locations.append([MazeLocation(ml.row, ml.column + 1), False])
        if ml.column - 1 >= 0 and self._grid[ml.row][ml.column - 1] != Cell.BLOCKED:
            locations.append([MazeLocation(ml.row, ml.column - 1), False])
        #######################################################################################################################
        if ml.row + 1 < self._rows and self._grid[ml.row + 1][ml.column + 1] != Cell.BLOCKED and ml.column + 1 < self._columns:
            locations.append([MazeLocation(ml.row + 1, ml.column + 1), True])
        if ml.row + 1 < self._rows and self._grid[ml.row + 1][ml.column - 1] != Cell.BLOCKED and ml.column - 1 >= 0:
            locations.append([MazeLocation(ml.row + 1, ml.column - 1), True])
        if ml.row - 1 >= 0 and self._grid[ml.row - 1][ml.column + 1] != Cell.BLOCKED and ml.column + 1 < self._columns:
            locations.append([MazeLocation(ml.row - 1, ml.column + 1),True])
        if ml.row - 1 >= 0 and self._grid[ml.row - 1][ml.column - 1] != Cell.BLOCKED and ml.column - 1 >= 0:
            locations.append([MazeLocation(ml.row - 1, ml.column - 1), True])
        return locations

    def mark(self, path: List[MazeLocation]):
        for maze_location in path:
            self._grid[maze_location.row][maze_location.column] = Cell.PATH
        self._grid[self.start.row][self.start.column] = Cell.START
        self._grid[self.goal.row][self.goal.column] = Cell.GOAL
    
    def clear(self, path: List[MazeLocation]):
        for maze_location in path:
            self._grid[maze_location.row][maze_location.column] = Cell.EMPTY
        self._grid[self.start.row][self.start.column] = Cell.START
        self._grid[self.goal.row][self.goal.column] = Cell.GOAL


def euclidean_distance(goal: MazeLocation) -> Callable[[MazeLocation], float]:
    def distance(ml: MazeLocation) -> float:
        xdist: int = ml.column - goal.column
        ydist: int = ml.row - goal.row
        return sqrt((xdist * xdist) + (ydist * ydist))
    return distance

def vel_euclidean_distance(goal: MazeLocation) -> Callable[[MazeLocation], float]:
    def distance(ml: MazeLocation) -> float:
        xdist: int = ml.column - goal.column
        ydist: int = ml.row - goal.row
        return sqrt((xdist * xdist) + (ydist * ydist))/20
    return distance

def manhattan_distance(goal: MazeLocation) -> Callable[[MazeLocation], float]:
    def distance(ml: MazeLocation) -> float:
        xdist: int = abs(ml.column - goal.column)
        ydist: int = abs(ml.row - goal.row)
        return (xdist + ydist)
    return distance

def vel_manhattan_distance(goal: MazeLocation) -> Callable[[MazeLocation], float]:
    def distance(ml: MazeLocation) -> float:
        xdist: int = abs(ml.column - goal.column)
        ydist: int = abs(ml.row - goal.row)
        return (xdist + ydist)/20
    return distance

###########################################################################################################################################################

class MazeThree:
    def __init__(self,layers: int = 20, rows: int = 10, columns: int = 10, sparseness: float = 0.2, \
        start: ThreeMazeLocation = ThreeMazeLocation(0, 0, 0), goal: ThreeMazeLocation = ThreeMazeLocation(9, 9, 9), cost_matrix = np.zeros((1,1,1))) -> None:
        # initialize basic instance variables
        self._layers: int = layers
        self._rows: int = rows
        self._columns: int = columns
        self.start: ThreeMazeLocation = start
        self.goal: ThreeMazeLocation = goal
        self.collision = 0
        # fill the grid with empty cells
        self._grid: List[List[List[Cell]]] = [[[Cell.EMPTY for c in range(columns)] for r in range(rows)] for l in range(layers)]
        # cost_matrix 代入
        self.cost_matrix = cost_matrix
        # 地図作成
        self._png_fill(layers, rows, columns)
        # fill the start and goal locations in
        for i in range(goal.layer):
            self._grid[i][start.row][start.column] = Cell.START
            self._grid[i][goal.row][goal.column] = Cell.GOAL


    def _png_fill(self,layers: int, rows: int, columns: int):
        for layer in range(layers):
            for row in range(rows):
                for column in range(columns):
                    cost = int(self.cost_matrix[layer][row][column])
                    if cost == 0 | cost == 100:
                        self._grid[layer][row][column] = Cell.BLOCKED
            # 画像の端は最初から障害物にしておく
            for row in range(rows):
                self._grid[layer][row][0] = Cell.BLOCKED
                self._grid[layer][row][columns-1] = Cell.BLOCKED
            for column in range(columns):
                self._grid[layer][0][column] = Cell.BLOCKED
                self._grid[layer][rows-1][column] = Cell.BLOCKED

    # return a nicely formatted version of the maze for printing
    def __str__(self) -> str:
        output: str = ""
        for layer_ite in self._grid:
            for row in layer_ite:
                output += "".join([c.value for c in row]) + "\n"
            output += "@"*130 + "\n"
        return output

    def goal_test(self, ml: ThreeMazeLocation) -> bool:
        return ml == self.goal

    # 時刻によらないゴール判定
    def goal_test_ind(self, ml: ThreeMazeLocation) -> bool:
        if ml.row == self.goal.row and ml.column == self.goal.column:
            return True
        else:
            return False

    def successors(self, ml: MazeLocation) -> List[MazeLocation]:
        locations: List[MazeLocation] = []
        if ml.row + 1 < self._rows and self._grid[ml.row + 1][ml.column] != Cell.BLOCKED:
            locations.append(MazeLocation(ml.row + 1, ml.column))
        if ml.row - 1 >= 0 and self._grid[ml.row - 1][ml.column] != Cell.BLOCKED:
            locations.append(MazeLocation(ml.row - 1, ml.column))
        if ml.column + 1 < self._columns and self._grid[ml.row][ml.column + 1] != Cell.BLOCKED:
            locations.append(MazeLocation(ml.row, ml.column + 1))
        if ml.column - 1 >= 0 and self._grid[ml.row][ml.column - 1] != Cell.BLOCKED:
            locations.append(MazeLocation(ml.row, ml.column - 1))
        return locations

    # 次の時刻の現在地を含めた9方向をopenにしている，斜めに移動したときはTrue 縦横に移動したときはFalseを返している
    def euc_successors(self, ml: ThreeMazeLocation) -> List[Union[ThreeMazeLocation, bool]]:
        locations: List[Union[ThreeMazeLocation, bool]] = []
        th_time = self.goal.layer
        if ml.layer < th_time:
            layer = ml.layer + 1
        else:
            layer = ml.layer
            print("not enough map")
#        print(ml.layer)
        # 次の時刻の現在地
        locations.append([ThreeMazeLocation(layer, ml.row, ml.column), False])
        # 次の時刻の8方向
        if ml.row + 1 < self._rows and self._grid[layer][ml.row + 1][ml.column] != Cell.BLOCKED:
            locations.append([ThreeMazeLocation(layer, ml.row + 1, ml.column), False])
        if ml.row - 1 >= 0 and self._grid[layer][ml.row - 1][ml.column] != Cell.BLOCKED:
            locations.append([ThreeMazeLocation(layer, ml.row - 1, ml.column), False])
        if ml.column + 1 < self._columns and self._grid[layer][ml.row][ml.column + 1] != Cell.BLOCKED:
            locations.append([ThreeMazeLocation(layer, ml.row, ml.column + 1), False])
        if ml.column - 1 >= 0 and self._grid[layer][ml.row][ml.column - 1] != Cell.BLOCKED:
            locations.append([ThreeMazeLocation(layer, ml.row, ml.column - 1), False])
        #######################################################################################################################
        #print(ml.row, ml.column)
        if ml.row + 1 < self._rows and self._grid[layer][ml.row + 1][ml.column + 1] != Cell.BLOCKED and ml.column + 1 < self._columns:
            locations.append([ThreeMazeLocation(layer, ml.row + 1, ml.column + 1), True])
        if ml.row + 1 < self._rows and self._grid[layer][ml.row + 1][ml.column - 1] != Cell.BLOCKED and ml.column - 1 >= 0:
            locations.append([ThreeMazeLocation(layer, ml.row + 1, ml.column - 1), True])
        if ml.row - 1 >= 0 and self._grid[layer][ml.row - 1][ml.column + 1] != Cell.BLOCKED and ml.column + 1 < self._columns:
            locations.append([ThreeMazeLocation(layer, ml.row - 1, ml.column + 1),True])
        if ml.row - 1 >= 0 and self._grid[layer][ml.row - 1][ml.column - 1] != Cell.BLOCKED and ml.column - 1 >= 0:
            locations.append([ThreeMazeLocation(layer, ml.row - 1, ml.column - 1), True])
        if len(locations) == 0:
            print("------------------------------collision----------------------------------")
            self.collision += 1
        return locations
    def mark(self, path: List[MazeLocation]):
        for maze_location in path:
            self._grid[maze_location.row][maze_location.column] = Cell.PATH
        self._grid[self.start.row][self.start.column] = Cell.START
        self._grid[self.goal.row][self.goal.column] = Cell.GOAL
    
    # 3次元用のmark関数
    def threemark(self, path: List[ThreeMazeLocation]):
        map_len = len(path)
        for maze_location in path:
            launch = maze_location.layer
            for i in range(launch, map_len):
                try:
                    self._grid[i][maze_location.row][maze_location.column] = Cell.PATH
                except IndexError:
                    self._grid[self._layers-1][maze_location.row][maze_location.column] = Cell.PATH 
            self._grid[maze_location.layer][self.start.row][self.start.column] = Cell.START
            self._grid[maze_location.layer][self.goal.row][self.goal.column] = Cell.GOAL

    def clear(self, path: List[MazeLocation]):
        for maze_location in path:
            self._grid[maze_location.row][maze_location.column] = Cell.EMPTY
        self._grid[self.start.row][self.start.column] = Cell.START
        self._grid[self.goal.row][self.goal.column] = Cell.GOAL



def three_vel_euclidean_distance(goal: ThreeMazeLocation) -> Callable[[ThreeMazeLocation], float]:
    def distance(ml: ThreeMazeLocation) -> float:
        xdist: int = ml.column - goal.column
        ydist: int = ml.row - goal.row
        return sqrt((xdist * xdist) + (ydist * ydist))/20
    return distance

########################################################################################################################################################
if __name__ == "__main__":
    # sato
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', required=True, help='input bagfile')
    parser.add_argument('-o', required=True, help='output bagfile')
    parser.add_argument('--st_r', required=True)
    parser.add_argument('--st_c', required=True)
    parser.add_argument('--go_r', required=True)
    parser.add_argument('--go_c', required=True)
#    parser.add_argument('--num', required=True)
#    parser.add_argument('--data', required=True)
#    parser.add_argument('--cond', required=True)
    args = parser.parse_args()

    st_r = int(args.st_r)
    st_c = int(args.st_c)
    go_r = int(args.go_r)
    go_c = int(args.go_c)
    def one_tern(dir_name, start=ThreeMazeLocation(0,st_r,st_c), goal=ThreeMazeLocation(20,go_r,go_c)):
        im_file_list = sorted(glob.glob(args.i + dir_name+"/*png"))
        print("finish reading png")
    
    ########################################################################################################################################################
    # 以下3次元探索
        initial_map = cv2.imread(im_file_list[0], cv2.IMREAD_GRAYSCALE)
        shape_0 = initial_map.shape[0]
        shape_1 = initial_map.shape[1]
        space_time_map = np.empty((0,shape_0,shape_1))
        print("go stack")
        print(len(im_file_list))
        space_time_map = np.vstack((space_time_map, [cv2.imread(im_file_list[-1], cv2.IMREAD_GRAYSCALE)]))
        for i in range(len(im_file_list)-1):
            space_time_map = np.vstack((space_time_map, [cv2.imread(im_file_list[i], cv2.IMREAD_GRAYSCALE)]))
            print(im_file_list[i])
        print("finish stack")
        
        layers_num = space_time_map.shape[0]
        rows_num = space_time_map.shape[1]
        columns_num = space_time_map.shape[2]
        #スタート，ゴール設定 
        start_pos = start
        map_len = len(space_time_map)
        goal_pos = goal
    
        m_three :MazeThree = MazeThree(layers=layers_num, rows=rows_num, columns=columns_num, start=start_pos, goal=goal_pos, cost_matrix=space_time_map)
        heuristic_func: Callable[[ThreeMazeLocation], float] = three_vel_euclidean_distance(m_three.goal)
        print("go search")
        time_start = time.time()
        time_solution: Optional[Node[ThreeMazeLocation]] = astar_three(m_three.start, m_three.goal_test_ind, m_three.euc_successors, \
                                                                         heuristic_func, cost_matrix=space_time_map)
        print(time.time() - time_start)
        print("search finish")
        print("number of collision")
        print(m_three.collision)
        if time_solution is None:
            print("No solution found using A*!")
            three_path: List[ThreeMazeLocation] = [start]
        else:
            three_path: List[ThreeMazeLocation] = node_to_path(time_solution)
            m_three.threemark(three_path)
            #print(m_three)
        try:
            butu = three_path[1]
        except IndexError:
            butu = three_path[0]
        return butu


    correct_list = sorted(glob.glob(args.i + "origin/*png"))
    ini_map = cv2.imread(correct_list[0], cv2.IMREAD_GRAYSCALE)
    sh_0 = ini_map.shape[0]
    sh_1 = ini_map.shape[1]
    correct_map = np.empty((0,sh_0,sh_1))
    print("go stack")
    for i in range(len(correct_list)):
        correct_map = np.vstack((correct_map, [cv2.imread(correct_list[i], cv2.IMREAD_GRAYSCALE)]))
        print(i)
    print("finish stack")
    layers_num = correct_map.shape[0]
    rows_num = correct_map.shape[1]
    columns_num = correct_map.shape[2]

    s_pos = ThreeMazeLocation(0, st_r, st_c)
    map_len = len(correct_map) - 1
    g_pos = ThreeMazeLocation(map_len, go_r, go_c)

    m_c :MazeThree = MazeThree(layers=layers_num, rows=rows_num, columns=columns_num, start=s_pos, goal=g_pos, cost_matrix=correct_map)
    path_buf = []
    collision_count = 0
    for i in range(len(correct_list)):
        print(i)
        try:
            print("*"*50)
            print(m_c._grid[i][next_state.row][next_state.column])
            if m_c._grid[i][next_state.row][next_state.column] == Cell.BLOCKED:
                path_buf.append(next_state)
                collision_count += 1
            else:
                state = ThreeMazeLocation(0,next_state.row,next_state.column)
                next_state = one_tern(f'{i:03}',start=state)
                path_buf.append(next_state)
                print(next_state)
                if next_state == ThreeMazeLocation(1,go_r,go_c):
                    break
        except NameError:
            next_state = one_tern(f'{i:03}')
            path_buf.append(next_state)
    print(next_state)
    print("collision: ", collision_count)


#    output_csv = [[args.data + ' ' + str(len(path_buf))]]
#    with open('/mnt/share/yutasato/rita2022/result/3d_'+args.num+'_'+args.cond+'_arrival_time.csv', 'a') as f:
#        writer = csv.writer(f)
#        writer.writerows(output_csv)
#
#    output_csv = [[args.data + ' ' + str(collision_count)]]
#    with open('/mnt/share/yutasato/rita2022/result/3d_'+args.num+'_'+args.cond+'_stop_time.csv', 'a') as f:
#        writer = csv.writer(f)
#        writer.writerows(output_csv)
#
#    output_csv = [[args.o], [str(next_state)], [str(collision_count)]]
#    with open('/home/digital/workspace/rita2022/result/result_3.csv', 'a') as f:
#        writer = csv.writer(f)
#        writer.writerows(output_csv)

    last_time = len(path_buf) - 1
    hoge_time = last_time + 1
    for i in range(hoge_time):
        save_map = cv2.imread(correct_list[i])
        for row in range(sh_0):
            for column in range(sh_1):
                if m_c._grid[i][row][column] == Cell.START:
                    row_begin = row - 10
                    column_begin = column - 10
                    for row_ran in range(row_begin, row_begin+20):
                        for column_ran in range(column_begin, column_begin+20):
                            save_map[row_ran][column_ran][0] = 0
                            save_map[row_ran][column_ran][1] = 0
                            save_map[row_ran][column_ran][2] = 255
                if m_c._grid[i][row][column] == Cell.GOAL:
                    row_begin = row - 10
                    column_begin = column -10 
                    for row_ran in range(row_begin, row_begin+20):
                        for column_ran in range(column_begin, column_begin+20):
                            save_map[row_ran][column_ran][0] = 255
                            save_map[row_ran][column_ran][1] = 0
                            save_map[row_ran][column_ran][2] = 0
        row_start = path_buf[i].row - 3
        column_start = path_buf[i].column - 3
        for row in range(row_start, row_start+6):
            for column in range(column_start, column_start+6):
                save_map[row][column][0] = 0
                save_map[row][column][1] = 255
                save_map[row][column][2] = 0
        file_name = args.o + f'{i:05}' + '.png'
        cv2.imwrite(file_name, save_map)


    save_map = cv2.imread(correct_list[last_time])
    for row in range(sh_0):
        for column in range(sh_1):
            if m_c._grid[i][row][column] == Cell.START:
                row_begin = row - 10
                column_begin = column - 10
                for row_ran in range(row_begin, row_begin+20):
                    for column_ran in range(column_begin, column_begin+20):
                        save_map[row_ran][column_ran][0] = 0
                        save_map[row_ran][column_ran][1] = 0
                        save_map[row_ran][column_ran][2] = 255
            if m_c._grid[i][row][column] == Cell.GOAL:
                row_begin = row - 10
                column_begin = column -10 
                for row_ran in range(row_begin, row_begin+20):
                    for column_ran in range(column_begin, column_begin+20):
                        save_map[row_ran][column_ran][0] = 255
                        save_map[row_ran][column_ran][1] = 0
                        save_map[row_ran][column_ran][2] = 0
    for t in range(len(path_buf)):
        row_start = path_buf[t].row
        column_start = path_buf[t].column
        save_map[row_start][column_start][0] = 0
        save_map[row_start][column_start][1] = 255
        save_map[row_start][column_start][2] = 0
    file_name = args.o + 'result' + '.png'
    cv2.imwrite(file_name, save_map)







