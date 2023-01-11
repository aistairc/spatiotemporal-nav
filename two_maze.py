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
from generic_search_two_d import dfs, bfs, node_to_path, astar, astar_png, astar_three, Node

import glob
import cv2
import numpy as np
import time
import argparse
import csv

number_of_collision = 0

class Cell(str, Enum):
    EMPTY = " "
    BLOCKED = "X"
    START = "S"
    GOAL = "G"
    PATH = "*"
    TMP_PATH = "T"


class MazeLocation(NamedTuple):
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
                if (cost == 0) | (cost == 100):
                    self._grid[row][column] = Cell.BLOCKED
        for row in range(rows):
            self._grid[row][0] = Cell.BLOCKED
            self._grid[row][columns-1] = Cell.BLOCKED
        for column in range(columns):
            self._grid[0][column] = Cell.BLOCKED
            self._grid[rows-1][column] = Cell.BLOCKED

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
        global number_of_collision
        locations: List[Union[MazeLocation, bool]] = []
        locations.append([MazeLocation(ml.row, ml.column), False])
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
        if len(locations) == 1:
            print("------------------------------collision----------------------------------")
            number_of_collision += 1
        return locations

    def mark(self, path: List[MazeLocation], tmp_path: List[MazeLocation]):
        for maze_location in path:
            self._grid[maze_location.row][maze_location.column] = Cell.PATH
        for maze_location in tmp_path:
            self._grid[maze_location.row][maze_location.column] = Cell.TMP_PATH
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

    im_file_list = sorted(glob.glob(args.i + "*png"))
    print("finish reading png")

    initial_map = cv2.imread(im_file_list[0], cv2.IMREAD_GRAYSCALE)
    shape_0 = initial_map.shape[0]
    shape_1 = initial_map.shape[1]
    space_time_map = np.empty((0,shape_0,shape_1))
    print("go stack")
    print(len(im_file_list))
    for i in range(len(im_file_list)):
        space_time_map = np.vstack((space_time_map, [cv2.imread(im_file_list[i], cv2.IMREAD_GRAYSCALE)]))
        print(i)
    print("finish stack")
    start_img: MazeLocation = MazeLocation(st_r, st_c)
    goal_img: MazeLocation = MazeLocation(go_r, go_c)
    road: List[MazeLocation] = []
    meiro: List[Maze] = []
    m_png :Maze = Maze(rows=shape_0, columns=shape_1, start=start_img, goal=goal_img, cost_matrix=space_time_map[0])
    meiro.append(m_png)

    time_start = time.time()
    count = 0
    for i in range(len(im_file_list)):
        #print(i)
        m_png :Maze = Maze(rows=shape_0, columns=shape_1, start=start_img, goal=goal_img, cost_matrix=space_time_map[i])
        distance_png: Callable[[MazeLocation], float] = vel_euclidean_distance(m_png.goal)
        solution4: Optional[Node[MazeLocation]] = astar_png(m_png.start, m_png.goal_test, m_png.euc_successors, distance_png, cost_matrix=space_time_map[i])
        if solution4 is None:
            print("No solution found using A*!")
            path4: List[MazeLocation] = []
        else:
            path4: List[MazeLocation] = node_to_path(solution4)
            if path4[1] == goal_img:
                print("break")
                meiro.append(m_png)
                break
            start_img = path4[1]
            road.append(path4[1])

        m_png.mark(road, path4)
        meiro.append(m_png)
        count = i
    count = count + 2 # 出力pngの数にしてる
    print(time.time() - time_start)
    print(number_of_collision)
    print(count)

#    output_csv = [[args.data +' ' +str(count)]]
#    with open('/mnt/share/yutasato/rita2022/result/2d_'+args.num+'_'+args.cond+'_arrival_time.csv', 'a') as f:
#        writer = csv.writer(f)
#        writer.writerows(output_csv)
#
#    output_csv = [[args.data +' ' +str(number_of_collision)]]
#    with open('/mnt/share/yutasato/rita2022/result/2d_'+args.num+'_'+args.cond+'_stop_time.csv', 'a') as f:
#        writer = csv.writer(f)
#        writer.writerows(output_csv)

#    output_csv = [[args.o], [str(count)], [str(number_of_collision)]]
#    with open('/home/digital/workspace/rita2022/result/result_2.csv', 'a') as f:
#        writer = csv.writer(f)
#        writer.writerows(output_csv)


    print("write png")
    for i in range(len(im_file_list)):
        if i == len(meiro):
            break
        save_map = cv2.imread(im_file_list[i])
        row = st_r
        column = st_c
        row_begin = row - 10
        column_begin = column -10 
        for row_ran in range(row_begin, row_begin+20):
            for column_ran in range(column_begin, column_begin+20):
                save_map[row_ran][column_ran][0] = 0
                save_map[row_ran][column_ran][1] = 0
                save_map[row_ran][column_ran][2] = 255
        for row in range(shape_0):
            for column in range(shape_1):
                if meiro[i]._grid[row][column] == Cell.GOAL:
                    row_begin = row - 10
                    column_begin = column -10 
                    for row_ran in range(row_begin, row_begin+20):
                        for column_ran in range(column_begin, column_begin+20):
                            save_map[row_ran][column_ran][0] = 255
                            save_map[row_ran][column_ran][1] = 0
                            save_map[row_ran][column_ran][2] = 0
        for row in range(shape_0):
            for column in range(shape_1):
                if meiro[i]._grid[row][column] == Cell.PATH:
                    save_map[row][column][0] = 0
                    save_map[row][column][1] = 255
                    save_map[row][column][2] = 0

        for row in range(shape_0):
            for column in range(shape_1):
                if meiro[i]._grid[row][column] == Cell.TMP_PATH:
                    save_map[row][column][0] = 255
                    save_map[row][column][1] = 255
                    save_map[row][column][2] = 255

        for row in range(shape_0):
            for column in range(shape_1):
                if meiro[i]._grid[row][column] == Cell.START:
                    row_begin = row - 3
                    column_begin = column -3
                    for row_ran in range(row_begin, row_begin+6):
                        for column_ran in range(column_begin, column_begin+6):
                            save_map[row_ran][column_ran][0] = 0
                            save_map[row_ran][column_ran][1] = 255
                            save_map[row_ran][column_ran][2] = 0
        #file_name = './' + f'{i:02}' + '.png'
        file_name = args.o + f'{i:05}' + '.png'
        cv2.imwrite(file_name, save_map)

