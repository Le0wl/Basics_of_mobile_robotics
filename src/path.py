import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors
import astar as a
from constants import *
from aruco import*
import time

class Map:
    def __init__(self):
        self.max = UNIT_NUMBER 
        self.grid = self.init_grid()

        # self.collision_proof = self.add_margin(obstacles, margin)

        
    def init_grid(self):
        # Ar = ArucoMarker((marker_id) for marker_id in range(1, 6)) 
        size = (self.max, self.max)
        grid = np.zeros(size, dtype=int)
        # grid = Ar.Map_indices
        return(grid)
    
    def update_map(self, matrix):
        self.grid = matrix.copy()
        self.with_margin = matrix.copy()
        # self.add_margin()  

    def other_corner(self, i,j):
        while self.grid[i][j]==1:
            i += 1
        i -=1
        while self.grid[i][j]==1:
            j += 1
        j-=1
        return (i,j)
        
    def add_margin(self):
        margin = int(self.max / 10)
        with_margin = self.grid.copy()
        for i in range(self.max-1):
            for j in range(self.max-1):
                if self.grid[i][j]==1 and self.grid[i][j-1] == 0 and self.grid[i-1][j] == 0 :
                    end = self.other_corner(i,j)
                    # print(end)
                    hight,width = end[0]-i+2*margin, end[1]-j+2*margin
                    if margin <= i < self.max-margin and margin <= j < self.max-margin:
                        for di in range(hight):
                            for dj in range(width):
                                if self.grid[i-margin + di][j - margin + dj] ==1:
                                    continue
                                with_margin[i-margin + di][j - margin + dj] = 2
        return(with_margin)

    
    def __len__(self):
        return(len(self.grid))
    
    def get_map(self):
        return(self.grid)
    
    def getElement(self, i, j):
        return self.grid[i][j]
    
    def setElement(self, i, j, element):
        self.grid[i][j] = element

    def plot_map(self, path):
        map = self.grid
        for i in range(len(path)):
            map[path[i]] = 2
        cmap = colors.ListedColormap(['white', 'black', 'red'])
        plt.imshow(map, cmap=cmap, interpolation='nearest')
        plt.show()

def smooth_path(path, maze):
    path = path[::-1]
    smoothed_path = [path[0]]

    for i in range(1, len(path) - 1):
        x, y = path[i]

        # Check if the straight line between the previous and next points is clear
        if not is_obstacle_between(smoothed_path[-1], path[i + 1], maze):
            # Skip the current point if the path is clear
            continue

        smoothed_path.append((x, y))

    # Add the last point in the original path
    smoothed_path.append(path[-1])

    return smoothed_path[::-1]

def is_obstacle_between(point1, point2, maze):

    x1, y1 = point1
    x2, y2 = point2

    # Check if there is an obstacle along the straight line between the two points
    dx = x2 - x1
    dy = y2 - y1
    steps = max(abs(dx), abs(dy))

    for i in range(1, steps):
        x = x1 + int(i * dx / steps)
        y = y1 + int(i * dy / steps)

        if maze[x][y] != 0:  # Assuming 0 represents a clear path
            return True  # There is an obstacle

    return False  # The path is clear

def get_path(map,robot, goal):
    #dots = [(2, 3)]
    #goal = (17, 46)
    #obstacles = [((5,5),(10,7)),((36,5),(40,20))]
    # margin = 3
    # maze = Map(np.array(obstacles), margin)
    maze = Map()
    maze.update_map(map)
    #print how many 1 in the matrix
    #print(np.count_nonzero(maze.grid == 1))
    
    path = a.astar(maze.get_map(), tuple(robot), tuple(goal))
    path = smooth_path(path, maze.get_map())
    #maze.plot_map(path)
    print("MAP: ",map)
    #remove the first element of the path
    #print("PATH: ",path)
    if len(path) != 0:
        path.pop(0)
    # maze.plot_map(path)
    #print(map)
    #time.sleep(0.5)
    #print("\033")
    return(path)

# def main():

    # robot = np.array([2, 3])
    # goal = np.array([40, 30])
    # obstacles = np.array([[[5,15],[10,30]],[[36,5],[40,15]],[[16,5],[20,20]]]) 
    # weg = get_path(robot, goal)
    # print(weg)
    # return()

# if __name__ == "__main__":
#     main()