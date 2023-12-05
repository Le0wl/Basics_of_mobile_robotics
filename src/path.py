import math
import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors
import astar as a
from constants import *
from aruco import*

class Map:
    def __init__(self):
        self.max = UNIT_NUMBER
        self.grid = self.init_grid()
        self.with_margin = self.init_grid()

        
    def init_grid(self):
        size = (self.max, self.max)
        grid = np.zeros(size, dtype=int)
        return(grid)
    
    def update_map(self, matrix):
        self.grid = matrix.copy()
        self.with_margin = matrix.copy()
        self.add_margin(3)  

    def put2(self, with_margin, i,j):

        if self.grid[i][j] == 1:
            return(with_margin)
        with_margin[i][j] = 2
        return(with_margin)
 

        
    def add_margin(self, margin):
        with_margin = self.with_margin
        for i in range(self.max-1):
            for j in range(self.max-1):
                if self.grid[i][j]==1 and self.grid[i][j-1] == 0 :
                    if j >= margin:
                        for k in range(1,margin):
                            if self.grid[i][j-k] == 1:
                                continue
                            with_margin[i][j-k] = 2
                    else:
                        for k in range(1,j):
                            if self.grid[i][j-k] == 1:
                                continue
                            with_margin[i][j-k] = 2
                if self.grid[i][j]==1 and self.grid[i][j+1] == 0 :
                    if j <= self.max - margin:
                        for k in range(1,margin):
                            if self.grid[i][j+k] == 1:
                                continue
                            with_margin[i][j+k] = 2
                    else:
                        for k in range(1,self.max - j):
                            if self.grid[i][j+k] == 1:
                                continue
                            with_margin[i][j+k] = 2
                if self.grid[i][j]==1 and self.grid[i-1][j] == 0 :
                    if i >= margin:
                        for k in range(1,margin):
                            if self.grid[i-k][j] == 1:
                                continue
                            with_margin[i-k][j] = 2
                    else:
                        for k in range(i):
                            if self.grid[i-k][j] == 1:
                                continue
                            with_margin[i-k][j] = 2
                if self.grid[i][j]==1 and self.grid[i+1][j] == 0 :
                    if i <= self.max - margin:
                        for k in range(1,margin):
                            if self.grid[i+k][j] == 1:
                                continue
                            with_margin[i+k][j] = 2
                    else:
                        for k in range(1,self.max - i):
                            if self.grid[i+k][j] == 1:
                                continue
                            with_margin[i+k][j] = 2              

        # return(with_margin)

    
    def __len__(self):
        return(len(self.grid))
    
    def get_map(self):
        return(self.with_margin)
    
    def getElement(self, i, j):
        return self.grid[i][j]
    
    def setElement(self, i, j, element):
        self.grid[i][j] = element

    def plot_map(self, path):
        map = self.with_margin.copy()
        for i in range(len(path)):
            map[path[i]] = 3
        cmap = colors.ListedColormap(['white', 'black', 'green', 'red'])
        plt.imshow(map, cmap=cmap, interpolation='nearest')
        plt.show()

def make_matrix(obstacles):
        size = (UNIT_NUMBER, UNIT_NUMBER)
        grid = np.zeros(size, dtype=int)
        for k in range(len(obstacles)):
            obsta = obstacles[k]
            beginning = obsta[0]
            end = obsta[1]
            hight,width = end[0]-beginning[0],end[1]-beginning[1]
            for i in range(hight):
                for j in range(width):
                    grid[beginning[0] + i][beginning[1]+ j] = 1
        return(grid)
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

def get_path(robot, goal, matrix):
    maze = Map()
    maze.update_map(matrix)
    path = a.astar(maze.get_map(), tuple(robot), tuple(goal))
    if type(path) is str:
        print(path)
        return(0)
    path = smooth_path(path, maze.get_map())
    print(path)
    maze.plot_map(path)
    return(path)

def main():
    #making a test map when the camera is not around
    robot = np.array([0, 0])
    goal = np.array([45, 30])
    obstacles = np.array([[[5,15],[10,20]],[[36,6],[40,22]],[[16,15],[20,20]], [[30,30],[40,45]]]) 
    matrix = make_matrix(obstacles)

    get_path(robot, goal, matrix) 
    return()

if __name__ == "__main__":
    main()