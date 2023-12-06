import math
import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors
import astar as a
from constants import *
#from aruco import*
import time

class Map:
    def __init__(self):
        self.max = UNIT_NUMBER
        self.grid = self.init_grid()
        self.with_margin = self.init_grid()

    # intitates empty map  
    def init_grid(self):
        size = (self.max, self.max)
        grid = np.zeros(size, dtype=int)
        return(grid)
    
    # updated map with the new input, adds margin
    def update_map(self, matrix):
        self.grid = matrix.copy()
        self.with_margin = matrix.copy()
        self.add_margin()  

    # checks if coordinates are NOT on the map
    def is_out_of_bounds(self, x, y):
        if x < 0 or x >= self.max or y < 0 or y >= self.max:
            return True
        else:
            return False

    # finds the bottom right corner of a rectangular obstacle when given the top left
    def other_corner(self, i,j):
        while self.grid[i][j]==1:
            if self.is_out_of_bounds(i,j):
                break
            i += 1
        i -=1
        while self.grid[i][j]==1:
            if self.is_out_of_bounds(i,j):
                break
            j += 1
        j -=1
        return (i,j)
        
    # adds margin to the obstacles to avoid collition
    def add_margin(self):
        margin = int(self.max / 10)
        with_margin = self.with_margin
        for i in range(self.max-1):
            for j in range(self.max-1):
                if self.grid[i][j]==1 and self.grid[i][j-1] == 0 and self.grid[i-1][j] == 0 :
                    end = self.other_corner(i,j)
                    hight,width = end[0]-i, end[1]-j
                    for di in range(hight+2*margin+1):
                        for dj in range(width+2*margin+1):
                            if self.is_out_of_bounds(i-margin + di,j - margin + dj):
                                continue
                            if self.grid[i-margin + di][j - margin + dj] ==1:
                                continue
                            with_margin[i-margin + di][j - margin + dj] = 2

    # returns the length of the map
    def __len__(self):
        return(len(self.grid))
    
    # returns the map wiht margin
    def get_map(self):
        return(self.with_margin)
    
    # gets a specific element of the map without margin
    def getElement(self, i, j):
        return self.grid[i][j]
    
    # sets a specifis element of the map without margin
    def setElement(self, i, j, element):
        self.grid[i][j] = element

    # plots the path on the map without margin
    def plot_map(self, path):
        map = self.grid.copy()
        for i in range(len(path)):
            map[path[i]] = 3
        cmap = colors.ListedColormap(['white', 'black', 'green', 'red'])
        plt.imshow(map, cmap=cmap, interpolation='nearest')
        plt.show()

# makes a matrix for testing when the camera is not available
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

# smooths the path returns only the relevant corners
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

        if maze[x][y] != 0:  # Assuming 0 is a clear path
            return True  # There is an obstacle

    return False  # The path is clear

def get_path(robot, goal, matrix):
    maze = Map()
    maze.update_map(matrix)
    path = a.astar(maze.get_map(), tuple(robot), tuple(goal))

    #checks for errormessages
    if type(path) is str:
        print(path)
        return(0)
    
    maze.plot_map(path)
    path = smooth_path(path, maze.get_map())
    print(path)
    maze.plot_map(path)
    return(path)

def main():
    #making a test map when the camera is not around
    robot = np.array([1, 0])
    goal = np.array([20, 25])
    obstacles = np.array([[[0,5],[10,10]],[[17,10],[18,20]]]) 
    matrix = make_matrix(obstacles)
    
    get_path(robot, goal, matrix) 
    return()

if __name__ == "__main__":
    main()