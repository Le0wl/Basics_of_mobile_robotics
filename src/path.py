import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors
import astar as a
from constants import *

class Map:
    def __init__(self):
        self.max = UNIT_NUMBER
        self.grid = self.init_grid()
        self.with_margin = self.init_grid()

        
    def init_grid(self):
        size = (self.max, self.max)
        grid = np.zeros(size, dtype=int)
        return(grid)
    
    def update_map(self, matirx):
        self.grid = matirx
        self.with_margin = self.add_margin(10)                    
        
    def add_margin(self, margin):
        with_margin = self.grid
        for i in range(self.max-1):
            for j in range(self.max-1):
                if self.grid[i][j]==1 and self.grid[i][j-1] == 0 :
                    if j >= margin:
                        for k in range(margin):
                            with_margin[i][j-k] = 1
                    else:
                        for k in range(j):
                            with_margin[i][j-k] = 1
                if self.grid[i][j]==1 and self.grid[i][j+1] == 0 :
                    if j <= self.max - margin:
                        for k in range(margin):
                            with_margin[i][j+k] = 1
                    else:
                        for k in range(self.max - j):
                            with_margin[i][j+k] = 1
                if self.grid[i][j]==1 and self.grid[i-1][j] == 0 :
                    if i >= margin:
                        for k in range(margin):
                            with_margin[i-k][j] = 1
                    else:
                        for k in range(i):
                            with_margin[i-k][j] = 1
                if self.grid[i][j]==1 and self.grid[i+1][j] == 0 :
                    if i <= self.max - margin:
                        for k in range(margin):
                            with_margin[i+k][j] = 1
                    else:
                        for k in range(self.max - i):
                            with_margin[i+k][j] = 1

        return(with_margin)
        # inside = True
        # size = (self.max, self.max)
        # grid = np.zeros(size, dtype=int)
        # for k in range(len(obstacles)):
        #     obsta = obstacles[k]
        #     beginning = obsta[0] 
        #     for j in range(2):
        #         if beginning[j] < margin:
        #             beginning[j] = 0
        #         elif beginning[j] >= self.max:
        #             inside = False
        #         else: 
        #             beginning[j] -= margin

        #     end = obsta[1]
        #     for j in range(2):
        #         if end[j] > self.max+margin:
        #             end[j] = self.max
        #         elif end[j] <= 0:
        #             inside = False
        #         else: 
        #             end[j] += margin
        #     if inside == False:
        #         break
        
        #     hight,width = end[0]-beginning[0]+2*margin, end[1]-beginning[1]+2*margin
        #     for i in range(hight):
        #         for j in range(width):
        #             grid[beginning[0] + i][beginning[1]+ j] = 1
        # return(grid)
    
    def __len__(self):
        return(len(self.grid))
    
    def get_map(self):
        return(self.with_margin)
    
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

def get_path(robot, goal, matirx):
    #dots = [(2, 3)]
    #goal = (17, 46)
    #obstacles = [((5,5),(10,7)),((36,5),(40,20))]
    margin = 3
    maze = Map()
    maze.update_map(matirx)
    path = a.astar(maze.get_map(), tuple(robot), tuple(goal))
    maze.plot_map(path)
    return(path)

def make_matirx (obstacles):
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
        return grid

def main():
    robot = np.array([2, 3])
    goal = np.array([40, 30])
    obstacles = np.array([[[5,15],[10,30]],[[36,5],[40,15]],[[16,5],[20,20]]]) 
    matrix = make_matirx(obstacles)
    weg = get_path(robot, goal, matrix)
    print(weg)
    return()

if __name__ == "__main__":
    main()