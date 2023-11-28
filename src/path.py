import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors
import astar as a

class Map:
    def __init__(self, obstacles):
        self.grid = self.init_grid(obstacles)   

    def init_grid(self, obstacles):
        size = (50, 50)
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

    def __len__(self):
        return(len(self.grid))
    
    def get_map(self):
        return(self.grid)
    
    def getElement(self, i, j):
        return self.grid[i][j]
    
    def setElement(self, i, j, element):
        self.grid[i][j] = element

    def plot_map(self, path):
        for i in range(len(path)):
            self.grid[path[i]] = 2
        cmap = colors.ListedColormap(['white', 'black', 'red'])
        plt.imshow(self.grid, cmap=cmap, interpolation='nearest')
        plt.show()



def path(robot, goal, obstacles):
    #robot = (0, 0)
    #goal = (17, 46)
    #obstacles = [((5,5),(10,7)),((36,5),(40,20))]
    maze = Map(obstacles)

    path = a.astar(maze.get_map(), robot, goal)
    maze.plot_map(path)
    return(path)

