import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors
import astar as a

class Map:
    def __init__(self, obstacles, scale):
        self.grid = self.init_grid(obstacles)
        self.collision_proof = self.margin(obstacles, scale)

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
    
    def margin(self, obstacles, scale):
        size = (50, 50)
        grid = np.zeros(size, dtype=int)
        for k in range(len(obstacles)):
            obsta = obstacles[k]
            beginning = obsta[0]
            end = obsta[1]
            hight,width = end[0]-beginning[0]+2*scale,end[1]-beginning[1]+2*scale
            for i in range(hight):
                for j in range(width):
                    grid[beginning[0] + i - scale][beginning[1]+ j- scale] = 1
        return(grid)
    
    def __len__(self):
        return(len(self.grid))
    
    def get_map(self):
        return(self.collision_proof)
    
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

def robo_pos(dots):
    robot = (1,1)
    nose, tail= dots[0],dots[1]
    robot = round((nose[0]+tail[0])/2),round((nose[1]+tail[1])/2)
    scale = round(np.sqrt((nose[0]-tail[0])^2+(nose[1]-tail[1])^2))
    return(robot, scale)

def get_path(dots, goal, obstacles):
    #dots = [(0, 0),(2, 3)]
    #goal = (17, 46)
    #obstacles = [((5,5),(10,7)),((36,5),(40,20))]
    robot, scale = robo_pos(dots)
    maze = Map(obstacles, scale)
    path = a.astar(maze.get_map(), robot, goal)
    maze.plot_map(path)
    return(path)

def main():
    dots = [(0, 0),(2, 3)]
    goal = (30, 20)
    obstacles = [((5,5),(10,7)),((36,5),(40,20)),((16,5),(20,20))] 
    weg = get_path(dots, goal, obstacles)
    print(weg)
    return()

if __name__ == "__main__":
    main()