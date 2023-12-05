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
        Ar = ArucoMarker((marker_id) for marker_id in range(1, 6)) 
        # size = (self.max, self.max)
        # grid = np.zeros(size, dtype=int)
        grid = Ar.Map_indices
        return(grid)
        # for k in range(len(obstacles)):
        #     obsta = obstacles[k]
        #     beginning = obsta[0]
        #     end = obsta[1]
        #     hight,width = end[0]-beginning[0],end[1]-beginning[1]
        #     for i in range(hight):
        #         for j in range(width):
        #             grid[beginning[0] + i][beginning[1]+ j] = 1
        # return(grid)
    
    def update_map(self, matirx):
        self.grid = matirx
        
    def add_margin(self, obstacles, margin):
        inside = True
        size = (self.max, self.max)
        grid = np.zeros(size, dtype=int)
        for k in range(len(obstacles)):
            obsta = obstacles[k]
            beginning = obsta[0] 
            for j in range(2):
                if beginning[j] < margin:
                    beginning[j] = 0
                elif beginning[j] >= self.max:
                    inside = False
                else: 
                    beginning[j] -= margin

            end = obsta[1]
            for j in range(2):
                if end[j] > self.max+margin:
                    end[j] = self.max
                elif end[j] <= 0:
                    inside = False
                else: 
                    end[j] += margin
            if inside == False:
                break
        
            hight,width = end[0]-beginning[0]+2*margin, end[1]-beginning[1]+2*margin
            for i in range(hight):
                for j in range(width):
                    grid[beginning[0] + i][beginning[1]+ j] = 1
        return(grid)
    
    def __len__(self):
        return(len(self.grid))
    
    # def get_map(self):
    #     return(self.collision_proof)
    
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

def get_path(map,robot, goal):
    #dots = [(2, 3)]
    #goal = (17, 46)
    #obstacles = [((5,5),(10,7)),((36,5),(40,20))]
    # margin = 3
    # maze = Map(np.array(obstacles), margin)
    maze = Map()
    #print how many 1 in the matrix
    #print(np.count_nonzero(maze.grid == 1))
    
    path = a.astar(map, tuple(robot), tuple(goal))
    #remove the first element of the path
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