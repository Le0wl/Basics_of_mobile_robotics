import numpy as np
import matplotlib.pyplot as plt
# import pathfinding as pf
from matplotlib import colors
import math
from constants import *

class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

    def calculate_heuristic(self, end):
        # Euclidean distance heuristic
        self.h = math.sqrt((self.position[0] - end.position[0])**2 + (self.position[1] - end.position[1])**2)


# Existing code...

def astar(maze_1, start, end, MAX_ITERATIONS=1000, MAX_LIST_SIZE=1000):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""
    
    maze = [row[::-1] for row in maze_1]
    #maze = [col[-1::] for col in maze]
    #exchange lines and columns
    #maze = np.transpose(maze_1)
    #flip matrix 90 degrees clockwise
    #maze = np.rot90(maze_1, k=1, axes=(0, 1))
    #flip matrix 90 degrees counter-clockwise
    maze = np.rot90(maze_1, k=3, axes=(0, 1))

    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    open_list = []
    closed_list = []
    path = []

    open_list.append(start_node)
    
    iterations = 0
    nodes_expanded = 0

    while len(open_list) > 0 and iterations < MAX_ITERATIONS:
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        open_list.pop(current_index)
        closed_list.append(current_node)
        nodes_expanded += 1
        
        if current_node == end_node:
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            #print("Path found!")
            return path[::-1]

        #if len(open_list) > MAX_LIST_SIZE or len(closed_list) > MAX_LIST_SIZE:
            #open_list = []
            #closed_list = []

        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) - 1) or node_position[1] < 0:
                continue
            
            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] == OBSTACLE:
                #print(f"Encountered obstacle at {node_position}")
                continue

            new_node = Node(current_node, node_position)

            new_node.g = current_node.g + 1
            new_node.calculate_heuristic(end_node)
            new_node.f = new_node.g + new_node.h

            if new_node in closed_list:
                continue

            if any(open_node == new_node and new_node.g > open_node.g for open_node in open_list):
                continue

            open_list.append(new_node)
            iterations += 1

    #print("Path not found!")
    #print(f"Nodes expanded: {nodes_expanded}")
    return path