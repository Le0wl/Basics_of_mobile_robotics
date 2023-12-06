import matplotlib.pyplot as plt
from collections import deque
import numpy as np
from constants import *

def is_valid(x, y, grid, robot_size):
    n = len(grid)
    for i in range(max(0, x - robot_size + 1), min(n, x + robot_size)):
        for j in range(max(0, y - robot_size + 1), min(n, y + robot_size)):
            if not (0 <= i < n and 0 <= j < n) or grid[i][j] == 1:
                return False
    return True

def find_shortest_path(grid, start, goal, robot_size):
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    queue = deque([(start, [])])
    visited = set()
    n = len(grid)

    goal = tuple(goal)  # Convert goal to tuple if it's a list or numpy array

    while queue:
        (x, y), path = queue.popleft()
        if (x, y) == goal:
            return path + [(x, y)]

        if (x, y) not in visited and 0 <= x < n and 0 <= y < n:
            visited.add((x, y))
            for dx, dy in directions:
                new_x, new_y = x + dx, y + dy
                if is_valid(new_x, new_y, grid, robot_size) and 0 <= new_x < n and 0 <= new_y < n:
                    queue.append(((new_x, new_y), path + [(x, y)]))

    return None



def plot_grid(grid, path, start, goal):
    n = len(grid)
    plt.figure(figsize=(8, 8))
    plt.imshow(grid, cmap='Greys', interpolation='nearest')

    if path:
        path_x, path_y = zip(*path)
        plt.plot(path_y, path_x, marker='o', color='red')

    plt.plot(start[1], start[0], marker='o', color='green', markersize=10)
    plt.plot(goal[1], goal[0], marker='o', color='blue', markersize=10)

    plt.xlim(-0.5, n - 0.5)
    plt.ylim(n - 0.5, -0.5)
    plt.gca().invert_yaxis()
    plt.grid(visible=True, color='black', linestyle='-', linewidth=0.5)
    plt.show()

def find_direction_changes(path):
    if len(path) <= 2:
        return path

    direction_changes = [path[0]]

    for i in range(2, len(path)):
        dx1 = path[i - 1][0] - path[i - 2][0]
        dy1 = path[i - 1][1] - path[i - 2][1]
        dx2 = path[i][0] - path[i - 1][0]
        dy2 = path[i][1] - path[i - 1][1]

        if (dx1, dy1) != (dx2, dy2):
            direction_changes.append(path[i - 1])

    direction_changes.append(path[-1])

    return direction_changes

# Example grid with obstacles
grid = np.zeros((UNIT_NUMBER,UNIT_NUMBER))

def get_path_rect(grid, start, goal):
    #fip rows
    #print("GRID: ",grid)
    

    grid = np.flip(grid,0)
    #print("GRID: ",grid)
    #reverse x and y of start and goal
    start = (start[1],start[0])
    goal = (goal[1],goal[0])

    path = find_shortest_path(grid, start, goal, 3)
    #print("PATH: ",path)
    if path is None:
        return []
    else:
        path = find_direction_changes(path)
        direction_changes_path = find_direction_changes(path)
        # switch x and y coordinates
        path = [(y, x) for x, y in path]
        direction_changes_path = [(y, x) for x, y in direction_changes_path]

        #print("Path with direction changes:", direction_changes_path)
        if len(path) != 0:
            path.pop(0)

        return path
