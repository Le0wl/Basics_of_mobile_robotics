import numpy as np
def main():
    robot = np.array([2, 3])
    goal = np.array([40, 30])
    obstacles = np.array([[[5,15],[10,30]],[[36,5],[40,15]],[[16,5],[20,20]]]) 
    print(robot)
    # weg = get_path(robot, goal, obstacles)
    # print(weg)
    return()

if __name__ == "__main__":
    main()