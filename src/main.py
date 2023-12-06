import numpy as np
import matplotlib.pyplot as plt
import time 
import threading
import sys 
import asyncio
import cv2
import random

sys.path.append('.\src')

from robot import *
from map import *
import aruco
from env import *
from constants import *
from path import *
from astar_path_planning import *



map_base = environment()
robot = robot()


markers = [ArucoMarker(marker_id) for marker_id in range(1, 6)]  # Create instances for each ArUco marker

print("Starting camera...")

# Create a thread for the ArUco marker detection
aruco_detect = threading.Thread(target=main_aruco, args=(*markers,))
# Create a thread for the robot
robot_th = threading.Thread(target=robot.run_robot, args=())

aruco_detect.start()
robot_th.start()

time.sleep(3)
print("Running threads...")

def update_main():

    camera_blocked = markers[0].camera_blocked

#============================= SETTING MAP CORNERS ======================================================
    map_base.top_left = markers[1].pos
    map_base.top_right = markers[2].pos
    map_base.bottom_left = markers[3].pos
    map_base.bottom_right = markers[4].pos

#============================= MAP UPDATE ======================================================
    #map_base.update_map()

    robot.pos = markers[0].pos
    robot.phi = markers[0].angle
  
    distance_vertical = map_base.get_vertical_distance()
   
    distance_horizontal = map_base.get_horizontal_distance()

    map_base.map = np.zeros((UNIT_NUMBER,UNIT_NUMBER,2))
    # get size of marker

 #============================= MAP GENERATION ======================================================
    ratio = 1.2

    for i in range(UNIT_NUMBER):
        for j in range(UNIT_NUMBER):
            
            x_pos = j*ratio + (2*i+1)*(distance_horizontal - j*2*ratio)/(2*UNIT_NUMBER)
            y_pos = -(2*j+1)*distance_vertical/(2*UNIT_NUMBER)

            map_base.map[i,j] = map_base.top_left + np.array([x_pos,y_pos])
           
    aruco.set_map(map_base.map)

    goal_idx = markers[4].goal_idx

    rob_idx = markers[0].robot_idx

    mat = Map()

    mat.grid = markers[4].Map_indices

    path = get_path_rect(mat.grid,rob_idx, goal_idx)
    
    markers[0].path = path
    #print("Camera blocked: ", camera_blocked)
    
    if len(path) != 0: #and np.linalg.norm(robot.pos-markers[4].centroid_goal) > 15:
        robot.trajectory = aruco.Map_camera[path[0][0]][path[0][1]]
        robot.state = 'FORWARD'

    goal_coord = aruco.Map_camera[goal_idx[0]][goal_idx[1]]
    distance_to_goal = np.linalg.norm(robot.pos - goal_coord)
    
    if distance_to_goal < 50:
        robot.state = 'FINISH'
    else:
        robot.state = 'FORWARD'
  
#============================= ANGLE CALCULATION ====================================================
    angle =np.rad2deg(np.arctan2(-robot.trajectory[1] + robot.pos[1],robot.trajectory[0] - robot.pos[0])) + 180

    if angle > 180:
        angle = angle - 360
        
    robot.teta = robot.phi - angle + 180

    if(robot.teta > 180):
        robot.teta = robot.teta - 360
        
    time.sleep(0.1)
#=========================== OUTPUT IS SPEED OF MOTORS =============================================
    v = robot.v
    if camera_blocked:
        v = {"motor.left.target": [0],
             "motor.right.target": [0],}
    return v
    

if __name__ == "__main__":
    while True:
        update_main()
       