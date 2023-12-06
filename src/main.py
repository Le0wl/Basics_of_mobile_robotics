import numpy as np
import matplotlib.pyplot as plt
import time 
import threading
import sys 

sys.path.append('.\src')

from robot import *
import aruco
from aruco import *
from env import *
from constants import *
from astar_path_planning import *
from kalman import *

hist_pos = np.zeros((2, 1))
hist_camera = np.zeros((2,1))

map_base = environment()
robot = robot()
kalman = Kalman()

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

def update_main(speed, angular_speed):

    ex_time = time.time()
    camera_blocked = markers[0].camera_blocked

#============================= SETTING MAP CORNERS ======================================================
    map_base.top_left = markers[1].pos
    map_base.top_right = markers[2].pos
    map_base.bottom_left = markers[3].pos
    map_base.bottom_right = markers[4].pos

#============================= MAP UPDATE ======================================================
    x_est, P_est = kalman.kalman_filter(markers[0].pos[0], markers[0].pos[1], markers[0].angle, speed, angular_speed, camera_blocked)

    markers[0].robot_est = x_est

    robot.pos = x_est[0:2]

    robot.phi = x_est[2]
  
    distance_vertical = map_base.get_vertical_distance()
    distance_horizontal = map_base.get_horizontal_distance()

    map_base.map = np.zeros((UNIT_NUMBER,UNIT_NUMBER,2))

 #============================= MAP GENERATION ======================================================

    for i in range(UNIT_NUMBER):
        for j in range(UNIT_NUMBER):
            
            x_pos = j*RATIO + (2*i+1)*(distance_horizontal - j*2*RATIO)/(2*UNIT_NUMBER)
            y_pos = -(2*j+1)*distance_vertical/(2*UNIT_NUMBER)

            map_base.map[i,j] = map_base.top_left + np.array([x_pos,y_pos])
           
    vision.set_map(map_base.map)

    goal_idx = markers[4].goal_idx

    rob_idx = markers[0].robot_idx

    map_base.map = markers[4].Map_indices

    path = get_path_rect(map_base.map,rob_idx, goal_idx)
    
    markers[0].path = path

    goal_coord = vision.Map_camera[goal_idx[0]][goal_idx[1]]

    distance_to_goal = math.sqrt((robot.pos[0] - goal_coord[0])**2 + (robot.pos[1] - goal_coord[1])**2)

    if distance_to_goal < DISTANCE_THRESHOLD:
        robot.state = 'FINISH'
    elif len(path) != 0: 
        robot.trajectory = vision.Map_camera[path[0][0]][path[0][1]]
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

    #print("Execution time: ", time.time() - ex_time)
    #np.append(hist_pos, robot.pos, axis=1)
    #np.append(hist_camera, markers[0].pos, axis=1)

    return v
    

if __name__ == "__main__":
    while True:
        update_main()
       