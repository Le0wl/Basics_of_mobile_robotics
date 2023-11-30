import numpy as np
import matplotlib.pyplot as plt
import time 
import threading
import sys 
import asyncio
import cv2

sys.path.append('.\src')

from robot import *
from map import *
import aruco
from env import *




#map = ObjectTracker()

#map.start()
trajectory = np.array([[0,0],[0,0]])

#camera_th = threading.Thread(target=lambda: asyncio.run(map.camera_feed()))
#map.camera_feed()

map_base = environment()
robot = robot()
#detect = RobotPosOrient()

markers = [ArucoMarker(marker_id) for marker_id in range(1, 6)]  # Create instances for each ArUco marker

# Create a thread for the ArUco marker detection
aruco_detect = threading.Thread(target=main_aruco, args=(*markers,))
# Create a thread for the robot
robot_th = threading.Thread(target=robot.run_robot, args=(markers[0].pos,markers[0].angle,))

aruco_detect.start()
robot_th.start()

MAP_SIZE_METERS = 0.5
MAP_UNITS = 4
print("MAIN")



def update_main():
    map_base.top_left = markers[1].pos
    map_base.top_right = markers[2].pos
    map_base.bottom_left = markers[3].pos
    map_base.bottom_right = markers[4].pos
    map_base.update_map()
    #robot.run_robot(markers[0].pos,markers[0].angle)
    robot.pos = markers[0].pos
    robot.phi = markers[0].angle
    angle = markers[0].angle

    """
    v = {"motor.left.target": [0],
             "motor.right.target": [0],}
    if(abs(angle) > 5):
        robot.turn(20,angle)
    else:
        robot.stop()
    """

    
    
    #print("TOP RIGHT: ",map_base.top_right)
    time.sleep(0.1)
    
    #distance_horizontal = np.linalg.norm(map_base.bottom_left - map_base.bottom_right)
    distance_vertical = np.linalg.norm(map_base.bottom_left - map_base.top_left)
    #distance_horizontal = map_base.bottom_right[0] - map_base.bottom_left[0] - markers[3].marker_pxl_size/2 - markers[4].marker_pxl_size/2
    distance_horizontal = np.linalg.norm(map_base.top_left - map_base.bottom_right)

    map_base.map = np.zeros((MAP_UNITS,MAP_UNITS,2))
    # get size of marker

    """
    for i in range(3):
        map_base.map[i,0] = markers[3].pos + (markers[3].marker_pxl_size/2) + np.array([(2*i+1)*distance_horizontal/6 , -markers[3].marker_pxl_size])
        #print("INTERVAL: ", (2*i+1)*distance_horizontal/6)
    """

    #We need to generate the coordinates for all the units in the map our image is a trapezoid so we need to transform it to a rectangle
    #if np.linalg.norm(map_base.top_right - map_base.top_left) > 0 and np.linalg.norm(map_base.bottom_right - map_base.bottom_left) > 0:
        #ratio = np.linalg.norm(map_base.bottom_right - map_base.bottom_left) / np.linalg.norm(map_base.top_right - map_base.top_left)*3
    #else:
    ratio = 1
    
    angle_map = markers[1].angle
    #print("ANGLE: ",angle_map)

    for i in range(MAP_UNITS):
        for j in range(MAP_UNITS):
            #map_base.map[i,j] = map_base.bottom_left + np.array([(2*i*ratio+1)*distance_horizontal/(2*MAP_UNITS) , -(2*j*ratio+1)*distance_vertical/(2*MAP_UNITS) ])
            #map_base.map[i,j] = map_base.bottom_left + np.array([j*ratio,0]) + np.array([(2*i+1)*(distance_horizontal - j*2*ratio)/(2*MAP_UNITS) , -(2*j+1)*distance_vertical/(2*MAP_UNITS) ])
            x_pos = j*ratio + (2*i+1)*(distance_horizontal - j*2*ratio)/(2*MAP_UNITS)
            y_pos = -(2*j+1)*distance_vertical/(2*MAP_UNITS)
            map_base.map[i,j] = map_base.top_left + np.array([x_pos,y_pos])
            #print("MAP: ",map_base.map[i,j])
            #- np.array([i*ratio,0])
    aruco.set_map(map_base.map)
    robot.trajectory = np.array(map_base.map[2,2])
    #print("MAP: ",map_base.map)
    #print("ANGLE  " ,np.rad2deg(np.arctan2(map_base.map[2,2][1] - map_base.bottom_left[1],map_base.map[2,2][0] - map_base.bottom_left[0])), "ROBOT: ",robot.pos,"GOAL: ",robot.trajectory)
    #print("ORIGIN: ",map_base.origin, "GOAL: ",robot.trajectory)
    #print("ROBOT: ",robot.pos,"GOAL: ",robot.trajectory)
    #print("ROBOT: ", robot.phi)

    angle =np.rad2deg(np.arctan2(-robot.trajectory[1] + robot.pos[1],robot.trajectory[0] - robot.pos[0])) + 180
    if angle > 180:
        angle = angle - 360
        #print("ANGLE: ",angle)
        #angle = robot.phi - angle + 90
        # between 270 and -90
        #if(angle > 180):
            #angle = angle - 360
        # angle goes from left to right starting from 180
        
        
    robot.teta = robot.phi - angle + 180

    if(robot.teta > 180):
        robot.teta = robot.teta - 360

        #print("GOAL", angle, "TETA", robot.teta)

    time.sleep(0.3)

    v = robot.v
    return v
    

if __name__ == "__main__":
    while True:
        update_main()
        #print("ROBOT: ", robot.phi, "GOAL: ",robot.teta)
        #print("ROBOT: ", robot.pos, "GOAL: ",robot.trajectory)
        angle =np.rad2deg(np.arctan2(-robot.trajectory[1] + robot.pos[1],robot.trajectory[0] - robot.pos[0])) + 180
        if angle > 180:
            angle = angle - 360
        #print("ANGLE: ",angle)
        #angle = robot.phi - angle + 90
        # between 270 and -90
        #if(angle > 180):
            #angle = angle - 360
        # angle goes from left to right starting from 180
        
        
        robot.teta = robot.phi - angle + 180

        if(robot.teta > 180):
            robot.teta = robot.teta - 360

        #print("GOAL", angle, "TETA", robot.teta)
        time.sleep(0.5)











