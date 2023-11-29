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
from aruco import *
from env import *




#map = ObjectTracker()

#map.start()
trajectory = np.array([[0,0],[0,0]])

#camera_th = threading.Thread(target=lambda: asyncio.run(map.camera_feed()))
#map.camera_feed()
"""
camera_th = threading.Thread(target=map.camera_feed)
robot_th = threading.Thread(target=robot.run_robot, args=(map.pos_red,trajectory))

camera_th.start()
robot_th.start()
marker_pos = np.array([[0,0],[0,0]])
while True:
    print('Robot position: ',map.pos_red)
    #print("TEST", map.pos_red[0,0])
    angle = np.arctan2(map.pos_red[1,1] - map.pos_red[1,0], map.pos_red[0,1] - map.pos_red[0,0])
    #marker_pos[:,0] = map.pos_red[:][0]
    #marker_pos[:,1] = map.pos_red[:][1]
    #print('Robot angle: ',np.rad2deg(angle))
    #time.sleep(0.05)


#thymio_markers = np.array([[0,0],[0,0]])
"""
map_base = environment()
robot = robot()
#detect = RobotPosOrient()

markers = [ArucoMarker(marker_id) for marker_id in range(1, 6)]  # Create instances for each ArUco marker
aruco_detect = threading.Thread(target=main_aruco, args=(*markers,))

aruco_detect.start()

MAP_SIZE_METERS = 0.5
MAP_UNITS = 6


while True:
    map_base.top_left = markers[1].pos
    map_base.top_right = markers[2].pos
    map_base.bottom_left = markers[3].pos
    map_base.bottom_right = markers[4].pos








