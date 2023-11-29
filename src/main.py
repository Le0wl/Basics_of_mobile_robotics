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

map_base = environment()
robot = robot()
#detect = RobotPosOrient()

markers = [ArucoMarker(marker_id) for marker_id in range(1, 6)]  # Create instances for each ArUco marker

# Create a thread for the ArUco marker detection
aruco_detect = threading.Thread(target=main_aruco, args=(*markers,))
# Create a thread for the robot
robot_th = threading.Thread(target=robot.update, args=(markers[0].pos,))

aruco_detect.start()

MAP_SIZE_METERS = 0.5
MAP_UNITS = 3


while True:
    map_base.top_left = markers[1].pos
    map_base.top_right = markers[2].pos
    map_base.bottom_left = markers[3].pos
    map_base.bottom_right = markers[4].pos
    map_base.update_map()
    #print("TOP LEFT: ",map_base.top_left)
    #clear terminal
    print("ANGLE: ",markers[0].angle)

      
    #print("TOP RIGHT: ",map_base.top_right)
    time.sleep(1)
    print("\033c") 










