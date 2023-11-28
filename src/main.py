import numpy as np
import matplotlib.pyplot as plt
import time 
import threading
import sys 
import asyncio

sys.path.append('.\src')

from robot import *
from map import *


robot = robot()

map = ObjectTracker()

map.start()
trajectory = np.array([[0,0],[0,0]])

#camera_th = threading.Thread(target=lambda: asyncio.run(map.camera_feed()))
#map.camera_feed()

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







