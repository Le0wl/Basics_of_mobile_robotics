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

map.main()

#camera_th = threading.Thread(target=lambda: asyncio.run(map.camera_feed()))
#map.camera_feed()
camera_th = threading.Thread(target=map.camera_feed)
camera_th.start()

while True:
    print('Robot position: ',map.pos_red)


#thymio_markers = np.array([[0,0],[0,0]])







