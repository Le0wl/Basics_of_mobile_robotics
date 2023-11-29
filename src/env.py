from aruco import *
from robot import *

class environment:
    def __init__(self):
        self.top_left = np.array([0,0])
        self.top_right = np.array([0,0])
        self.bottom_left = np.array([0,0])
        self.bottom_right = np.array([0,0])
        self. map_height = ((self.top_left[1] - self.bottom_left[1]) + (self.top_right[1] - self.bottom_right[1])) / 2
        self.map_width = ((self.top_right[0] - self.top_left[0]) + (self.bottom_right[0] - self.bottom_left[0])) / 2
        

