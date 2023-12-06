from aruco import *
from robot import *
import time
from constants import *
class environment:
    def __init__(self):
        self.top_left = np.array([0,0])
        self.top_right = np.array([0,0])
        self.bottom_left = np.array([0,0])
        self.bottom_right = np.array([0,0])

        self.map = np.zeros((UNIT_NUMBER,UNIT_NUMBER,2))

    def get_horizontal_distance(self):
        return np.linalg.norm(self.top_left - self.bottom_right)
    
    def get_vertical_distance(self):
        return np.linalg.norm(self.top_left - self.bottom_left)

                
        




