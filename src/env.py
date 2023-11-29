from aruco import *
from robot import *
import time
MAP_UNITS = 3
class environment:
    def __init__(self):
        self.top_left = np.array([0,0])
        self.top_right = np.array([0,0])
        self.bottom_left = np.array([0,0])
        self.bottom_right = np.array([0,0])
        # map height is taken from top right of bottom left and bottom right of top left so we need remove unit size
        self.map_height = abs(((self.top_left[1] - self.bottom_left[1]) + (self.top_right[1] - self.bottom_right[1])) / 2) 
        # map width is taken from top right of top left and bottom right of bottom left so we need remove unit size
        self.map_width = abs(((self.top_right[0] - self.top_left[0]) + (self.bottom_right[0] - self.bottom_left[0])) / 2)
        
        self.unit_size = (self.map_height +  self.map_width) / 2 / MAP_UNITS

        # The origin of the map is the center of the bottom left unit
        self.map_width = self.map_width - self.unit_size
        self.map_height = self.map_height - self.unit_size
        self.origin = self.bottom_left + np.array([self.unit_size / 2, self.unit_size / 2])

        # A 6by6 matrix containing x y coordinates of units
        self.map = np.zeros((MAP_UNITS,MAP_UNITS,2))

    def update_map(self):
        self.map_height = abs(((self.top_left[1] - self.bottom_left[1]) + (self.top_right[1] - self.bottom_right[1])) / 2)
        self.map_width = abs(((self.top_right[0] - self.top_left[0]) + (self.bottom_right[0] - self.bottom_left[0])) / 2)
        self.unit_size = (self.map_height +  self.map_width) / 2 / MAP_UNITS
        self.origin = self.bottom_left
        # With the ratio between the top vertices and the bottom vertices, we can calculate the position of each unit
        # in the map
        ratio = (self.top_right - self.top_left) / (self.bottom_right - self.bottom_left)

        # Calculate the position of each unit in the map
        for i in range(MAP_UNITS):
            for j in range(MAP_UNITS):
                self.map[i,j] = self.origin + np.array([self.unit_size * ratio[0] * j, self.unit_size * ratio[1] * i])
                
        #print("MAP: ",self.map)
                
        time.sleep(1)




