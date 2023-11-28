# Create a class for the robot
# Class contains parameters such as speed, state...
# It also contains the states like position, angle...
import numpy as np

__init__ = ['robot']
CRUISING_SPEED = 200
TURNING_SPEED = 50
class robot:
    def __init__(self):
        
        self.pos_marker_top = np.array([0,0])                           # position of the top marker
        self.pos_marker_bottom = np.array([0,0])                        # position of the bottom marker
        self.pos = np.array([0,0])                                      # position of the robot
        self.pos = (self.pos_marker_top + self.pos_marker_bottom) / 2   # position of the robot
        self.phi = np.arctan2(self.pos_marker_top[1] - self.pos_marker_bottom[1],
                               self.pos_marker_top[0] - self.pos_marker_bottom[0])
        self.phi_dot = 0                                                # angular velocity of the robot
        self.teta = 0                                                   # trajectory angle of the robot
        self.speed = np.array([0,0])                                    #speed of right and left wheels
        self.state = 'STOP'                                             # state of the robot

    def go_forward(self,speed):
        self.state = 'FORWARD'  # state of the robot
        v = {"motor.left.target": [CRUISING_SPEED],
             "motor.right.target": [CRUISING_SPEED],}
        return v        

    def turn(self,speed):
        self.state = 'TURN'     # state of the robot
        if(self.teta < 0):
            v = {"motor.left.target": [-TURNING_SPEED],
                 "motor.right.target": [TURNING_SPEED],}

        else:
            v = {"motor.left.target": [TURNING_SPEED],
                 "motor.right.target": [-TURNING_SPEED],}
        return v

    def stop(self):
        self.state = 'STOP'             # state of the robot
        v = {"motor.left.target": [0],
             "motor.right.target": [0],}
        return v
    
    def update(self, trajectory_angle):
        self.speed = self.pos
        self.pos = (np.add(self.pos_marker_top,self.pos_marker_bottom))/2  
        self.phi_dot = self.phi
        self.phi = np.arctan2(self.pos_marker_top[1] - self.pos_marker_bottom[1],
                               self.pos_marker_top[0] - self.pos_marker_bottom[0])
        self.teta = self.phi - np.deg2rad(trajectory_angle)   
        self.phi_dot = self.phi - self.phi_dot
        self.speed = self.pos - self.speed                     
        
    def print_status(self):
        print('Robot position: ',self.pos)
        print('Robot angle: ',np.rad2deg(self.phi))
        print('Robot angular velocity: ',self.phi_dot)
        print('Robot speed: ',self.speed)
        print('Robot state: ',self.state)
        print('Robot trajectory angle: ',np.rad2deg(self.teta))
        print('Robot top marker position: ',self.pos_marker_top)
        print('Robot bottom marker position: ',self.pos_marker_bottom)
        print('---------------------------')