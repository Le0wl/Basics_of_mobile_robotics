# Create a class for the robot
# Class contains parameters such as speed, state...
# It also contains the states like position, angle...
import numpy as np
import map
import time

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
        self.v = {"motor.left.target": [0],
                  "motor.right.target": [0],}

    def go_forward(self,speed):
        self.state = 'FORWARD'  # state of the robot
        self.v = {"motor.left.target": [CRUISING_SPEED],
             "motor.right.target": [CRUISING_SPEED],}
               

    def turn(self,speed,angle):
        self.state = 'TURN'     # state of the robot
        if(angle < 0):
            self.v = {"motor.left.target": [-TURNING_SPEED],
                 "motor.right.target": [TURNING_SPEED],}

        else:
            self.v = {"motor.left.target": [TURNING_SPEED],
                 "motor.right.target": [-TURNING_SPEED],}
         

    def stop(self):
        self.state = 'STOP'             # state of the robot
        self.v = {"motor.left.target": [0],
             "motor.right.target": [0],}

    
    def update(self, marker_position):
        self.pos = (marker_position[:,0] + marker_position[:,1]) / 2       
        self.phi = np.arctan2(marker_position[1,1] - marker_position[1,0],
                               marker_position[0,1] - marker_position[0,0])          
        
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

    def run_robot(self,marker_position, tranjectory):
        """
        This function is used to control the robot. It is callaed in a thread and runs at 10HZ.
        """

        while True:
            next_goal = tranjectory[:,0]
            #self.phi = np.arctan2(marker_position[1,1] - marker_position[1,0],
                                #marker_position[0,1] - marker_position[0,0])
            self.teta = np.arctan2(next_goal[1] - self.pos[1],next_goal[0] - self.pos[0])
            """
            while np.linalg.norm(next_goal - self.pos) > 0.1:
                self.update(marker_position)
                
                if np.abs(self.phi - self.teta) > 0.1:
                    v = self.turn(TURNING_SPEED,self.phi - self.teta)
                    self.state = 'TURN'
                    print('Turning')
                else:
                    v = self.go_forward(CRUISING_SPEED)
                    self.state = 'FORWARD'
                    print('Going forward')

                if np.linalg.norm(next_goal - self.pos) < 0.1:
                    #go to next trajectory point
                    tranjectory = np.delete(tranjectory,0,1)
                    print('Next goal')
                    self.state = 'STOP'
                    break
                if tranjectory.size == 0:
                    self.stop()
                    print('Goal reached')
                    self.state = 'FINISH'
                    break
                    """
            time.sleep(0.1)
    

                    

