# Create a class for the robot
# Class contains parameters such as speed, state...
# It also contains the states like position, angle...
import numpy as np
import time

__init__ = ['robot']
#CRUISING_SPEED = 80
#TURNING_SPEED = 50
class robot:
    def __init__(self):
        self.pos = np.array([0,0]) # position of the robot
        self.phi = 0 # angle of the robot
        self.phi_dot = 0  # angular velocity of the robot                             
        self.teta = 0 # angle between the robot and the next point                           
        self.speed = np.array([0,0])  # speed of the robot
        self.state = 'STOP' # state of the robot                     
        self.trajectory = self.pos  # trajectory of the robot

        self.v = np.array([0,0])  # speed of the motors
        #{"motor.left.target": [0],
                 # "motor.right.target": [0],}

    def go_forward(self,angle):
        self.state = 'FORWARD'  # state of the robot
        #CRUISING_SPEED = int(np.abs(2*np.rad2deg(np.arctan2(self.trajectory[1] - self.pos[1],self.trajectory[0] - self.pos[0]))/1.5) + 50)
        CRUISING_SPEED = 120 
        SPEED_LEFT = int(CRUISING_SPEED + angle*2)
        SPEED_RIGHT = int(CRUISING_SPEED - angle*2)
        #self.v = {"motor.left.target": [SPEED_LEFT],
             #"motor.right.target": [SPEED_RIGHT],}
        self.v = np.array([SPEED_LEFT,SPEED_RIGHT])
               

    def turn(self,speed,angle):
        self.state = 'TURN'     # state of the robot
        TURNING_SPEED = int(abs(self.teta)) 
        
        if(angle < 0):
            #self.v = {"motor.left.target": [-TURNING_SPEED],
                 #"motor.right.target": [TURNING_SPEED ],}
            self.v = np.array([-TURNING_SPEED,TURNING_SPEED])

        else:
            #self.v = {"motor.left.target": [TURNING_SPEED],
                 #"motor.right.target": [-TURNING_SPEED],}
            self.v = np.array([TURNING_SPEED,-TURNING_SPEED])
         

    def stop(self):
        self.state = 'STOP'             # state of the robot
        #self.v = {"motor.left.target": [0],
             #"motor.right.target": [0],}
        self.v = np.array([0,0])

    def run_robot(self):
        """
        This function is used to control the robot. It is callaed in a thread and runs at 10HZ.
        """

        while True:
          
            if self.state != 'FINISH':
            
                if np.abs(self.teta) > 40:
                    self.turn(0,self.teta)
                    self.state = 'TURN'
                else:
                    self.go_forward(self.teta)
                    self.state = 'FORWARD'
                
            else:
                self.stop()
                break

            if False: 
                self.stop()
                #print('Goal reached')
                self.state = 'FINISH'
                break
                
            time.sleep(0.1)
    

                    

