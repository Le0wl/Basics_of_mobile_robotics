# Create a class for the robot
# Class contains parameters such as speed, state...
# It also contains the states like position, angle...
import numpy as np
import map
import time

__init__ = ['robot']
#CRUISING_SPEED = 80
#TURNING_SPEED = 50
class robot:
    def __init__(self):
        self.pos = np.array([0,0])                                      # position of the robot
        self.phi = 0
        self.phi_dot = 0                                                # angular velocity of the robot
        self.teta = 0                                                   # trajectory angle of the robot
        self.speed = np.array([0,0])                                    #speed of right and left wheels
        self.state = 'STOP'                                             # state of the robot
        self.trajectory = self.pos                 # trajectory of the robot
        self.v = {"motor.left.target": [0],
                  "motor.right.target": [0],}

    def go_forward(self,angle):
        self.state = 'FORWARD'  # state of the robot
        #CRUISING_SPEED = int(np.abs(2*np.rad2deg(np.arctan2(self.trajectory[1] - self.pos[1],self.trajectory[0] - self.pos[0]))/1.5) + 50)
        CRUISING_SPEED = 200 
        SPEED_LEFT = int(CRUISING_SPEED+ angle*5)
        SPEED_RIGHT = int(CRUISING_SPEED - angle*5)
        self.v = {"motor.left.target": [SPEED_LEFT],
             "motor.right.target": [SPEED_RIGHT],}
               

    def turn(self,speed,angle):
        self.state = 'TURN'     # state of the robot
        TURNING_SPEED = int(abs(self.teta/1.8) +10)
        
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

    def run_robot(self,marker_position,angle):
        """
        This function is used to control the robot. It is callaed in a thread and runs at 10HZ.
        """

        while True:
            #next_goal = self.trajectory

            #self.teta = np.rad2deg(np.arctan2(next_goal[1] - self.pos[1],next_goal[0] - self.pos[0]))
            #self.teta = 90
            #print("GOAL: ",next_goal,"POS: ",self.pos)
            #print("INNNNNNNNNNN")
            #self.pos = marker_position    
            #self.phi = marker_angle 
            
            #while np.linalg.norm(next_goal - self.pos) > 30:
                #print("teta:  ",self.teta)
            #print("DISTANCE: ",np.linalg.norm(next_goal - self.pos))
            #print(np.linalg.norm(self.trajectory - self.pos))
            if self.state != 'FINISH':
            
                if np.abs(self.teta) > 15:
                    self.turn(0,self.teta)
                    self.state = 'TURN'
                    #print("PHI: ",self.phi, "TETA: ",self.teta)
                    #print('Turning')
                else:
                    self.go_forward(self.teta)
                    self.state = 'FORWARD'
                #print('Going forward')
            else:
                self.stop()
                #print('Goal reached')
                break

            #if np.linalg.norm(self.trajectory - self.pos) < 10:
                #go to next trajectory point
                #self.trajectory = np.delete(self.trajectory,0,1)
                #print('Next goal',self.trajectory)
                
                #self.state = 'fake'
                #break
            if False: #self.trajectory.size < 1:
                self.stop()
                #print('Goal reached')
                self.state = 'FINISH'
                break
                
            time.sleep(0.1)
    

                    

