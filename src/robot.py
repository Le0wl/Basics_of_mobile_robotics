# Create a class for the robot
# Class contains parameters such as speed, state...
# It also contains the states like position, angle...
__init__ = ['robot']


class robot:
    def __init__(self):
        self.pos = [0,0]                 # position of the robot
        self.pos_marker_top = [0,0]      # position of the top marker
        self.pos_marker_bottom = [0,0]   # position of the bottom marker
        self.phi = 0                     # angle relative to map 
        self.teta = 0                    # angle relative to trajectory
        self.speed = [0,0]               #speed of right and left wheels
        self.state = 'STOP'              # state of the robot

    def go_forward(self,speed):
        self.state = 'FORWARD'  # state of the robot
        self.speed = [speed,speed]  # speed of right and left wheels
        v = {"motor.left.target": [50],
             "motor.right.target": [50],}
        return v
        #node.send_set_variables(v)

        

    def turn(self,speed):
        self.state = 'TURN'     # state of the robot
        if(self.teta < 0):
            self.speed = [-speed,speed] # turn left

        else:
            self.speed = [speed,-speed] # turn right
            v = {"motor.left.target": [self.speed[0]],
                 "motor.right.target": [self.speed[1]],}
        #node.send_set_variables(v)

    def stop(self):
        self.state = 'STOP'     # state of the robot
        self.speed = [0,0]      # speed of right and left wheels 
        v = {"motor.left.target": [0],
             "motor.right.target": [0],}
        return 'STOPPED'
        #node.send_set_variables(v)
        
    def print_status(self):
        print('Robot position: ',self.pos)
        print('Robot angle: ',self.phi)
        print('Robot speed: ',self.speed)
        print('Robot state: ',self.state)
        print('Robot trajectory angle: ',self.teta)
        print('Robot top marker position: ',self.pos_marker_top)
        print('Robot bottom marker position: ',self.pos_marker_bottom)
        print('---------------------------')