# Create a class for the robot
# Class contains parameters such as speed, state...
# It also contains the states like position, angle...

class robot:
    def __init__(self):
        self.pos = [0,0]                 # position of the robot
        self.pos_marker_top = [0,0]      # position of the top marker
        self.pos_marker_bottom = [0,0]   # position of the bottom marker
        self.phi = 0                     # angle relative to map 
        self.teta = 0                    # angle relative to trajectory
        self.speed = [0,0]               #speed of right and left wheels
        self.state = 'STOP'              # state of the robot

    def go_forward(self):
        self.state = 'FORWARD'  # state of the robot
        self.speed = [100,100]  # both wheels go forward

        return  "motor.left.target": [100],
                "motor.right.target": [100],
                await node.set_variables(v)
        

    def turn(self):
        self.state = 'TURN'     # state of the robot
        if(self.teta < 0):
            self.speed = [-100,100] # turn left
        else:
            self.speed = [100,-100] # turn right

    def stop(self):
        self.state = 'STOP'     # state of the robot
        self.speed = [0,0]      # both wheels stop
        return  "motor.left.target": [0],
                "motor.right.target": [0],
                await node.set_variables(v)
