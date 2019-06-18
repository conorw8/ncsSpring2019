import math

class Rover:

    def __init__(self, name, first_x, first_y, separation, Kv, Kt):
        self.name = name
        self.firstRead = True
        self.initial_x = float(first_x)
        self.initial_y = float(first_y)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.distance_goal = separation
        self.Kv = Kv
        self.Kt = Kt
        self.last_error = 0.0
        self.driveToBase = False
        self.count = 0

    def distance(self, leader):
        x_error = leader.x - self.x
        y_error = leader.y - self.y
        error = math.sqrt(math.pow(x_error, 2) + math.pow(y_error, 2))
        #print("%s distance error: %s" % (self.name, error))
        if error < self.distance_goal:
            return 0.0
        else:
            return (self.Kv * error)


    def steering(self, leader):
        error = leader.theta - self.theta
        if error < 0.0:
            error = self.last_error
        self.last_error = error
        #print("%s heading error: %s" % (self.name, error))
        return (self.Kt * error)

    def distanceToBase(self, leader, agent1, agent2, base):
        velocity = 0.0
        x_error = leader.x - self.x
        y_error = leader.y - self.y
        x_error1 = agent1.x - self.x
        y_error1 = agent1.y - self.y
        x_error2 = agent2.x - self.x
        y_error2 = agent2.y - self.y
        x_baseError = base[0] - self.x
        y_baseError = base[1] - self.y
        error = math.sqrt(math.pow(x_error, 2) + math.pow(y_error, 2))
        error1 = math.sqrt(math.pow(x_error1, 2) + math.pow(y_error1, 2))
        error2 = math.sqrt(math.pow(x_error2, 2) + math.pow(y_error2, 2))

        #print("Distance to leader: %s" % error)
        #print("Distance to constituent1: %s" % error1)
        #print("Distance to constituent2: %s" % error2)

        baseError = math.sqrt(math.pow(x_baseError, 2) + math.pow(y_baseError, 2))
        if self.driveToBase != True:
            velocity = 0.3
        else:
            if(baseError < 0.25):
                #print("stop")
                velocity = 0.0
            else:
                velocity = 0.1 * baseError

        return velocity

    def steerToBase(self, leader, agent1, agent2, base):
        if self.driveToBase == True:
            print("driving to base")
            velocity = 0.0
            #print("Heading to the base")
            x_error = base[0] - self.x
            y_error = base[1] - self.y
            errorToBase = math.radians(math.atan2(y_error, x_error))
            print("heading error to base: %s" % errorToBase)
            velocity = 0.5 * errorToBase
        else:
            #print("follow the leader")
            velocity = 0.0
            if leader.theta > 0.01 and leader.theta < 3.14:
                if self.count == 80:
                    self.driveToBase = True
                    velocity = 0.0
                else:
                    velocity = -1.45
                    self.count += 1
            else:
                if self.count == 80:
                    self.driveToBase = True
                    velocity = 0.0
                else:
                    velocity = -1.45
                    self.count += 1

        return velocity
