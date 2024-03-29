import math
import time

GOAL_WIDTH = 1900
FIELD_LENGTH = 10280
FIELD_WIDTH = 8240

boosts = [
        [0.0, -4240.0, 70.0],
        [-1792.0, -4184.0, 70.0],
        [ 1792.0, -4184.0, 70.0],
        [-3072.0, -4096.0, 73.0], 
        [ 3072.0, -4096.0, 73.0],
        [- 940.0, -3308.0, 70.0],
        [  940.0, -3308.0, 70.0],
        [    0.0, -2816.0, 70.0],
        [-3584.0, -2484.0, 70.0],
        [ 3584.0, -2484.0, 70.0],
        [-1788.0, -2300.0, 70.0],
        [ 1788.0, -2300.0, 70.0],
        [-2048.0, -1036.0, 70.0],
        [    0.0, -1024.0, 70.0],
        [ 2048.0, -1036.0, 70.0],
        [-3584.0,     0.0, 73.0],
        [-1024.0,     0.0, 70.0],
        [ 1024.0,     0.0, 70.0],
        [ 3584.0,     0.0, 73.0],
        [-2048.0,  1036.0, 70.0],
        [    0.0,  1024.0, 70.0],
        [ 2048.0,  1036.0, 70.0],
        [-1788.0,  2300.0, 70.0],
        [ 1788.0,  2300.0, 70.0],
        [-3584.0,  2484.0, 70.0],
        [ 3584.0,  2484.0, 70.0],
        [    0.0,  2816.0, 70.0],
        [- 940.0,  3310.0, 70.0],
        [  940.0,  3308.0, 70.0],
        [-3072.0,  4096.0, 73.0],
        [ 3072.0,  4096.0, 73.0],
        [-1792.0,  4184.0, 70.0],
        [ 1792.0,  4184.0, 70.0],
        [    0.0,  4240.0, 70.0]
    ]



class Vector3:
    def __init__(self, data):
        self.data = data
    def __add__(self,value):
        return Vector3([self.data[0]+value.data[0],self.data[1]+value.data[1],self.data[2]+value.data[2]])
    def __sub__(self,value):
        return Vector3([self.data[0]-value.data[0],self.data[1]-value.data[1],self.data[2]-value.data[2]])
    def __mul__(self,value):
        return (self.data[0]*value.data[0] + self.data[1]*value.data[1] + self.data[2]*value.data[2])
    def magnitude(self):
        return math.sqrt((self.data[0]*self.data[0]) + (self.data[1] * self.data[1])+ (self.data[2]* self.data[2]))
    def normalize(self):
        mag = self.magnitude()
        if mag != 0:
            return Vector3([self.data[0]/mag, self.data[1]/mag, self.data[2]/mag])
        else:
            return Vector3([0,0,0])
    def dot(self,value):
        return self.data[0]*value[0] + self.data[1]*value[1] + self.data[2]*value[2]
    def scale(self, scale):
        return Vector3([self.data[0] * scale, self.data[1] * scale, self.data[2] * scale])
    def flatten(self):
        return Vector3([self.data[0], self.data[1], 0])

class obj:
    def __init__(self):
        #for car
        self.location = Vector3([0,0,0]) # x,y,z
        self.velocity = Vector3([0,0,0])
        self.rotation = Vector3([0,0,0]) # pitch, yaw, roll
        self.rvelocity = Vector3([0,0,0])

        #for ball
        self.local_location = Vector3([0,0,0]) # x,y,z
        self.boost = 0

def quad(a,b,c):
    inside = (b**2) - (4*a*c)
    if inside < 0 or a == 0:
        return 0.1
    else:
        n = ((-b - math.sqrt(inside))/(2*a))
        p = ((-b + math.sqrt(inside))/(2*a))
        if p > n:
            return p
        return n

def future(ball,time):
    x = ball.location.data[0] + (ball.velocity.data[0] * time)
    y = ball.location.data[1] + (ball.velocity.data[1] * time)
    z = ball.location.data[2] + (ball.velocity.data[2] * time)
    return Vector3([x,y,z])
    
def timeZ(ball):
    rate = 0.97
    return quad(-325, ball.velocity.data[2] * rate, ball.location.data[2]-92.75)

def dpp(target_loc,target_vel,our_loc,our_vel): #returns the rate of change in distance between the target object and the agent
    target_loc = toLocation(target_loc)
    our_loc = toLocation(our_loc)
    our_vel = toLocation(our_vel)
    d = distance2D(target_loc,our_loc)
    if d != 0:
        return (((target_loc.data[0] - our_loc.data[0]) * (target_vel.data[0] - our_vel.data[0])) + ((target_loc.data[1] - our_loc.data[1]) * (target_vel.data[1] - our_vel.data[1])))/d
    else:
        return 0

def to_local(target_object,our_object):
    x = (toLocation(target_object) - our_object.location) * our_object.matrix[0]
    y = (toLocation(target_object) - our_object.location) * our_object.matrix[1]
    z = (toLocation(target_object) - our_object.location) * our_object.matrix[2]
    return Vector3([x,y,z])

def rotator_to_matrix(our_object):
    r = our_object.rotation.data
    CR = math.cos(r[2])
    SR = math.sin(r[2])
    CP = math.cos(r[0])
    SP = math.sin(r[0])
    CY = math.cos(r[1])
    SY = math.sin(r[1])

    matrix = []
    matrix.append(Vector3([CP*CY, CP*SY, SP]))
    matrix.append(Vector3([CY*SP*SR-CR*SY, SY*SP*SR+CR*CY, -CP * SR]))
    matrix.append(Vector3([-CR*CY*SP-SR*SY, -CR*SY*SP+SR*CY, CP*CR]))
    return matrix

def ballReady(agent):
    ball = agent.ball
    if abs(ball.velocity.data[2]) < 150 and timeZ(agent.ball) < 1:
        return True
    return False

def ballProject(agent):
    goal = Vector3([0,-sign(agent.team)*FIELD_LENGTH/2,100])
    goal_to_ball = (agent.ball.location - goal).normalize()
    difference = agent.me.location - agent.ball.location
    return difference * goal_to_ball
    
def sign(x):
    if x <= 0:
        return -1
    else:
        return 1

def cap(x, low, high):
    if x < low:
        return low
    elif x > high:
        return high
    else:
        return x

def steer(angle):
    final = ((10 * angle+sign(angle))**3) / 20
    return cap(final,-1,1)

def throttle(speed, agent_speed):
    final = ((speed - agent_speed)/100)
    if final > 0.55:
        boost = True
    else:
        boost = False

    if final > -0.55:
        return cap(final,0,1),boost
    else:
        return cap(final,-1,1),boost

def angle2(target_location,object_location):
    difference = toLocation(target_location) - toLocation(object_location)
    return math.atan2(difference.data[1], difference.data[0])


def velocity2D(target_object):
    return math.sqrt(target_object.velocity.data[0]**2 + target_object.velocity.data[1]**2)

def toLocal(target,our_object):
    if isinstance(target,obj):
        return target.local_location
    else:
        return to_local(target,our_object)

def toLocation(target):
    if isinstance(target,Vector3):
        return target
    elif isinstance(target,list):
        return Vector3(target)
    else:
        return target.location

def distance2D(target_object, our_object):
    difference = toLocation(target_object) - toLocation(our_object)
    return math.sqrt(difference.data[0]**2 + difference.data[1]**2)

def dodging(agent):
    time_diff = time.time() - agent.start
    if time_diff < 1.3 :
        return True
    return False

def radius(v):
    return 139.059 + (0.1539 * v) + (0.0001267716565 * v * v)

def kickoff(agent):
    if agent.ball.location.data[0] == 0 and agent.ball.location.data[1] == 0:
        return True
    return False