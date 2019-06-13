import math
import time
from Util import *
from States import *

from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket

class Deetzz(BaseAgent):

    def initialize_agent(self):
        #This runs once before the bot starts up
        self.me = obj()
        self.ball = obj()
        self.start = time.time()
        self.steer_correction_radians: float = 0

        self.state = calcShot()
        self.controller = calcController
    
    def checkState(self):
        if self.state.expired:
            if calcShot().available(self) == True:
                self.state = calcShot()
            elif quickShot().available(self) == True:
                self.state = quickShot()
            elif wait().available(self) == True:
                self.state = wait()
            else:
                self.state = quickShot()
        
    def get_output(self, game: GameTickPacket) -> SimpleControllerState:
        self.preprocess(game) # Put some game data in easy to use variables

        self.checkState() # Checks to see which state the bot needs to be set to

        return self.state.execute(self)
    
    def preprocess(self, game):
        self.players = []
        car = game.game_cars[self.index]
        self.me.location.data = [car.physics.location.x, car.physics.location.y, car.physics.location.z]
        self.me.velocity.data = [car.physics.velocity.x, car.physics.velocity.y, car.physics.velocity.z]       
        self.me.rotation.data = [car.physics.rotation.pitch, car.physics.rotation.yaw, car.physics.rotation.roll]
        self.me.rvelocity.data = [car.physics.angular_velocity.x, car.physics.angular_velocity.y, car.physics.angular_velocity.z]  
        self.me.matrix = rotator_to_matrix(self.me)
        self.me.boost = car.boost
        self.me.grounded = car.has_wheel_contact

        ball = game.game_ball
        self.ball.location.data = [ball.physics.location.x,ball.physics.location.y,ball.physics.location.z]
        self.ball.velocity.data = [ball.physics.velocity.x,ball.physics.velocity.y,ball.physics.velocity.z]
        self.ball.rotation.data = [ball.physics.rotation.pitch,ball.physics.rotation.yaw,ball.physics.rotation.roll]
        self.ball.rvelocity.data = [ball.physics.angular_velocity.x,ball.physics.angular_velocity.y,ball.physics.angular_velocity.z]

        self.ball.local_location = to_local(self.ball,self.me)

        self.boosts = game.game_boosts

        for i in range(game.num_cars):
            if i != self.index:
                car = game.game_cars[i]
                temp = obj()
                temp.index = i
                temp.team = car.team
                temp.location.data = [car.physics.location.x, car.physics.location.y, car.physics.location.z]
                temp.velocity.data = [car.physics.velocity.x, car.physics.velocity.y, car.physics.velocity.z]       
                temp.rotation.data = [car.physics.rotation.pitch, car.physics.rotation.yaw, car.physics.rotation.roll]
                temp.rvelocity.data = [car.physics.angular_velocity.x, car.physics.angular_velocity.y, car.physics.angular_velocity.z]  
                self.me.boost = car.boost   
                flag = False
                for item in self.players:
                    if item.index == i:
                        item = temp
                        flag = True
                        break
                if flag:
                    self.players.append(temp)
