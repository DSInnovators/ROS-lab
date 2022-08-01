import pygame
import math

from vehicle_control import *
from road_detection import *

screen_width = 1500
screen_height = 800
check_point = ((1200, 660), (1250, 120), (190, 200), (1030, 270), (250, 475), (650, 690))

class Car:
    def __init__(self, car_file, map_file, pos):
        # self.surface = pygame.image.load(car_file)
        # self.map = pygame.image.load(map_file)
        # self.surface = pygame.transform.scale(self.surface, (100, 100))
        # self.rotate_surface = self.surface
        self.pos = pos
        self.angle = 0
        self.speed = 0.3
        self.center = [self.pos[0] + 50, self.pos[1] + 50]
        self.radars = []
        self.radars_for_draw = []
        self.is_alive = True
        self.current_check = 0
        self.prev_distance = 0
        self.cur_distance = 0
        self.goal = False
        self.check_flag = False
        self.distance = 0
        self.time_spent = 0
        self.slope = 5
        self.reward = 0
        self.car_action = 0 #  0 forward, 1 left, 2 right
        self.road_state = 3
       

    def check_checkpoint(self):
        p = check_point[self.current_check]
        self.prev_distance = self.cur_distance
        dist = get_distance(p, self.center)
        if dist < 70:
            self.current_check += 1
            self.prev_distance = 9999
            self.check_flag = True
            if self.current_check >= len(check_point):
                self.current_check = 0
                self.goal = True
            else:
                self.goal = False

        self.cur_distance = dist

    def update(self):
        
        #todo move vehicle
        # move_vehicle(self.action)
        normal_speed = 0.5
        turn_speed = 0.3
        command_publisher = CommandPublisher()
        key_code = 'i'
        if self.car_action == 1:  # direction == MOVE_LEFT:
            key_code = 'u'
            command_publisher.set_speed(turn_speed)
        if self.car_action == 2:  # direction == MOVE_RIGHT:
            key_code = 'o'
            command_publisher.set_speed(turn_speed)
        # if self.car_action == 1:    #direction == TURN_LEFT:
        #     key_code = 'j'
        #     command_publisher.set_speed(turn_speed)
        # if self.car_action == 2:    # direction == TURN_RIGHT:
        #     key_code = 'l'
        #     command_publisher.set_speed(turn_speed)
        if self.car_action == 0:  # direction == MOVE_FORWARD:
            key_code = 'i'
            command_publisher.set_speed(normal_speed)
        # if direction == MOVE_STOP:
        #     key_code = 't'

        command_publisher.publish_command(key_code)
        
               
        

class PyGame2D:
    def __init__(self):
        pygame.init()
        # self.screen = pygame.display.set_mode((screen_width, screen_height))
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("Arial", 30)
        self.car = Car('car.png', 'map.png', [700, 650])
        self.game_speed = 60
        self.mode = 0

    def action(self, action):
        # if action == 0:
        #     self.car.speed += 2
        # if action == 1:
        #     self.car.angle += 5
        # elif action == 2:
        #     self.car.angle -= 5
        
        self.car.distance += self.car.speed
            
        self.car.car_action = action
        # print("--- action  ", action, " for road_state ", self.car.road_state)
        self.car.update()
        # self.car.check_collision()
        # self.car.check_checkpoint()

        # self.car.radars.clear()
        # # for d in range(-90, 120, 45):
        # #     self.car.check_radar(d)
        # self.car.check_radar(0)

    def evaluate(self):
        reward = 0
        """
        if self.car.check_flag:
            self.car.check_flag = False
            reward = 2000 - self.car.time_spent
            self.car.time_spent = 0
        """
        # if not self.car.is_alive:
        #     reward = -10000 + self.car.distance

        # elif self.car.goal:
        #     reward = 10000
        # print("evaluate slope ", self.car.slope)
        
        if self.car.road_state == 0: # no road so car is not alive
            reward = -10000 + self.car.distance
        elif self.car.distance >= 999: #self.car.prev_distance:
            reward = 10000
        # elif self.car.road_state == 3: # both side
        #     reward = 20
        else: # one side 
            reward = 20              
            
        return reward

    def is_done(self):      
        if self.car.road_state == 0 or self.car.distance > 999:
            return True
      
        return False

    def observe(self):             
        # todo return state value as observe
        ret = self.car.road_state
        # print("road state: ", ret)
        return ret
    
    def set_road_state(self, road_state):
        self.car.road_state = road_state
  
