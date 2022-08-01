import gym
from gym import spaces
import numpy as np
from gym_game.envs.pygame_2d import PyGame2D
import os

class CustomEnv(gym.Env):
    #metadata = {'render.modes' : ['human']}
    def __init__(self):
        self.pygame = PyGame2D()
        self.action_space = spaces.Discrete(3)
        # self.observation_space = spaces.Box(np.array([0, 0, 0, 0, 0]), np.array([10, 10, 10, 10, 10]), dtype=np.int)
        self.observation_space = spaces.Box(low=0, high=3, shape=(1,), dtype=np.int)  # mazhar 1 radar
        # self.observation_space = spaces.Box(np.array([0, 0]), np.array([1,1]), dtype=np.int)
        

    def reset(self):
        os.system('rosservice call /gazebo/reset_simulation "{}" ')
        del self.pygame        
        self.pygame = PyGame2D()
        obs = self.pygame.observe()        
        return obs

    def step(self, action):
        self.pygame.action(action)
        obs = self.pygame.observe()
        reward = self.pygame.evaluate()
        done = self.pygame.is_done()
        return obs, reward, done, {}

    def render(self, mode="human", close=False):
        # self.pygame.view()
        pass

    # def set_slope(self, slope):
    #     self.pygame.set_slope(slope)
        
    def set_road_state(self, road_state):
        self.pygame.set_road_state(road_state)
        
