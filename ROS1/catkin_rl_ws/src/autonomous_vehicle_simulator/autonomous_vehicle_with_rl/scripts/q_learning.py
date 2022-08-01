import signal
import sys
import numpy as np
import math
import random

import gym
import gym_game
# from __future__ import print_function
# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
import traceback
from os.path import exists

MODEL_FILE_NPY = "data.npy"
episode_file = "episode.txt"

from vehicle_control import *
from road_detection import *

# Define your image topic
image_topic = "/vehicle_camera/image_raw"
env = gym.make("Pygame-v0")
MAX_EPISODES = 9999
MAX_TRY = 5000
epsilon = 1
epsilon_decay = 0.999
learning_rate = 0.1
gamma = 0.6
sub = None
q_table = None
current_episode = 0
q_table = np.zeros(0)


def signal_handler(signal, frame):
    print("\nprogram exiting gracefully")
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

def image_callback(msg):
    # print("Received an image!")
    global env, sub    
    try:
        # Convert your ROS Image message to OpenCV2
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(msg, "bgr8")
        # print("unsubscribe image -----------")
        sub.unregister()   
             

        # Detect road and get the decision to move forward or turn
        road_detection = RoadDetection(image)
        road_state = road_detection.process_image()
        # env.set_slope(slope)
        env.set_road_state(road_state)

        # control vehicle
        # move_vehicle(decision)

    except Exception as e:
        # print("exception: ")
        print(e)
        traceback.print_exc()

def simulate():
    global epsilon, epsilon_decay
    global env, sub, q_table, current_episode
    
    try:
        for episode in range(current_episode, MAX_EPISODES):
            # Init environment
            state = env.reset()
            total_reward = 0

            # AI tries up to MAX_TRY times
            for t in range(MAX_TRY):
                sub = rospy.Subscriber(image_topic, Image, image_callback)
                # print("---- subscribe image!")

                # In the beginning, do random action to learn
                # if random.uniform(0, 1) < epsilon:            
                if current_episode == 0 and random.uniform(0, 1) < epsilon:
                    action = env.action_space.sample()
                else:
                    action = np.argmax(q_table[state])

                # Do action and get result
                next_state, reward, done, _ = env.step(action)
                total_reward += reward

                # Get correspond q value from state, action pair
                # print("state: ", state)
                q_value = q_table[state][action]
                best_q = np.max(q_table[next_state])

                # Q(state, action) <- (1 - a)Q(state, action) + a(reward + rmaxQ(next state, all actions))
                q_table[state][action] = (1 - learning_rate) * q_value + learning_rate * (reward + gamma * best_q)

                # Set up for the next iteration
                state = next_state

                # Draw games
                env.render()

                # When episode is done, print reward
                if done or t >= MAX_TRY - 1:
                    print("######################### Episode %d finished after %i time steps with total reward = %f." % (episode, t, total_reward))
                    break

            # exploring rate decay
            if epsilon >= 0.005:
                epsilon *= epsilon_decay
                
            # save to file
            if episode >= 100 and episode % 50 == 0:
                f = open(episode_file, "w")
                f.write(str(episode))
                f.close()
                np.save(MODEL_FILE_NPY, q_table)
                
    except KeyboardInterrupt:
        print('interrupted!')

def init_rl():
    global q_table, current_episode
    env = gym.make("Pygame-v0")
    MAX_EPISODES = 9999
    MAX_TRY = 5000
    epsilon = 1
    epsilon_decay = 0.999
    learning_rate = 0.1
    gamma = 0.6
    num_box = tuple((env.observation_space.high +
                    np.ones(env.observation_space.shape)).astype(int))
    
    if exists(MODEL_FILE_NPY):
        f = open(episode_file,"r")
        for line in f.readlines():
            current_episode = int(line)
            print(current_episode)
        f.close()

        print("======== loaded trained data ===========")
        q_table = np.load(MODEL_FILE_NPY)
    else:
        current_episode = 0
        print("------------------- no trained data  ---------------")
        q_table = np.zeros(num_box + (env.action_space.n,))
        
    
    
    
    
    

