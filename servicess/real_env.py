import json

import gymnasium as gym
import numpy as np
from gymnasium import spaces


class RealRobotEnv(gym.Env):
    def __init__(self, autopilot_service, verbose=True):
        super(RealRobotEnv, self).__init__()
        self.reduced_current_position = [0, 0, 0]
        self.last_min_distance = 10
        self.front_left_wheel_joint = 1
        self.front_right_wheel_joint = 2
        self.rear_left_wheel_joint = 3
        self.rear_right_wheel_joint = 4
        self.new_action = -1
        self.previous_bearing = -1
        self.last_collision_probability = 0
        self.dumb_fuck_points = 0
        self.reward = 0
        self.action_space = spaces.Discrete(5)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(365,), dtype=np.float32)  # LiDAR data
        self.previous_positions = []
        self.last_distance_to_target = 10000000
        # Target position and tolerance
        self.tolerance = 0.1
        self.stuck_count = 0
        self.last_action = 0
        self.spinning = 0
        self.verbose = verbose
        self.autopilot_service = autopilot_service

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        observation = self.autopilot_service.get_last_observation()
        info = {}
        observation = observation if observation is not None else np.zeros(365,)
        return observation, info

    def step(self, action):
        self.new_action = action
        reward = self.calculate_reward()
        truncated = False
        observation = self.autopilot_service.get_last_observation()
        done = self.check_done(observation)
        self.previous_bearing = self.autopilot_service.last_compass
        observation = observation if observation is not None else np.zeros(365,)
        return observation, reward, done, truncated, {}

    def calculate_reward(self):
        current_bearing = self.autopilot_service.last_compass
        lidar_data = self.autopilot_service.last_lidar
        min_distance = np.min(lidar_data)
        self.reward = 0
        if min_distance <= 0.4064:  # Assuming 1 meter is a close distance to an obstacle
            self.reward = -300.0
        if min_distance > self.last_min_distance:
            self.reward += 100.0
        if self.last_action == self.new_action and self.new_action == 0:
            self.stuck_count += 1
            self.reward -= 10.0 * self.stuck_count  # Penalty for getting stuck
        else:
            self.stuck_count = 0
        current_position = (self.autopilot_service.latitude, self.autopilot_service.longitude, 0)

        self.reduced_current_position = (
            np.round(current_position[0],3),
            np.round(current_position[1], 3),
            np.round(current_position[2], 3)
        )
        if self.reduced_current_position in self.previous_positions:
            self.reward -= 50.0
        else:
            self.reward += 50.0
        self.previous_positions.append(self.reduced_current_position)

        if self.new_action == 1 and min_distance > .4:
            self.reward += 150.0
        self.last_action = self.new_action
        self.last_min_distance = min_distance
        min_distance_position = np.argmin(lidar_data)
        max_distance_position = np.argmax(lidar_data)

        if min_distance_position in range((360 - (45 // 2)), 360) or min_distance_position in range(0, (45 // 2)):
            if self.new_action == 1 and min_distance <= 0.4064:
                self.dumb_fuck_points += 1
                self.reward -= 100.0 * self.dumb_fuck_points
            elif self.new_action in [4, 3, 2]:
                self.dumb_fuck_points = 0
                self.reward += 400.0

        if ((max_distance_position in range((360 - (45 // 2)), 360) or max_distance_position in range(0, (45 // 2))) and
                min_distance > 0.4064):
            if self.new_action == 2:
                self.reward -= 500.0
            else:
                self.reward += 500
        if min_distance_position in range((180 - (45 // 2)), (180 + (45 // 2))):
            if self.new_action == 2:
                self.reward -= 300
            if self.last_action == 2:
                self.reward -= 400.0
        if max_distance_position in range((90 - (45 // 2)), (90 + (45 // 2))):
            if self.new_action == 4:
                self.reward += 300.0
                if self.last_action == 4:
                    self.reward += 40.0
            else:
                self.reward -= 300
        if max_distance_position in range((180 - (45 // 2)), (180 + (45 // 2))):
            if self.new_action == 3:
                self.reward += 300.0
                if self.last_action == 3:
                    self.reward += 40.0
            else:
                self.reward -= 300
        if max_distance_position in range((270 - (45 // 2)), (270 + (45 // 2))):
            if self.new_action == 3:
                self.reward += 300.0
            else:
                self.reward -= 300

        if self.verbose:
            print(f"Min Distance: {min_distance}")
            print(f"Min Distance Angle: {min_distance_position}")
            print(f"Max Distance Angle: {max_distance_position}")
            print(f"Current bearing: {current_bearing} rad")
            print(f"Current position: {current_position}")
            print(f"Reward: {self.reward}")
            print(f"Action: {self.new_action}")
        return self.reward

    def check_done(self, obs):
        return False
