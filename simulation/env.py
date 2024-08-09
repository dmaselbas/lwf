import gymnasium as gym
from gymnasium import spaces
import pybullet as p
import pybullet_data
import numpy as np


class RobotEnv(gym.Env):
    def __init__(self, render=True, verbose=True,
                 maze_path="simulation/maze.urdf",
                 robot_path="simulation/robot.urdf"):
        super(RobotEnv, self).__init__()
        self.reduced_current_position = [0, 0, 0]
        self.maze_path = maze_path
        self.robot_path = robot_path
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
        if render:
            p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")
        p.loadURDF(self.maze_path, [1, -1, 0.25])
        self.robot_id = p.loadURDF(self.robot_path, [-1, -1, 0.15])
        self.action_space = spaces.Discrete(5)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(365,), dtype=np.float32)  # LiDAR data
        self.boundary_ids = None
        self.previous_position, _ = p.getBasePositionAndOrientation(self.robot_id)
        self.previous_positions = []
        self.last_distance_to_target = 10000000
        # Target position and tolerance
        self.tolerance = 0.1
        self.stuck_count = 0
        self.last_action = 0
        self.spinning = 0
        self.verbose = verbose
        self.waypoint_index = 0
        self.waypoints = np.array([
            [0.5, -1.0, 0.15],
            [0.5, -2.0, 0.15],
            [0.5, -2.5, 0.15],
            [-1.5, -2.5, 0.15],
            [-1.5, -3.0, 0.15],
            [-1.0, -3.0, 0.15],
            [-1.5, -4.0, 0.15],
            [-4.0, -5.0, 0.15],
        ])
        self.target_position = self.waypoints[self.waypoint_index]

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.reward = 0
        p.resetSimulation()
        p.setGravity(0, 0, -10)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.robot_id = p.loadURDF(self.robot_path, [-1, -1.2, 0.2])
        self.add_boundaries()
        observation = self.get_lidar_data().astype(np.float32)
        min_dist_angle = np.argmin(observation)
        max_dist_angle = np.argmax(observation)
        current_position, _ = p.getBasePositionAndOrientation(self.robot_id)
        self.last_distance_to_target = np.linalg.norm(np.array(current_position) - self.target_position)
        bearing = self.get_bearing()
        observation = np.concatenate((observation,
                                      np.array([
                                          current_position[0],
                                          current_position[1],
                                          bearing,
                                          min_dist_angle,
                                          max_dist_angle])))
        info = {}  # You can add any additional info here if needed
        return observation, info

    def add_boundaries(self):
        p.loadURDF("plane.urdf")
        p.loadURDF(self.maze_path, [1, -1, 0.3])

    def step(self, action):
        left_wheel_velocity = 0
        right_wheel_velocity = 0

        if action == 0:  # Stop
            left_wheel_velocity = 0
            right_wheel_velocity = 0
        elif action == 1:  #Reverse
            left_wheel_velocity = 10.0
            right_wheel_velocity = 10.0
        elif action == 2:  # Forward
            left_wheel_velocity = -10.0
            right_wheel_velocity = -10.0
        elif action == 3:  # Turn left
            left_wheel_velocity = -10.0
            right_wheel_velocity = 10.0
        elif action == 4:  # Turn right
            left_wheel_velocity = 10.0
            right_wheel_velocity = -10.0
        self.new_action = action
        # Apply velocities to the left and right wheels
        p.setJointMotorControl2(self.robot_id,
                                self.front_left_wheel_joint,
                                p.VELOCITY_CONTROL,
                                targetVelocity=left_wheel_velocity)
        p.setJointMotorControl2(self.robot_id,
                                self.front_right_wheel_joint,
                                p.VELOCITY_CONTROL,
                                targetVelocity=right_wheel_velocity)
        p.setJointMotorControl2(self.robot_id,
                                self.rear_left_wheel_joint,
                                p.VELOCITY_CONTROL,
                                targetVelocity=left_wheel_velocity)
        p.setJointMotorControl2(self.robot_id,
                                self.rear_right_wheel_joint,
                                p.VELOCITY_CONTROL,
                                targetVelocity=right_wheel_velocity)
        self.new_action = action
        p.stepSimulation()
        observation = self.get_lidar_data().astype(np.float32)
        min_dist_angle = np.argmin(observation)
        max_dist_angle = np.argmax(observation)
        current_position, _ = p.getBasePositionAndOrientation(self.robot_id)
        bearing = self.get_bearing()
        observation = np.concatenate((observation,
                                      np.array([
                                          current_position[0],
                                          current_position[1],
                                          bearing,
                                          min_dist_angle,
                                          max_dist_angle])))
        reward = self.calculate_reward()
        done = self.is_done()
        truncated = False
        self.previous_bearing = self.get_bearing()  # Update previous bearing
        return observation, reward, done, truncated, {}

    def get_lidar_data(self):
        lidar_data = []
        num_rays = 360
        ray_length = 10.0  # The maximum distance to measure (in meters)

        # Get the robot's current position and orientation
        robot_pos, robot_ori = p.getBasePositionAndOrientation(self.robot_id)
        robot_ori_matrix = p.getMatrixFromQuaternion(robot_ori)
        robot_ori_matrix = np.array(robot_ori_matrix).reshape(3, 3)

        # Define the sensor height from the ground
        sensor_height = 0.2

        for i in range(num_rays):
            angle = (i * 2 * np.pi) / num_rays
            ray_start = np.array(robot_pos) + np.array([0, 0, sensor_height])
            ray_end = ray_start + ray_length * np.dot(robot_ori_matrix, [np.cos(angle), np.sin(angle), 0])

            ray_result = p.rayTest(ray_start.tolist(), ray_end.tolist())
            if ray_result[0][0] != -1:
                # There is a hit
                hit_fraction = ray_result[0][2]
                distance = hit_fraction * ray_length
            else:
                # No hit
                distance = ray_length
            lidar_data.append(distance)

        return np.array(lidar_data)

    def get_bearing(self):
        _, orientation = p.getBasePositionAndOrientation(self.robot_id)
        euler = p.getEulerFromQuaternion(orientation)
        return euler[2]  # Yaw angle in radians

    def calculate_reward(self):
        current_bearing = self.get_bearing()
        lidar_data = self.get_lidar_data()
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
        current_position, _ = p.getBasePositionAndOrientation(self.robot_id)
        current_position, _ = p.getBasePositionAndOrientation(self.robot_id)
        distance_to_target = np.linalg.norm(np.array(current_position) - self.target_position)

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

        if distance_to_target < self.tolerance:
            self.reward += 10000.0  # Reward for reaching the target
            self.waypoint_index += 1
            self.target_position = self.waypoints[self.waypoint_index]
            print(f"Next waypoint: {self.waypoints[self.waypoint_index]}")

        # self.reward for getting closer to the target
        self.reward += (self.last_distance_to_target - distance_to_target) * 10
        self.last_distance_to_target = distance_to_target
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
            print(f"Distance to target: {distance_to_target} m")
            print(f"Reward: {self.reward}")
            print(f"Action: {self.new_action}")
        return self.reward

    def is_done(self):
        current_position, _ = p.getBasePositionAndOrientation(self.robot_id)
        distance_to_target = np.linalg.norm(np.array(current_position) - self.target_position)
        return bool(distance_to_target < self.tolerance) and self.waypoint_index == len(self.waypoints) - 1

    def render(self, mode='human'):
        pass

    def close(self):
        p.disconnect()
