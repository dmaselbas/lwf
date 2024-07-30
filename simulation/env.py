import gymnasium as gym
from gymnasium import spaces
import pybullet as p
import pybullet_data
import numpy as np


class RobotEnv(gym.Env):
    def __init__(self):
        super(RobotEnv, self).__init__()
        self.front_left_wheel_joint = 1
        self.front_right_wheel_joint = 2
        self.rear_left_wheel_joint = 3
        self.rear_right_wheel_joint = 4
        self.new_action = -1
        self.previous_bearing = -1
        self.last_collision_probability = 0
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")
        self.robot_id = p.loadURDF("simulation/robot.urdf", [0, 0, 0])
        self.action_space = spaces.Discrete(5)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(360,), dtype=np.float32)  # LiDAR data
        self.boundary_ids = None
        self.previous_position, _ = p.getBasePositionAndOrientation(self.robot_id)
        # Target position and tolerance
        self.target_position = np.array([2.0, 2.0, 0.1])  # Example target position
        self.tolerance = 0.1
        self.stuck_count = 0
        self.last_action = 0
        self.spinning = 0

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        p.resetSimulation()
        p.setGravity(0, 0, -10)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.robot_id = p.loadURDF("simulation/robot.urdf", [0, 0, 0.1])
        self.add_boundaries()
        self.previous_bearing = self.get_bearing()
        self.previous_position, _ = p.getBasePositionAndOrientation(self.robot_id)
        observation = self.get_lidar_data().astype(np.float32)
        info = {}  # You can add any additional info here if needed
        return observation, info

    def add_boundaries(self):
        # Clear any existing boundaries
        if self.boundary_ids is not None:
            for boundary_id in self.boundary_ids:
                p.removeBody(boundary_id)
        self.boundary_ids = []

        # Define the size and position of the boundaries
        boundary_thickness = 0.1
        boundary_height = 0.5
        area_size = 10  # Define the size of the square area

        # Add four boundary walls
        self.boundary_ids.append(
                p.createMultiBody(
                        baseMass=0,
                        baseCollisionShapeIndex=p.createCollisionShape(
                                shapeType=p.GEOM_BOX,
                                halfExtents=[area_size / 2, boundary_thickness / 2, boundary_height / 2]
                                ),
                        basePosition=[0, area_size / 2, boundary_height / 2]
                        )
                )
        self.boundary_ids.append(
                p.createMultiBody(
                        baseMass=0,
                        baseCollisionShapeIndex=p.createCollisionShape(
                                shapeType=p.GEOM_BOX,
                                halfExtents=[area_size / 2, boundary_thickness / 2, boundary_height / 2]
                                ),
                        basePosition=[0, -area_size / 2, boundary_height / 2]
                        )
                )
        self.boundary_ids.append(
                p.createMultiBody(
                        baseMass=0,
                        baseCollisionShapeIndex=p.createCollisionShape(
                                shapeType=p.GEOM_BOX,
                                halfExtents=[boundary_thickness / 2, area_size / 2, boundary_height / 2]
                                ),
                        basePosition=[area_size / 2, 0, boundary_height / 2]
                        )
                )
        self.boundary_ids.append(
                p.createMultiBody(
                        baseMass=0,
                        baseCollisionShapeIndex=p.createCollisionShape(
                                shapeType=p.GEOM_BOX,
                                halfExtents=[boundary_thickness / 2, area_size / 2, boundary_height / 2]
                                ),
                        basePosition=[-area_size / 2, 0, boundary_height / 2]
                        )
                )
        p.loadURDF("plane.urdf")

    def step(self, action):
        left_wheel_velocity = 0
        right_wheel_velocity = 0

        if action == 0:  # Stop
            left_wheel_velocity = 0
            right_wheel_velocity = 0
        elif action == 1:  # Forward
            left_wheel_velocity = 10.0
            right_wheel_velocity = 10.0
        elif action == 2:  # Backward
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
        reward = self.calculate_reward()
        done = self.is_done()
        truncated = False
        self.previous_bearing = self.get_bearing()  # Update previous bearing
        return observation, reward, done, truncated, {}

    def get_lidar_data(self):
        lidar_data = []
        num_rays = 360
        ray_length = 10.0
        for i in range(num_rays):
            angle = (i * 2 * np.pi) / num_rays
            ray_end = [ray_length * np.cos(angle), ray_length * np.sin(angle), 0.2]
            ray_result = p.rayTest([0, 0, 0.2], ray_end)
            lidar_data.append(ray_result[0][2] if ray_result[0][0] != -1 else ray_length)
        return np.array(lidar_data)

    def get_bearing(self):
        _, orientation = p.getBasePositionAndOrientation(self.robot_id)
        euler = p.getEulerFromQuaternion(orientation)
        return euler[2]  # Yaw angle in radians

    def calculate_reward(self):
        current_bearing = self.get_bearing()
        lidar_data = self.get_lidar_data()
        min_distance = np.min(lidar_data)
        collision_probability = min(1, min_distance + 0.55)
        reward = 0

        if collision_probability > 0.95:  # Assuming 1 meter is a close distance to an obstacle
            reward = 0 # Larger penalty for closer obstacles
            if self.last_action == self.new_action:
                reward -= 100.0  # Small penalty for repeated actions

        if collision_probability < self.last_collision_probability:
            reward += 10.0
        self.last_collision_probability = collision_probability

        # Reward for turning away from the obstacle
        if 0.9 <= collision_probability < 0.98:
            obstacle_direction = np.argmin(lidar_data)
            if (obstacle_direction < 90 or obstacle_direction > 270) and (
                    np.pi / 2 < current_bearing < 3 * np.pi / 2):
                reward += 50.0  # Reward for turning left away from obstacle
            elif (90 < obstacle_direction < 270) and (
                    current_bearing < np.pi / 2 or current_bearing > 3 * np.pi / 2):
                reward += 50.0  # Reward for turning right away from obstacle
        if collision_probability >= 0.98 and self.new_action in [0, 2]:
            reward += 10.0
        if collision_probability >= 0.99:
            reward -= 200.0

        current_position, _ = p.getBasePositionAndOrientation(self.robot_id)
        distance_to_target = np.linalg.norm(np.array(current_position) - self.target_position)
        if distance_to_target < self.tolerance:
            reward += 10.0  # Large reward for reaching the target

        self.previous_position = current_position

        if self.new_action == 1 and collision_probability < 0.95:
            reward += 30.0  # Small penalty for not following the command
            self.spinning = min(0, (self.spinning - 1))
            if self.new_action == 1 and self.last_action == 1:
                reward = reward + (reward * 0.25)  # Small reward for following the command

        if self.new_action in [3, 4, 2] and collision_probability < 0.90:
            self.spinning = self.spinning + 1
            reward = reward - (self.spinning * 5.0)  # Penalty for spinning
        self.last_action = self.new_action
        print(f"Reward: {reward}")
        return max(reward, 256)

    def is_done(self):
        current_position, _ = p.getBasePositionAndOrientation(self.robot_id)
        distance_to_target = np.linalg.norm(np.array(current_position) - self.target_position)
        return bool(distance_to_target < self.tolerance)

    def render(self, mode='human'):
        pass

    def close(self):
        p.disconnect()
