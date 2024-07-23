import atexit
import json
import threading
import time
from typing import Callable

from devices.compass import HMC5883L
from devices.drive import DriveController
from devices.gps import GPSController
from devices.lidar import LidarController
from devices.pwm_controller import PWMController


class CollisionAvoidanceSystem:

    def __init__(self, on_update_callback: Callable,
                 drive: DriveController,
                 lidar: LidarController,
                 gps: GPSController):
        self.collision_probability = 1
        self.on_update_callback = on_update_callback

        self.lidar = lidar
        self.gps = gps
        self.drive = drive
        self.lidar_data = None
        self.gps_data = None
        self.running = True
        self.last_speed = 0
        self.last_direction = None
        self.last_heading = 0
        self.taking_avoidance_action = False
        self.system_enabled = True
        atexit.register(self.shutdown)

        self.thread = threading.Thread(target=self.run, daemon=True, name="CollisionAvoidanceSystem")
        self.thread.start()

    def shutdown(self):
        self.running = False

    def turn_on(self):
        self.system_enabled = True

    def turn_off(self):
        self.system_enabled = False

    def handle_lidar_update(self, lidar_data):
        self.lidar_data = lidar_data
        self.calculate_collision_probability()

    def handle_gps_update(self, gps_data):
        print(f"Received gps update: {gps_data}")
        self.gps_data = gps_data

    def avoid_collision(self):
        if not self.system_enabled:
            time.sleep(5)
            return
        if not self.taking_avoidance_action:
            self.taking_avoidance_action = True
            self.last_speed = self.drive.get_speed()
            self.last_direction = self.drive.get_direction()
            self.last_heading = self.gps.get_heading()
            self.drive.set_speed(0)
            while self.calculate_collision_probability() > .90:
                if not self.system_enabled:
                    return
                dir_probs = self.calculate_directional_collision_probabilities()
                min_prob_direction = min(dir_probs, key=dir_probs.get)
                max_prob_direction = max(dir_probs, key=dir_probs.get)
                if min_prob_direction == max_prob_direction:
                    self.drive.stop()
                elif min_prob_direction == "rear":
                    self.drive.reverse()
                    self.drive.set_speed(513)
                elif min_prob_direction == "left":
                    self.drive.left()
                    self.drive.set_speed(513)
                elif min_prob_direction == "right":
                    self.drive.right()
                    self.drive.set_speed(513)
                elif min_prob_direction == "front":
                    self.drive.forward()
                    self.drive.set_speed(513)
                elif min_prob_direction == "rear":
                    self.drive.reverse()
                    self.drive.set_speed(513)
                else:
                    self.drive.stop()
            self.resume()

    def correct_heading(self):
        last_heading_difference = self.last_heading - self.gps.get_heading()
        mode = "right"
        while self.gps.get_heading() != self.last_heading:
            heading_difference = self.last_heading - self.gps.get_heading()
            if heading_difference > last_heading_difference:
                mode = "left"
                last_heading_difference = heading_difference
            if mode == "left":
                self.drive.left()
                self.drive.set_speed(513)
            else:
                self.drive.right()
                self.drive.set_speed(513)

    def resume(self):
        # self.correct_heading()
        self.taking_avoidance_action = False
        self.drive.set_direction(self.last_direction)
        self.drive.set_speed(self.last_speed)

    def calculate_directional_collision_probabilities(self):
        self.lidar_data = self.lidar.get_last_reading()
        if self.lidar_data is None or self.lidar_data.empty:
            return {
                "front": 0.98,
                "rear":  0.98,
                "right": 0.98,
                "left":  0.98
                }

        # Define the angle ranges for each direction
        directions = {
            "front": (315, 45),
            "right": (45, 135),
            "rear":  (135, 225),
            "left":  (225, 315)
            }

        collision_probabilities = {}

        for direction, (start_angle, end_angle) in directions.items():
            if start_angle > end_angle:
                direction_data = self.lidar_data[
                    (self.lidar_data["angle"] >= start_angle) | (self.lidar_data["angle"] <= end_angle)]
            else:
                direction_data = self.lidar_data[
                    (self.lidar_data["angle"] >= start_angle) & (self.lidar_data["angle"] <= end_angle)]


            if not direction_data.empty:
                min_distance = direction_data["distance"].min()
                min_distance = max(1.0, min_distance)
                collision_distance = 16.0  # Placeholder value for demonstration purposes
                collision_probability = min(1.0, max(1.0, collision_distance) / min_distance)
            else:
                collision_probability = 1

            collision_probabilities[direction] = collision_probability
        print(collision_probabilities)
        return collision_probabilities

    def calculate_collision_probability(self):
        self.lidar_data = self.lidar.get_last_reading()
        if self.lidar_data is not None and not self.lidar_data.empty:
            # Drop the lowest 5 points
            lidar_distance = self.lidar_data["distance"].min()  # Assuming lidar distance is in meters
            collision_distance = 16.0  # Placeholder value for demonstration purposes
            self.collision_probability = collision_distance / max(1.0, lidar_distance)
            self.collision_probability = min(1.0, self.collision_probability)  # Ensure probability is within [0, 1]
        return self.collision_probability

    def run(self):
        while self.running:
            try:
                self.lidar_data = self.lidar.get_last_reading()
                self.lidar_data = self.lidar_data.drop(self.lidar_data.nsmallest(40, 'distance').index)
                if self.lidar_data is not None:
                    self.calculate_collision_probability()
                    self.on_update_callback({
                        "lidar":                 self.lidar_data,
                        "collision_probability": self.collision_probability
                        })
                if self.collision_probability > 0.95:
                    self.avoid_collision()
            except Exception as e:
                print(f"Error in collision avoidance system: {e}")
                continue
            finally:
                time.sleep(.25)
