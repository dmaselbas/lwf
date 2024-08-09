from stable_baselines3 import PPO
from simulation.env import RobotEnv
import numpy as np

env = RobotEnv()
model = PPO.load("ppo_robot", env=env, device="auto")

obs, inf = env.reset()
while True:# Adjusted loop count
    action, _states = model.predict(obs)
    print(f"Action: {action}")
    obs, reward, done, truncated, info = env.step(action)  # Adjusted unpacking
    if done:
        env.close()
        break
