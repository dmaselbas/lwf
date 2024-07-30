from stable_baselines3 import DQN
from simulation.env import RobotEnv

env = RobotEnv()
model = DQN.load("dqn_robot")

obs = env.reset()
for i in range(1000):
    action, _states = model.predict(obs)
    obs, reward, done, info = env.step(action)
    if done:
        obs = env.reset()
env.close()
