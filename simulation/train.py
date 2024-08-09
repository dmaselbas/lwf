from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from simulation.env import RobotEnv

env = RobotEnv()
# check_env(env)  # Optional, to check if the environment follows the Gym API
env.compute_reward = env.calculate_reward
model = DQN('MlpPolicy', env, verbose=1)
#model = DQN.load("dqn_robot", env=env)
for i in range(100):
    env.reset()
    model.learn(total_timesteps= i * 500)
    model.save("ppo_robot")
