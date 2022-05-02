import numpy as np
import random
from numpy.random import choice

ids = ["111111111", "222222222"]


ALPHA = 0.1
GAMMA = 0.9
INITIAL_TEMPERATURE = 100
TEMPERATURE_STEP = 0.2

class DroneAgent:
    def __init__(self, n, m):
        self.mode = 'train'  # do not change this!
        self.T = INITIAL_TEMPERATURE
        self.policy = {}
        self.action_space = ['reset', 'wait', 'pick', 'move_up', 'move_down', 'move_left', 'move_right', 'deliver']

    # so we can put it a dictionary key - must be hashable
    def get_observation_as_hashable(self, observation):
        return observation["drone_location"], tuple(observation["packages"]), observation["target_location"]

    def get_q(self, observation, action):
        return self.policy.get((observation, action), 0.0)

    def select_action(self, obs0):
        hashable_obs0 = self.get_observation_as_hashable(obs0)
        b_constants = np.exp(np.array([self.get_q(hashable_obs0, action) for action in self.action_space]) / self.T)
        probabilities = b_constants / sum(b_constants)
        action = choice(self.action_space, 1, p=probabilities)[0]
        if self.T >= 1 + TEMPERATURE_STEP:
            self.T -= TEMPERATURE_STEP
        return action

    def train(self):
        self.mode = 'train'  # do not change this!

    def eval(self):
        self.mode = 'eval'  # do not change this!

    def update(self, obs0, action, obs1, reward):
        obs0_hashable = self.get_observation_as_hashable(obs0)
        obs1_hashable = self.get_observation_as_hashable(obs1)
        q_max = max([self.get_q(obs1_hashable, a) for a in self.action_space])
        old_q = self.policy.get((obs0_hashable, action), None)
        if old_q is None:
            self.policy[(obs0_hashable, action)] = reward
        else:
            self.policy[(obs0_hashable, action)] = old_q + ALPHA * (reward + GAMMA * q_max - old_q)

