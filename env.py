import abc
import random

class Env(abc.ABC):
    def __init__(self):
        pass

    @abc.abstractmethod
    def reset(self, task: str):
        pass

    @abc.abstractmethod
    def step(self, action):
        pass

class MockEnv(Env):
    def reset(self, task: str):
        return [random.random() for _ in range(4)]
    
    def step(self, action):
        obs = [random.random() for _ in range(4)]
        reward = random.random()
        done = reward > 0.8
        return obs, reward, done

