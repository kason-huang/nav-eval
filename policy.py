import abc
import random
import websocket
import pickle

class Policy(abc.ABC):
    @abc.abstractmethod
    def act(self, obs):
        pass

class WebSocketPolicy(Policy):
    def __init__(self, ip:str, port: str):
        self.url = f"ws://{ip:port}"
        self.ws = websocket.create_connection(self.url)
    
    def act(self, obs):
        self.ws.send(pickle.dumps(obs))
        action = pickle.loads(self.ws.recv())
        return action
    
class MockPolicy(Policy):
    def act(self, obs):
        """
        self.actions2idx = OrderedDict({
            'STOP': [0],
            "↑": [1],
            "←": [2],
            "→": [3]
        })
        """
        values = [0, 1, 2, 3]
        weights = [5, 65, 15, 15]  # 对应每个值的权重（百分比或任意比例）
        return random.choices(values, weights=weights, k=1)[0]