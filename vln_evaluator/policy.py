import abc
import random
import websockets.sync.client
import pickle
from typing import Dict, Any

class Policy(abc.ABC):
    @abc.abstractmethod
    def act(self, obs):
        """
        Get action from observation.

        Args:
            obs: Observation dict containing {'rgb', 'depth', 'instruction'}

        Returns:
            action: Action ID (0=STOP, 1=FORWARD, 2=LEFT, 3=RIGHT)
        """
        pass

class WebSocketPolicy(Policy):
    def __init__(self, ip: str, port: str):
        self.url = f"ws://{ip}:{port}"
        self.ws = websockets.sync.client.connect(self.url)

    def act(self, obs: Dict[str, Any]) -> int:
        """
        Send observation to VLM server and get action.

        Args:
            obs: Observation dict containing {'rgb', 'depth', 'instruction'}

        Returns:
            action: Action ID (0=STOP, 1=FORWARD, 2=LEFT, 3=RIGHT)
        """
        self.ws.send(pickle.dumps(obs))
        action = pickle.loads(self.ws.recv())
        return action

class MockPolicy(Policy):
    def act(self, obs: Dict[str, Any]) -> int:
        """
        Random action for testing.

        Args:
            obs: Observation dict (not used for random policy)

        Returns:
            action: Random action ID (0=STOP, 1=FORWARD, 2=LEFT, 3=RIGHT)
        """
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