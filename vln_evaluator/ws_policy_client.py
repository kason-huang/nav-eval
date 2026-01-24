#!/usr/bin/env python3
"""
WebSocket Policy Client for VLN Evaluation

Connects to remote VLM server via WebSocket to get navigation actions.
"""

import pickle
import json
from typing import Dict, Any, Optional
import numpy as np

try:
    import websocket
except ImportError:
    websocket = None


class WebSocketPolicyClient:
    """
    WebSocket client for remote VLM policy

    Sends RGB, depth, and instruction to remote server, receives action.
    """

    def __init__(self, url: str, use_pickle: bool = True):
        """
        Initialize WebSocket client

        Args:
            url: WebSocket server URL (e.g., 'ws://localhost:8080')
            use_pickle: If True, use pickle for serialization; otherwise use JSON
        """
        if websocket is None:
            raise ImportError('websocket-client is required. Install with: pip install websocket-client')

        self.url = url
        self.use_pickle = use_pickle
        self.ws = None
        self.connected = False

    def connect(self, url: Optional[str] = None):
        """
        Connect to WebSocket server

        Args:
            url: Optional URL override
        """
        if url:
            self.url = url

        try:
            self.ws = websocket.create_connection(self.url, timeout=10)
            self.connected = True
            print(f'Connected to VLM server: {self.url}')
        except Exception as e:
            raise ConnectionError(f'Failed to connect to {self.url}: {e}')

    def disconnect(self):
        """Disconnect from server"""
        if self.ws:
            self.ws.close()
            self.connected = False
            print('Disconnected from VLM server')

    def act(self, rgb: np.ndarray, depth: np.ndarray, instruction: str) -> int:
        """
        Get action from remote VLM

        Args:
            rgb: RGB image (H, W, 3)
            depth: Depth image (H, W)
            instruction: Navigation instruction text

        Returns:
            Action ID (0=STOP, 1=FORWARD, 2=LEFT, 3=RIGHT)
        """
        if not self.connected:
            raise RuntimeError('Not connected to VLM server. Call connect() first.')

        # Prepare observation
        obs = {
            'rgb': rgb,
            'depth': depth,
            'instruction': instruction
        }

        # Send and receive
        try:
            if self.use_pickle:
                # Use pickle for binary data (faster for images)
                self.ws.send(pickle.dumps(obs))
                response = pickle.loads(self.ws.recv())
            else:
                # Use JSON (slower but more compatible)
                # Convert numpy arrays to lists for JSON serialization
                obs_json = {
                    'rgb': rgb.tolist(),
                    'depth': depth.tolist(),
                    'instruction': instruction
                }
                self.ws.send(json.dumps(obs_json))
                response = json.loads(self.ws.recv())

            # Parse action
            action = self._parse_action(response)

            return action

        except Exception as e:
            print(f'Error communicating with VLM server: {e}')
            # Return STOP action on error
            return 0

    def _parse_action(self, response: Any) -> int:
        """
        Parse action from server response

        Args:
            response: Response from server (int, dict, or string)

        Returns:
            Action ID
        """
        if isinstance(response, int):
            return response
        elif isinstance(response, dict):
            return response.get('action', response.get('action_id', 0))
        elif isinstance(response, str):
            # Map action names to IDs
            action_map = {
                'STOP': 0,
                'stop': 0,
                'FORWARD': 1,
                'forward': 1,
                'LEFT': 2,
                'left': 2,
                'RIGHT': 3,
                'right': 3
            }
            return action_map.get(response, 0)
        else:
            return 0

    def __enter__(self):
        """Context manager entry"""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.disconnect()


# For compatibility with existing policy.py
class WebSocketPolicy(WebSocketPolicyClient):
    """Alias for WebSocketPolicyClient"""

    def __init__(self, ip: str = '127.0.0.1', port: int = 8080, **kwargs):
        url = f'ws://{ip}:{port}'
        super().__init__(url, **kwargs)


if __name__ == '__main__':
    # Test WebSocket client
    print('WebSocket Policy Client Module')
    print('')
    print('Usage:')
    print('  from vln_evaluator import WebSocketPolicyClient')
    print('  client = WebSocketPolicyClient("ws://localhost:8080")')
    print('  client.connect()')
    print('  action = client.act(rgb, depth, "Walk forward")')
    print('  client.disconnect()')
