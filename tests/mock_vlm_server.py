#!/usr/bin/env python3
"""
Mock VLM Server for Testing

A simple WebSocket server that simulates VLM behavior for testing the evaluation system.
"""

import argparse
import json
import pickle
import random
import time
import numpy as np
from typing import Dict, Any

try:
    from websocket_server import WebsocketServer
except ImportError:
    print("Installing websocket-server...")
    import subprocess
    import sys
    subprocess.check_call([sys.executable, '-m', 'pip', 'install', 'websocket-server', '-q'])
    from websocket_server import WebsocketServer


class MockVLMServer:
    """Mock VLM WebSocket Server"""

    def __init__(self, host='0.0.0.0', port=8080, use_pickle=True, smart_mode=False):
        self.host = host
        self.port = port
        self.use_pickle = use_pickle
        self.smart_mode = smart_mode  # If True, uses simple logic to navigate
        self.step_count = 0
        self.position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.goal_position = None

    def start(self):
        """Start the WebSocket server"""
        print(f"Mock VLM Server starting on {self.host}:{self.port}")
        print(f"Mode: {'Pickle' if self.use_pickle else 'JSON'}")
        print(f"Smart Mode: {self.smart_mode}")

        server = WebsocketServer(host=self.host, port=self.port)
        server.set_fn_new_client(self._on_new_client)
        server.set_fn_message_received(self._on_message)
        server.run_forever()

    def _on_new_client(self, client, server):
        """Called when new client connects"""
        print(f"New client connected: {client['id']}")
        self.step_count = 0
        self.position = {'x': 0.0, 'y': 0.0, 'z': 0.0}

    def _on_message(self, client, server, message):
        """Called when message received from client"""
        try:
            # Parse observation
            if self.use_pickle:
                obs = pickle.loads(message)
            else:
                obs = json.loads(message)

            rgb = obs.get('rgb')
            depth = obs.get('depth')
            instruction = obs.get('instruction', '')

            # Get action
            action = self._get_action(obs)

            # Log
            self.step_count += 1
            print(f"[Step {self.step_count}] Instruction: {instruction[:50] if instruction else ''}...")
            rgb_shape = np.array(rgb).shape if rgb is not None and hasattr(rgb, 'shape') else type(rgb).__name__
            depth_shape = np.array(depth).shape if depth is not None and hasattr(depth, 'shape') else type(depth).__name__
            print(f"  RGB shape: {rgb_shape}")
            print(f"  Depth shape: {depth_shape}")
            print(f"  Action: {self._action_name(action)}")

            # Send response
            if self.use_pickle:
                response = pickle.dumps(action)
            else:
                response = json.dumps(action)

            server.send_message(client, response)

        except Exception as e:
            print(f"Error processing message: {e}")
            import traceback
            traceback.print_exc()

    def _get_action(self, obs: Dict[str, Any]) -> int:
        """Get action based on mode"""
        if self.smart_mode:
            return self._smart_action(obs)
        else:
            return self._random_action()

    def _random_action(self) -> int:
        """Random action with weighted distribution"""
        # Weighted: FORWARD (65%), LEFT (15%), RIGHT (15%), STOP (5%)
        values = [0, 1, 2, 3]
        weights = [5, 65, 15, 15]
        return random.choices(values, weights=weights, k=1)[0]

    def _smart_action(self, obs: Dict[str, Any]) -> int:
        """Simple navigation logic"""
        instruction = obs.get('instruction', '').lower()

        # Extract goal position from instruction (very basic parsing)
        goal_distance = 3.0  # default
        if 'meter' in instruction:
            try:
                goal_distance = float(instruction.split('meter')[0].split()[-1])
            except:
                pass

        # Simple logic: move forward until close to goal, then stop
        distance_traveled = abs(self.position['x']) + abs(self.position['z'])

        if distance_traveled >= goal_distance * 0.8:
            # Close to goal, 50% chance to stop
            if random.random() < 0.5:
                return 0  # STOP

        # Mostly move forward, occasionally turn
        if self.step_count % 10 == 0:
            # Every 10 steps, randomly turn
            return random.choice([2, 3])  # LEFT or RIGHT
        else:
            return 1  # FORWARD

    @staticmethod
    def _action_name(action: int) -> str:
        """Get action name"""
        names = {0: 'STOP', 1: 'FORWARD', 2: 'LEFT', 3: 'RIGHT'}
        return names.get(action, 'UNKNOWN')


def main():
    parser = argparse.ArgumentParser(description='Mock VLM Server for Testing')
    parser.add_argument('--host', type=str, default='0.0.0.0', help='Host to bind to')
    parser.add_argument('--port', type=int, default=8080, help='Port to bind to')
    parser.add_argument('--json', action='store_true', help='Use JSON instead of pickle')
    parser.add_argument('--smart', action='store_true', help='Enable smart navigation mode')
    args = parser.parse_args()

    server = MockVLMServer(
        host=args.host,
        port=args.port,
        use_pickle=not args.json,
        smart_mode=args.smart
    )
    server.start()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\nMock VLM Server stopped.")
