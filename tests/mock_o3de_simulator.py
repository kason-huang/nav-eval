#!/usr/bin/env python3
"""
Mock O3DE Simulator for Testing

Simulates O3DE behavior without requiring actual O3DE installation.
"""

import math
import random
import time
from typing import Dict, List, Tuple, Optional
import numpy as np


class MockO3DESimulator:
    """
    Mock O3DE Simulator for testing

    Simulates robot movement in a 2D plane without actual O3DE engine.
    """

    def __init__(self, success_threshold: float = 0.2):
        self.success_threshold = success_threshold

        # Robot state
        self.position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.rotation = 0.0  # yaw angle in radians
        self.goal_position = None

        # Episode state
        self.current_step = 0
        self.max_steps = 50
        self.trajectory = []
        self.collision_count = 0

        # Mock sensors (64x64 RGB-D)
        self.rgb_shape = (64, 64, 3)
        self.depth_shape = (64, 64)

    def reset(self, episode) -> Dict[str, np.ndarray]:
        """
        Reset simulator with episode data

        Args:
            episode: Episode object with start_position, goal_position, etc.

        Returns:
            Initial observation {'rgb': ndarray, 'depth': ndarray}
        """
        self.position = episode.start_position.copy()
        self.rotation = 0.0
        self.goal_position = episode.goal_position.copy()
        self.current_step = 0
        self.max_steps = episode.max_steps or 50
        self.trajectory = [self.position.copy()]
        self.collision_count = 0

        print(f"Mock O3DE Simulator reset:")
        print(f"  Start: {self.position}")
        print(f"  Goal: {self.goal_position}")
        print(f"  Max steps: {self.max_steps}")

        # Generate mock sensor data
        return self._get_observation()

    def step(self, action: int) -> Tuple[Dict, float, bool, Dict]:
        """
        Execute one step

        Args:
            action: Action ID (0=STOP, 1=FORWARD, 2=LEFT, 3=RIGHT)

        Returns:
            obs, reward, done, info
        """
        self.current_step += 1

        # Execute action (simulate movement)
        if action == 0:  # STOP
            pass
        elif action == 1:  # FORWARD 25cm
            self._move_forward(0.25)
        elif action == 2:  # LEFT 15 degrees
            self._rotate(math.radians(15))
        elif action == 3:  # RIGHT 15 degrees
            self._rotate(-math.radians(15))

        # Record trajectory
        self.trajectory.append(self.position.copy())

        # Simulate collision (random)
        if random.random() < 0.05:  # 5% chance of collision
            self.collision_count += 1
            collision = True
        else:
            collision = False

        # Calculate distance to goal
        distance = self._distance_to_goal()

        # Check if done
        done = False
        if action == 0 and distance < self.success_threshold:
            done = True
            print(f"Step {self.current_step}: SUCCESS - Distance: {distance:.3f}m")
        elif self.current_step >= self.max_steps:
            done = True
            print(f"Step {self.current_step}: TIMEOUT - Distance: {distance:.3f}m")

        # Get observation
        obs = self._get_observation()

        # Build info
        info = {
            'position': self.position.copy(),
            'distance_to_goal': distance,
            'collision': collision,
            'collision_count': self.collision_count,
            'step': self.current_step
        }

        return obs, 0.0, done, info

    def get_distance_to_goal(self) -> float:
        """Calculate distance to goal"""
        return self._distance_to_goal()

    def get_trajectory(self) -> List[Dict]:
        """Get trajectory"""
        return self.trajectory.copy()

    def close(self):
        """Cleanup"""
        print("Mock O3DE Simulator closed")

    # Private methods

    def _get_observation(self) -> Dict[str, np.ndarray]:
        """Generate mock sensor data"""
        # Mock RGB - gradient based on position
        rgb = np.zeros(self.rgb_shape, dtype=np.uint8)
        for i in range(self.rgb_shape[0]):
            for j in range(self.rgb_shape[1]):
                # Create gradient based on position
                val = int((i + j + self.position['x'] * 10) % 256)
                rgb[i, j] = [val, val // 2, 255 - val]

        # Mock depth - random noise
        depth = np.random.randint(0, 10000, size=self.depth_shape, dtype=np.uint16)

        return {'rgb': rgb, 'depth': depth}

    def _move_forward(self, distance: float):
        """Move forward by distance (meters)"""
        self.position['x'] += distance * math.cos(self.rotation)
        self.position['z'] += distance * math.sin(self.rotation)
        print(f"Moved forward {distance}m -> pos: {self.position}")

    def _rotate(self, angle: float):
        """Rotate by angle (radians)"""
        self.rotation += angle
        self.rotation = math.atan2(math.sin(self.rotation), math.cos(self.rotation))
        print(f"Rotated {math.degrees(angle):.1f} deg -> rot: {math.degrees(self.rotation):.1f}")

    def _distance_to_goal(self) -> float:
        """Calculate Euclidean distance to goal"""
        if self.goal_position is None:
            return float('inf')

        dx = self.position['x'] - self.goal_position['x']
        dy = self.position['y'] - self.goal_position['y']
        dz = self.position['z'] - self.goal_position['z']

        return math.sqrt(dx*dx + dy*dy + dz*dz)


# Mock Episode class for testing
class MockEpisode:
    """Mock Episode class"""
    def __init__(self, episode_id='test_001', scene_id='test_scene',
                 start_position=None, start_rotation=None,
                 goal_position=None, instruction='Test instruction',
                 max_steps=50, scene_config_path=None):
        self.episode_id = episode_id
        self.scene_id = scene_id
        self.start_position = start_position or {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.start_rotation = start_rotation or {'x': 0, 'y': 0, 'z': 0}
        self.goal_position = goal_position or {'x': 3.0, 'y': 0.0, 'z': 0.0}
        self.instruction = instruction
        self.max_steps = max_steps
        self.scene_config_path = scene_config_path


if __name__ == '__main__':
    """Test the mock simulator"""
    print("Testing Mock O3DE Simulator")
    print("=" * 50)

    sim = MockO3DESimulator(success_threshold=0.2)

    # Create test episode
    episode = MockEpisode(
        episode_id='test_001',
        start_position={'x': 0, 'y': 0, 'z': 0},
        goal_position={'x': 1.5, 'y': 0, 'z': 0},
        instruction='Walk forward 1.5 meters',
        max_steps=20
    )

    # Reset
    obs = sim.reset(episode)
    print(f"\nInitial obs shapes:")
    print(f"  RGB: {obs['rgb'].shape}")
    print(f"  Depth: {obs['depth'].shape}")

    # Run a few steps
    print("\nRunning test steps...")
    actions = [1, 1, 1, 1, 1, 0]  # Forward x5 then STOP

    for action in actions:
        obs, reward, done, info = sim.step(action)
        print(f"  Step {info['step']}: action={action}, "
              f"pos={info['position']}, dist={info['distance_to_goal']:.3f}m")
        if done:
            break

    # Print trajectory
    print(f"\nTrajectory ({len(sim.get_trajectory())} points):")
    for i, pos in enumerate(sim.get_trajectory()):
        print(f"  {i}: {pos}")

    sim.close()
    print("\nMock O3DE Simulator test completed!")
