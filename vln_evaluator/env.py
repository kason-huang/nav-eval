"""
VLN Environment Interface

This module defines the Env interface following Habitat-lab conventions:
- Simulator: Low-level simulation interface (O3DE physics, sensors)
- Env: High-level environment wrapper (task logic, rewards, termination)
- Episode: Single task configuration (start, goal, instruction)
"""

import abc
from typing import Dict, Any, Tuple, Optional
from dataclasses import dataclass

import numpy as np


@dataclass
class Episode:
    """Episode data structure for VLN evaluation"""
    episode_id: str
    scene_id: str
    instruction: str
    start_position: Dict[str, float]
    start_rotation: Dict[str, float]
    goal_position: Dict[str, float]

    # Optional fields
    scene_config_path: Optional[str] = None
    max_steps: Optional[int] = None
    success_threshold: Optional[float] = None


class Env(abc.ABC):
    """
    Abstract Environment interface following Habitat-lab conventions.

    The Env wraps a Simulator and adds task-specific logic:
    - Episode management
    - Reward computation (if applicable)
    - Termination conditions
    - Info dict with metrics
    """

    def __init__(self):
        self.current_episode: Optional[Episode] = None

    @abc.abstractmethod
    def reset(self, episode: Episode) -> Dict[str, np.ndarray]:
        """
        Reset environment with a new episode.

        Args:
            episode: Episode object containing task configuration

        Returns:
            Initial observation {'rgb': ndarray, 'depth': ndarray}
        """
        self.current_episode = episode
        pass

    @abc.abstractmethod
    def step(self, action: int) -> Tuple[Dict[str, np.ndarray], float, bool, Dict[str, Any]]:
        """
        Execute one step in the environment.

        Args:
            action: Action ID to execute

        Returns:
            obs: Observation dict {'rgb': ndarray, 'depth': ndarray}
            reward: Reward value (0.0 for VLN)
            done: Whether episode has terminated
            info: Info dict with metrics (position, distance_to_goal, etc.)
        """
        pass

    @abc.abstractmethod
    def close(self):
        """Clean up environment resources"""
        pass

    def get_current_episode(self) -> Optional[Episode]:
        """Get the current episode"""
        return self.current_episode


class MockEnv(Env):
    """Mock environment for testing without actual simulator"""

    def __init__(self):
        super().__init__()
        self.position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.step_count = 0

    def reset(self, episode: Episode) -> Dict[str, np.ndarray]:
        """Reset with episode configuration"""
        self.current_episode = episode
        self.position = episode.start_position.copy()
        self.step_count = 0

        # Return mock observation
        return {
            'rgb': np.random.randint(0, 255, (64, 64, 3), dtype=np.uint8),
            'depth': np.random.randint(0, 10000, (64, 64), dtype=np.uint16)
        }

    def step(self, action: int) -> Tuple[Dict[str, np.ndarray], float, bool, Dict[str, Any]]:
        """Execute step"""
        self.step_count += 1

        # Mock observation
        obs = {
            'rgb': np.random.randint(0, 255, (64, 64, 3), dtype=np.uint8),
            'depth': np.random.randint(0, 10000, (64, 64), dtype=np.uint16)
        }

        # Mock reward and done
        reward = 0.0
        done = self.step_count >= 20

        # Mock info with all required keys
        info = {
            'position': self.position.copy(),
            'step': self.step_count,
            'distance_to_goal': max(0.0, 2.0 - self.step_count * 0.1),  # Mock distance decreasing
            'collision_count': 0,
            'trajectory': [self.position.copy()]
        }

        return obs, reward, done, info

    def close(self):
        """Cleanup"""
        pass

