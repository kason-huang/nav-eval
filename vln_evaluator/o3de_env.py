"""
O3DE Environment Implementation

This module provides the O3DE-based implementation of the Env interface.
"""

from typing import Dict, Any, Tuple, Optional
import numpy as np

from .env import Env, Episode


class O3DEEnv(Env):
    """
    O3DE-based VLN Environment.

    This Env wraps the O3DESimulator and provides the standard RL interface.
    """

    def __init__(self, simulator_config: Optional[Dict[str, Any]] = None):
        """
        Initialize O3DE Environment.

        Args:
            simulator_config: Config dict for O3DESimulator
                - mode: 'peb' or 'socket'
                - success_threshold: float
                - collision_threshold: float
                - linear_speed: float
                - angular_speed: float
        """
        super().__init__()
        self.simulator_config = simulator_config or {}
        self.simulator = None

    def _get_simulator(self):
        """Lazy load simulator"""
        if self.simulator is None:
            from .o3de_simulator import O3DESimulator
            self.simulator = O3DESimulator(**self.simulator_config)
        return self.simulator

    def reset(self, episode: Episode) -> Dict[str, np.ndarray]:
        """
        Reset environment with episode.

        Args:
            episode: Episode configuration

        Returns:
            Initial observation
        """
        self.current_episode = episode

        # Convert Episode to simulator's Episode format
        sim_episode = Episode(
            episode_id=episode.episode_id,
            scene_id=episode.scene_id,
            start_position=episode.start_position,
            start_rotation=episode.start_rotation,
            goal_position=episode.goal_position,
            instruction=episode.instruction,
            scene_config_path=episode.scene_config_path,
            max_steps=episode.max_steps,
            success_threshold=episode.success_threshold
        )

        return self._get_simulator().reset(sim_episode)

    def step(self, action: int) -> Tuple[Dict[str, np.ndarray], float, bool, Dict[str, Any]]:
        """
        Execute one step.

        Args:
            action: Action ID (0=STOP, 1=FORWARD, 2=LEFT, 3=RIGHT)

        Returns:
            obs, reward, done, info
        """
        return self._get_simulator().step(action)

    def close(self):
        """Cleanup simulator"""
        if self.simulator is not None:
            self.simulator.close()
            self.simulator = None
