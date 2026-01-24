#!/usr/bin/env python3
"""
R2R Dataset Loader for VLN Evaluation

Loads episodes from R2R-format JSON files.
"""

import json
from dataclasses import dataclass
from pathlib import Path
from typing import List, Dict, Any, Optional


@dataclass
class R2REpisode:
    """Episode data from R2R dataset"""
    episode_id: str
    scene_id: str
    instruction: str
    start_position: Dict[str, float]
    start_rotation: Dict[str, float]
    goal_position: Dict[str, float]

    # Optional metadata
    scene_config_path: Optional[str] = None
    max_steps: Optional[int] = None
    success_threshold: Optional[float] = None


class R2RDatasetLoader:
    """Loader for R2R-format dataset files"""

    @staticmethod
    def load_from_file(dataset_path: str) -> List[R2REpisode]:
        """
        Load episodes from R2R format JSON file

        Args:
            dataset_path: Path to dataset JSON file

        Returns:
            List of R2REpisode objects
        """
        with open(dataset_path, 'r', encoding='utf-8') as f:
            data = json.load(f)

        # Handle both wrapped and unwrapped formats
        episodes_data = data.get('episodes', data) if isinstance(data, dict) else data

        episodes = []
        for ep_data in episodes_data:
            episode = R2RDatasetLoader._parse_episode(ep_data)
            episodes.append(episode)

        return episodes

    @staticmethod
    def _parse_episode(ep_data: Dict[str, Any]) -> R2REpisode:
        """Parse episode data from dict"""
        # Required fields
        episode_id = ep_data.get('episode_id', ep_data.get('id', ''))
        scene_id = ep_data.get('scene_id', '')
        instruction = ep_data.get('instruction', '')

        # Position and rotation
        start_position = ep_data.get('start_position', {})
        if not start_position:
            # Try alternate field names
            start_position = ep_data.get('start_pos', {})

        start_rotation = ep_data.get('start_rotation', {})
        if not start_rotation:
            start_rotation = ep_data.get('start_rot', {})

        goal_position = ep_data.get('goal_position', {})
        if not goal_position:
            goal_position = ep_data.get('goal', {})

        # Optional fields
        scene_config_path = ep_data.get('scene_config_path')
        max_steps = ep_data.get('max_steps')
        success_threshold = ep_data.get('success_threshold')

        return R2REpisode(
            episode_id=episode_id,
            scene_id=scene_id,
            instruction=instruction,
            start_position=start_position,
            start_rotation=start_rotation,
            goal_position=goal_position,
            scene_config_path=scene_config_path,
            max_steps=max_steps,
            success_threshold=success_threshold
        )

    @staticmethod
    def save_to_file(episodes: List[R2REpisode], output_path: str):
        """
        Save episodes to R2R format JSON file

        Args:
            episodes: List of R2REpisode objects
            output_path: Output file path
        """
        episodes_data = []
        for ep in episodes:
            ep_dict = {
                'episode_id': ep.episode_id,
                'scene_id': ep.scene_id,
                'instruction': ep.instruction,
                'start_position': ep.start_position,
                'start_rotation': ep.start_rotation,
                'goal_position': ep.goal_position
            }
            if ep.scene_config_path:
                ep_dict['scene_config_path'] = ep.scene_config_path
            if ep.max_steps:
                ep_dict['max_steps'] = ep.max_steps
            if ep.success_threshold:
                ep_dict['success_threshold'] = ep.success_threshold

            episodes_data.append(ep_dict)

        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump({'episodes': episodes_data}, f, indent=2, ensure_ascii=False)


def create_sample_dataset(output_path: str = 'datasets/sample_r2r.json'):
    """Create a sample R2R dataset for testing"""
    episodes = [
        R2REpisode(
            episode_id='ep_001',
            scene_id='scene_001',
            instruction='Walk straight forward for 3 meters',
            start_position={'x': 0.0, 'y': 0.0, 'z': 0.0},
            start_rotation={'x': 0, 'y': 0, 'z': 0},
            goal_position={'x': 3.0, 'y': 0.0, 'z': 0.0},
            max_steps=50
        ),
        R2REpisode(
            episode_id='ep_002',
            scene_id='scene_001',
            instruction='Walk forward 2 meters, then turn left and walk 1 meter',
            start_position={'x': 0.0, 'y': 0.0, 'z': 0.0},
            start_rotation={'x': 0, 'y': 0, 'z': 0},
            goal_position={'x': 2.0, 'y': 0.0, 'z': 1.0},
            max_steps=50
        ),
    ]

    R2RDatasetLoader.save_to_file(episodes, output_path)
    print(f'Sample dataset created: {output_path}')
    return episodes


if __name__ == '__main__':
    # Create sample dataset
    create_sample_dataset()
