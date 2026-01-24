#!/usr/bin/env python3
"""
VLN Evaluator - Main Evaluation Module

Orchestrates VLN evaluation using O3DE simulator and remote VLM.
"""

import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Dict, Any, Optional, Callable
from datetime import datetime
import numpy as np

# Import simulator and related modules
import sys
sys.path.insert(0, str(Path(__file__).parent.parent))

from o3de_simulator import O3DESimulator, Episode
from vln_evaluator.dataset_loader import R2RDatasetLoader, R2REpisode
from vln_evaluator.ws_policy_client import WebSocketPolicyClient


@dataclass
class EpisodeResult:
    """Result of a single episode evaluation"""
    episode_id: str
    scene_id: str
    instruction: str
    success: bool
    failure_reason: Optional[str] = None
    final_distance_to_goal: float = 0.0
    steps: int = 0
    collision_count: int = 0
    trajectory: List[Dict[str, float]] = field(default_factory=list)


@dataclass
class EvaluationResult:
    """Overall evaluation result"""
    total_episodes: int = 0
    success_count: int = 0
    success_rate: float = 0.0
    avg_distance_error: float = 0.0
    avg_steps: float = 0.0
    avg_collision_count: float = 0.0
    timeout_count: int = 0
    episodes: List[EpisodeResult] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization"""
        return {
            'summary': {
                'total_episodes': self.total_episodes,
                'success_count': self.success_count,
                'success_rate': self.success_rate,
                'avg_distance_error': self.avg_distance_error,
                'avg_steps': self.avg_steps,
                'avg_collision_count': self.avg_collision_count,
                'timeout_count': self.timeout_count
            },
            'episodes': [
                {
                    'episode_id': ep.episode_id,
                    'scene_id': ep.scene_id,
                    'instruction': ep.instruction,
                    'success': ep.success,
                    'failure_reason': ep.failure_reason,
                    'final_distance_to_goal': ep.final_distance_to_goal,
                    'steps': ep.steps,
                    'collision_count': ep.collision_count,
                    'trajectory': ep.trajectory
                }
                for ep in self.episodes
            ]
        }


class VLNEvaluator:
    """
    VLN Evaluator - Main evaluation orchestration

    Coordinates between:
    - O3DE Simulator (environment)
    - Remote VLM (policy)
    - Dataset (episodes)
    """

    def __init__(self,
                 simulator_config: Optional[Dict[str, Any]] = None,
                 vlm_config: Optional[Dict[str, Any]] = None,
                 default_max_steps: int = 50,
                 default_success_threshold: float = 0.2):
        """
        Initialize VLN Evaluator

        Args:
            simulator_config: O3DE simulator configuration
            vlm_config: VLM WebSocket client configuration
            default_max_steps: Default max steps per episode
            default_success_threshold: Default success distance threshold (meters)
        """
        self.simulator_config = simulator_config or {}
        self.vlm_config = vlm_config or {}
        self.default_max_steps = default_max_steps
        self.default_success_threshold = default_success_threshold

        # Initialize simulator
        self.simulator = O3DESimulator(**self.simulator_config)

        # Initialize VLM client
        self.vlm_client = WebSocketPolicyClient(**self.vlm_config)

    def evaluate_dataset(self,
                        dataset_path: str,
                        episode_ids: Optional[List[str]] = None,
                        progress_callback: Optional[Callable[[int, int], None]] = None) -> EvaluationResult:
        """
        Evaluate on a dataset

        Args:
            dataset_path: Path to R2R dataset JSON file
            episode_ids: Optional list of episode IDs to evaluate (if None, evaluate all)
            progress_callback: Optional callback(current, total) for progress updates

        Returns:
            EvaluationResult with overall statistics
        """
        # Load dataset
        print(f'Loading dataset from: {dataset_path}')
        r2r_episodes = R2RDatasetLoader.load_from_file(dataset_path)
        print(f'Loaded {len(r2r_episodes)} episodes')

        # Filter episodes if episode_ids specified
        if episode_ids:
            episode_id_set = set(episode_ids)
            r2r_episodes = [ep for ep in r2r_episodes if ep.episode_id in episode_id_set]
            print(f'Filtered to {len(r2r_episodes)} episodes')

        # Connect to VLM
        print(f'Connecting to VLM server: {self.vlm_config.get("url", "ws://localhost:8080")}')
        self.vlm_client.connect()

        # Evaluate each episode
        results = EvaluationResult()

        for idx, r2r_ep in enumerate(r2r_episodes):
            if progress_callback:
                progress_callback(idx + 1, len(r2r_episodes))

            print(f'\n[{idx+1}/{len(r2r_episodes)}] Evaluating episode: {r2r_ep.episode_id}')
            print(f'  Instruction: {r2r_ep.instruction}')

            ep_result = self.evaluate_episode(r2r_ep)
            results.episodes.append(ep_result)

            # Print result
            if ep_result.success:
                print(f'  ✓ SUCCESS - Distance: {ep_result.final_distance_to_goal:.3f}m, Steps: {ep_result.steps}')
            else:
                print(f'  ✗ FAILED - Reason: {ep_result.failure_reason}, Distance: {ep_result.final_distance_to_goal:.3f}m')

        # Disconnect from VLM
        self.vlm_client.disconnect()

        # Compute statistics
        results.total_episodes = len(results.episodes)
        results.success_count = sum(1 for ep in results.episodes if ep.success)
        results.success_rate = results.success_count / results.total_episodes if results.total_episodes > 0 else 0.0
        results.avg_distance_error = np.mean([ep.final_distance_to_goal for ep in results.episodes])
        results.avg_steps = np.mean([ep.steps for ep in results.episodes])
        results.avg_collision_count = np.mean([ep.collision_count for ep in results.episodes])
        results.timeout_count = sum(1 for ep in results.episodes if ep.failure_reason == 'timeout')

        return results

    def evaluate_episode(self, r2r_ep: R2REpisode) -> EpisodeResult:
        """
        Evaluate a single episode

        Args:
            r2r_ep: R2REpisode to evaluate

        Returns:
            EpisodeResult
        """
        # Convert R2REpisode to Episode for simulator
        max_steps = r2r_ep.max_steps or self.default_max_steps
        success_threshold = r2r_ep.success_threshold or self.default_success_threshold

        episode = Episode(
            episode_id=r2r_ep.episode_id,
            scene_id=r2r_ep.scene_id,
            start_position=r2r_ep.start_position,
            start_rotation=r2r_ep.start_rotation,
            goal_position=r2r_ep.goal_position,
            instruction=r2r_ep.instruction,
            scene_config_path=r2r_ep.scene_config_path,
            max_steps=max_steps,
            success_threshold=success_threshold
        )

        # Reset simulator
        try:
            obs = self.simulator.reset(episode)
        except Exception as e:
            print(f'  Error resetting simulator: {e}')
            return EpisodeResult(
                episode_id=r2r_ep.episode_id,
                scene_id=r2r_ep.scene_id,
                instruction=r2r_ep.instruction,
                success=False,
                failure_reason='reset_error',
                steps=0
            )

        # Evaluation loop
        success = False
        failure_reason = None
        collision_count = 0

        for step in range(max_steps):
            try:
                # Get action from VLM
                rgb = obs['rgb']
                depth = obs['depth']
                action = self.vlm_client.act(rgb, depth, r2r_ep.instruction)

                # Execute action
                obs, reward, done, info = self.simulator.step(action)

                # Update info
                collision_count = info.get('collision_count', 0)

                # Check if done
                if done:
                    success = (info['distance_to_goal'] < success_threshold)
                    if not success:
                        failure_reason = 'timeout'
                    break

            except Exception as e:
                print(f'  Error during evaluation: {e}')
                failure_reason = f'execution_error: {str(e)}'
                break

        # Get final state
        final_distance = self.simulator.get_distance_to_goal()
        trajectory = self.simulator.get_trajectory()

        return EpisodeResult(
            episode_id=r2r_ep.episode_id,
            scene_id=r2r_ep.scene_id,
            instruction=r2r_ep.instruction,
            success=success,
            failure_reason=failure_reason,
            final_distance_to_goal=final_distance,
            steps=step + 1,
            collision_count=collision_count,
            trajectory=trajectory
        )

    def save_results(self, results: EvaluationResult, output_path: str):
        """
        Save evaluation results to JSON file

        Args:
            results: EvaluationResult to save
            output_path: Output file path
        """
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(results.to_dict(), f, indent=2, ensure_ascii=False)

        print(f'\nResults saved to: {output_path}')

    def close(self):
        """Close evaluator and cleanup"""
        self.simulator.close()

    def __enter__(self):
        """Context manager entry"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.close()


# =============================================================================
# Convenience Functions
# =============================================================================

def evaluate_vln(dataset_path: str,
                output_path: str,
                simulator_config: Optional[Dict[str, Any]] = None,
                vlm_config: Optional[Dict[str, Any]] = None,
                episode_ids: Optional[List[str]] = None):
    """
    Convenience function to run VLN evaluation

    Args:
        dataset_path: Path to R2R dataset JSON
        output_path: Path to save results JSON
        simulator_config: O3DE simulator config
        vlm_config: VLM WebSocket client config
        episode_ids: Optional list of episode IDs to evaluate

    Returns:
        EvaluationResult
    """
    vlm_config = vlm_config or {}
    simulator_config = simulator_config or {}

    with VLNEvaluator(simulator_config=simulator_config, vlm_config=vlm_config) as evaluator:
        # Progress callback
        def progress_callback(current, total):
            print(f'\n=== Progress: {current}/{total} episodes ===')

        # Run evaluation
        results = evaluator.evaluate_dataset(
            dataset_path=dataset_path,
            episode_ids=episode_ids,
            progress_callback=progress_callback
        )

        # Save results
        evaluator.save_results(results, output_path)

        # Print summary
        print('\n' + '='*50)
        print('EVALUATION SUMMARY')
        print('='*50)
        print(f'Total Episodes:  {results.total_episodes}')
        print(f'Success Count:   {results.success_count}')
        print(f'Success Rate:    {results.success_rate:.2%}')
        print(f'Avg Distance:    {results.avg_distance_error:.3f}m')
        print(f'Avg Steps:       {results.avg_steps:.1f}')
        print(f'Timeout Count:   {results.timeout_count}')
        print('='*50)

        return results


if __name__ == '__main__':
    print('VLN Evaluator Module')
    print('')
    print('Usage:')
    print('  from vln_evaluator import VLNEvaluator, evaluate_vln')
    print('')
    print('  # Option 1: Use convenience function')
    print('  evaluate_vln(')
    print('      dataset_path="datasets/sample_r2r.json",')
    print('      output_path="results/eval_results.json",')
    print('      vlm_config={"url": "ws://localhost:8080"}')
    print('  )')
    print('')
    print('  # Option 2: Use evaluator class')
    print('  with VLNEvaluator() as evaluator:')
    print('      results = evaluator.evaluate_dataset("datasets/sample_r2r.json")')
    print('      evaluator.save_results(results, "results/eval_results.json")')
