#!/usr/bin/env python3
"""
VLN Evaluator - Main Evaluation Module

Orchestrates VLN evaluation using Env and Policy.
"""

import json
import traceback
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Dict, Any, Optional, TYPE_CHECKING, Callable

import numpy as np

from vln_evaluator.env import Env, Episode
from vln_evaluator.dataset_loader import R2RDatasetLoader, R2REpisode

if TYPE_CHECKING:
    from vln_evaluator.policy import Policy


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
    - Env: Environment (contains Simulator internally)
    - Policy: Action selection (local or remote VLM)
    - Dataset: Episodes to evaluate
    """

    def __init__(self,
                 env: Env,
                 policy: 'Policy',
                 default_max_steps: int = 50,
                 default_success_threshold: float = 0.2):
        """
        Initialize VLN Evaluator

        Args:
            env: Environment instance (e.g., O3DEEnv, MockEnv)
            policy: Policy instance (e.g., WebSocketPolicy, MockPolicy)
            default_max_steps: Default max steps per episode
            default_success_threshold: Default success distance threshold (meters)
        """
        self.env = env
        self.policy = policy
        self.default_max_steps = default_max_steps
        self.default_success_threshold = default_success_threshold

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
        # Convert R2REpisode to Episode for env
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

        # Reset environment
        try:
            obs = self.env.reset(episode)
        except Exception as e:
            print(f'  Error resetting environment: {e}')
            traceback.print_exc()
            return EpisodeResult(
                episode_id=r2r_ep.episode_id,
                scene_id=r2r_ep.scene_id,
                instruction=r2r_ep.instruction,
                success=False,
                failure_reason='reset_error',
                steps=0
            )

        # Add instruction to observation for VLM policies
        obs['instruction'] = r2r_ep.instruction

        # Evaluation loop
        success = False
        failure_reason = None
        collision_count = 0
        trajectory = []
        final_distance = float('inf')

        for step in range(max_steps):
            try:
                # Get action from policy
                action = self.policy.act(obs)

                # Execute action through env
                obs, reward, done, info = self.env.step(action)

                # Re-add instruction to observation for next step
                obs['instruction'] = r2r_ep.instruction

                # Extract info from step return
                collision_count = info.get('collision_count', 0)
                trajectory = info.get('trajectory', [])
                final_distance = info.get('distance_to_goal', float('inf'))

                # Check if done
                if done:
                    success = (final_distance < success_threshold)
                    if not success:
                        failure_reason = 'timeout'
                    break

            except Exception as e:
                print(f'  Error during evaluation: {e}')
                failure_reason = f'execution_error: {str(e)}'
                break

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
        self.env.close()

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
                env: Env,
                policy: 'Policy',
                episode_ids: Optional[List[str]] = None):
    """
    Convenience function to run VLN evaluation

    Args:
        dataset_path: Path to R2R dataset JSON
        output_path: Path to save results JSON
        env: Environment instance (e.g., O3DEEnv, MockEnv)
        policy: Policy instance (e.g., WebSocketPolicy, MockPolicy)
        episode_ids: Optional list of episode IDs to evaluate

    Returns:
        EvaluationResult
    """
    with VLNEvaluator(env=env, policy=policy) as evaluator:
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
    print('  from vln_evaluator.env import MockEnv, O3DEEnv')
    print('  from vln_evaluator.policy import MockPolicy, WebSocketPolicy')
    print('')
    print('  # Option 1: Use convenience function')
    print('  env = MockEnv()')
    print('  policy = MockPolicy()')
    print('  evaluate_vln(')
    print('      dataset_path="datasets/sample_r2r.json",')
    print('      output_path="results/eval_results.json",')
    print('      env=env,')
    print('      policy=policy')
    print('  )')
    print('')
    print('  # Option 2: Use evaluator class')
    print('  env = O3DEEnv(simulator_config={"mode": "socket"})')
    print('  policy = WebSocketPolicy("localhost", "8080")')
    print('  with VLNEvaluator(env=env, policy=policy) as evaluator:')
    print('      results = evaluator.evaluate_dataset("datasets/sample_r2r.json")')
    print('      evaluator.save_results(results, "results/eval_results.json")')
