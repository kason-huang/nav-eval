#!/usr/bin/env python3
"""
VLN Evaluator Demo

This demo shows how to use the VLN evaluation system with the following components:
- Episode: Single task configuration
- Simulator: Low-level simulation (O3DE or Mock)
- Env: High-level environment wrapper
- Policy: Action selection (local or remote)

Following Habitat-lab conventions:
- Episode contains task config (start, goal, instruction)
- Simulator manages physics and sensors
- Env wraps Simulator and adds task logic
- Policy selects actions based on observations
"""

import sys
from typing import List, Dict

from vln_evaluator.env import Env, Episode, MockEnv, O3DEEnv
from vln_evaluator.policy import Policy, MockPolicy


def create_test_episodes() -> List[Episode]:
    """Create test episodes for demo"""
    return [
        Episode(
            episode_id='demo_001',
            scene_id='scene_001',
            instruction='Walk forward 2 meters',
            start_position={'x': 0.0, 'y': 0.0, 'z': 0.0},
            start_rotation={'x': 0, 'y': 0, 'z': 0},
            goal_position={'x': 2.0, 'y': 0.0, 'z': 0.0},
            max_steps=20,
            success_threshold=0.2
        ),
        Episode(
            episode_id='demo_002',
            scene_id='scene_001',
            instruction='Walk forward and turn left',
            start_position={'x': 0.0, 'y': 0.0, 'z': 0.0},
            start_rotation={'x': 0, 'y': 0, 'z': 0},
            goal_position={'x': 1.0, 'y': 0.0, 'z': 1.0},
            max_steps=20,
            success_threshold=0.3
        )
    ]


def run_episode(env: Env, episode: Episode, policy: Policy) -> Dict:
    """
    Run a single episode with given env and policy.

    Args:
        env: Environment instance
        episode: Episode configuration
        policy: Policy for action selection

    Returns:
        Episode result dict
    """
    print(f"\n{'='*70}")
    print(f"Running Episode: {episode.episode_id}")
    print(f"{'='*70}")
    print(f"Instruction: {episode.instruction}")
    print(f"Start: {episode.start_position}")
    print(f"Goal: {episode.goal_position}")
    print(f"Max steps: {episode.max_steps}")

    # Reset environment with episode
    obs = env.reset(episode)
    print(f"\nObservation shape: RGB={obs['rgb'].shape}, Depth={obs['depth'].shape}")

    # Episode loop
    success = False
    total_reward = 0.0

    for step in range(episode.max_steps):
        # Get action from policy
        action = policy.act(obs)
        action_names = ['STOP', 'FORWARD', 'LEFT', 'RIGHT']
        print(f"Step {step+1}: Action = {action} ({action_names[action]})")

        # Execute action
        obs, reward, done, info = env.step(action)
        total_reward += reward

        # Print info
        if 'position' in info:
            pos = info['position']
            print(f"  Position: x={pos['x']:.2f}, y={pos['y']:.2f}, z={pos['z']:.2f}")
        if 'distance_to_goal' in info:
            print(f"  Distance to goal: {info['distance_to_goal']:.3f}m")

        # Check termination
        if done:
            if 'distance_to_goal' in info:
                success = info['distance_to_goal'] < episode.success_threshold
                print(f"\nEpisode finished: success={success}")
                print(f"Final distance to goal: {info['distance_to_goal']:.3f}m")
            else:
                print(f"\nEpisode finished")
            break

    return {
        'episode_id': episode.episode_id,
        'success': success,
        'steps': step + 1,
        'total_reward': total_reward,
        'final_info': info
    }


def demo_with_mock_components():
    """
    Demo 1: Using MockEnv and MockPolicy

    This shows the basic usage without requiring actual O3DE or VLM server.
    """
    print("\n" + "="*70)
    print("DEMO 1: Mock Components (No O3DE/VLM Required)")
    print("="*70)

    # Create environment and policy
    env = MockEnv()
    policy = MockPolicy()

    # Create test episodes
    episodes = create_test_episodes()

    # Run evaluation
    results = []
    for episode in episodes:
        result = run_episode(env, episode, policy)
        results.append(result)

    # Print summary
    print(f"\n{'='*70}")
    print("EVALUATION SUMMARY")
    print(f"{'='*70}")
    success_count = sum(1 for r in results if r['success'])
    print(f"Total episodes: {len(results)}")
    print(f"Success count: {success_count}")
    print(f"Success rate: {success_count/len(results):.2%}")

    print("\nPer-episode results:")
    for r in results:
        status = "✓ SUCCESS" if r['success'] else "✗ FAILED"
        print(f"  {r['episode_id']}: {status}, steps={r['steps']}, reward={r['total_reward']:.2f}")

    env.close()


def demo_with_o3de_components():
    """
    Demo 2: Using O3DEEnv (requires O3DE and ROS2)

    This shows how to use the actual O3DE simulator.
    """
    print("\n" + "="*70)
    print("DEMO 2: O3DE Components (Requires O3DE + ROS2)")
    print("="*70)

    try:
        # Create O3DE environment
        env = O3DEEnv(simulator_config={
            'mode': 'socket',
            'success_threshold': 0.2,
            'collision_threshold': 0.3,
            'linear_speed': 0.1,
            'angular_speed': 0.3
        })

        # Use mock policy for demo
        policy = MockPolicy()

        # Create test episodes
        episodes = create_test_episodes()

        # Run evaluation
        results = []
        for episode in episodes:
            result = run_episode(env, episode, policy)
            results.append(result)

        # Print summary
        print(f"\n{'='*70}")
        print("EVALUATION SUMMARY")
        print(f"{'='*70}")
        success_count = sum(1 for r in results if r['success'])
        print(f"Total episodes: {len(results)}")
        print(f"Success count: {success_count}")
        print(f"Success rate: {success_count/len(results):.2%}")

        env.close()

    except Exception as e:
        print(f"\n✗ O3DE not available: {e}")
        print("  Please ensure O3DE and ROS2 are installed and configured.")
        print("  Use Demo 1 (Mock Components) for testing without dependencies.")


def main():
    """Run all demos"""
    print("\n" + "="*70)
    print("VLN EVALUATOR DEMO")
    print("="*70)
    print("\nThis demo shows the relationship between:")
    print("  - Episode: Task configuration (start, goal, instruction)")
    print("  - Simulator: Low-level simulation (O3DE or Mock)")
    print("  - Env: Environment wrapper (task logic, rewards, termination)")
    print("  - Policy: Action selection (local or remote VLM)")

    # Demo 1: Mock components (always works)
    demo_with_mock_components()

    # Demo 2: O3DE components (requires O3DE + ROS2)
    # Uncomment to test with actual O3DE:
    # demo_with_o3de_components()

    print("\n" + "="*70)
    print("DEMO COMPLETED")
    print("="*70)


if __name__ == '__main__':
    main()
