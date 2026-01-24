#!/usr/bin/env python3
"""
Test Script for VLN Evaluation System

Uses Mock Env and Mock Policy to test the evaluation flow.
"""

import json
import os
import sys
import time
import subprocess
from pathlib import Path
import numpy as np

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from vln_evaluator import (
    VLNEvaluator, evaluate_vln,
    MockEnv, O3DEEnv,
    MockPolicy, WebSocketPolicy
)
from vln_evaluator.dataset_loader import R2RDatasetLoader, R2REpisode


def test_mock_env_only():
    """Test 1: MockEnv Only (No Policy)"""
    print("\n" + "="*70)
    print("TEST 1: MockEnv (No Policy)")
    print("="*70)

    from vln_evaluator.env import Episode

    env = MockEnv()

    # Create test episode
    episode = Episode(
        episode_id='test_001',
        scene_id='test_scene',
        instruction='Walk forward 1 meter',
        start_position={'x': 0, 'y': 0, 'z': 0},
        start_rotation={'x': 0, 'y': 0, 'z': 0},
        goal_position={'x': 1.0, 'y': 0, 'z': 0},
        max_steps=10
    )

    # Reset
    obs = env.reset(episode)
    print(f"✓ Reset successful")
    print(f"  RGB shape: {obs['rgb'].shape}")
    print(f"  Depth shape: {obs['depth'].shape}")

    # Run steps with predefined actions
    actions = [1, 1, 1, 1, 0]  # Forward x4 then STOP

    for i, action in enumerate(actions):
        obs, reward, done, info = env.step(action)
        print(f"  Step {i+1}: action={action}, dist={info['distance_to_goal']:.3f}m")

        if done:
            success = info['distance_to_goal'] < 0.2
            print(f"  ✓ Episode done: success={success}")
            break

    env.close()
    print("✓ Test 1 PASSED\n")
    return True


def test_mock_policy_only():
    """Test 2: MockPolicy Only"""
    print("\n" + "="*70)
    print("TEST 2: MockPolicy")
    print("="*70)

    policy = MockPolicy()

    # Test act() calls
    rgb = np.random.randint(0, 255, (64, 64, 3), dtype=np.uint8)
    depth = np.random.randint(0, 10000, (64, 64), dtype=np.uint16)
    obs = {
        'rgb': rgb,
        'depth': depth,
        'instruction': 'Walk forward 2 meters'
    }

    print("\nCalling MockPolicy.act() 5 times...")
    action_names = ['STOP', 'FORWARD', 'LEFT', 'RIGHT']

    for i in range(5):
        action = policy.act(obs)
        print(f"  Call {i+1}: action = {action} ({action_names[action]})")

    print("✓ Test 2 PASSED\n")
    return True


def test_vln_evaluator_with_mocks():
    """Test 3: VLNEvaluator with MockEnv and MockPolicy"""
    print("\n" + "="*70)
    print("TEST 3: VLNEvaluator (MockEnv + MockPolicy)")
    print("="*70)

    # Create Env and Policy
    env = MockEnv()
    policy = MockPolicy()

    # Create VLNEvaluator
    evaluator = VLNEvaluator(env=env, policy=policy,
                             default_max_steps=20,
                             default_success_threshold=0.3)

    print("✓ VLNEvaluator initialized with Mock components")

    # Create test episode
    episode = R2REpisode(
        episode_id='test_001',
        scene_id='test_scene',
        instruction='Walk forward 2 meters',
        start_position={'x': 0, 'y': 0, 'z': 0},
        start_rotation={'x': 0, 'y': 0, 'z': 0},
        goal_position={'x': 2.0, 'y': 0, 'z': 0},
        max_steps=20
    )

    print(f"\n--- Evaluating {episode.episode_id} ---")
    print(f"Instruction: {episode.instruction}")

    # Evaluate episode
    result = evaluator.evaluate_episode(episode)

    print(f"\nResult:")
    print(f"  Success: {result.success}")
    print(f"  Failure Reason: {result.failure_reason}")
    print(f"  Final Distance: {result.final_distance_to_goal:.3f}m")
    print(f"  Steps: {result.steps}")
    print(f"  Collision Count: {result.collision_count}")
    print(f"  Trajectory Points: {len(result.trajectory)}")

    evaluator.close()

    print("\n✓ Test 3 PASSED\n")
    return True


def test_vln_evaluator_with_websocket():
    """Test 4: VLNEvaluator with MockEnv and WebSocketPolicy"""
    print("\n" + "="*70)
    print("TEST 4: VLNEvaluator (MockEnv + WebSocketPolicy)")
    print("="*70)

    print("\nStarting Mock VLM Server in smart mode...")
    server_process = subprocess.Popen(
        [sys.executable, 'tests/mock_vlm_server.py', '--port', '8081', '--smart'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )

    time.sleep(2)

    try:
        # Create Env and Policy
        env = MockEnv()
        policy = WebSocketPolicy('localhost', '8081')

        # Create VLNEvaluator
        evaluator = VLNEvaluator(env=env, policy=policy,
                                 default_max_steps=20,
                                 default_success_threshold=0.3)

        print("✓ VLNEvaluator initialized with MockEnv + WebSocketPolicy")

        # Create test episodes
        episodes = [
            R2REpisode(
                episode_id='test_001',
                scene_id='test_scene',
                instruction='Walk forward 2 meters',
                start_position={'x': 0, 'y': 0, 'z': 0},
                start_rotation={'x': 0, 'y': 0, 'z': 0},
                goal_position={'x': 2.0, 'y': 0, 'z': 0},
                max_steps=20
            ),
            R2REpisode(
                episode_id='test_002',
                scene_id='test_scene',
                instruction='Walk forward 1.5 meters',
                start_position={'x': 0, 'y': 0, 'z': 0},
                start_rotation={'x': 0, 'y': 0, 'z': 0},
                goal_position={'x': 1.5, 'y': 0, 'z': 0},
                max_steps=20
            )
        ]

        results = []

        # Evaluate episodes
        for ep in episodes:
            print(f"\n--- Evaluating {ep.episode_id} ---")
            print(f"Instruction: {ep.instruction}")

            result = evaluator.evaluate_episode(ep)
            results.append(result)

            print(f"  Success: {result.success}")
            print(f"  Distance: {result.final_distance_to_goal:.3f}m")
            print(f"  Steps: {result.steps}")

        # Print summary
        print("\n" + "-"*70)
        print("EVALUATION SUMMARY")
        print("-"*70)
        success_count = sum(1 for r in results if r.success)
        print(f"Total episodes: {len(results)}")
        print(f"Success count: {success_count}")
        print(f"Success rate: {success_count/len(results):.2%}")

        evaluator.close()

        print("\n✓ Test 4 PASSED\n")
        return True

    except Exception as e:
        print(f"✗ Test 4 FAILED: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        server_process.terminate()
        try:
            server_process.wait(timeout=5)
        except:
            server_process.kill()


def test_evaluate_dataset():
    """Test 5: evaluate_dataset() with mock components"""
    print("\n" + "="*70)
    print("TEST 5: evaluate_dataset() with Mock Components")
    print("="*70)

    # Create test dataset
    test_episodes = [
        R2REpisode(
            episode_id='test_001',
            scene_id='test_scene',
            instruction='Walk forward 1 meter',
            start_position={'x': 0, 'y': 0, 'z': 0},
            start_rotation={'x': 0, 'y': 0, 'z': 0},
            goal_position={'x': 1.0, 'y': 0, 'z': 0},
            max_steps=15
        ),
        R2REpisode(
            episode_id='test_002',
            scene_id='test_scene',
            instruction='Walk forward 1.5 meters',
            start_position={'x': 0, 'y': 0, 'z': 0},
            start_rotation={'x': 0, 'y': 0, 'z': 0},
            goal_position={'x': 1.5, 'y': 0, 'z': 0},
            max_steps=20
        ),
        R2REpisode(
            episode_id='test_003',
            scene_id='test_scene',
            instruction='Walk forward 2 meters',
            start_position={'x': 0, 'y': 0, 'z': 0},
            start_rotation={'x': 0, 'y': 0, 'z': 0},
            goal_position={'x': 2.0, 'y': 0, 'z': 0},
            max_steps=25
        )
    ]

    # Save to file
    test_dataset_path = 'tests/test_dataset.json'
    R2RDatasetLoader.save_to_file(test_episodes, test_dataset_path)
    print(f"✓ Created test dataset: {test_dataset_path}")

    # Create evaluator with mocks
    env = MockEnv()
    policy = MockPolicy()

    with VLNEvaluator(env=env, policy=policy) as evaluator:
        print("✓ VLNEvaluator initialized")

        # Evaluate dataset
        results = evaluator.evaluate_dataset(test_dataset_path)

        # Print summary
        print("\n" + "-"*70)
        print("DATASET EVALUATION RESULTS")
        print("-"*70)
        print(f"Total episodes: {results.total_episodes}")
        print(f"Success count: {results.success_count}")
        print(f"Success rate: {results.success_rate:.2%}")
        print(f"Avg distance error: {results.avg_distance_error:.3f}m")
        print(f"Avg steps: {results.avg_steps:.1f}")

        print("\nPer-episode results:")
        for ep in results.episodes:
            status = "✓" if ep.success else "✗"
            print(f"  {status} {ep.episode_id}: success={ep.success}, "
                  f"steps={ep.steps}, dist={ep.final_distance_to_goal:.3f}m")

    # Cleanup
    if os.path.exists(test_dataset_path):
        os.remove(test_dataset_path)
        print(f"\n✓ Cleaned up test dataset")

    print("\n✓ Test 5 PASSED\n")
    return True


def test_dataset_loader():
    """Test 6: Dataset Loader"""
    print("\n" + "="*70)
    print("TEST 6: R2R Dataset Loader")
    print("="*70)

    # Create test dataset
    test_episodes = [
        R2REpisode(
            episode_id='test_001',
            scene_id='scene_001',
            instruction='Test instruction 1',
            start_position={'x': 0, 'y': 0, 'z': 0},
            start_rotation={'x': 0, 'y': 0, 'z': 0},
            goal_position={'x': 1, 'y': 0, 'z': 0},
            max_steps=50
        ),
        R2REpisode(
            episode_id='test_002',
            scene_id='scene_001',
            instruction='Test instruction 2',
            start_position={'x': 0, 'y': 0, 'z': 0},
            start_rotation={'x': 0, 'y': 0, 'z': 0},
            goal_position={'x': 2, 'y': 0, 'z': 1},
            max_steps=50
        )
    ]

    # Save to file
    test_dataset_path = 'tests/test_dataset.json'
    R2RDatasetLoader.save_to_file(test_episodes, test_dataset_path)
    print(f"✓ Saved test dataset to {test_dataset_path}")

    # Load from file
    loaded_episodes = R2RDatasetLoader.load_from_file(test_dataset_path)
    print(f"✓ Loaded {len(loaded_episodes)} episodes from dataset")

    # Verify
    assert len(loaded_episodes) == len(test_episodes)
    for loaded, original in zip(loaded_episodes, test_episodes):
        assert loaded.episode_id == original.episode_id
        assert loaded.instruction == original.instruction
        print(f"  ✓ Episode {loaded.episode_id}: {loaded.instruction}")

    # Cleanup
    if os.path.exists(test_dataset_path):
        os.remove(test_dataset_path)
        print(f"✓ Cleaned up test dataset")

    print("✓ Test 6 PASSED\n")
    return True


def main():
    """Run all tests"""
    print("\n" + "="*70)
    print("VLN EVALUATION SYSTEM - TEST SUITE")
    print("="*70)
    print("\nThis test suite uses Mock Env and Mock Policy to verify")
    print("the evaluation flow without requiring actual O3DE or VLM server.\n")

    results = {}

    # Test 1: MockEnv Only
    results['test1'] = test_mock_env_only()

    # Test 2: MockPolicy Only
    results['test2'] = test_mock_policy_only()

    # Test 3: VLNEvaluator with Mocks
    results['test3'] = test_vln_evaluator_with_mocks()

    # Test 4: VLNEvaluator with WebSocket
    results['test4'] = test_vln_evaluator_with_websocket()

    # Test 5: evaluate_dataset()
    results['test5'] = test_evaluate_dataset()

    # Test 6: Dataset Loader
    results['test6'] = test_dataset_loader()

    # Print summary
    print("\n" + "="*70)
    print("TEST SUMMARY")
    print("="*70)
    passed = sum(1 for v in results.values() if v)
    total = len(results)

    for test_name, passed_test in results.items():
        status = "✓ PASSED" if passed_test else "✗ FAILED"
        print(f"  {test_name}: {status}")

    print("\n" + f"Results: {passed}/{total} tests passed")

    if passed == total:
        print("\n✓ ALL TESTS PASSED!")
        return 0
    else:
        print(f"\n✗ {total - passed} test(s) failed")
        return 1


if __name__ == '__main__':
    exit_code = main()
    sys.exit(exit_code)
