#!/usr/bin/env python3
"""
Test Script for VLN Evaluation System

Uses Mock VLM Server and Mock O3DE Simulator to test the evaluation flow.
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

from vln_evaluator.dataset_loader import R2REpisode, R2RDatasetLoader
from tests.mock_o3de_simulator import MockO3DESimulator, MockEpisode
from vln_evaluator.ws_policy_client import WebSocketPolicyClient


def test_mock_simulator_only():
    """Test 1: Mock Simulator Only (No VLM)"""
    print("\n" + "="*70)
    print("TEST 1: Mock O3DE Simulator (No VLM)")
    print("="*70)

    sim = MockO3DESimulator(success_threshold=0.2)

    # Create test episode
    episode = MockEpisode(
        episode_id='test_001',
        start_position={'x': 0, 'y': 0, 'z': 0},
        goal_position={'x': 1.0, 'y': 0, 'z': 0},
        instruction='Walk forward 1 meter',
        max_steps=10
    )

    # Reset
    obs = sim.reset(episode)
    print(f"✓ Reset successful")
    print(f"  RGB shape: {obs['rgb'].shape}")
    print(f"  Depth shape: {obs['depth'].shape}")

    # Run steps with predefined actions
    actions = [1, 1, 1, 1, 0]  # Forward x4 then STOP
    total_reward = 0

    for i, action in enumerate(actions):
        obs, reward, done, info = sim.step(action)
        total_reward += reward
        print(f"  Step {i+1}: action={action}, dist={info['distance_to_goal']:.3f}m")

        if done:
            success = info['distance_to_goal'] < 0.2
            print(f"  ✓ Episode done: success={success}")
            break

    sim.close()
    print("✓ Test 1 PASSED\n")
    return True


def test_vlm_client_only():
    """Test 2: VLM Client Only (Requires Mock Server)"""
    print("\n" + "="*70)
    print("TEST 2: VLM Client (Mock Server Required)")
    print("="*70)

    print("\nStarting Mock VLM Server...")
    # Start mock server in background
    server_process = subprocess.Popen(
        [sys.executable, 'tests/mock_vlm_server.py', '--port', '8080'],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT  # Capture both to see server output
    )

    # Wait for server to start
    time.sleep(3)

    try:
        # Create VLM client
        client = WebSocketPolicyClient(url='ws://localhost:8080')
        client.connect()
        print("✓ Connected to Mock VLM Server")

        # Test act() calls
        rgb = np.random.randint(0, 255, (64, 64, 3), dtype=np.uint8)
        depth = np.random.randint(0, 10000, (64, 64), dtype=np.uint16)
        instruction = "Walk forward 2 meters"

        print("\nSending 5 test observations...")
        for i in range(5):
            try:
                action = client.act(rgb, depth, instruction)
                action_names = ['STOP', 'FORWARD', 'LEFT', 'RIGHT']
                print(f"  Step {i+1}: Received action = {action} ({action_names[action]})")
            except Exception as act_error:
                print(f"  Step {i+1}: Error getting action: {act_error}")
                raise

        client.disconnect()
        print("✓ Test 2 PASSED\n")
        return True

    except Exception as e:
        print(f"✗ Test 2 FAILED: {e}")
        import traceback
        traceback.print_exc()
        print()
        return False
    finally:
        # Stop server
        server_process.terminate()
        try:
            server_process.wait(timeout=5)
        except:
            server_process.kill()


def test_full_evaluation_mock():
    """Test 3: Full Evaluation with Mock Simulator and VLM"""
    print("\n" + "="*70)
    print("TEST 3: Full Evaluation (Mock Simulator + Mock VLM)")
    print("="*70)

    print("\nStarting Mock VLM Server in smart mode...")
    server_process = subprocess.Popen(
        [sys.executable, 'tests/mock_vlm_server.py', '--port', '8081', '--smart'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )

    time.sleep(2)

    try:
        # Create simulator and VLM client
        sim = MockO3DESimulator(success_threshold=0.3)
        vlm_client = WebSocketPolicyClient(url='ws://localhost:8081')
        vlm_client.connect()

        print("✓ Mock components initialized")

        # Create test episodes
        episodes = [
            MockEpisode(
                episode_id='test_001',
                start_position={'x': 0, 'y': 0, 'z': 0},
                goal_position={'x': 2.0, 'y': 0, 'z': 0},
                instruction='Walk forward 2 meters',
                max_steps=20
            ),
            MockEpisode(
                episode_id='test_002',
                start_position={'x': 0, 'y': 0, 'z': 0},
                goal_position={'x': 1.0, 'y': 0, 'z': 1.0},
                instruction='Walk forward and turn left',
                max_steps=20
            )
        ]

        results = []

        # Run evaluation
        for ep in episodes:
            print(f"\n--- Evaluating {ep.episode_id} ---")
            print(f"Instruction: {ep.instruction}")

            # Reset
            obs = sim.reset(ep)

            # Episode loop
            success = False
            for step in range(ep.max_steps):
                # Get action from VLM
                action = vlm_client.act(obs['rgb'], obs['depth'], ep.instruction)

                # Execute
                obs, reward, done, info = sim.step(action)

                if done:
                    success = info['distance_to_goal'] < sim.success_threshold
                    print(f"  Episode done: success={success}, dist={info['distance_to_goal']:.3f}m")
                    break

            results.append({
                'episode_id': ep.episode_id,
                'success': success,
                'steps': info['step'],
                'distance': info['distance_to_goal']
            })

        # Print summary
        print("\n" + "-"*70)
        print("EVALUATION SUMMARY")
        print("-"*70)
        success_count = sum(1 for r in results if r['success'])
        print(f"Total episodes: {len(results)}")
        print(f"Success count: {success_count}")
        print(f"Success rate: {success_count/len(results):.2%}")
        print("\nPer-episode results:")
        for r in results:
            print(f"  {r['episode_id']}: success={r['success']}, steps={r['steps']}, dist={r['distance']:.3f}m")

        sim.close()
        vlm_client.disconnect()

        print("✓ Test 3 PASSED\n")
        return True

    except Exception as e:
        print(f"✗ Test 3 FAILED: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        server_process.terminate()
        server_process.wait()


def test_dataset_loader():
    """Test 4: Dataset Loader"""
    print("\n" + "="*70)
    print("TEST 4: R2R Dataset Loader")
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

    print("✓ Test 4 PASSED\n")
    return True


def main():
    """Run all tests"""
    print("\n" + "="*70)
    print("VLN EVALUATION SYSTEM - TEST SUITE")
    print("="*70)
    print("\nThis test suite uses Mock components to verify the evaluation flow")
    print("without requiring actual O3DE or VLM server.\n")

    import numpy as np  # For test data generation

    results = {}

    # Test 1: Mock Simulator Only
    results['test1'] = test_mock_simulator_only()

    # Test 2: VLM Client Only (with mock server)
    results['test2'] = test_vlm_client_only()

    # Test 3: Full Evaluation
    results['test3'] = test_full_evaluation_mock()

    # Test 4: Dataset Loader
    results['test4'] = test_dataset_loader()

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
