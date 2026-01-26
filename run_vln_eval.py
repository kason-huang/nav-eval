#!/usr/bin/env python3
"""
VLN Evaluation Runner

Example script to run VLN evaluation with O3DE simulator.
"""

import argparse
import yaml
from pathlib import Path

from vln_evaluator import VLNEvaluator, evaluate_vln, O3DEEnv, MockPolicy
# from vln_evaluator import VLNEvaluator, evaluate_vln, O3DEEnv, WebSocketPolicy

def load_config(config_path: str) -> dict:
    """Load YAML configuration file"""
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


def main():
    parser = argparse.ArgumentParser(description='VLN Evaluation with O3DE Simulator')
    parser.add_argument('--dataset', type=str, default='datasets/sample_r2r.json',
                        help='Path to R2R dataset JSON file')
    parser.add_argument('--output', type=str, default='results/eval_results.json',
                        help='Path to save evaluation results')
    parser.add_argument('--config', type=str, default='configs/vln_eval.yaml',
                        help='Path to evaluation configuration YAML')
    parser.add_argument('--episodes', type=str, nargs='+', default=None,
                        help='Specific episode IDs to evaluate (if not specified, evaluate all)')
    parser.add_argument('--vlm-url', type=str, default=None,
                        help='Override VLM WebSocket URL')
    parser.add_argument('--sim-mode', type=str, default=None,
                        choices=['peb', 'socket'],
                        help='Override O3DE simulator mode')

    args = parser.parse_args()

    # Load configuration
    print(f'Loading configuration from: {args.config}')
    config = load_config(args.config)

    # Override config with command line arguments
    if args.vlm_url:
        config['vlm']['url'] = args.vlm_url
    if args.sim_mode:
        config['simulator']['mode'] = args.sim_mode

    # Prepare configs for evaluator
    vlm_config = {
        'url': config['vlm']['url'],
        'use_pickle': config['vlm'].get('use_pickle', True)
    }

    simulator_config = {
        'mode': config['simulator']['mode'],
        'success_threshold': config['simulator']['success_threshold'],
        'collision_threshold': config['simulator']['collision_threshold'],
        'linear_speed': config['simulator']['linear_speed'],
        'angular_speed': config['simulator']['angular_speed']
    }

    # Run evaluation
    print('\n' + '='*60)
    print('VLN EVALUATION')
    print('='*60)
    print(f'Dataset:     {args.dataset}')
    print(f'Output:      {args.output}')
    print(f'VLM URL:     {vlm_config["url"]}')
    print(f'Simulator:   {simulator_config["mode"]} mode')
    print('='*60)

    # Parse VLM URL to extract ip and port
    # from urllib.parse import urlparse
    # parsed_url = urlparse(vlm_config["url"])
    # ip = parsed_url.hostname or 'localhost'
    # port = parsed_url.port or 8080

    # Create env and policy instances
    env = O3DEEnv(simulator_config)
    # policy = WebSocketPolicy(ip, str(port))
    policy = MockPolicy()

    evaluate_vln(
        dataset_path=args.dataset,
        output_path=args.output,
        env=env,
        policy=policy,
        episode_ids=args.episodes
    )


if __name__ == '__main__':
    main()
