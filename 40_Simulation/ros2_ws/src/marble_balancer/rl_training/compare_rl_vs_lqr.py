#!/usr/bin/env python3
"""
compare_rl_vs_lqr.py - Simple, clean model comparison tool

This script:
1. Launches Gazebo infrastructure (no manual setup needed)
2. Loads an RL model
3. Runs identical episodes with RL enabled vs disabled
4. Reports metrics: reward, max error, mean error, success rate

Usage:
  python3 compare_rl_vs_lqr.py --model models_td3_v2/best_model_td3_v2.zip --episodes 5 --tcp-lissajous true

Metrics Output:
  ✓ Episode-by-episode comparison (reward, max_error_x, max_error_y)
  ✓ Summary statistics (mean, std, win rate)
  ✓ Controller performance (RL vs Pure LQR)
"""

import argparse
import os
import sys
import time
import subprocess
import pickle
import numpy as np
import rclpy
from stable_baselines3 import TD3

# Add workspace to path
_ws_root = '/home/Gery/Marble_Balancing_Robotic_Arm/40_Simulation/ros2_ws'
_rl_training_path = os.path.join(_ws_root, 'src', 'marble_balancer', 'rl_training')
if _rl_training_path not in sys.path:
    sys.path.insert(0, _rl_training_path)

from gazebo_rl_env_v2 import GazeboRLEnvV2


def print_header(text):
    """Print formatted section header."""
    print(f"\n{'='*80}")
    print(f"  {text}")
    print(f"{'='*80}\n")


def print_results_table(rl_episodes, lqr_episodes):
    """Print comparison table with metrics."""
    print(f"\n{'Episode':<10} {'RL Reward':<15} {'LQR Reward':<15} {'RL RMSE':<12} {'LQR RMSE':<12} {'RL Max':<12} {'LQR Max':<12} {'Winner':<10}")
    print("-" * 110)
    
    for i, (rl_ep, lqr_ep) in enumerate(zip(rl_episodes, lqr_episodes)):
        winner = "RL ✓" if rl_ep['reward'] > lqr_ep['reward'] else ("LQR ✗" if lqr_ep['reward'] > rl_ep['reward'] else "DRAW")
        max_err_rl = max(rl_ep.get('max_error_x', 0), rl_ep.get('max_error_y', 0))
        max_err_lqr = max(lqr_ep.get('max_error_x', 0), lqr_ep.get('max_error_y', 0))
        rmse_rl = rl_ep.get('rmse', 0.0)
        rmse_lqr = lqr_ep.get('rmse', 0.0)
        print(f"{i+1:<10} {rl_ep['reward']:<15.3f} {lqr_ep['reward']:<15.3f} {rmse_rl:<12.4f} {rmse_lqr:<12.4f} {max_err_rl:<12.4f} {max_err_lqr:<12.4f} {winner:<10}")


def print_metrics(rl_episodes, lqr_episodes):
    """Print summary metrics."""
    print_header("SUMMARY METRICS")
    
    rl_rewards = [ep['reward'] for ep in rl_episodes]
    lqr_rewards = [ep['reward'] for ep in lqr_episodes]
    rl_rmse = [ep.get('rmse', 0.0) for ep in rl_episodes]
    lqr_rmse = [ep.get('rmse', 0.0) for ep in lqr_episodes]
    rl_max_errors = [max(ep.get('max_error_x', 0), ep.get('max_error_y', 0)) for ep in rl_episodes]
    lqr_max_errors = [max(ep.get('max_error_x', 0), ep.get('max_error_y', 0)) for ep in lqr_episodes]
    
    print(f"REWARD:")
    print(f"  RL  (avg): {np.mean(rl_rewards):+.3f} ± {np.std(rl_rewards):.3f}")
    print(f"  LQR (avg): {np.mean(lqr_rewards):+.3f} ± {np.std(lqr_rewards):.3f}")
    improvement = ((np.mean(rl_rewards) - np.mean(lqr_rewards)) / abs(np.mean(lqr_rewards)) * 100) if np.mean(lqr_rewards) != 0 else 0
    print(f"  Improvement: {improvement:+.1f}%\n")
    
    print(f"RMSE ERROR (m):")
    print(f"  RL  (avg): {np.mean(rl_rmse):.4f} ± {np.std(rl_rmse):.4f}")
    print(f"  LQR (avg): {np.mean(lqr_rmse):.4f} ± {np.std(lqr_rmse):.4f}")
    rmse_improvement = ((np.mean(lqr_rmse) - np.mean(rl_rmse)) / np.mean(lqr_rmse) * 100) if np.mean(lqr_rmse) != 0 else 0
    print(f"  Improvement: {rmse_improvement:+.1f}%\n")
    
    print(f"MAX ERROR (m):")
    print(f"  RL  (avg): {np.mean(rl_max_errors):.4f} ± {np.std(rl_max_errors):.4f}")
    print(f"  LQR (avg): {np.mean(lqr_max_errors):.4f} ± {np.std(lqr_max_errors):.4f}")
    error_improvement = ((np.mean(lqr_max_errors) - np.mean(rl_max_errors)) / np.mean(lqr_max_errors) * 100) if np.mean(lqr_max_errors) != 0 else 0
    print(f"  Improvement: {error_improvement:+.1f}%\n")
    
    rl_wins = sum(1 for r, l in zip(rl_rewards, lqr_rewards) if r > l)
    print(f"WIN RATE:")
    print(f"  RL:  {rl_wins}/{len(rl_episodes)} ({100*rl_wins/len(rl_episodes):.0f}%)")
    print(f"  LQR: {len(rl_episodes)-rl_wins}/{len(rl_episodes)} ({100*(len(rl_episodes)-rl_wins)/len(rl_episodes):.0f}%)\n")


def run_comparison(model_path, episodes, tcp_lissajous, spawn_radius):
    """Run comparison: RL vs pure LQR."""
    
    # Handle relative paths: convert to absolute if needed
    if not os.path.isabs(model_path):
        # If relative, assume it's relative to rl_training directory
        rl_training_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(rl_training_dir, model_path)
    
    # Ensure .zip extension (SB3 expects it, but don't double-add)
    if not model_path.endswith('.zip'):
        model_path += '.zip'
    
    if not os.path.exists(model_path):
        raise FileNotFoundError(f"Model not found: {model_path}")
    
    print_header("RL Model Comparison (RL vs Pure LQR)")
    print(f"Model: {os.path.basename(model_path)}")
    print(f"Episodes: {episodes}")
    print(f"TCP Lissajous: {tcp_lissajous}")
    print(f"Spawn Radius: {spawn_radius}m\n")
    
    # Initialize ROS2
    if not rclpy.ok():
        rclpy.init()
    
    # Create single environment
    print("[*] Creating environment (Gazebo starting...)")
    env = GazeboRLEnvV2(
        use_tcp_lissajous=tcp_lissajous,
        spawn_radius=spawn_radius,
        stage=0
    )
    
    # Load model
    print("[*] Loading RL model...")
    model = TD3.load(model_path, env=env)
    
    rl_episodes = []
    lqr_episodes = []
    
    print_header("Running Comparison")
    print(f"{'Episode':<10} {'RL Reward':<15} {'LQR Reward':<15} {'RL Max Err':<15} {'LQR Max Err':<15} {'Winner':<10}")
    print("-" * 90)
    
    for ep_num in range(episodes):
        # ─────────── RL Episode ───────────────
        env.unwrapped.set_stage(2)  # RL enabled (lambda = 15 deg/s)
        obs_rl, _ = env.reset()
        
        rl_reward = 0.0
        rl_max_error_x = 0.0
        rl_max_error_y = 0.0
        rl_errors = []
        done = False
        
        while not done:
            action, _ = model.predict(obs_rl, deterministic=True)
            obs_rl, reward, terminated, truncated, info = env.step(action)
            done = terminated or truncated
            rl_reward += reward
            
            if len(obs_rl) > 0:
                x_err = abs(obs_rl[0])
                y_err = abs(obs_rl[2])
                rl_max_error_x = max(rl_max_error_x, x_err)
                rl_max_error_y = max(rl_max_error_y, y_err)
                # Track errors for RMSE
                rl_errors.append(np.sqrt(x_err**2 + y_err**2))
        
        rl_rmse = np.sqrt(np.mean(np.array(rl_errors)**2)) if rl_errors else 0.0
        
        rl_episodes.append({
            'reward': rl_reward,
            'max_error_x': rl_max_error_x,
            'max_error_y': rl_max_error_y,
            'rmse': rl_rmse,
        })
        
        # Sleep between episodes
        time.sleep(1.0)
        
        # ─────────── LQR Episode ───────────────
        env.unwrapped.set_stage(0)  # LQR only (lambda = 0)
        obs_lqr, _ = env.reset()
        
        lqr_reward = 0.0
        lqr_max_error_x = 0.0
        lqr_max_error_y = 0.0
        lqr_errors = []
        done = False
        
        while not done:
            action = np.array([0.0, 0.0])  # Zero RL action
            obs_lqr, reward, terminated, truncated, info = env.step(action)
            done = terminated or truncated
            lqr_reward += reward
            
            if len(obs_lqr) > 0:
                x_err = abs(obs_lqr[0])
                y_err = abs(obs_lqr[2])
                lqr_max_error_x = max(lqr_max_error_x, x_err)
                lqr_max_error_y = max(lqr_max_error_y, y_err)
                # Track errors for RMSE
                lqr_errors.append(np.sqrt(x_err**2 + y_err**2))
        
        lqr_rmse = np.sqrt(np.mean(np.array(lqr_errors)**2)) if lqr_errors else 0.0
        
        lqr_episodes.append({
            'reward': lqr_reward,
            'max_error_x': lqr_max_error_x,
            'max_error_y': lqr_max_error_y,
            'rmse': lqr_rmse,
        })
        
        # Sleep between episodes
        time.sleep(1.0)
        
        # Print row
        winner = "RL ✓" if rl_reward > lqr_reward else ("LQR ✗" if lqr_reward > rl_reward else "DRAW")
        max_err_rl = max(rl_max_error_x, rl_max_error_y)
        max_err_lqr = max(lqr_max_error_x, lqr_max_error_y)
        print(f"{ep_num+1:<10} {rl_reward:<15.3f} {lqr_reward:<15.3f} {max_err_rl:<15.4f} {max_err_lqr:<15.4f} {winner:<10}")
    
    # Print summary
    print_metrics(rl_episodes, lqr_episodes)
    
    env.close()
    rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Compare RL vs Pure LQR controller')
    parser.add_argument('--model', type=str, required=True,
                        help='Path to RL model .zip file')
    parser.add_argument('--episodes', type=int, default=5,
                        help='Number of episodes (default: 5)')
    parser.add_argument('--tcp-lissajous', type=str, default='false',
                        help='Enable TCP Lissajous (true/false)')
    parser.add_argument('--spawn-radius', type=float, default=0.12,
                        help='Marble spawn radius (default: 0.12)')
    
    args = parser.parse_args()
    tcp_liss = args.tcp_lissajous.lower() in ['true', 'yes', '1']
    
    try:
        run_comparison(
            model_path=args.model,
            episodes=args.episodes,
            tcp_lissajous=tcp_liss,
            spawn_radius=args.spawn_radius
        )
    except KeyboardInterrupt:
        print("\n\n[*] Interrupted by user")
    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()
