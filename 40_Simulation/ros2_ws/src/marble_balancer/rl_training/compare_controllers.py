#!/usr/bin/env python3
"""
Compare RL Residual Controller vs Pure LQR on identical test episodes.

SINGLE ENVIRONMENT PATTERN:
- Creates ONE instance of GazeboRLEnvV2
- Toggles between stage 2 (RL) and stage 0 (LQR) between episodes
- Avoids ROS2 node conflicts and service timeouts

Usage:
  python3 compare_controllers.py --model models_td3_v2/best_model_td3_v2.zip --episodes 10
  python3 compare_controllers.py --model models_td3_v2/checkpoint_td3_v2_500000_steps.zip --episodes 5 --tcp-lissajous true
"""

import argparse
import os
import sys
import time
import pickle
import numpy as np
import rclpy
from stable_baselines3 import TD3
from gazebo_rl_env_v2 import GazeboRLEnvV2
import matplotlib.pyplot as plt


def load_normalization(norm_path):
    """Load running mean/std stats."""
    try:
        with open(norm_path, 'rb') as f:
            stats = pickle.load(f)
        return stats
    except FileNotFoundError:
        print(f"⚠ Normalization file not found: {norm_path}")
        return None


def compare_controllers(model_path, norm_path, num_episodes=5, tcp_lissajous=False, spawn_radius=0.12):
    """
    Compare RL + LQR vs Pure LQR using a SINGLE environment with stage toggling.
    
    Args:
        model_path: Path to .zip model file
        norm_path: Path to running_stats_v2.pkl
        num_episodes: Number of comparison episodes
        tcp_lissajous: Enable TCP Lissajous disturbance
        spawn_radius: Marble spawn radius
    """
    
    print(f"\n{'='*80}")
    print(f"RL + LQR vs Pure LQR Comparison (Single Environment Pattern)")
    print(f"Model: {os.path.basename(model_path)}")
    print(f"{'='*80}\n")
    
    # Check if model exists
    if not os.path.exists(model_path):
        print(f"❌ Model not found: {model_path}")
        return
    
    # Handle normalization path relative to script location
    if not os.path.isabs(norm_path):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        norm_path = os.path.join(script_dir, norm_path)
    
    # Load normalization stats if available
    norm_stats = load_normalization(norm_path)
    if norm_stats:
        print(f"[INFO] Loaded normalization stats\n")
    
    # Load model
    print(f"[INFO] Loading model: {os.path.basename(model_path)}")
    
    # CREATE SINGLE ENVIRONMENT (not two!)
    # This avoids ROS2 node name conflicts and service timeouts
    print(f"[INFO] Creating single shared environment (toggling stage per episode)...")
    env = GazeboRLEnvV2(
        use_tcp_lissajous=tcp_lissajous,
        spawn_radius=spawn_radius,
        stage=0  # Start with LQR-only
    )
    
    # Load model with shared environment
    model = TD3.load(model_path, env=env)
    print(f"[INFO] Ready to compare.\n")
    
    # Storage for metrics
    rl_rewards = []
    lqr_rewards = []
    rl_lengths = []
    lqr_lengths = []
    rl_max_errors = []
    lqr_max_errors = []
    
    print(f"{'Episode':<10} {'RL Reward':<15} {'LQR Reward':<15} {'RL Max Err':<12} {'LQR Max Err':<12} {'Winner':<8}")
    print("-" * 80)
    
    for ep in range(num_episodes):
        # ─────── RL + LQR Episode ───────
        # Set to stage 2 (full RL authority: 15 deg/s)
        env.unwrapped.set_stage(2)
        obs_rl, _ = env.reset()
        rl_reward = 0.0
        rl_length = 0
        rl_max_err = 0.0
        done_rl = False
        
        while not done_rl:
            if norm_stats:
                obs_norm = (obs_rl - norm_stats['mean']) / (np.sqrt(norm_stats['var']) + 1e-8)
                action, _ = model.predict(obs_norm, deterministic=True)
            else:
                action, _ = model.predict(obs_rl, deterministic=True)
            
            obs_rl, reward, terminated, truncated, info = env.step(action)
            done_rl = terminated or truncated
            rl_reward += reward
            rl_length += 1
            
            if len(obs_rl) > 0:
                err = np.sqrt(obs_rl[0]**2 + obs_rl[2]**2)
                rl_max_err = max(rl_max_err, err)
        
        rl_rewards.append(rl_reward)
        rl_lengths.append(rl_length)
        rl_max_errors.append(rl_max_err)
        
        # BREATHING ROOM: Allow Gazebo physics and ROS2 middleware to catch up
        # This prevents IK service timeouts and "already registered" conflicts
        print(f"  [pausing 1.0s between episodes...]", end='', flush=True)
        time.sleep(1.0)
        print(" ✓")
        
        # ─────── Pure LQR Episode ───────
        # Set to stage 0 (no RL, lambda = 0)
        env.unwrapped.set_stage(0)
        obs_lqr, _ = env.reset()
        lqr_reward = 0.0
        lqr_length = 0
        lqr_max_err = 0.0
        done_lqr = False
        
        while not done_lqr:
            # Pure LQR: take zero action (RL output is ignored)
            action = np.array([0.0, 0.0])
            
            obs_lqr, reward, terminated, truncated, info = env.step(action)
            done_lqr = terminated or truncated
            lqr_reward += reward
            lqr_length += 1
            
            if len(obs_lqr) > 0:
                err = np.sqrt(obs_lqr[0]**2 + obs_lqr[2]**2)
                lqr_max_err = max(lqr_max_err, err)
        
        lqr_rewards.append(lqr_reward)
        lqr_lengths.append(lqr_length)
        lqr_max_errors.append(lqr_max_err)
        
        # BREATHING ROOM between episodes
        print(f"  [pausing 1.0s between episodes...]", end='', flush=True)
        time.sleep(1.0)
        print(" ✓")
        
        # Determine winner
        winner = "RL" if rl_reward > lqr_reward else ("LQR" if lqr_reward > rl_reward else "DRAW")
        
        print(f"{ep+1:<10} {rl_reward:<15.2f} {lqr_reward:<15.2f} {rl_max_err:<12.4f} {lqr_max_err:<12.4f} {winner:<8}")
    
    # Statistics
    print("-" * 80)
    print(f"\n{'='*80}")
    print(f"COMPARISON SUMMARY ({num_episodes} episodes)")
    print(f"{'='*80}\n")
    
    rl_mean_reward = np.mean(rl_rewards)
    lqr_mean_reward = np.mean(lqr_rewards)
    improvement = ((rl_mean_reward - lqr_mean_reward) / abs(lqr_mean_reward)) * 100 if lqr_mean_reward != 0 else 0
    
    print(f"{'REWARD':<30}")
    print(f"  RL + LQR:          {rl_mean_reward:+8.2f} ± {np.std(rl_rewards):6.2f}")
    print(f"  Pure LQR:          {lqr_mean_reward:+8.2f} ± {np.std(lqr_rewards):6.2f}")
    print(f"  Improvement:       {improvement:+8.2f}%\n")
    
    print(f"{'EPISODE LENGTH (steps)':<30}")
    print(f"  RL + LQR:          {np.mean(rl_lengths):8.1f} ± {np.std(rl_lengths):6.1f}")
    print(f"  Pure LQR:          {np.mean(lqr_lengths):8.1f} ± {np.std(lqr_lengths):6.1f}\n")
    
    print(f"{'MAX MARBLE ERROR (m)':<30}")
    print(f"  RL + LQR:          {np.mean(rl_max_errors):8.4f} ± {np.std(rl_max_errors):6.4f}")
    print(f"  Pure LQR:          {np.mean(lqr_max_errors):8.4f} ± {np.std(lqr_max_errors):6.4f}\n")
    
    # Win statistics
    rl_wins = sum(1 for r, l in zip(rl_rewards, lqr_rewards) if r > l)
    print(f"{'EPISODES WON':<30}")
    print(f"  RL + LQR:          {rl_wins}/{num_episodes} ({100*rl_wins/num_episodes:.0f}%)")
    print(f"  Pure LQR:          {num_episodes-rl_wins}/{num_episodes} ({100*(num_episodes-rl_wins)/num_episodes:.0f}%)\n")
    
    print(f"{'='*80}\n")
    
    env.close()


if __name__ == '__main__':
    # Initialize ROS2 context for environment nodes
    if not rclpy.ok():
        rclpy.init()
    
    parser = argparse.ArgumentParser(description='Compare RL + LQR vs Pure LQR (Single Environment)')
    parser.add_argument('--model', type=str, required=True,
                        help='Path to RL model .zip file')
    parser.add_argument('--norm', type=str, default='models_td3_v2/running_stats_v2.pkl',
                        help='Path to normalization stats (relative to script directory)')
    parser.add_argument('--episodes', type=int, default=5,
                        help='Number of comparison episodes (default: 5)')
    parser.add_argument('--tcp-lissajous', type=str, default='false',
                        help='Enable TCP Lissajous (true/false, default: false)')
    parser.add_argument('--spawn-radius', type=float, default=0.12,
                        help='Marble spawn radius in meters (default: 0.12)')
    
    args = parser.parse_args()
    tcp_liss = args.tcp_lissajous.lower() in ['true', 'yes', '1']
    
    try:
        compare_controllers(
            model_path=args.model,
            norm_path=args.norm,
            num_episodes=args.episodes,
            tcp_lissajous=tcp_liss,
            spawn_radius=args.spawn_radius
        )
    finally:
        rclpy.shutdown()
