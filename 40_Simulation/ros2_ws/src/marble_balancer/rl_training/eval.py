"""
eval.py
-------
Evaluate a trained SAC policy and plot marble trajectories.

Usage:
  python eval.py --model models/best_model.zip
  python eval.py --model models/best_model.zip --norm models/vec_normalize.pkl
  python eval.py --model models/best_model.zip --stage 2 --episodes 10
"""

import os
import sys
import argparse
import numpy as np
import matplotlib
matplotlib.use('Agg')  # non-interactive backend — avoids Qt/OpenCV conflict
import matplotlib.pyplot as plt
import matplotlib.patches as patches

sys.path.insert(0, os.path.dirname(__file__))
from ball_plate_env import BallPlateEnv, PLATE_HALF

from stable_baselines3 import SAC
from stable_baselines3.common.vec_env import VecNormalize, DummyVecEnv


def evaluate(args):
    # ── Load model ────────────────────────────────────────────────────────────
    model = SAC.load(args.model)
    print(f'Loaded model: {args.model}')

    # ── Build env ─────────────────────────────────────────────────────────────
    env = BallPlateEnv(stage=args.stage)

    if args.norm and os.path.exists(args.norm):
        dummy = DummyVecEnv([lambda: BallPlateEnv(stage=args.stage)])
        vec_norm = VecNormalize.load(args.norm, dummy)
        vec_norm.training = False
        vec_norm.norm_reward = False
        use_norm = True
        print(f'Loaded normalisation stats: {args.norm}')
    else:
        use_norm = False

    # ── Run episodes ──────────────────────────────────────────────────────────
    all_trajectories = []
    all_rewards      = []

    for ep in range(args.episodes):
        obs, _ = env.reset()
        traj   = [(env._state[0], env._state[2])]
        ep_reward = 0.0
        done = False

        while not done:
            if use_norm:
                obs_in = vec_norm.normalize_obs(obs.reshape(1, -1)).flatten()
            else:
                obs_in = obs
            action, _ = model.predict(obs_in, deterministic=True)
            obs, reward, terminated, truncated, _ = env.step(action)
            traj.append((env._state[0], env._state[2]))
            ep_reward += reward
            done = terminated or truncated

        all_trajectories.append(traj)
        all_rewards.append(ep_reward)
        outcome = 'FELL OFF' if terminated else 'survived'
        print(f'Episode {ep+1:2d}: reward={ep_reward:8.1f}  steps={len(traj)}  {outcome}')

    print(f'\nMean reward: {np.mean(all_rewards):.1f} ± {np.std(all_rewards):.1f}')
    survival = sum(1 for t, r in zip(all_trajectories, all_rewards) if len(t) == 501)
    print(f'Survival rate: {survival}/{args.episodes}')

    # ── Plot ──────────────────────────────────────────────────────────────────
    fig, axes = plt.subplots(1, 2, figsize=(13, 6))
    fig.suptitle(f'SAC Residual Controller — {args.episodes} episodes  '
                 f'(mean reward {np.mean(all_rewards):.1f})', fontsize=12)

    # Left: all trajectories on plate
    ax = axes[0]
    ax.set_facecolor('#1a1a2e')
    ax.add_patch(patches.Rectangle(
        (-PLATE_HALF, -PLATE_HALF), 2*PLATE_HALF, 2*PLATE_HALF,
        linewidth=2, edgecolor='#8888cc', facecolor='none'))
    ax.axhline(0, color='#555577', linewidth=0.8, linestyle='--')
    ax.axvline(0, color='#555577', linewidth=0.8, linestyle='--')

    cmap = plt.cm.plasma
    for i, traj in enumerate(all_trajectories):
        xs = [p[0] for p in traj]
        ys = [p[1] for p in traj]
        color = cmap(i / max(len(all_trajectories) - 1, 1))
        ax.plot(xs, ys, color=color, linewidth=0.8, alpha=0.7)
        ax.plot(xs[-1], ys[-1], 'o', color=color, markersize=4)

    ax.set_xlim(-0.22, 0.22)
    ax.set_ylim(-0.22, 0.22)
    ax.set_aspect('equal')
    ax.set_xlabel('X (m)', color='white')
    ax.set_ylabel('Y (m)', color='white')
    ax.set_title('Marble Trajectories', color='white')
    ax.tick_params(colors='white')
    for sp in ax.spines.values():
        sp.set_edgecolor('#444466')

    # Right: reward per episode + mean distance from centre
    ax2 = axes[1]
    ax2.set_facecolor('#1a1a2e')
    mean_dists = []
    for traj in all_trajectories:
        dists = [np.sqrt(p[0]**2 + p[1]**2) for p in traj]
        mean_dists.append(np.mean(dists))

    ep_nums = range(1, args.episodes + 1)
    ax2.bar(ep_nums, [r / 100 for r in all_rewards], color='#44aaff', alpha=0.7,
            label='Reward (/100)')
    ax2.plot(ep_nums, [d * 100 for d in mean_dists], 'o-', color='#ff6644',
             linewidth=1.5, markersize=4, label='Mean dist (cm)')
    ax2.axhline(np.mean([d * 100 for d in mean_dists]), color='#ff6644',
                linestyle='--', linewidth=1, alpha=0.5)
    ax2.set_xlabel('Episode', color='white')
    ax2.set_title('Reward & Mean Distance from Centre', color='white')
    ax2.legend(facecolor='#2a2a4e', labelcolor='white')
    ax2.tick_params(colors='white')
    for sp in ax2.spines.values():
        sp.set_edgecolor('#444466')

    fig.patch.set_facecolor('#1a1a2e')
    plt.tight_layout()
    out = os.path.join(os.path.dirname(args.model), 'eval_trajectories.png')
    plt.savefig(out, dpi=120, bbox_inches='tight')
    print(f'Saved plot: {out}')
    plt.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--model',    required=True,
                        help='Path to model .zip')
    parser.add_argument('--norm',     default='',
                        help='Path to vec_normalize.pkl')
    parser.add_argument('--stage',    type=int, default=3,
                        help='Curriculum stage to evaluate at (default: 3 = full)')
    parser.add_argument('--episodes', type=int, default=10)
    evaluate(parser.parse_args())
