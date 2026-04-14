"""
train_cmaes_gazebo_v2.py
------------------------
CMA-ES validator for the TD3 residual policy (V2: 18-D observation).

Optimises a linear policy (18 → 2) using CMA-ES and compares its best
reward against the trained TD3 baseline.  If CMA-ES finds a solution
>10 % better than TD3, TD3 may be stuck in a local optimum.

Linear policy:  action = tanh(W @ obs + b)
  W: (2 × 18) weight matrix
  b: (2,) bias
  Total parameters: 38 (vs 58 in v1 with 28-D obs)

NEW FEATURES IN V2:
  • 18-D observation space (vs 32-D in v1)
  • 6-D TCP velocity window for learning pseudo-force dynamics
  • Random Velocity Walk disturbance (50% of episodes)
  • Sharper centering reward function
  • Tilt penalty to reduce oscillation
  • Headless Gazebo compatible

IMPORTANT — wall-time warning (Gazebo mode):
  Each CMA-ES fitness evaluation requires N_EVAL_EPS Gazebo episodes.
  At popsize=8, 100 iterations, N_EVAL_EPS=3, ~150 s/episode:
    8 × 100 × 3 × 150 s ≈ 100 hours.
  Use --offline (default) to run against ball_plate_env.py (1000× faster).
  Use --no-offline ONLY for a final sim-to-real gap check.

Usage (from rl_training/ directory):
  python train_cmaes_gazebo_v2.py                          # offline (fast)
  python train_cmaes_gazebo_v2.py --no-offline             # Gazebo (slow!)
  python train_cmaes_gazebo_v2.py --td3-model models_td3_v2/best_model_td3_v2.zip
  python train_cmaes_gazebo_v2.py --stage 2 --max-iters 200

Install deps (once):
  pip install cma gymnasium stable-baselines3[extra]
"""

import os
import sys
import pickle
import argparse
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    import cma
except ImportError:
    raise SystemExit('cma not installed.  Run: pip install cma')

# ── Policy ────────────────────────────────────────────────────────────────────

OBS_DIM    = 21    # V2: 21-D observation space (3D TCP velocity window)
ACTION_DIM = 2
N_PARAMS   = OBS_DIM * ACTION_DIM + ACTION_DIM   # 44


def policy_linear(params: np.ndarray, obs: np.ndarray) -> np.ndarray:
    """Linear map: (21,) → (2,) with tanh squash to [-1, 1]."""
    W = params[:OBS_DIM * ACTION_DIM].reshape(ACTION_DIM, OBS_DIM)
    b = params[OBS_DIM * ACTION_DIM:]
    return np.tanh(W @ obs + b).astype(np.float32)


def evaluate_params(env, params: np.ndarray, n_eps: int = 3) -> float:
    """Run n_eps episodes and return mean total reward."""
    total = 0.0
    for _ in range(n_eps):
        obs, _ = env.reset()
        done   = False
        ep_r   = 0.0
        while not done:
            action = policy_linear(params, obs)
            obs, r, terminated, truncated, _ = env.step(action)
            ep_r  += r
            done   = terminated or truncated
        total += ep_r
    return total / n_eps


# ── TD3 baseline ─────────────────────────────────────────────────────────────

def evaluate_td3(env, model_path: str, norm_path: str, n_eps: int = 20) -> float:
    """Evaluate trained TD3 model and return mean reward."""
    if not os.path.exists(model_path):
        print(f'TD3 model not found: {model_path} — skipping TD3 comparison.')
        return -np.inf

    from stable_baselines3 import TD3
    model = TD3.load(model_path)

    rms_mean = None
    rms_std  = None
    if norm_path and os.path.exists(norm_path):
        with open(norm_path, 'rb') as f:
            stats = pickle.load(f)
        rms_mean = stats['mean'].astype(np.float32)
        rms_std  = np.sqrt(stats['var'].astype(np.float32) + 1e-8)

    rewards = []
    for _ in range(n_eps):
        obs, _ = env.reset()
        done   = False
        ep_r   = 0.0
        while not done:
            obs_n = obs
            if rms_mean is not None:
                obs_n = np.clip((obs - rms_mean) / rms_std, -3.0, 3.0)
            action, _ = model.predict(obs_n.astype(np.float32), deterministic=True)
            obs, r, terminated, truncated, _ = env.step(action)
            ep_r += r
            done  = terminated or truncated
        rewards.append(ep_r)

    mean_r = float(np.mean(rewards))
    std_r  = float(np.std(rewards))
    print(f'TD3 baseline: mean={mean_r:.2f} ± {std_r:.2f}  (n={n_eps})')
    return mean_r


# ── Main ─────────────────────────────────────────────────────────────────────

def train(args):
    # ── Build environment ─────────────────────────────────────────────────────
    if args.offline:
        from ball_plate_env import BallPlateEnv
        env = BallPlateEnv(stage=args.stage)
        print('Mode: OFFLINE (ball_plate_env.py — ~1000× faster than Gazebo)')
    else:
        print('Mode: GAZEBO (live sim — very slow, see wall-time warning in docstring)')
        import rclpy
        from gazebo_rl_env_v2 import GazeboRLEnvV2
        rclpy.init()
        env = GazeboRLEnvV2(stage=args.stage)

    # ── TD3 baseline ──────────────────────────────────────────────────────────
    _here      = os.path.dirname(os.path.abspath(__file__))
    models_dir = os.path.join(_here, 'models_td3_v2')
    td3_path   = args.td3_model or os.path.join(models_dir, 'best_model_td3_v2.zip')
    norm_path  = args.td3_norm  or os.path.join(models_dir, 'running_stats_v2.pkl')

    n_td3_eps = 20 if args.offline else 5
    td3_reward = evaluate_td3(env, td3_path, norm_path, n_eps=n_td3_eps)

    # ── CMA-ES ────────────────────────────────────────────────────────────────
    out_dir = os.path.join(_here, 'models_cmaes_v2')
    os.makedirs(out_dir, exist_ok=True)

    x0     = np.zeros(N_PARAMS)
    sigma0 = 0.5

    opts = {
        'maxiter':   args.max_iters,
        'popsize':   args.popsize,
        'verbose':   3,
        'tolfun':    1e-4,
        'tolx':      1e-4,
    }

    print(f'\nCMA-ES: {args.max_iters} iters × popsize {args.popsize} '
          f'× {args.n_eval_eps} eval eps')
    print(f'Policy: linear ({OBS_DIM}-D obs → 2-D action), {N_PARAMS} parameters')

    es = cma.CMAEvolutionStrategy(x0, sigma0, opts)

    best_reward = -np.inf
    best_params = x0.copy()

    while not es.stop():
        candidates = es.ask()
        fitnesses  = []
        for p in candidates:
            r = evaluate_params(env, p, n_eps=args.n_eval_eps)
            fitnesses.append(-r)   # CMA-ES minimises

        es.tell(candidates, fitnesses)
        es.logger.add()
        es.disp()

        ep_best = -min(fitnesses)
        if ep_best > best_reward:
            best_reward = ep_best
            best_params = candidates[int(np.argmin(fitnesses))].copy()
            save_path   = os.path.join(out_dir, 'best_params_v2.pkl')
            with open(save_path, 'wb') as f:
                pickle.dump({'params': best_params, 'reward': best_reward}, f)

    # ── Summary ───────────────────────────────────────────────────────────────
    print(f'\n{"="*60}')
    print(f'CMA-ES best reward:  {best_reward:.2f}')
    print(f'TD3 baseline reward: {td3_reward:.2f}')
    if td3_reward > -np.inf:
        ratio = best_reward / (abs(td3_reward) + 1e-8)
        if best_reward > td3_reward * 1.1:
            print('WARNING: CMA-ES > TD3 * 1.1 — TD3 may be in a local optimum!')
        else:
            print('TD3 performance validated — no significant local optimum detected.')
    print(f'CMA-ES params saved to: {os.path.join(out_dir, "best_params_v2.pkl")}')

    if not args.offline:
        env.destroy_node()
        import rclpy
        rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='CMA-ES validator for TD3 residual policy (V2: 18-D obs)')
    parser.add_argument('--offline',    action='store_true', default=True,
                        help='Use ball_plate_env.py (fast, default)')
    parser.add_argument('--no-offline', dest='offline', action='store_false',
                        help='Use live Gazebo (very slow — see docstring warning)')
    parser.add_argument('--stage',      type=int,   default=2,
                        help='Curriculum stage to evaluate at (default 2)')
    parser.add_argument('--max-iters',  type=int,   default=100,
                        help='CMA-ES iterations (default 100)')
    parser.add_argument('--popsize',    type=int,   default=8,
                        help='CMA-ES population size (default 8)')
    parser.add_argument('--n-eval-eps', type=int,   default=3,
                        help='Episodes per candidate evaluation (default 3)')
    parser.add_argument('--td3-model',  type=str,   default='',
                        help='Path to TD3 model .zip for comparison (v2 model)')
    parser.add_argument('--td3-norm',   type=str,   default='',
                        help='Path to running_stats_v2.pkl for TD3 normalisation')
    train(parser.parse_args())
