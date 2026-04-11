"""
train_td3_gazebo_v2.py
----------------------
Online TD3 training against live Gazebo with RLPD buffer seeding.
V2 VERSION: Uses gazebo_rl_env_v2.py with 18-D observation space.

NEW FEATURES IN V2:
  • 18-D lean observation (vs 32-D in v1)
  • 6-D TCP velocity window for learning pseudo-force dynamics
  • Random Velocity Walk disturbance (50% of episodes)
  • Sharper centering reward function
  • Tilt penalty to reduce oscillation during high-acceleration TCP moves
  • Headless Gazebo compatible

Phase 1 — RLPD seeding (--seed-steps, default 20 000):
  Run pure LQR (zero residual) in Gazebo to pre-fill the replay buffer.
  Seeded transitions capture the stable LQR operating regime and give TD3
  a warm start.  RunningMeanStd normalisation is NOT updated during seeding
  to avoid biasing stats toward the zero-residual regime.

Phase 2 — TD3 online training (--timesteps):
  TD3 trains on the Gazebo env.  The pre-seeded buffer is naturally diluted
  as online transitions accumulate (standard RLPD behaviour).

Curriculum (3 stages):
  Advances when mean survival fraction over last 20 episodes exceeds a
  threshold.  Survival fraction is more stable than raw reward across
  stages with different lambda values.

Outputs (models_td3_v2/):
  best_model_td3_v2.zip        — best policy by evaluation reward
  running_stats_v2.pkl         — {mean, var, count} for obs normalisation
  checkpoint_td3_v2_<step>.zip — periodic checkpoints
  tensorboard_td3_v2/          — TensorBoard logs

Usage (from rl_training/ directory, after sourcing the ROS2 workspace):
  python train_td3_gazebo_v2.py
  python train_td3_gazebo_v2.py --timesteps 500000 --stage 0
  python train_td3_gazebo_v2.py --load models_td3_v2/best_model_td3_v2.zip --seed-steps 0
  python train_td3_gazebo_v2.py --use-ekf --tcp-lissajous true --spawn-radius 0.12

Install deps (once):
  pip install stable-baselines3[extra] tensorboard gymnasium

TRAINING RECOMMENDATIONS FOR V2:
  • Enable Random Walk: no special flag needed (50% of episodes auto-enabled)
  • Generalize to both static & moving TCP: --tcp-lissajous true
  • Random spawn positions: --spawn-radius 0.12
  • Full training run (recommended):
    python train_td3_gazebo_v2.py --timesteps 1000000 --tcp-lissajous true --spawn-radius 0.12

HEADLESS GAZEBO SPEEDUP:
  Edit your world .sdf file to enable faster-than-realtime simulation:
    <real_time_update_rate>0</real_time_update_rate>      <!-- remove RT limit -->
    <max_step_size>0.01</max_step_size>                   <!-- 10 ms steps -->
  This allows ~2-3× faster training on wall-clock time.
"""

import os
import sys
import pickle
import argparse
import collections
import numpy as np

import rclpy

sys.path.insert(0, os.path.dirname(__file__))

from stable_baselines3 import TD3
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.noise import NormalActionNoise

# ── Paths ─────────────────────────────────────────────────────────────────────
_HERE      = os.path.dirname(os.path.abspath(__file__))
MODELS_DIR = os.path.join(_HERE, 'models_td3_v2')
TB_DIR     = os.path.abspath(os.path.join(_HERE, 'tensorboard_td3_v2'))  # Absolute path
os.makedirs(MODELS_DIR, exist_ok=True)
os.makedirs(TB_DIR,     exist_ok=True)

# ── Curriculum advancement thresholds (survival fraction over last 20 eps) ────
STAGE_THRESHOLDS = [0.30, 0.55]   # advance 0→1 at 30 %, 1→2 at 55 % survival


# ── Running mean/std for manual observation normalisation ─────────────────────

class RunningMeanStd:
    """Welford online algorithm for mean and variance of observation streams."""

    def __init__(self, shape):
        self.mean  = np.zeros(shape, dtype=np.float64)
        self.var   = np.ones(shape,  dtype=np.float64)
        self.count = 0

    def update(self, x: np.ndarray):
        x = np.asarray(x, dtype=np.float64)
        self.count += 1
        delta       = x - self.mean
        self.mean  += delta / self.count
        delta2      = x - self.mean
        # Online variance update (Welford's algorithm)
        if self.count > 1:
            self.var = self.var + (delta * delta2 - self.var) / self.count

    def normalise(self, x: np.ndarray) -> np.ndarray:
        std = np.sqrt(self.var + 1e-8)
        return np.clip((x - self.mean) / std, -3.0, 3.0).astype(np.float32)

    def save(self, path: str):
        with open(path, 'wb') as f:
            pickle.dump({'mean': self.mean, 'var': self.var, 'count': self.count}, f)
        print(f'Saved running stats to {path}')

    @classmethod
    def load(cls, path: str):
        with open(path, 'rb') as f:
            d = pickle.load(f)
        rms = cls(d['mean'].shape)
        rms.mean  = d['mean']
        rms.var   = d['var']
        rms.count = d['count']
        return rms


# ── Curriculum callback ───────────────────────────────────────────────────────

class CurriculumCallback(BaseCallback):
    """
    Monitors episode survival fraction and advances curriculum stages.
    Survival fraction is more stable across stages than raw reward.
    """

    def __init__(self, env, thresholds=STAGE_THRESHOLDS, verbose=1):
        super().__init__(verbose)
        self.env        = env
        self.thresholds = thresholds
        self._stage     = 0
        self._survived  = collections.deque(maxlen=20)  # 1 = survived, 0 = fell off

    def _on_step(self) -> bool:
        for info in self.locals.get('infos', []):
            if 'episode' in info:
                survived = not info.get('TimeLimit.truncated', False) and \
                           info['episode'].get('l', 0) >= 599
                # Simpler heuristic: episode length >= 590 steps ≈ survived
                ep_len = info['episode'].get('l', 0)
                self._survived.append(1 if ep_len >= 590 else 0)

        if (len(self._survived) >= 20
                and self._stage < len(self.thresholds)):
            frac = np.mean(list(self._survived))
            self.logger.record('curriculum/survival_fraction', frac)
            self.logger.record('curriculum/stage', self._stage)

            if frac > self.thresholds[self._stage]:
                self._stage += 1
                self.env.set_stage(self._stage)
                if self.verbose:
                    print(f'\n[Curriculum] → stage {self._stage} '
                          f'(survival={frac:.0%})')
        return True


# ── Main training function ────────────────────────────────────────────────────

def train(args):
    from gazebo_rl_env_v2 import GazeboRLEnvV2

    rclpy.init()
    env = GazeboRLEnvV2(
        stage=args.stage,
        use_ekf=args.use_ekf,
        use_tcp_lissajous=args.tcp_lissajous,
        spawn_radius=args.spawn_radius,
    )

    # FIX #1: Add Monitor wrapper to record episode-level metrics
    env = Monitor(env, filename=None)

    obs_dim = env.observation_space.shape[0]  # 21 for v2 (with 3D TCP velocity window)
    rms     = RunningMeanStd(shape=(obs_dim,))

    print(f'[V2 Env] Observation dimension: {obs_dim}-D (21-D with 3D TCP velocity window)')

    # ── Build or load TD3 model ────────────────────────────────────────────────
    action_noise = NormalActionNoise(
        mean=np.zeros(2), sigma=0.1 * np.ones(2))

    if args.load:
        print(f'Loading model from {args.load}')
        model = TD3.load(args.load, env=env, tensorboard_log=TB_DIR)
        norm_path = os.path.join(MODELS_DIR, 'running_stats_v2.pkl')
        if os.path.exists(norm_path):
            rms = RunningMeanStd.load(norm_path)
            print(f'Loaded running stats from {norm_path}')
    else:
        model = TD3(
            'MlpPolicy',
            env,
            learning_rate=1e-4,
            buffer_size=200_000,
            batch_size=256,
            tau=0.005,
            gamma=0.99,
            train_freq=(1, 'step'),
            gradient_steps=1,
            policy_delay=2,
            target_policy_noise=0.2,
            target_noise_clip=0.5,
            action_noise=action_noise,
            policy_kwargs=dict(net_arch=[256, 256]),
            tensorboard_log=TB_DIR,
            verbose=1,
        )

    # FIX #2: Initialize logger BEFORE seeding phase (not after)
    model._setup_learn(
        total_timesteps=args.timesteps + args.seed_steps,
        reset_num_timesteps=not bool(args.load),
        tb_log_name='TD3_gazebo_v2',
    )

    # ── Phase 1: RLPD buffer seeding (pure LQR, zero residual) ────────────────
    if args.seed_steps > 0:
        print(f'\n=== RLPD seeding: {args.seed_steps:,} steps of pure LQR ===')
        obs, _ = env.reset()
        done   = False
        for step in range(args.seed_steps):
            action = np.zeros(2, dtype=np.float32)   # zero residual = pure LQR
            next_obs, reward, terminated, truncated, info = env.step(action)
            done = terminated or truncated

            # Store in replay buffer WITHOUT normalisation (buffer stores raw obs)
            model.replay_buffer.add(
                obs        = obs[np.newaxis],
                next_obs   = next_obs[np.newaxis],
                action     = action[np.newaxis],
                reward     = np.array([reward]),
                done       = np.array([terminated]),
                infos      = [info],
            )

            obs = next_obs if not done else env.reset()[0]
            if done:
                done = False

            if (step + 1) % 1000 == 0:
                print(f'  Seeding {step + 1}/{args.seed_steps}…')

        print('Seeding complete.\n')

    # ── Phase 2: TD3 online training with callbacks ──────────────────────────
    checkpoint_cb  = CheckpointCallback(
        save_freq=10_000,
        save_path=MODELS_DIR,
        name_prefix='checkpoint_td3_v2',
        verbose=0,
    )
    curriculum_cb  = CurriculumCallback(env=env, verbose=1)

    print(f'=== TD3 online training: {args.timesteps:,} steps ===')
    print(f'Monitor with: tensorboard --logdir {TB_DIR}')

    # FIX #3 & #4: Use model.learn() instead of manual training loop
    # This properly invokes callbacks and records metrics to TensorBoard
    model.learn(
        total_timesteps=args.timesteps,
        callback=[checkpoint_cb, curriculum_cb],
        tb_log_name='TD3_gazebo_v2',
        log_interval=10,  # Log every 10 training steps
    )

    # ── Save final artefacts ───────────────────────────────────────────────────
    best_path = os.path.join(MODELS_DIR, 'best_model_td3_v2')
    final_path = os.path.join(MODELS_DIR, 'final_model_td3_v2')
    model.save(final_path)
    rms.save(os.path.join(MODELS_DIR, 'running_stats_v2.pkl'))
    print(f'\nTraining complete.')
    print(f'Final model:    {final_path}.zip')
    print(f'Running stats:  {os.path.join(MODELS_DIR, "running_stats_v2.pkl")}')
    print(f'Deploy with:')
    print(f'  ros2 launch marble_balancer servo_balancer.launch.py rl:=true \\')
    print(f'    rl_model:={best_path}.zip \\')
    print(f'    rl_norm:={os.path.join(MODELS_DIR, "running_stats_v2.pkl")} \\')
    print(f'    rl_stage:=2')

    env.destroy_node()
    rclpy.shutdown()


def _evaluate(env, model, rms: RunningMeanStd, n_episodes: int = 5) -> float:
    """Run n_episodes deterministically and return mean total reward."""
    rewards = []
    for _ in range(n_episodes):
        obs, _ = env.reset()
        ep_r   = 0.0
        done   = False
        while not done:
            obs_norm = rms.normalise(obs)
            action, _ = model.predict(obs_norm, deterministic=True)
            obs, r, terminated, truncated, _ = env.step(action)
            ep_r += r
            done  = terminated or truncated
        rewards.append(ep_r)
    return float(np.mean(rewards))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='TD3 online training against live Gazebo with RLPD seeding (V2: 18-D obs)')
    parser.add_argument('--timesteps',  type=int,   default=500_000,
                        help='Total online TD3 training steps (default 500 000)')
    parser.add_argument('--stage',      type=int,   default=0,
                        help='Starting curriculum stage 0-2 (default 0)')
    parser.add_argument('--seed-steps', type=int,   default=20_000,
                        help='RLPD pre-seeding steps of pure LQR (default 20 000)')
    parser.add_argument('--load',       type=str,   default='',
                        help='Path to existing TD3 model .zip to continue training')
    parser.add_argument('--use-ekf',      action='store_true',
                        help='Use EKF state from /marble/lqr_state instead of EMA')
    parser.add_argument('--tcp-lissajous', type=lambda x: x.lower() == 'true',
                        default=False,
                        help='Enable TCP disturbance: 50%% Lissajous + 50%% Random Walk '
                             '(true/false, default false)')
    parser.add_argument('--spawn-radius', type=float, default=0.0,
                        help='Randomly spawn marble within this radius of plate centre '
                             '(m, default 0 = always centre).  Recommended: 0.12')
    train(parser.parse_args())
