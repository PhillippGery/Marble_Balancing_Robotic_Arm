"""
train.py
--------
SAC training with curriculum for the ball-on-plate residual controller.

Usage (from rl_training/ directory):
  python train.py
  python train.py --timesteps 600000 --envs 8 --stage 2  # resume from stage 2
  python train.py --load models/best_model.zip           # continue training

Outputs:
  models/best_model.zip          — best policy by eval reward
  models/checkpoint_<step>.zip   — periodic checkpoints
  models/vec_normalize.pkl       — observation normalisation stats (needed for deploy)
  tensorboard/                   — training curves (run: tensorboard --logdir tensorboard/)

Install deps first:
  pip install gymnasium stable-baselines3[extra] tensorboard
"""

import os
import sys
import argparse
import numpy as np

# Allow running from any directory
sys.path.insert(0, os.path.dirname(__file__))
from ball_plate_env import BallPlateEnv, STAGES

from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import (
    BaseCallback, EvalCallback, CheckpointCallback
)
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import VecNormalize

MODELS_DIR = os.path.join(os.path.dirname(__file__), 'models')
TB_DIR     = os.path.join(os.path.dirname(__file__), 'tensorboard')
os.makedirs(MODELS_DIR, exist_ok=True)
os.makedirs(TB_DIR, exist_ok=True)


# ── Reward thresholds to advance curriculum stages ────────────────────────────
# Mean episode reward (over last 20 eval episodes) needed to advance
STAGE_ADVANCE_THRESHOLDS = [-40.0, -15.0, -5.0]   # advance 0→1, 1→2, 2→3


class CurriculumCallback(BaseCallback):
    """
    Monitors mean eval reward and advances curriculum stages.
    Also logs current stage to tensorboard.
    """

    def __init__(self, envs, eval_env, thresholds=STAGE_ADVANCE_THRESHOLDS, verbose=1):
        super().__init__(verbose)
        self.envs       = envs          # VecEnv used for training
        self.eval_env   = eval_env      # separate eval env
        self.thresholds = thresholds
        self._stage     = 0
        self._ep_rewards = []

    def _on_step(self) -> bool:
        # Collect episode rewards from info buffers
        for info in self.locals.get('infos', []):
            if 'episode' in info:
                self._ep_rewards.append(info['episode']['r'])

        if len(self._ep_rewards) >= 20 and self._stage < len(self.thresholds):
            mean_r = np.mean(self._ep_rewards[-20:])
            self.logger.record('curriculum/mean_reward', mean_r)
            self.logger.record('curriculum/stage', self._stage)

            if mean_r > self.thresholds[self._stage]:
                self._stage += 1
                self.envs.env_method('set_stage', self._stage)
                self.eval_env.env_method('set_stage', self._stage)
                clip_deg = np.degrees(STAGES[self._stage][0])
                use_lis  = STAGES[self._stage][1]
                if self.verbose:
                    print(f'\n[Curriculum] Advancing to stage {self._stage}: '
                          f'clip=±{clip_deg:.0f}°/s  lissajous={use_lis}')
        return True


def make_env(stage=0):
    return lambda: BallPlateEnv(stage=stage)


def train(args):
    n_envs = args.envs
    total_steps = args.timesteps

    # ── Environments ──────────────────────────────────────────────────────────
    train_envs = make_vec_env(make_env(args.stage), n_envs=n_envs)
    train_envs = VecNormalize(train_envs, norm_obs=True, norm_reward=True,
                               clip_obs=5.0)

    eval_env = make_vec_env(make_env(args.stage), n_envs=1)
    eval_env = VecNormalize(eval_env, norm_obs=True, norm_reward=False,
                             training=False, clip_obs=5.0)

    # ── Model ─────────────────────────────────────────────────────────────────
    if args.load:
        print(f'Loading model from {args.load}')
        model = SAC.load(args.load, env=train_envs)
        norm_path = args.load.replace('.zip', '_vecnorm.pkl')
        if os.path.exists(norm_path):
            train_envs = VecNormalize.load(norm_path, train_envs.venv)
            print(f'Loaded VecNormalize stats from {norm_path}')
    else:
        model = SAC(
            'MlpPolicy',
            train_envs,
            learning_rate=3e-4,
            buffer_size=500_000,
            batch_size=256,
            tau=0.005,
            gamma=0.99,
            train_freq=1,
            gradient_steps=1,
            ent_coef='auto',
            policy_kwargs=dict(net_arch=[256, 256]),
            tensorboard_log=TB_DIR,
            verbose=1,
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────
    checkpoint_cb = CheckpointCallback(
        save_freq=max(10_000 // n_envs, 1),
        save_path=MODELS_DIR,
        name_prefix='checkpoint',
        verbose=0,
    )

    eval_cb = EvalCallback(
        eval_env,
        best_model_save_path=MODELS_DIR,
        log_path=os.path.join(MODELS_DIR, 'eval_logs'),
        eval_freq=max(5_000 // n_envs, 1),
        n_eval_episodes=20,
        deterministic=True,
        verbose=1,
    )

    curriculum_cb = CurriculumCallback(
        envs=train_envs,
        eval_env=eval_env,
        verbose=1,
    )

    # ── Train ─────────────────────────────────────────────────────────────────
    print(f'Training SAC for {total_steps:,} steps on {n_envs} envs.')
    print(f'Starting at curriculum stage {args.stage}.')
    print(f'Monitor with: tensorboard --logdir {TB_DIR}')

    model.learn(
        total_timesteps=total_steps,
        callback=[checkpoint_cb, eval_cb, curriculum_cb],
        reset_num_timesteps=not bool(args.load),
    )

    # ── Save final model + normalisation stats ─────────────────────────────────
    final_path = os.path.join(MODELS_DIR, 'final_model')
    model.save(final_path)
    norm_path  = os.path.join(MODELS_DIR, 'vec_normalize.pkl')
    train_envs.save(norm_path)
    print(f'Saved final model: {final_path}.zip')
    print(f'Saved normalisation stats: {norm_path}')
    print(f'Deploy with: rl_model:={final_path}.zip  rl_norm:={norm_path}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--timesteps', type=int, default=600_000)
    parser.add_argument('--envs',      type=int, default=4)
    parser.add_argument('--stage',     type=int, default=0,
                        help='Starting curriculum stage (0-3)')
    parser.add_argument('--load',      type=str, default='',
                        help='Path to existing model .zip to continue training')
    train(parser.parse_args())
