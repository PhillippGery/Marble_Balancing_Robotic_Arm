# RL Training Plan — TD3 Residual Controller

## Goal
Train a TD3 residual controller that outperforms LQR alone, specifically on:
- Y axis tracking under TCP Lissajous disturbance (primary target)
- Keeping ball closer to plate centre (tighter position control)

---

## Current Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| Algorithm | TD3 + RLPD seeding | 40K pure-LQR seed steps |
| Timesteps | 1,000,000 | Phase 1 |
| `tcp_lissajous_prob` | 0.7 | 70% of episodes have TCP moving |
| Period range | uniform(10, 20) s | Deployment = 15 s (mid-range) |
| Amplitude range | uniform(0.20, 0.40) m | Deployment = 0.30 m |
| `spawn_radius` | 0 (Phase 1) → 0.12 (Phase 2) | See two-phase plan below |
| Curriculum stages | ±5 → ±10 → ±15 °/s | Advances on survival fraction |
| Network | [256, 256] MLP | Actor + Critic |
| Learning rate | 1e-4 | |
| Buffer size | 200,000 | |

---

## Reward Function

```python
# Dual-scale position: broad term guides from far, fine term rewards precision near origin
pos  = 0.7 * exp(-30  * (x² + 3.0*y²))
     + 0.3 * exp(-300 * (x² + 3.0*y²))

# Extra Y centering during TCP Lissajous episodes (agent uses phase encoding to pre-correct)
if tcp_episode_active:
    pos += 0.4 * exp(-300 * y²)

vel    = -0.1*vx² - 0.40*vy²   # stronger Y damping (TCP drives Y at 2× freq)
smooth = -0.02 * ||Δaction||²
surv   = 0.05
shape  = LQR potential shaping (Riccati P matrix)
```

---

## Two-Phase Training Plan

### Phase 1 — Learn to balance (700K steps, centre spawn)
Marble always spawns at plate centre. Agent learns basic stabilisation and advances
through curriculum stages before being exposed to off-centre starts.

```bash
ros2 launch marble_balancer rl_training.launch.py \
  timesteps:=700000 tcp_lissajous:=true spawn_radius:=0 \
  seed_steps:=40000 headless:=true
```

### Phase 2 — Generalise (300K steps, random spawn)
Load the best Phase 1 model. Short fine-tuning run exposing the agent to off-centre
marble starts (±12 cm) to build robustness without forgetting basic balancing.

```bash
ros2 launch marble_balancer rl_training.launch.py \
  timesteps:=300000 tcp_lissajous:=true spawn_radius:=0.12 \
  seed_steps:=0 headless:=true \
  load:=$(pwd)/src/marble_balancer/rl_training/models_td3/best_model_td3.zip
```

---

## Deployment

```bash
ros2 launch marble_balancer servo_balancer.launch.py \
  tcp_lissajous:=true \
  rl:=true \
  rl_model:=$(pwd)/src/marble_balancer/rl_training/models_td3/best_model_td3.zip \
  rl_norm:=$(pwd)/src/marble_balancer/rl_training/models_td3/running_stats.pkl \
  rl_stage:=2
```

Deployment TCP Lissajous parameters (`servo_balancer.launch.py`):
- amplitude: 0.30 m
- period: 15 s
- fa=1, fb=2, delta=π/2

---

## Convergence Check

Monitor via TensorBoard:
```bash
tensorboard --logdir src/marble_balancer/rl_training/tensorboard_td3/
```

- `eval/mean_reward` — flat for 150K+ steps = converged
- `train/actor_loss` — stable small oscillations = converged
- Terminal prints `→ New best` every 10K steps — stops printing when plateaued

If no improvement for 200K steps, stop training and move to Phase 2.

---

## Demo vs Training Settings

**Before demo:** set `MOVE_TIME_SEC = 3` in `marble_balancer/go_to_pose.py:33`
**Before training:** set `MOVE_TIME_SEC = 1` for faster resets (~2.5× more steps/hour)
