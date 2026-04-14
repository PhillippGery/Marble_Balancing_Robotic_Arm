# TD3 Online Training Pipeline

## Overview

The RL system trains a **TD3 (Twin Delayed Deep Deterministic Policy Gradient)** residual controller that adds small corrective angular velocity commands on top of the LQR base controller. Training runs entirely inside live Gazebo — the simulated robot is the training environment.

```
Marble on plate
      │
      ▼
  Gazebo sim  ──odom──►  GazeboRLEnv  ──state──►  LQR + TD3 residual  ──twist──►  MoveIt Servo  ──joints──►  UR5e
      ▲                                                                                                           │
      └───────────────────────────────────────────────────────────────────────────────────────────────────────────┘
```

---

## What it Uses

| Component | Role |
|-----------|------|
| **Gazebo Classic** | Physics simulation — marble dynamics, plate collision, odometry |
| **MoveIt Servo** | Real-time joint velocity control from twist commands |
| **go_to_pose** | Homing the arm between episodes |
| **TD3 (stable-baselines3)** | Off-policy actor-critic RL algorithm |
| **LQR** | Base controller; TD3 only adds a small residual |
| **TF2** | Plate orientation (pitch/roll angles) from kinematics |
| **Jacobian** | Plate angular velocity from joint velocities |

---

## How It Trains — Step by Step

### 1. Launch
```bash
ros2 launch marble_balancer rl_training.launch.py

# For unattended training runs, suppress the Gazebo GUI:
ros2 launch marble_balancer rl_training.launch.py headless:=true
```
This starts: Gazebo + UR5e + MoveIt Servo + `go_to_pose` (initial arm home).
It does **NOT** start `marble_servo_controller` — `GazeboRLEnv` is the controller during training.
After `go_to_pose` exits, `train_td3_gazebo.py` starts automatically (1 s delay).

---

### 2. Phase 1 — RLPD Buffer Seeding (40,000 steps, pure LQR)

Before TD3 trains, the replay buffer is pre-filled with **pure LQR transitions** (zero RL residual). This is called **RLPD (Replay-weighted Policy Distillation)** seeding.

- Action sent every step: `[0, 0]` (no residual)
- The LQR controller alone balances the marble
- All transitions `(obs, action, reward, next_obs)` are stored in the replay buffer
- `RunningMeanStd` normalisation **IS** updated during seeding (TCP velocities are now in range after normalization fix)

**Why:** TD3 needs many samples before it can learn anything useful. Starting with stable LQR transitions gives it a warm start — it knows what "good behaviour" looks like from day one. 40K steps covers ~66+ full Lissajous cycles for a thorough warm-start.

---

### 3. Phase 2 — TD3 Online Training (500,000 steps default)

#### Per-step loop:
```
1. Normalise obs with RunningMeanStd
2. Query TD3 actor → action ∈ [-1, 1]  (random for first 1,000 steps)
3. Compute: u_total = clip(LQR + action * λ, ±60°/s)
4. Publish twist to MoveIt Servo
5. Receive next_obs, reward from Gazebo
6. Store (obs_norm, action, reward, next_obs_norm) in replay buffer
7. If buffer > batch size: call model.train() — 1 gradient step
```

#### TD3 gradient step (every env step after 1,000 steps):
- Samples a mini-batch of 256 from the 200,000-transition replay buffer
- Updates **two critic networks** (Q1, Q2) to minimise Bellman error
- Every **2 steps** (policy_delay=2): updates the **actor** to maximise Q1
- Target networks updated with soft update: `τ = 0.005`

---

## The Control Law

At every step, the total command sent to the arm is:

```
u_lqr     = clip(-K @ state, ±60°/s)        # LQR baseline
residual  = action * λ                        # TD3 residual (scaled)
u_total   = clip(u_lqr + residual, ±60°/s)  # combined command

omega_alpha_cmd = -u_total[0]   # negated — plate_tcp yaw ≈ 180° at home
omega_beta_cmd  = -u_total[1]

twist.angular.x = omega_beta_cmd    # → controls Y marble dynamics
twist.angular.y = omega_alpha_cmd   # → controls X marble dynamics
```

The **sign negation** on both axes is essential — without it the arm moves opposite to what the controller intends due to the plate being mounted with ~180° yaw rotation at the home pose.

---

## Observation Space (36-D)

Every observation vector fed to TD3 has 36 dimensions:

```
obs[0:8]   = state / _NORM              # normalised plant state
obs[8:28]  = action_history.flatten()   # last 10 actions, oldest-first
obs[28:32] = twist_cmd / _TWIST_NORM    # LQR angular cmds + TCP velocity
obs[32:36] = [sin(φ_x), cos(φ_x), sin(φ_y), cos(φ_y)]  # Lissajous phase
```

### State (8-D): `[x, vx, y, vy, α, ω_α, β, ω_β]`

| Index | State | Meaning | Normaliser |
|-------|-------|---------|-----------|
| 0 | x | marble X position on plate (m) | 0.20 |
| 1 | vx | marble X velocity (m/s) | 0.50 |
| 2 | y | marble Y position on plate (m) | 0.20 |
| 3 | vy | marble Y velocity (m/s) | 0.50 |
| 4 | α | plate pitch angle (rad) | 0.30 |
| 5 | ω_α | plate pitch rate (rad/s) | MAX_RATE (1.047 rad/s = 60°/s) |
| 6 | β | plate roll angle (rad) | 0.30 |
| 7 | ω_β | plate roll rate (rad/s) | MAX_RATE (1.047 rad/s = 60°/s) |

### Twist Command (4-D)
`[ω_β_cmd, ω_α_cmd, tcp_vx, tcp_vy]` — the full twist sent to MoveIt Servo this step.

| Index | Content | Normaliser |
|-------|---------|-----------|
| 28 | ω_β_cmd | MAX_RATE |
| 29 | ω_α_cmd | MAX_RATE |
| 30 | tcp_vx | 0.20 m/s (sized for 0.30 m amplitude, 12 s period) |
| 31 | tcp_vy | 0.35 m/s (sized for 0.30 m amplitude, 12 s period, fb=2) |

### Action History (20-D)
Last 10 actions the agent took, each 2-D `[Δω_α, Δω_β]` in `[-1, 1]`.
Oldest action is at index 0. This gives the agent **memory** — it can detect oscillation patterns and dampen them.

### Lissajous Phase Encoding (4-D)
`[sin(φ_x), cos(φ_x), sin(φ_y), cos(φ_y)]` — where in the Lissajous cycle the TCP currently is.
- `φ_x = fa * ω₀ * t + δ`
- `φ_y = fb * ω₀ * t`

sin/cos encoding avoids discontinuity at 0/2π. All four values are **0.0** when TCP Lissajous is not active in this episode. This lets the agent distinguish "TCP is stationary" from any particular phase.

Already in `[-1, 1]`, so no additional normalisation needed.

### Observation Normalisation
A manual `RunningMeanStd` (Welford online algorithm) tracks the mean and variance of all 36 dimensions in real-time. Observations are normalised before storing in the buffer and before policy queries:
```python
obs_norm = clip((obs - mean) / sqrt(var + 1e-8), -3, 3)
```
This is saved to `running_stats.pkl` alongside the model and loaded at deployment.

---

## Action Space (2-D)

```
action ∈ [-1, 1]²     →     residual = action * λ  (rad/s)
```

λ (lambda) is the **curriculum residual budget**:
| Stage | λ | Max residual |
|-------|---|-------------|
| 0 | 5°/s | ±0.087 rad/s |
| 1 | 10°/s | ±0.175 rad/s |
| 2 | 15°/s | ±0.262 rad/s |

---

## Reward Function

Each step returns a scalar reward composed of 5 terms:

```python
# Y weighted 2.5× — TCP Lissajous drives Y at 2× frequency (fb=2) → ~4× pseudo-force vs X
pos    =  exp(-50 * (x² + 2.5*y²))       # +1 at centre, → 0 at edges; Y penalised more
vel    = -0.1 * vx² - 0.25 * vy²         # Y velocity penalised 2.5× vs X
smooth = -0.02 * ||action - prev_action||² # penalise jerky residual
surv   =  0.05                            # +0.05 every step survived
shape  =  0.99 * Φ(s') - Φ(s)            # potential shaping

# On fall-off:
reward -= 50.0
```

### Potential Shaping (Ng et al. 1999)
```
Φ(s) = -s^T P s
```
`P` is the solution to the Algebraic Riccati Equation from `lqr_math.compute_dlqr()`. It is positive-definite, so `Φ` is zero at the origin and negative everywhere else. The shaping term `0.99 * Φ(s') - Φ(s)` rewards the agent for **moving the state toward the origin** without changing the optimal policy (policy-invariant shaping guarantee).

---

## Episode Management

Each episode:

1. **Delete marble** — calls `/delete_entity`, waits 0.6 s for physics to flush
2. **Home arm** — spawns `go_to_pose` as subprocess, waits for it to exit (≤15 s timeout)
3. **Spawn marble** — calls `/spawn_entity` 10 cm above plate surface; verifies marble doesn't fall through (3-attempt retry)
4. **Wait for landing** — polls until `/marble/landed` fires (20 s timeout)
5. **Reset state** — zeros action history, velocity estimates, shaping baseline

### Termination
An episode ends when any of the following occur:
- `|x| > 0.20 m` or `|y| > 0.20 m` — marble off plate (XY)
- `marble_z < plate_top_z - 5 cm` — marble fell through plate
- `step_count ≥ 600` — 20 s time limit reached (truncated, not terminated)

---

## Curriculum

`CurriculumCallback` monitors the **survival fraction** (episodes lasting ≥590 steps) over the last 20 episodes and advances the stage automatically:

| Trigger | Action |
|---------|--------|
| Survival fraction > 40% | Advance to stage 1 (λ = 10°/s) |
| Survival fraction > 65% | Advance to stage 2 (λ = 15°/s) |

Thresholds are higher than the original [30%, 55%] because the harder task (TCP Lissajous + asymmetric reward + domain randomization) needs a more stable policy before widening the residual budget. Survival fraction is more stable than raw reward across stages because λ changes the reward scale.

---

## TD3 Hyperparameters

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `learning_rate` | 1e-4 | Adam optimizer step size |
| `buffer_size` | 200,000 | Replay buffer capacity |
| `batch_size` | 256 | Mini-batch size per gradient step |
| `tau` | 0.005 | Target network soft-update rate |
| `gamma` | 0.99 | Discount factor |
| `policy_delay` | 2 | Actor updated every 2 critic updates |
| `target_policy_noise` | 0.2 | Noise on target actions (prevents Q overfit) |
| `target_noise_clip` | 0.5 | Clips target noise |
| `action_noise` | Normal(0, 0.1) | Exploration noise on actor output |
| `net_arch` | [256, 256] | 2 hidden layers, 256 units each |
| `gradient_steps` | 1 | Gradient steps per env step |

---

## Outputs (`rl_training/models_td3/`)

| File | Contents |
|------|----------|
| `best_model_td3.zip` | Policy weights with highest evaluation reward |
| `running_stats.pkl` | `{mean, var, count}` for obs normalisation at deployment |
| `checkpoint_td3_<N>.zip` | Checkpoint every 10,000 steps |
| `tensorboard_td3/` | TensorBoard logs |

---

## Monitoring

```bash
tensorboard --logdir src/marble_balancer/rl_training/tensorboard_td3/
```

---

## Training for Generalization

Two optional training flags build a **single model** that generalizes across scenarios:

### TCP Lissajous Disturbance (`--tcp-lissajous true`)

When enabled, the TCP (end-effector) traces a Lissajous figure-eight during training — identical to the `tcp_lissajous_node.py` motion used at deployment.

**Key design decision: 50% per-episode activation (not always-on)**

Each episode independently activates TCP Lissajous with 50% probability:
```python
self._tcp_episode_active = (np.random.random() < 0.5)
```

If always-on, the model would only learn to balance under TCP motion. If always-off, the model wouldn't generalise to TCP Lissajous at deployment. The 50/50 split trains a single model on both distributions simultaneously.

The TCP phase is also randomised at each episode reset (not always starting at zero), so the model sees all phases of the curve.

**TCP velocity injected into the servo twist command each step:**
```
vx = A_x * ω_fa * cos(ω_fa * t + δ)
vy = A_y * ω_fb * cos(ω_fb * t)
```
where amplitude and period are **domain-randomised per episode**: `A_x = A_y ~ U(0.20, 0.40) m`, `period ~ U(10, 15) s`, `fa=1`, `fb=2`, `δ = π/2`. Deployment uses 0.30 m / 12 s (within the training range). The starting phase `t₀` is also randomised so the agent sees all cycle positions.

### Random Marble Spawn (`--spawn-radius R`)

When `spawn_radius > 0`, each episode spawns the marble at a uniformly random position within a disc of radius `R` metres from the plate centre:

```python
angle  = uniform(0, 2π)
radius = R * sqrt(uniform(0, 1))   # sqrt for uniform disc (not biased to centre)
offset = [radius*cos(angle), radius*sin(angle)]  # plate-frame
```

The plate-frame offset is converted to world coordinates using the plate TCP rotation matrix (TF quaternion → 3×3 rotation; plate X/Y are columns 0 and 1 of R).

**Recommended:** `--spawn-radius 0.12` (12 cm, within the 20 cm plate half-size limit).

### Combined command

```bash
ros2 launch marble_balancer rl_training.launch.py timesteps:=1000000 tcp_lissajous:=true spawn_radius:=0.12 seed_steps:=40000

# For unattended / overnight training — suppress Gazebo GUI:
ros2 launch marble_balancer rl_training.launch.py timesteps:=1000000 tcp_lissajous:=true spawn_radius:=0.12 seed_steps:=40000 headless:=true
```

This produces one model that handles: stationary TCP, moving TCP (any amplitude 20–40 cm / period 10–15 s), centred spawn, and off-centre spawn — 1M steps recommended for the harder task.

---

## CMA-ES Validator

After training, run CMA-ES to check TD3 isn't stuck in a local optimum:

```bash
cd src/marble_balancer/rl_training
python train_cmaes_gazebo.py --td3-model models_td3/best_model_td3.zip
```

Optimises a **linear policy** against `ball_plate_env.py` (28-D offline obs, 56 params). Note: the offline env does not include TCP velocity or phase encoding (it's a simplified physics model), so CMA-ES validates the LQR+residual structure but does not fully replicate the 36-D training obs. If CMA-ES finds a reward >10% better than TD3, TD3 may be in a local optimum and needs retraining. Defaults to `--offline` mode (~minutes). Use `--no-offline` for live Gazebo (~100 hours).

---

## Deployment

```bash
ros2 launch marble_balancer servo_balancer.launch.py \
  rl:=true \
  rl_model:=$(pwd)/src/marble_balancer/rl_training/models_td3/best_model_td3.zip \
  rl_norm:=$(pwd)/src/marble_balancer/rl_training/models_td3/running_stats.pkl \
  rl_stage:=2
```

`rl_residual_node.py` loads the TD3 model and `running_stats.pkl`, subscribes to `/marble/lqr_state`, and publishes the combined LQR+residual twist to `/marble_servo_rl/delta_twist_cmds`. The mux_controller routes this to MoveIt Servo when `rl:=true`.
