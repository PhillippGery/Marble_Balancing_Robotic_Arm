# Reinforcement Learning Methods — Marble Balancing Robotic Arm

## Overview

The control architecture uses a **residual RL** strategy: a learned policy outputs a small correction (Δω) that is *added on top of* an existing discrete-time LQR baseline. This means the RL agent never needs to learn basic stabilisation from scratch — the LQR handles that — and only needs to learn the residual correction that the linear controller cannot provide (e.g. responding to TCP Lissajous disturbances, nonlinear friction, model mismatch).

```
marble_odom + joint_states
        │
        ▼
  [LQR Controller]  ──── ω_lqr ────►  [+ residual] ──► MoveIt Servo ──► UR5e
                                             ▲
                                      [RL Policy]
                                      (Δω_alpha, Δω_beta)
```

The combined command is always clipped to `±MAX_RATE` (60 °/s in Gazebo, 45 °/s in the offline env), preserving the safety envelope of the LQR while allowing the policy to correct it.

---

## 1. State Vector and Physics Model

All RL environments share the same 8-dimensional state vector, matching `marble_servo_controller.py` exactly:

```
s = [x, vx, y, vy, alpha, omega_alpha, beta, omega_beta]
```

| Index | Variable | Description |
|-------|----------|-------------|
| 0 | `x` | Marble X position on plate (m) |
| 1 | `vx` | Marble X velocity (m/s) |
| 2 | `y` | Marble Y position on plate (m) |
| 3 | `vy` | Marble Y velocity (m/s) |
| 4 | `alpha` | Plate pitch angle (rad) — controls X marble dynamics |
| 5 | `omega_alpha` | Plate pitch rate (rad/s) |
| 6 | `beta` | Plate roll angle (rad) — controls Y marble dynamics |
| 7 | `omega_beta` | Plate roll rate (rad/s) |

The underlying dynamics follow a linearised ball-on-plate model with a PT1 servo delay:

```
ẍ = -C·alpha + friction(vx) + tcp_disturbance_x
ÿ = -C·beta  + friction(vy) + tcp_disturbance_y
alpha_dot  = omega_alpha
omega_alpha_dot = (u_alpha - omega_alpha) / T_ROBOT
```

where `C ≈ 7.0 m/s²` (gravity-scaled coupling constant), `T_ROBOT = 0.35 s` (PT1 servo time constant), and friction is `tanh`-smoothed Coulomb + viscous.

The LQR is computed by solving the discrete algebraic Riccati equation (DARE) using SciPy, with ZOH discretisation at 30 Hz.

---

## 2. Offline SAC on the Fast Physics Environment

### Motivation

Training directly in Gazebo is slow — each episode requires spawning a marble, homing the arm, and running at real-time speed. The offline environment (`ball_plate_env.py`) reimplements the same PT1 physics model in pure Python, running **100–1000× faster** with no ROS2 dependency. This made it practical to prototype the reward function, curriculum, and observation design before committing to expensive Gazebo training.

### Algorithm: SAC

Soft Actor-Critic (SAC) was chosen for the offline phase because:
- It is off-policy and sample-efficient, well-suited to the parallelised vectorised environment.
- Its entropy-regularised objective encourages exploration, which is useful during curriculum stage 0 when the marble is rarely perturbed and the LQR alone nearly solves the task.
- Stable-Baselines3's `VecNormalize` wrapper handles observation normalisation automatically across multiple parallel environments.

**Hyperparameters (`train.py`):**

| Parameter | Value |
|-----------|-------|
| Learning rate | 3 × 10⁻⁴ |
| Replay buffer size | 500 000 |
| Batch size | 256 |
| Soft update τ | 0.005 |
| Discount γ | 0.99 |
| Entropy coefficient | `auto` (learned) |
| Network architecture | MLP [256, 256] |
| Parallel envs | 4 (default) |
| Observation normalisation | `VecNormalize` (clip ±5σ, reward normalised too) |

### Observation Space — 28-D

```
obs = [state_norm(8), action_history_flat(20)]
```

| Slice | Content | Normalisation |
|-------|---------|---------------|
| `[0:8]` | Physical state `s` | Divided by `[0.20, 0.50, 0.20, 0.50, 0.30, MAX_RATE, 0.30, MAX_RATE]`, clipped to ±3 |
| `[8:28]` | Last 10 actions, oldest-first, flattened | Raw `[-1, 1]` — no further scaling |

The action history lets the agent reason about its own recent behaviour without requiring recurrent architectures.

### Action Space — 2-D

```
action ∈ [-1, 1]²   →   residual = action × λ   (rad/s)
```

`λ` (the clip budget) grows with curriculum stage: ±5 → ±10 → ±15 °/s.

### Reward Function — Offline

```python
r  = -10.0 * (x_err² + y_err²)           # centering
r -= 0.5   * (vx² + vy²)                  # velocity damping
r -= 0.05  * ||residual||²                # effort penalty
r += 0.1                                  # survival bonus per step
if terminated:
    r -= 100.0                            # fall-off penalty
```

This is a simple quadratic centering reward. The effort penalty keeps residuals small (the LQR should do most of the work). The survival bonus gives a learning signal even when the marble stays near the centre (otherwise sparse reward).

### Curriculum — 3 Stages (Reward-Based)

Advancement is triggered when the mean episode reward over the last 20 evaluations exceeds a threshold:

| Stage | λ clip | Lissajous | Perturbations | Advance threshold |
|-------|--------|-----------|---------------|-------------------|
| 0 | ±5 °/s | off | off | reward > −40 |
| 1 | ±10 °/s | off | random impulse (0.5% chance/step) | reward > −15 |
| 2 | ±15 °/s | on (TCP sinusoidal forces) | on | — (final) |

Domain randomisation at each episode reset:
- `C` ∈ [6.5, 7.5] m/s²
- `T_ROBOT` ∈ [0.25, 0.45] s
- Coulomb friction `μ_c` ∈ [0.0, 0.04]
- Viscous friction `μ_v` ∈ [0.0, 0.08]

This makes the policy robust to model mismatch between the offline env and Gazebo.

### Limitation: Sim-to-Gazebo Gap

The offline environment uses linearised dynamics with Euler integration at 30 Hz. Gazebo applies full rigid-body physics at ~1000 Hz. The SAC model trained offline transfers partially but cannot handle the higher-fidelity Gazebo dynamics without further online fine-tuning. This was the primary motivation for moving to online TD3 training.

---

## 3. Online SAC Against Live Gazebo

### Motivation

The offline SAC policy showed clear improvement on the PT1 physics sim but degraded noticeably when deployed to Gazebo — particularly on the Y axis under TCP Lissajous disturbance. The simplified physics (linearised dynamics, Euler integration, no collision geometry) could not capture the effects present in Gazebo: marble bounce and slip, joint velocity noise, TF timing jitter, and the full UR5e kinematics. The logical next step was to keep SAC as the algorithm but move training fully online into Gazebo, giving the policy access to the real simulator dynamics.

`GazeboRLEnv` (`gazebo_rl_env.py`) implements a Gymnasium environment wrapping live Gazebo. The env is the controller during training — it replaces `marble_servo_controller`, reads real odometry and joint states, and publishes twist commands to MoveIt Servo directly. SAC was plugged into this env using the same observation and reward structure as the offline phase.

### Issues Encountered with Online SAC

Several problems emerged when running SAC online in a single Gazebo instance:

1. **No parallel environments.** SAC's sample efficiency advantage comes partly from running many vectorised environments simultaneously (the offline setup used 4 parallel `BallPlateEnv` instances). A single Gazebo instance cannot be parallelised — training is strictly sequential, one transition per real-time 30 Hz step. SAC's higher sample efficiency per gradient step does not compensate for this bottleneck.

2. **Entropy regularisation conflicts with RLPD seeding.** Pre-filling the buffer with pure-LQR transitions (zero residual) is the natural warm-start for this problem. For SAC, these zero-residual transitions have near-zero entropy, which drives the automatic entropy coefficient toward zero during early training and suppresses exploration exactly when it is most needed.

3. **VecNormalize is not compatible with a single online env.** The offline SAC relied on `VecNormalize` across multiple parallel envs. Online, observation statistics must be tracked incrementally with a running mean/std — a simpler scheme that SAC's training loop does not integrate with natively.

4. **Stochastic policy at a fixed frequency.** The 30 Hz control loop requires low-latency, consistent commands. SAC's stochastic policy samples from a learned distribution at every step, producing more variance in commands than is desirable for real-time servo control. Early in training this caused chattering on the Y axis that the marble oscillated with rather than against.

5. **Q-function instability.** With noisy Gazebo observations (odom at 50 Hz, joint states at 100 Hz, 30 Hz servo loop), the critic learned a high-variance Q-function early on. SAC's actor update is applied every step, meaning the policy immediately exploited unstable Q estimates, producing large residuals and frequent early terminations.

The combination of these issues meant online SAC did not converge reliably within a reasonable number of training steps. Curriculum stage 0 survival fraction plateaued below 40% and did not advance within several hundred thousand steps.

---

## 4. Online TD3 Against Live Gazebo

### Motivation

After observing that both offline SAC and online SAC failed to converge reliably in the Gazebo environment, training was switched to TD3. The same `GazeboRLEnv` is reused — only the algorithm changes. TD3 addresses all five failure modes identified with online SAC.

### Why TD3 Over Online SAC

TD3 directly addresses each of the five failure modes encountered with online SAC:

1. **No parallel env requirement.** TD3 is designed for single-environment off-policy training with a replay buffer. Its sample efficiency comes from the replay buffer and delayed updates rather than parallelisation, making it a natural fit for a single live Gazebo instance.

2. **RLPD seeding is natural.** TD3's deterministic off-policy loop works cleanly with zero-residual LQR pre-seeding. There is no entropy objective to conflict with — the buffer simply stores transitions and TD3 learns from them. The 40 000-step pure-LQR warm-start gives the critic a stable value baseline before any RL exploration begins.

3. **Delayed policy updates reduce overestimation.** TD3 updates the actor only every `policy_delay=2` critic steps. In an online Gazebo loop with noisy 50 Hz odometry and 30 Hz servo commands, the Q-function initially has high variance. Delaying actor updates prevents the policy from over-exploiting early, inaccurate value estimates — the exact failure mode that caused online SAC to issue large residuals early in training.

4. **Target policy smoothing.** TD3 adds clipped Gaussian noise to the target action (`σ=0.2, clip=0.5`) during critic updates, regularising the value function against sharp Q peaks. This is particularly relevant for the narrow plate geometry where falling off is catastrophic and terminal Q values are very negative.

5. **Deterministic policy at deployment.** The `rl_residual_node` calls `model.predict(obs, deterministic=True)`. SAC's stochastic policy adds variance at inference time with no benefit; TD3's deterministic actor is directly deployable.

**Hyperparameters (`train_td3_gazebo.py`):**

| Parameter | Value |
|-----------|-------|
| Learning rate | 1 × 10⁻⁴ |
| Replay buffer size | 200 000 |
| Batch size | 256 |
| Soft update τ | 0.005 |
| Discount γ | 0.99 |
| Policy delay | 2 (actor updated every 2nd critic step) |
| Target policy noise | σ=0.2, clip=0.5 |
| Exploration noise | `NormalActionNoise` σ=0.1 |
| Network architecture | MLP [256, 256] |
| Observation normalisation | `RunningMeanStd` (Welford online, manual) |

### RLPD Buffer Seeding

Before online training begins, the replay buffer is pre-filled with **40 000 steps of pure LQR behaviour** (zero residual). This is the Replay-guided Learning from Prior Demonstrations (RLPD) technique:

- The LQR is near-optimal for the nominal system. Seeding gives TD3 a distribution of stable, informative transitions from the start.
- The running normalisation statistics (`RunningMeanStd`) are updated *during* seeding so that the normaliser sees the LQR operating regime (low residuals, small marble displacements).
- The seeded buffer is never cleared — as online transitions accumulate, the LQR demonstrations are naturally diluted, allowing the policy to specialise beyond pure LQR.

```
Phase 1 — 40 000 steps:  pure LQR  (action = zeros)  →  fill replay buffer, build normaliser stats
Phase 2 — 500 000 steps:  TD3 exploration  →  learn residual corrections on top of LQR
```

### Observation Space — 36-D (Gazebo)

The Gazebo observation extends the offline 28-D space with two additional groups:

```
obs = [state_norm(8), action_history_flat(20), twist_cmd_norm(4), lissajous_phase(4)]
```

| Slice | Content | Normalisation |
|-------|---------|---------------|
| `[0:8]` | Physical state `s` | Divided by `[0.20, 0.50, 0.20, 0.50, 0.30, MAX_RATE, 0.30, MAX_RATE]`, clipped ±3 |
| `[8:28]` | Last 10 actions oldest-first | Raw `[-1, 1]` |
| `[28:32]` | `[ω_beta_cmd, ω_alpha_cmd, tcp_vx, tcp_vy]` | Divided by `[MAX_RATE, MAX_RATE, 0.20, 0.35]`, clipped ±3 |
| `[32:36]` | `[sin(φ_x), cos(φ_x), sin(φ_y), cos(φ_y)]` | Already in `[-1, 1]`; zeros when TCP is stationary |

**Twist command context `[28:32]`** tells the agent what the LQR is currently commanding and what TCP velocity is active. This lets the policy choose residuals that complement rather than fight the LQR.

**Lissajous phase encoding `[32:36]`** encodes the current phase of the TCP Lissajous trajectory as `sin/cos` pairs. Using sine and cosine avoids the 0/2π discontinuity. The agent can use these four values to *anticipate* upcoming TCP direction reversals and pre-correct on the Y axis before the marble displacement becomes large.

```
φ_x = fa · ω₀ · t + δ      (fa=1, ω₀=2π/15 rad/s, δ=π/2)
φ_y = fb · ω₀ · t           (fb=2, twice the frequency)
```

When TCP Lissajous is not active in the current episode, all four phase values are zero — a clear signal to the agent.

### Reward Function — Gazebo TD3

The Gazebo reward uses **dual-scale exponential centering** instead of the quadratic used offline. This provides:
- A broad gradient signal (`exp(-30·r²)`) when the marble is far from centre, preventing gradient vanishing for large displacements
- A sharp precision signal (`exp(-300·r²)`) near the origin, encouraging the agent to tighten around centre

```python
pos  = (0.7 * exp(-30.0  * (x² + 3.0 * y²))
      + 0.3 * exp(-300.0 * (x² + 3.0 * y²)))

# Extra Y bonus during TCP Lissajous episodes
if tcp_episode_active:
    pos += 0.4 * exp(-300.0 * y²)

vel    = -0.1 * vx² - 0.40 * vy²
smooth = -0.02 * ||action - prev_action||²
surv   = 0.05

reward = pos + vel + smooth + surv
if terminated:
    reward -= 50.0
```

**Y-axis asymmetry:** Y is weighted 3× in the position term and 4× in velocity damping relative to X. This compensates for the TCP Lissajous frequency structure: the Y component runs at `fb=2` (twice X's frequency), producing approximately 4× the pseudo-force on the marble. Without asymmetry the agent under-corrects on Y.

**Smoothness penalty** (`-0.02 * ||Δaction||²`) penalises large action changes between consecutive steps. This discourages the chattering behaviour seen in early training where the agent would oscillate between large positive and negative residuals.

**What was removed — potential shaping:** An earlier reward version included potential shaping from the Riccati matrix P (`Φ(s) = -s^T P s`). This was removed because the P-matrix produces very large terminal values, which caused critic loss spikes of 1000–4000 during early training and prevented convergence. The dual-scale exponential reward provides sufficient gradient signal without the instability.

### Curriculum — 3 Stages (Survival-Based)

For Gazebo training, curriculum advancement is based on **survival fraction** (fraction of episodes lasting ≥590 of 600 steps) rather than raw reward. Survival fraction is more consistent across stages because the reward scale changes with λ (larger residual budget → larger potential rewards and penalties).

| Stage | λ clip | TCP Lissajous | Advance threshold |
|-------|--------|---------------|-------------------|
| 0 | ±5 °/s | 0% | survival fraction > 40% (last 20 eps) |
| 1 | ±10 °/s | 0% | survival fraction > 65% |
| 2 | ±15 °/s | 70% | — (final stage) |

### TCP Lissajous Generalisation

When `tcp_lissajous:=true` is passed at launch, each episode independently activates TCP motion with probability 0.7. This means the agent experiences a mix of stationary-TCP and moving-TCP episodes within a single training run — a single model generalises to both deployment scenarios without separate policies.

At each TCP-active episode reset, amplitude and period are also domain-randomised:
- Amplitude: 20–40 cm (deployment uses 30 cm)
- Period: 10–20 s (deployment uses 15 s)
- Starting phase: random, so the agent sees all cycle positions equally

### Random Marble Spawn

`spawn_radius:=0.12` spawns the marble uniformly within 12 cm of the plate centre at each episode reset. This prevents the agent from overfitting to a single initial condition (centre spawn) and forces it to recover from off-centre starts.

---

## 4. CMA-ES — Local Optimum Detection

### Role

TD3 is a gradient-based method operating in a high-dimensional parameter space (the neural network weights). Gradient methods can converge to local optima, especially in the noisy Gazebo environment. CMA-ES is used as an **independent validator**: it optimises a simple linear policy from scratch using evolution and compares its best achievable reward to the TD3 baseline.

If CMA-ES finds a reward more than 10% higher than TD3, this is a warning that TD3 may be stuck in a local optimum and should be restarted or given a different initialisation.

### How CMA-ES Works

Covariance Matrix Adaptation Evolution Strategy (CMA-ES) is a **gradient-free** evolutionary optimisation algorithm. At each iteration it:

1. Samples a population of `popsize` candidate parameter vectors from a multivariate Gaussian: `x ~ N(m, σ² C)` where `m` is the current mean, `σ` the step size, and `C` the covariance matrix.
2. Evaluates each candidate's fitness (mean episode reward over `n_eval_eps` episodes).
3. Updates `m` toward the better-performing candidates (weighted recombination).
4. Adapts `C` to capture the shape of the fitness landscape (the covariance matrix learns which parameter directions improve reward fastest).
5. Adapts `σ` using a cumulative step-size adaptation rule to prevent premature convergence.

The algorithm is particularly effective for problems with up to a few hundred parameters and does not require gradients, making it robust to noise and discontinuities.

### Linear Policy

The CMA-ES policy is intentionally kept simple — a linear map with tanh squash:

```
action = tanh(W @ obs + b)
W: (2 × 28)  weight matrix
b: (2,)      bias
Total parameters: 58
```

A 58-parameter linear policy is simple enough for CMA-ES to optimise quickly yet expressive enough to achieve reasonable performance on the ball-plate task. It serves as a lower bound on what a structured approach can achieve — if the TD3 network (with ~100 000 parameters) cannot beat a 58-parameter linear policy, something is wrong with the TD3 training.

### Configuration

| Parameter | Value | Notes |
|-----------|-------|-------|
| `sigma0` | 0.5 | Initial step size |
| `popsize` | 8 | Candidates per iteration |
| `max_iters` | 100 | ~800 total evaluations |
| `n_eval_eps` | 3 | Episodes per candidate |
| Mode | offline (default) | Uses `ball_plate_env.py` (~1000× faster) |

**Wall-time note:** Running CMA-ES against live Gazebo (`--no-offline`) would take ~100 hours (8 × 100 × 3 × ~150 s/episode). The offline mode completes in minutes.

### Interpretation

```
CMA-ES best ≤ TD3 baseline × 1.1   →  TD3 validated, no significant local optimum
CMA-ES best > TD3 baseline × 1.1   →  WARNING: TD3 may be trapped — consider retraining
```

---

## 5. Deployment Architecture

The trained TD3 policy is deployed as `rl_residual_node.py`, a ROS2 node that runs alongside the standard LQR controller:

```
marble_servo_controller
    │  /marble/lqr_state  (8-D state + 2-D LQR output)
    │  /marble_servo/delta_twist_cmds  (raw LQR twist)
    ▼
rl_residual_node
    │  builds 36-D obs, calls TD3.predict()
    │  computes residual = action × λ_clip
    │  adds residual to LQR twist, re-clips to MAX_RATE
    ▼
/marble_servo_rl/delta_twist_cmds
    ▼
mux_controller  →  /servo_node/delta_twist_cmds  →  MoveIt Servo
```

The mux subscribes to `/marble_servo_rl/` instead of `/marble_servo/` when `rl:=true`, routing the combined LQR+RL command to MoveIt Servo while still allowing manual override.

**Normalisation at deployment:** The `running_stats.pkl` file (Welford mean/variance built during RLPD seeding and online training) normalises observations identically to training:
```python
obs_normalised = clip((obs - mean) / sqrt(var + 1e-8), -3, 3)
```

---

## 6. Summary of Methods

| Method | Environment | Algorithm | Obs dims | Key purpose |
|--------|-------------|-----------|----------|-------------|
| Offline SAC | `ball_plate_env.py` (fast PT1 sim) | SAC + VecNormalize | 28-D | Fast reward/curriculum prototyping |
| Online SAC | `gazebo_rl_env.py` (live Gazebo) | SAC | 32-D | Bridge offline → real dynamics; revealed SAC limitations in single-env online setting |
| Online TD3 | `gazebo_rl_env.py` (live Gazebo) | TD3 + RLPD seeding | 36-D | High-fidelity converged policy |
| CMA-ES | `ball_plate_env.py` (offline) | Evolutionary, 58-param linear | 28-D | Validate TD3 against local optima |

### Why This Progression

1. **Offline SAC first** — fast iteration to tune reward shaping, curriculum thresholds, and observation design without waiting on Gazebo spawn/home cycles (~5 s per episode). VecNormalize across 4 parallel envs makes this practical.

2. **Online SAC second** — the offline model degraded in Gazebo due to sim-to-real gap (linearised dynamics vs full rigid-body physics). Moving SAC online was the logical first attempt. This revealed that SAC's design assumptions (parallel envs, entropy regularisation, VecNormalize) are a poor fit for a single live Gazebo instance and identified the specific failure modes that needed addressing.

3. **Online TD3 third** — switches to an algorithm whose design assumptions match the single-env online setting exactly: replay buffer warm-start compatible with RLPD, delayed updates for noisy Q-functions, and a deterministic policy. Same `GazeboRLEnv` — only the training script changes.

4. **CMA-ES as a check** — ensures the trained TD3 policy is not simply a local optimum that a much simpler linear policy could match or beat.

---

## Appendix: File Reference

| File | Purpose |
|------|---------|
| `rl_training/ball_plate_env.py` | Fast offline Gymnasium env (PT1 physics, domain rand) |
| `rl_training/train.py` | Offline SAC training script |
| `rl_training/eval.py` | Evaluate SAC model, plot trajectories |
| `rl_training/gazebo_rl_env.py` | Online Gazebo Gymnasium env (wraps live ROS2) |
| `rl_training/train_td3_gazebo.py` | Online TD3 training with RLPD seeding |
| `rl_training/train_cmaes_gazebo.py` | CMA-ES local optimum validator |
| `marble_balancer/rl_residual_node.py` | ROS2 deployment node (TD3 inference at 30 Hz) |
| `marble_balancer/lqr_math.py` | LQR gain computation, shared by all components |
