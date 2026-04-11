# RL Improvement Plan — TCP Lissajous Balancing

## Status: All Changes Implemented ✓

All items below have been applied to the codebase. A new training run is required to produce the improved model.

---

## Context

The v1 TD3 residual controller improves Y-axis tracking (RMS Y: 4.0 cm → 2.5 cm, β saturation: 23.5% → 3.9%) but several bugs and missing features were limiting further improvement. This document records what was found and what was changed.

**Target after retraining (v2 model):** RMS Y < 2.0 cm, β saturation < 3%.

---

## Bugs Fixed

### Bug 1 — MAX_RATE Mismatch (deployment only, no retraining needed)
**File:** `marble_balancer/rl_residual_node.py:46`

Training used `MAX_RATE = deg2rad(60.0)` but the deployment node had `math.radians(45.0)`. This caused wrong normalisation for obs indices [5], [7] (ωα, ωβ) and [28], [29] (angular twist commands) at deployment.

**Fixed:** Changed to `math.radians(60.0)`. This fix benefits the existing model immediately — no retraining required.

---

### Bug 2 — TCP Velocity Normalization (saturated at ±3 every step)
**File:** `rl_training/gazebo_rl_env.py:71` and `marble_balancer/rl_residual_node.py:48`

With 0.30 m amplitude and 12 s period (as deployed), max tcp_vy ≈ 0.314 m/s. The old normalizer was 0.05 m/s → clipped to ±3 every single step. The agent effectively learned to ignore the TCP velocity signal entirely.

| | Training | Normaliser | Result |
|--|--|--|--|
| Max tcp_vx | 0.157 m/s | was 0.05 → **now 0.20** | was 3.14 → now 0.78 |
| Max tcp_vy | 0.314 m/s | was 0.05 → **now 0.35** | was 6.28 → now 0.90 |

**Fixed:** `_TWIST_NORM = np.array([MAX_RATE, MAX_RATE, 0.20, 0.35])` in both files.

---

## New Features Added

### 1. Lissajous Phase Encoding (obs 32D → 36D) ✓
**Files:** `gazebo_rl_env.py` (`_get_obs()`, `observation_space`), `rl_residual_node.py` (`_build_obs()`, phase counter)

TCP velocity alone tells the agent the current speed but not *where* it is in the cycle. Phase encoding lets the agent anticipate direction reversals and pre-correct rather than react.

Added `[sin(φ_x), cos(φ_x), sin(φ_y), cos(φ_y)]` as obs[32:36]:
- `φ_x = fa * ω₀ * t + δ`
- `φ_y = fb * ω₀ * t`

All four values are **0.0** when `tcp_episode_active = False` (agent learns zeros = no disturbance).

`rl_residual_node.py` tracks its own `_tcp_t` counter (reset at each marble landing), using the deployment parameters (ω₀ = 2π/12, fa=1, fb=2, δ=π/2).

---

### 2. Asymmetric Y-Axis Reward ✓
**File:** `gazebo_rl_env.py` `_compute_reward()` (~line 500)

TCP Lissajous drives Y at 2× frequency (fb=2) → ~4× pseudo-force vs X. Reward now reflects this:

```python
# Before:
pos = exp(-50 * (x² + y²))
vel = -0.1 * (vx² + vy²)

# After:
pos = exp(-50 * (x² + 2.5*y²))   # Y weighted 2.5×
vel = -0.1 * vx² - 0.25 * vy²   # Y velocity penalised 2.5×
```

The P matrix for potential shaping already implicitly weights Y (Q[2]=200, Q[3]=400), so shaping reinforces this further.

---

### 3. Domain Randomisation of TCP Parameters ✓
**File:** `gazebo_rl_env.py` `reset()` method

Each episode now samples a random amplitude and period:
```python
amp    ~ U(0.20, 0.40) m      # 20–40 cm
period ~ U(10.0, 15.0) s      # 10–15 s
```
Deployment (0.30 m / 12 s) is within the training range. Forces generalisation rather than memorising one fixed trajectory. Starting phase is also randomised each episode.

---

### 4. Increased RLPD Seeding Steps ✓
**File:** `train_td3_gazebo.py` line ~343

Default seed steps: 20,000 → **40,000**.

40K steps ≈ 66+ full Lissajous cycles at 12 s period → thorough warm-start buffer covering the full disturbance range.

---

### 5. Raised Curriculum Thresholds ✓
**File:** `train_td3_gazebo.py` line 60

`STAGE_THRESHOLDS`: `[0.30, 0.55]` → **`[0.40, 0.65]`**

Harder task (TCP + asymmetric reward + domain randomization) needs more time at each stage.

---

### 6. RunningMeanStd Updated During Seeding ✓
**File:** `train_td3_gazebo.py` seeding loop (~line 200)

`rms.update(obs)` is now called during seeding. TCP velocities are no longer saturated after the normalizer fix, so including them in the stats improves normalisation quality during Phase 2.

---

## Updated Observation Layout (36-D)

| Indices | Content | Dims |
|---------|---------|------|
| 0:8 | Normalised plant state `[x, vx, y, vy, α, ωα, β, ωβ]` | 8 |
| 8:28 | Action history (last 10 actions × 2-D) | 20 |
| 28:32 | Twist cmd `[ωβ_cmd, ωα_cmd, tcp_vx, tcp_vy]` | 4 |
| 32:36 | Lissajous phase `[sin(φ_x), cos(φ_x), sin(φ_y), cos(φ_y)]` | 4 |
| **Total** | | **36** |

When `tcp_episode_active = False`: indices 30:36 are all 0.0.

---

## Files Modified

| File | Changes |
|------|---------|
| `rl_training/gazebo_rl_env.py` | _TWIST_NORM fix; obs 32→36D; phase encoding in `_get_obs()`; asymmetric reward; domain-randomise TCP in `reset()` |
| `marble_balancer/rl_residual_node.py` | MAX_RATE 45°→60°; _TWIST_NORM fix; 36D obs; phase counter + sin/cos in `_build_obs()` |
| `rl_training/train_td3_gazebo.py` | seed-steps default 20K→40K; STAGE_THRESHOLDS [0.40, 0.65]; rms.update() during seeding |

---

## Recommended Training Run

```bash
ros2 launch marble_balancer rl_training.launch.py timesteps:=1000000 tcp_lissajous:=true spawn_radius:=0.12 seed_steps:=40000

# For unattended runs — suppress Gazebo GUI:
ros2 launch marble_balancer rl_training.launch.py timesteps:=1000000 tcp_lissajous:=true spawn_radius:=0.12 seed_steps:=40000 headless:=true
```

---

## Verification Checklist

1. Confirm obs shape is `(36,)`: check env startup log prints `observation_space: Box(-3.0, 3.0, (36,), float32)`
2. Confirm env log shows varying amplitude/period each episode: `amp=XX.Xcm  period=XX.Xs`
3. TensorBoard: `rollout/ep_rew_mean` trending up; `curriculum/stage` reaches 2 by ~400K steps; `curriculum/survival_fraction` reaches >0.65 before stage 2
4. Deployment metric targets vs v1 baseline: RMS Y < 2.0 cm (was 2.5 cm), β saturation < 3% (was 3.9%)

---

## Reference: Performance Baseline

| Metric | v1 RL model | Without RL | v2 target |
|--------|-------------|------------|-----------|
| RMS error X | 1.8 cm | 1.8 cm | ~1.8 cm |
| RMS error Y | **2.5 cm** | **4.0 cm** | **< 2.0 cm** |
| Max \|y\| | 3.4 cm | 8.2 cm | < 3.0 cm |
| β saturation | 3.9% | 23.5% | < 3% |

Measured on ~168 s runs with `tcp_lissajous:=true`.
