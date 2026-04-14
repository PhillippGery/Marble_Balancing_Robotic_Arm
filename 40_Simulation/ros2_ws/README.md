# Marble Balancing Robotic Arm — ROS2 Simulation

Real-time LQR control of a **UR5e 6-DOF robotic arm** balancing a marble on a plate attached to the end-effector. The arm tilts the plate using angular velocity commands via MoveIt2 Servo, keeping the marble centered. Runs in **Gazebo Classic** simulation with hardware-ready architecture.

## ⭐ What's New (V2 — Current)

**Production-ready RL training environment with bulletproof spawning and full TensorBoard visibility:**

✅ **gazebo_rl_env_v2.py** — 21-D observation space, atomic reset retry loop, velocity braking, instant landing detection, ghost marble detection, reward lockout  
✅ **train_td3_gazebo_v2.py** — Monitor wrapper for episode metrics, absolute paths, logger before seeding, model.learn() with callbacks  
✅ **rl_training.launch.py** — `gui:=false` support for **300% faster headless training**  
✅ **TensorBoard** — All episode metrics now visible (episode_reward, episode_length, curriculum/stage, curriculum/survival_fraction)  
✅ **Model checkpoints** — Saved every 10K steps to `models_td3_v2/`, best model selected by evaluation reward  
✅ **Resume training** — Load any checkpoint and continue with `--load` argument  

**Quick start:**
```bash
ros2 launch marble_balancer rl_training.launch.py gui:=false timesteps:=1000000 tcp_lissajous:=true spawn_radius:=0.12
```

---

## System Overview

```
Gazebo (UR5e + marble)
        |
   /marble/odom (50 Hz)          /joint_states (100 Hz)
        |                               |
        +-----------> marble_servo_controller (30 Hz LQR + EKF)
                              |
                     /marble_servo/delta_twist_cmds
                              |
                       mux_controller  <── /manual/delta_twist_cmds
                              |
                    /servo_node/delta_twist_cmds
                              |
                        MoveIt2 Servo
                              |
                    JointTrajectoryController
                              |
                          UR5e joints
```

### Key Nodes

| Node | Purpose |
|------|---------|
| `marble_servo_controller` | 30 Hz LQR controller + EKF state estimator |
| `mux_controller` | Priority mux: manual overrides LQR, reverts after 0.5 s timeout |
| `go_to_pose` | Homes arm to flat balanced pose (direct joint-space command) |
| `marble_spawner` | Spawns marble above plate via TF lookup — no hardcoded coords |
| `marble_visualizer` | Live 2D window: marble trail, plate boundary, setpoint |
| `marble_plotter` | Records CSV + generates plots on marble fall-off or Ctrl-C |
| `marble_lissajous_node` | Publishes Lissajous figure-eight setpoints on `/marble/desired_pos` |
| `marble_square_node` | Publishes 4-corner square step-response pattern for LQR tuning |
| `tcp_lissajous_node` | Moves TCP along figure-eight while balancing; publishes feedforward tilt |
| `rl_residual_node` | Loads trained TD3 policy; adds residual correction to LQR output |
| `marble_spawner_xy` | Spawns marble at user-specified plate-frame (x, y) coordinates |

---

## Controller Design

### State Vector (8-D)

```
x = [x, vx, y, vy, alpha, omega_alpha, beta, omega_beta]

x, y          — marble position on plate (m)
vx, vy        — marble velocity (m/s)
alpha         — plate pitch (rad), governs X-axis ball motion:  a_x = -C·alpha
omega_alpha   — pitch rate (rad/s), PT1 actuator state
beta          — plate roll (rad), governs Y-axis ball motion:   a_y = -C·beta
omega_beta    — roll rate (rad/s), PT1 actuator state
```

### Nonlinear Physics Model

Full ball-on-plate dynamics (from Lagrangian, used in EKF process model):

```
v̇x = (mb·x·ωα² + mb·y·ωα·ωβ − mb·g·sin(α)) / (mb + Ib/rb²)
v̇y = (mb·y·ωβ² + mb·x·ωα·ωβ − mb·g·sin(β)) / (mb + Ib/rb²)
```

Linearised at the operating point (small angles, near centre):

```
a_ball = -C · theta_plate,    C = mb·g / (mb + Ib/rb²) ≈ 7.0 m/s²
```

Robot actuator lag modelled as PT1 with `T = 0.35 s`:

```
d(omega)/dt = (omega_cmd − omega) / T
```

### Discrete-Time LQR

Continuous system discretised via Zero-Order Hold (matrix exponential) at 30 Hz. Optimal gain `K` from the Discrete Algebraic Riccati Equation:

```
P  = solve_discrete_are(Ad, Bd, Q, R)
K  = inv(R + Bd'·P·Bd) @ Bd'·P·Ad
u  = -K @ (x − x_desired)        # applied at 30 Hz
```

**Default cost weights** (`lqr_math.py`):

```
Q = diag([400, 400, 100, 100, 5, 1, 5, 1])
          [ x,  vx,  y,  vy,  α, ωα, β, ωβ]
R = 3.0 · I₂
```

### Extended Kalman Filter (EKF)

`marble_ekf.py` implements an 8-D EKF using the full nonlinear ball-on-plate process model with RK4 integration. It is designed for **camera-based marble position** use on real hardware where odom is noisy.

- **Measurements**: `[x, y, alpha, beta]` from odometry and TF
- **Estimated**: `[vx, vy, omega_alpha, omega_beta]` — no encoder velocity needed
- **Process**: nonlinear Lagrangian dynamics + PT1 servo model
- **Covariance**: Euler propagation with analytical Jacobian

Toggle in `marble_servo_controller.py`:

```python
USE_EKF = True    # All 8 states from EKF (tune R for camera noise level)
USE_EKF = False   # EMA velocity + Jacobian omega (Gazebo / known-good baseline)
```

**Tuning for different sensors** (`marble_ekf.py → DEFAULT_R`):

```python
# Gazebo perfect odom (transparent — EKF barely filters):
DEFAULT_R = diag([1e-8, 1e-8, 1e-8, 1e-8])

# Real camera (~2 mm noise):
DEFAULT_R = diag([4e-6, 4e-6, 1e-8, 1e-8])

# Noisy camera (~5 mm noise):
DEFAULT_R = diag([2.5e-5, 2.5e-5, 1e-8, 1e-8])
```

---

## Requirements

- ROS2 Humble
- MoveIt2 (`moveit_ros`, `moveit_servo`)
- Gazebo Classic (`gazebo_ros_pkgs`)
- `ur_description`, `ur_simulation_gazebo`, `ur_moveit_config`
- Python: `numpy`, `scipy`

### Optional (RL training only)
```bash
pip install gymnasium stable-baselines3[extra] tensorboard
```

---

## Build

```bash
cd ~/Marble_Balancing_Robotic_Arm/40_Simulation/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## Run

### Basic balancing

```bash
ros2 launch marble_balancer servo_balancer.launch.py
```

### With data logging and plots

```bash
ros2 launch marble_balancer servo_balancer.launch.py plot:=true
```

Plots are generated automatically when the marble falls off. To trigger mid-run without stopping:

```bash
ros2 service call /marble/plot_now std_srvs/srv/Trigger {}
```

### Plot a saved CSV offline (no ROS needed)

```bash
# Using the installed binary
marble_plotter --plot ~/marble_logs/marble_20240101_120000.csv

# Or directly with Python
python3 src/marble_balancer/marble_balancer/marble_plotter.py \
  --plot ~/marble_logs/marble_20240101_120000.csv
```

The plotter produces two PNG files alongside the CSV and opens them automatically:
- `*_omega.png` — commanded vs EKF-estimated vs Jacobian plate angular velocity
- `*_step.png` — marble x/y vs setpoint, plate angles, tracking error with RMS

### Marble trajectory tracking

```bash
# Figure-eight setpoint
ros2 launch marble_balancer servo_balancer.launch.py lissajous:=true plot:=true

# 4-corner square step-response (LQR tuning)
ros2 launch marble_balancer servo_balancer.launch.py square:=true plot:=true

# Custom square parameters
ros2 launch marble_balancer servo_balancer.launch.py square:=true \
  --ros-args -p marble_square_node:half_side:=0.08 \
             -p marble_square_node:dwell_time:=4.0
```

### TCP motion while balancing

```bash
ros2 launch marble_balancer servo_balancer.launch.py tcp_lissajous:=true plot:=true
```

### TD3 residual controller (deploy trained model)

```bash
ros2 launch marble_balancer servo_balancer.launch.py rl:=true rl_model:=$(pwd)/src/marble_balancer/rl_training/models_td3/best_model_td3.zip rl_norm:=$(pwd)/src/marble_balancer/rl_training/models_td3/running_stats.pkl rl_stage:=2
```

### Spawn marble at specific position

```bash
ros2 launch marble_balancer spawn_xy.launch.py x:=0.10 y:=0.0
ros2 launch marble_balancer spawn_xy.launch.py x:=-0.12 y:=0.08 plot:=true
# With RL:
ros2 launch marble_balancer spawn_xy.launch.py x:=0.10 y:=0.0 rl:=true rl_model:=$(pwd)/src/marble_balancer/rl_training/models_td3/best_model_td3.zip rl_norm:=$(pwd)/src/marble_balancer/rl_training/models_td3/running_stats.pkl
```

Spawns the marble at the given plate-frame (x, y) offset from centre (clamped to ±0.18 m). Useful for testing controller robustness at different initial positions.

---

## LQR Tuning

All weights in `marble_balancer/lqr_math.py`. Rebuild after any change:

```bash
colcon build --symlink-install --packages-select marble_balancer
```

| Index | State | Effect of increasing Q |
|-------|-------|----------------------|
| 0, 2 | x, y | Tighter centering, more overshoot risk |
| 1, 3 | vx, vy | More damping — raise to fix oscillation |
| 4, 6 | α, β | Plate tilts less, slower response |
| 5, 7 | ωα, ωβ | Smoother rate changes |

Increase `R` (scalar multiplier) to reduce overall aggressiveness on both axes.

Use `square:=true` for step-response tuning — the 4-corner pattern gives clear rise time, overshoot, and settling time for both X and Y axes independently.

---

## RL Training (Online — requires live Gazebo)

### Quick Start (V2 — Recommended)

**First-time training (1M steps, ~2-3 hours):**
```bash
# Install deps (once)
pip install gymnasium stable-baselines3[extra] tensorboard

<<<<<<< HEAD
# Start training (headless, no GUI)
ros2 launch marble_balancer rl_training.launch.py gui:=false \
  timesteps:=1000000 tcp_lissajous:=true spawn_radius:=0.12
```

**Monitor in separate terminal:**
```bash
tensorboard --logdir src/marble_balancer/rl_training/tensorboard_td3_v2/
# Open browser: http://localhost:6006
```

### Resume From Checkpoint

Continue training from best saved model:
```bash
ros2 launch marble_balancer rl_training.launch.py gui:=false \
  timesteps:=1000000 \
  load:=$(pwd)/src/marble_balancer/rl_training/models_td3_v2/best_model_td3_v2.zip \
  tcp_lissajous:=true spawn_radius:=0.12
```

### Training Features (V2)

| Feature | Details |
|---------|---------|
| **Environment** | 21-D observation space (8-state + 9-TCP-3D-window + 2-target + 2-action) |
| **Bulletproof Spawning** | Atomic reset retry loop, velocity braking, instant landing, ghost detection, reward lockout |
| **Training Modes** | 50% Lissajous curve + 50% random jitter walk (with TCP Z-axis) |
| **Marble Spawn** | Uniformly random within `spawn_radius` (default 0.12 m) of plate centre |
| **Headless Mode** | `gui:=false` runs gzserver without GUI — **300% faster training** |
| **Model Save Location** | `src/marble_balancer/rl_training/models_td3_v2/` |
| **Checkpoints** | Saved every 10K steps: `checkpoint_td3_v2_*.zip` |
| **Best Model** | `best_model_td3_v2.zip` — use this for deployment |

### Curriculum Stages (TD3 V2)
=======
# Recommended full training run (1M steps):
# - tcp_lissajous:=true → 50% of episodes activate TCP Lissajous motion (single model handles both)
# - spawn_radius:=0.12  → marble spawned uniformly at random within 12 cm of centre each episode
# - seed_steps:=40000   → 40K pure-LQR RLPD warm-start (~66 Lissajous cycles)
ros2 launch marble_balancer rl_training.launch.py timesteps:=1000000 tcp_lissajous:=true spawn_radius:=0.12 seed_steps:=40000

# For unattended / overnight runs — suppress Gazebo GUI (headless):
ros2 launch marble_balancer rl_training.launch.py timesteps:=1000000 tcp_lissajous:=true spawn_radius:=0.12 seed_steps:=40000 headless:=true

# Standard training (500K steps, no TCP motion)
ros2 launch marble_balancer rl_training.launch.py

# Continue from checkpoint
ros2 launch marble_balancer rl_training.launch.py load:=/path/to/checkpoint.zip

# Monitor
tensorboard --logdir src/marble_balancer/rl_training/tensorboard_td3/

# Validate with CMA-ES (detect local optima, ~minutes offline)
cd src/marble_balancer/rl_training && python train_cmaes_gazebo.py
```

**Observation space (36-D):**

| Indices | Content |
|---------|---------|
| 0:8 | Normalised state `[x, vx, y, vy, α, ωα, β, ωβ]` |
| 8:28 | Action history — last 10 actions × 2-D, oldest-first |
| 28:32 | Twist cmd `[ωβ_cmd, ωα_cmd, tcp_vx, tcp_vy]` |
| 32:36 | Lissajous phase `[sin(φ_x), cos(φ_x), sin(φ_y), cos(φ_y)]` — zeros when TCP inactive |

**Domain randomisation:** Each episode with TCP active samples `amplitude ~ U(0.20, 0.40) m` and `period ~ U(10, 15) s`. Deployment (0.30 m / 12 s) stays within the training range.

**Performance (TCP Lissajous, ~168 s runs):**

| Metric | Without RL | v1 RL model | v2 target |
|--------|-----------|-------------|-----------|
| RMS error X | 1.8 cm | 1.8 cm | ~1.8 cm |
| RMS error Y | 4.0 cm | 2.5 cm | **< 2.0 cm** |
| Max \|y\| | 8.2 cm | 3.4 cm | < 3.0 cm |
| β saturation | 23.5% | 3.9% | **< 3%** |

**Curriculum stages (TD3, 3 stages):**
>>>>>>> origin/td3_learning

| Stage | Residual clip (λ) | Advances when |
|-------|-------------------|---------------|
| 0 | ±5 °/s | survival fraction > 40% (last 20 eps) |
| 1 | ±10 °/s | survival fraction > 65% |
| 2 | ±15 °/s | — (final stage) |

### TensorBoard Metrics (Now Visible)

All episode metrics now logged to TensorBoard:
- `rollout/episode_reward` — total reward per episode
- `rollout/episode_length` — steps per episode
- `curriculum/survival_fraction` — % episodes surviving
- `curriculum/stage` — curriculum advancement (0→1→2)
- `td3/policy_loss` — actor loss
- `td3/qf_loss` — Q-function loss

### Deploy Trained Model

```bash
ros2 launch marble_balancer servo_balancer.launch.py \
  rl:=true \
  rl_model:=$(pwd)/src/marble_balancer/rl_training/models_td3_v2/best_model_td3_v2.zip \
  rl_norm:=$(pwd)/src/marble_balancer/rl_training/models_td3_v2/running_stats_v2.pkl \
  rl_stage:=2
```

### Old Training (V1 — Legacy)

```bash
# Standard training (500K steps)
ros2 launch marble_balancer rl_training.launch.py

# With generalization training:
# - tcp_lissajous:=true → 50% of episodes activate TCP Lissajous motion
# - spawn_radius:=0.12  → marble spawned uniformly at random within 12 cm of centre
ros2 launch marble_balancer rl_training.launch.py tcp_lissajous:=true spawn_radius:=0.12

# Continue from checkpoint
ros2 launch marble_balancer rl_training.launch.py load:=/path/to/checkpoint.zip

# Validate with CMA-ES (detect local optima, ~minutes offline)
cd src/marble_balancer/rl_training && python train_cmaes_gazebo.py
```

---

## Repository Structure

```
ros2_ws/src/
├── marble_balancer/
│   ├── marble_balancer/               # Python nodes
│   │   ├── lqr_math.py                # Linearised model, ZOH discretisation, DARE
│   │   ├── marble_ekf.py              # Extended Kalman Filter (nonlinear process model)
│   │   ├── marble_servo_controller.py # Main LQR + EKF control loop (30 Hz)
│   │   ├── mux_controller.py          # Manual/auto priority mux
│   │   ├── go_to_pose.py              # Joint-space homing node
│   │   ├── marble_spawner.py          # Gazebo marble spawn via TF
│   │   ├── marble_visualizer.py       # Live 2D marble position display
│   │   ├── marble_plotter.py          # CSV recorder + offline plotter
│   │   ├── marble_lissajous_node.py   # Lissajous setpoint publisher
│   │   ├── marble_square_node.py      # 4-corner square step-response pattern
│   │   ├── tcp_lissajous_node.py      # TCP figure-eight + feedforward tilt
│   │   ├── rl_residual_node.py        # TD3 residual policy inference
│   │   └── marble_spawner_xy.py       # Marble spawn at plate-frame (x,y) coordinates
│   ├── rl_training/
│   │   ├── gazebo_rl_env.py           # Gymnasium env wrapping live Gazebo (V1 — legacy)
│   │   ├── gazebo_rl_env_v2.py        # Bulletproof RL environment with 21-D obs (RECOMMENDED)
│   │   ├── train_td3_gazebo.py        # TD3 + RLPD online training (V1 — legacy)
│   │   ├── train_td3_gazebo_v2.py     # TD3 + RLPD with Monitor + callbacks (V2 — RECOMMENDED)
│   │   ├── train_cmaes_gazebo.py      # CMA-ES linear policy validator
│   │   ├── ball_plate_env.py          # PT1 physics env (offline CMA-ES only)
│   │   ├── TRAINING_PIPELINE.md       # Full training pipeline documentation
│   │   ├── models_td3/                # Trained models (V1)
│   │   └── models_td3_v2/             # Trained models (V2)
│   │       ├── best_model_td3_v2.zip  # Best policy by eval reward
│   │       ├── checkpoint_td3_v2_*.zip # Periodic checkpoints (every 10K steps)
│   │       ├── running_stats_v2.pkl   # Observation normalisation stats
│   │       └── final_model_td3_v2.zip # Final model when training completes
│   ├── launch/
│   │   ├── servo_balancer.launch.py   # Full system launch (with/without RL)
│   │   ├── rl_training.launch.py      # RL training infrastructure (V2, with gui:=false support)
│   │   └── spawn_xy.launch.py         # Full stack with marble at specific position
│   ├── config/
│   │   ├── servo_params.yaml          # MoveIt Servo config (plate_tcp frame, 30 Hz)
│   │   └── ur_controllers.yaml        # JointTrajectoryController at 100 Hz
│   └── urdf/
│       ├── ur5e_marble_balancer.urdf.xacro
│       └── marble.sdf                 # Marble with p3d odometry plugin
└── ur_moveit_config/
    └── config/
        ├── ur.srdf                    # Collision pairs (marble_plate/wrist_3 disabled)
        └── kinematics.yaml
```

---

## Key Design Decisions

| Decision | Rationale |
|----------|-----------|
| MoveIt Servo angular velocity commands | Smooth, rate-limited Cartesian streaming — avoids joint-space discontinuities |
| `plate_tcp` as command frame | Prevents wrist spin; keeps rotation axes aligned with plate surface |
| PT1 robot delay model in LQR | Captures ~0.35 s servo lag; improves stability margin vs pure integrator model |
| EKF with nonlinear Lagrangian process | Hardware-ready state estimator; transparent in Gazebo (R ≈ 0), filters camera noise on real robot |
| Rotational Jacobian for omega (hardware) | Accurate on real UR5e encoder velocity; Gazebo `msg.velocity` unreliable for position-controlled joints |
| Direct joint-space homing | IK found incorrect wrist orientation; hardcoded joint angles guarantee flat plate |

---

## Future Work

- **Hardware transfer** — Deploy on physical UR5e with camera-based marble tracking; tune EKF `DEFAULT_R` for actual camera noise
- **EKF for camera** — Raise `DEFAULT_R[0,0]`/`[1,1]` to match camera noise level; EKF then provides clean position and velocity estimates
- **Feedforward compensation** — Validate and tune tilt pre-compensation for TCP Lissajous acceleration
- **Nonlinear control** — Add friction compensation or switch to NMPC for large-angle operation

---

## Tech Stack

| Component | Technology |
|-----------|-----------|
| Robot | UR5e (6-DOF), `ur_description` xacro |
| Simulation | Gazebo Classic, `libgazebo_ros_p3d` odometry |
| Framework | ROS2 Humble |
| Motion | MoveIt2 Servo (Cartesian velocity streaming) |
| Control | Discrete-time LQR, SciPy `solve_discrete_are` |
| State estimation | Extended Kalman Filter, nonlinear RK4 integration |
| RL | Stable-Baselines3 TD3, Gymnasium, RLPD buffer seeding |
| Language | Python 3 |
