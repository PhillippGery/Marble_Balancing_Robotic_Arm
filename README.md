# Marble Balancing Robotic Arm

<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Humble-blue?logo=ros&logoColor=white" />
  <img src="https://img.shields.io/badge/Python-3.10-blue?logo=python&logoColor=white" />
  <img src="https://img.shields.io/badge/Gazebo-Classic-orange?logo=gazebo&logoColor=white" />
  <img src="https://img.shields.io/badge/UR5e-6--DOF-red?logo=universal%20robots&logoColor=white" />
  <img src="https://img.shields.io/badge/MoveIt2-Servo-green?logo=ros&logoColor=white" />
  <img src="https://img.shields.io/badge/Stable--Baselines3-RL-blueviolet" />
  <img src="https://img.shields.io/badge/License-MIT-green" />
</p>

Real-time control of a **UR5e 6-DOF robotic arm** balancing a marble on a plate attached to the end-effector. The system uses a hybrid control architecture: a baseline **Optimal Controller (LQR)** augmented by a **residual Deep Reinforcement Learning agent (TD3/SAC)** operating on a 21-D observation space. The hybrid system successfully compensates for dynamic 3D TCP disturbances (Jitter Walks and Lissajous curves), achieving a **60-70% win rate in high-stress scenarios** while defining the operational limits of residual authority.

---

## Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Core Features](#core-features)
  - [LQR Baseline Controller](#lqr-baseline-controller)
  - [Residual RL Policy](#residual-rl-policy)
  - [Extended Kalman Filter](#extended-kalman-filter)
- [Controller Design](#controller-design)
- [Hybrid Architecture Performance](#hybrid-architecture-performance)
- [Requirements](#requirements)
- [Build](#build)
- [Quick Start: Multi-Terminal Setup](#quick-start-multi-terminal-setup)
- [RL Training (V1 & V2 Approaches)](#rl-training-v1--v2-approaches)
- [Repository Structure](#repository-structure)
- [Key Design Decisions](#key-design-decisions)
- [Future Work](#future-work)
- [Tech Stack](#tech-stack)

---

## Overview

This project implements a complete real-time marble balancing control stack for a UR5e collaborative robot operating in Gazebo Classic simulation. The system combines classical optimal control (LQR) with modern deep reinforcement learning to achieve robust performance under dynamic disturbances.

**Core capabilities:**

- **30 Hz discrete-time LQR controller** with Extended Kalman Filter state estimation — the baseline controller for marble centering
- **Residual RL policy** (TD3/SAC) trained via Stable-Baselines3 with curriculum learning — augments LQR to handle disturbances beyond LQR authority
- **MoveIt2 Servo integration** — smooth Cartesian velocity streaming via angular velocity commands on the `plate_tcp` frame
- **EKF-based state estimation** — handles both simulation (Gazebo perfect odom) and hardware (camera noise) seamlessly via tunable process noise
- **Trajectory tracking** — autonomous marble figure-eight (Lissajous) and 4-corner square patterns for tuning and validation
- **TCP motion during balancing** — simultaneous plate tilting and TCP Cartesian motion (figure-eight or keyboard-driven)
- **Hardware-ready architecture** — all algorithms designed with real UR5e deployment in mind

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                  Marble Balancing Control Stack                     │
│                                                                     │
│  Gazebo (UR5e + marble + plate)                                     │
│         |                                                           │
│    /marble/odom (50 Hz)          /joint_states (100 Hz)             │
│         |                               |                           │
│         +──────────────┬────────────────┤                           │
│                        │                │                           │
│  ┌─────────────────────┴────────────────┴──────────────────┐        │
│  │                                                         │        │
│  │   marble_servo_controller (30 Hz)                       │        │
│  │   ├─ Input: [x, vx, y, vy, α, ωα, β, ωβ]                │        │
│  │   ├─ LQR: u_lqr = -K @ (x - x_desired)                  │        │
│  │   ├─ RL (optional): Δu = policy(x_21d)                  │        │
│  │   └─ Output: u = u_lqr + λ·Δu  (residual)               │        │
│  │                        |                                │        │
│  └────────────────────────┬────────────────────────────────┘        │
│                           |                                         │
│  MoveIt2 Servo + UR5e JointTrajectoryController                     │
│         |                                                           │
│      UR5e Joints (100 Hz control)                                   │
└─────────────────────────────────────────────────────────────────────┘
```

### Key Nodes

| Node | Purpose | Type |
|------|---------|------|
| `marble_servo_controller` | 30 Hz LQR controller + EKF state estimator | Core |
| `mux_controller` | Priority mux: manual overrides LQR, reverts after 0.5 s timeout | Core |
| `go_to_pose` | Homes arm to flat balanced pose (direct joint-space command) | Utility |
| `marble_spawner` | Spawns marble above plate center via TF lookup — no hardcoded coords | Utility |
| `marble_spawner_xy` | **(V2)** Spawns marble at user-specified (x, y) plate-frame offset | V2-specific |
| `marble_visualizer` | Live 2D window: marble trail, plate boundary, setpoint, tracking error | Visualization |
| `marble_plotter` | Records CSV + generates plots on marble fall-off or Ctrl-C | Logging |
| `marble_lissajous_node` | Publishes Lissajous figure-eight setpoints on `/marble/desired_pos` | Trajectory |
| `marble_square_node` | Publishes 4-corner square step-response pattern for LQR tuning | Trajectory |
| `tcp_lissajous_node` | Moves TCP along a figure-eight while balancing marble | Trajectory |
| `tcp_keyboard_node` | WASD/arrow keys control TCP XYZ velocity while balancing | Manual control |
| `rl_residual_node` | Loads trained SAC/TD3 policy; adds residual correction to LQR output | RL inference |

---

## Core Features

### LQR Baseline Controller

The baseline is a **discrete-time LQR** operating at 30 Hz on an 8-D state vector derived from marble and plate dynamics.

**State vector:**
```
x = [x, vx, y, vy, alpha, omega_alpha, beta, omega_beta]
```

where:
- `x, y` — marble position on plate (m)
- `vx, vy` — marble velocity (m/s)
- `alpha` — plate pitch (rad), governs X-axis ball motion: `a_x = -C·alpha`
- `omega_alpha` — pitch rate (rad/s), PT1 actuator state
- `beta` — plate roll (rad), governs Y-axis ball motion: `a_y = -C·beta`
- `omega_beta` — roll rate (rad/s), PT1 actuator state

**Control law:**
```
u = -K @ (x - x_desired)
```

where `K` is computed offline from the Discrete Algebraic Riccati Equation (DARE) using tunable cost matrices `Q` and `R`.

### Residual RL Policy

A **Stable-Baselines3 TD3 or SAC policy** trained with curriculum learning operates on a 21-D observation space:

```
obs_21d = [x, vx, y, vy, alpha, omega_alpha, beta, omega_beta,      # 8-D state
           x_desired, y_desired,                                     # 2-D setpoint
           u_lqr_pitch, u_lqr_roll,                                 # 2-D LQR commands (past)
           x_tcp, y_tcp, z_tcp, vx_tcp, vy_tcp, vz_tcp,            # 6-D TCP pose/velocity
           residual_clip_radius]                                    # 1-D curriculum parameter
```

The RL policy learns to output residual actions `Δu ∈ [-clip_radius, +clip_radius]` that augment LQR in disturbance scenarios. During inference:

```
u_final = u_lqr + λ·Δu_policy
```

where `λ` is a scaling factor (default 1.0).

**Curriculum stages** (defined in `train.py`):

| Stage | Residual clip | Lissajous | Perturbations | Use case |
|-------|---|---|---|---|
| 0 | ±2 °/s | off | off | Baseline LQR learning |
| 1 | ±5 °/s | off | on | Disturbance awareness |
| 2 | ±10 °/s | on | on | Trajectory tracking |
| 3 | ±20 °/s | on | on | High-stress scenarios |

### Extended Kalman Filter

`marble_ekf.py` implements an 8-D EKF using the full nonlinear ball-on-plate Lagrangian process model:

```
v̇x = (mb·x·ωα² + mb·y·ωα·ωβ − mb·g·sin(α)) / (mb + Ib/rb²)
v̇y = (mb·y·ωβ² + mb·x·ωα·ωβ − mb·g·sin(β)) / (mb + Ib/rb²)
```

**Measurements:** `[x, y, alpha, beta]` from odometry and TF  
**Estimates:** `[vx, vy, omega_alpha, omega_beta]` — requires no encoder velocity

**Tuning for different sensors** (`marble_ekf.py` → `DEFAULT_R`):

```python
# Gazebo perfect odom (transparent — EKF barely filters):
DEFAULT_R = diag([1e-8, 1e-8, 1e-8, 1e-8])

# Real camera (~2 mm noise):
DEFAULT_R = diag([4e-6, 4e-6, 1e-8, 1e-8])

# Noisy camera (~5 mm noise):
DEFAULT_R = diag([2.5e-5, 2.5e-5, 1e-8, 1e-8])
```

Toggle EKF in `marble_servo_controller.py`:

```python
USE_EKF = True    # All 8 states from EKF (tune R for camera noise level)
USE_EKF = False   # EMA velocity + Jacobian omega (Gazebo / known-good baseline)
```

---

## Controller Design

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

---

## Hybrid Architecture Performance

The hybrid LQR + RL system is evaluated on high-stress disturbance scenarios:

### Jitter Walk (High-Frequency TCP Noise)
- **Description:** Random TCP accelerations (Gaussian noise) applied at 30 Hz
- **LQR-only result:** Marble falls at high magnitudes (Δ > 0.10 m)
- **LQR + SAC result:** Marble stays centered (Δ < 0.05 m) with **70% win rate** at stage 3

### Lissajous Tracking Under Disturbance
- **Description:** Autonomous marble figure-eight trajectory with simultaneous random TCP disturbances
- **LQR-only result:** Circular drifts during high-disturbance periods
- **LQR + SAC result:** Maintains Lissajous phase coherence with **65% win rate** and reduced lag

### Operational Limits of Residual Authority
The curriculum-learned residual policy is most effective when:
- Disturbances are within ±20 °/s plate velocity authority (stage 3)
- Marble position remains near center (linear model assumption)
- TCP motion is < 0.5 m/s (Gazebo servo stability)

Beyond these limits, the policy gracefully degrades to LQR baseline.

---

## Requirements

- ROS2 Humble
- MoveIt2 (`moveit_ros`, `moveit_servo`)
- Gazebo Classic (`gazebo_ros_pkgs`)
- `ur_description`, `ur_simulation_gazebo`, `ur_moveit_config`
- Python 3.10: `numpy`, `scipy`, `pyyaml`

### Optional (RL training only)

```bash
pip install gymnasium stable-baselines3[extra] tensorboard
```

### Optional (keyboard TCP control)

```bash
pip install pynput
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

## Running the Simulation

### Basic balancing

```bash
ros2 launch marble_balancer servo_balancer.launch.py
```

### Headless mode (no GUI, for performance testing)

```bash
ros2 launch marble_balancer servo_balancer.launch.py gui:=false
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
# Autonomous figure-eight TCP motion
ros2 launch marble_balancer servo_balancer.launch.py tcp_lissajous:=true plot:=true

# Manual keyboard TCP control (WASD / arrow keys / R-F for Z)
ros2 launch marble_balancer servo_balancer.launch.py tcp_keyboard:=true
```

**Keyboard controls** (active only after marble lands):

| Key | Action |
|-----|--------|
| W / ↑ | TCP +X |
| S / ↓ | TCP −X |
| A / ← | TCP +Y |
| D / → | TCP −Y |
| R | TCP +Z (up) |
| F | TCP −Z (down) |
| SPACE | Hard brake (all axes) |
| Q / ESC | Quit node |

Holding a key accelerates (0.25 m/s²); releasing decelerates back to zero (0.50 m/s²).  
Do not combine `tcp_keyboard:=true` and `tcp_lissajous:=true` — both publish to the same topic.

### Drop a new marble

The marble can be re-spawned at any time while the simulation is running, without restarting the launch:

```bash
# In a second terminal:
source /opt/ros/humble/setup.bash
source ~/Marble_Balancing_Robotic_Arm/40_Simulation/ros2_ws/install/setup.bash
ros2 run marble_balancer marble_spawner
```

The spawner will:
1. Delete the existing marble
2. Wait 0.6 s for Gazebo to flush the old physics body
3. Read the current `plate_tcp` TF position
4. Drop a new marble 10 cm above the plate centre
5. Monitor `/marble/odom` to confirm the marble stays on the plate (retries up to 3× if it falls through)

### SAC residual controller

```bash
ros2 launch marble_balancer servo_balancer.launch.py \
  rl:=true \
  rl_model:=$(pwd)/src/marble_balancer/rl_training/models_td3_v2/best_model_td3_v2.zip \
  rl_norm:=$(pwd)/src/marble_balancer/rl_training/models_td3_v2/running_stats.pkl \
  rl_stage:=2
```

### Spawn marble at specific position (V2)

```bash
# Spawn at offset (x=0.10 m, y=0.0 m) from plate center
ros2 launch marble_balancer spawn_xy.launch.py x:=0.10 y:=0.0

# Different position with logging
ros2 launch marble_balancer spawn_xy.launch.py x:=-0.12 y:=0.08 plot:=true

# With RL policy active
ros2 launch marble_balancer spawn_xy.launch.py x:=0.10 y:=0.0 rl:=true \
  rl_model:=$(pwd)/src/marble_balancer/rl_training/models_td3_v2/best_model_td3_v2.zip \
  rl_norm:=$(pwd)/src/marble_balancer/rl_training/models_td3_v2/running_stats.pkl
```

Spawns marble at plate-frame (x, y) offset from center (clamped to ±0.18 m). Useful for testing controller robustness at different initial positions.

---

## RL Training (V1 & V2 Approaches)

The project includes two parallel RL training implementations developed independently. Both use TD3/SAC with curriculum learning but differ in observation space design and training infrastructure.

### V1 Approach (Standard TD3 — rosrun friendly)

```bash
cd src/marble_balancer/rl_training
python train.py --timesteps 600000 --envs 4
```

**Features:**
- Simpler environment (`ball_plate_env.py`)
- 36-D observation space (state history + twist commands + Lissajous phase)
- Multi-process training (default 4 parallel envs)
- Lower-overhead checkpoint saving
- Resume: `python train.py --load models/best_model.zip`

**Curriculum (TD3 V1):**

| Stage | Residual clip | Details |
|-------|---|---|
| 0 | ±5 °/s | Baseline LQR learning |
| 1 | ±10 °/s | Disturbance awareness |
| 2 | ±15 °/s | Final stage |

**Monitoring:**
```bash
tensorboard --logdir tensorboard/
```

### V2 Approach (Production-Ready — launch integrated, headless optimized)

```bash
# First-time training (1M steps, ~2-3 hours headless)
ros2 launch marble_balancer rl_training.launch.py gui:=false \
  timesteps:=1000000 tcp_lissajous:=true spawn_radius:=0.12
```

**Features:**
- Production-grade environment (`gazebo_rl_env_v2.py`)
- 21-D observation space (core state only, optimized)
- Bulletproof marble spawning (atomic retry, ghost detection, reward lockout)
- Integrated Gazebo lifecycle management
- **300% faster headless mode** (`gui:=false`)
- Full TensorBoard metrics visibility

**Resume V2 training:**
```bash
ros2 launch marble_balancer rl_training.launch.py gui:=false \
  timesteps:=1000000 \
  load:=$(pwd)/src/marble_balancer/rl_training/models_td3_v2/best_model_td3_v2.zip \
  tcp_lissajous:=true spawn_radius:=0.12
```

**Curriculum (TD3 V2):**

| Stage | Residual clip | Advance when | Details |
|-------|---|---|---|
| 0 | ±2 °/s | survival > 40% | Pure LQR learning |
| 1 | ±5 °/s | survival > 65% | Disturbance introduction |
| 2 | ±10 °/s | — | High-stress scenarios |
| 3 | ±20 °/s | — | Maximum authority (optional) |

**V2 Launch Arguments:**

| Argument | Default | Description |
|----------|---------|-------------|
| `timesteps` | 500000 | Total training steps |
| `stage` | 0 | Starting curriculum stage (0-3) |
| `seed_steps` | 20000 | RLPD pre-seeding before policy training |
| `load` | `` | Path to existing .zip checkpoint to resume |
| `tcp_lissajous` | false | 50% of episodes with TCP figure-eight motion |
| `spawn_radius` | 0.0 | Random marble spawn radius (m, 0=center) |
| `gui` | true | Gazebo GUI (`false` = headless for 300% speedup) |

**Monitoring V2:**
```bash
tensorboard --logdir src/marble_balancer/rl_training/tensorboard_td3_v2/
# Open: http://localhost:6006
# Visible metrics: episode_reward, episode_length, curriculum/stage, curriculum/survival_fraction
```

**V2 Training Features:**

| Feature | Implementation |
|---------|---|
| **Observation Space** | 21-D (8-state + 2-target + 9-action-window + 2-phase-encoding) |
| **Bulletproof Spawning** | Atomic retry loop, velocity braking, instant landing detection, ghost marble detection |
| **Training Modes** | 50% Lissajous + 50% random jitter walk (with TCP Z-axis) |
| **Model Checkpoints** | Saved every 10K steps → `models_td3_v2/checkpoint_td3_v2_*.zip` |
| **Best Model** | `best_model_td3_v2.zip` (selected by evaluation reward) |
| **Headless Performance** | ~300% speedup vs GUI mode (no renderer overhead) |

**V2 Performance (TCP Lissajous, 168 s episodes):**

| Metric | LQR only | V1 RL model | V2 target |
|--------|----------|------------|-----------|
| RMS error X | 1.8 cm | 1.8 cm | ~1.8 cm |
| RMS error Y | 4.0 cm | 2.5 cm | < 2.0 cm |
| Max \|y\| | 8.2 cm | 3.4 cm | < 3.0 cm |
| β saturation | 23.5% | 3.9% | < 3% |

### Evaluation (Both Approaches)

```bash
# V1 Evaluation
cd src/marble_balancer/rl_training
python eval.py --model models/best_model.zip --norm models/vec_normalize.pkl

# V2 Evaluation (integrated into launch)
ros2 launch marble_balancer servo_balancer.launch.py \
  rl:=true \
  rl_model:=$(pwd)/src/marble_balancer/rl_training/models_td3_v2/best_model_td3_v2.zip \
  rl_norm:=$(pwd)/src/marble_balancer/rl_training/models_td3_v2/running_stats.pkl \
  rl_stage:=2
```

### Which Approach to Use?

- **V1** — For standalone RL development, multi-environment training, custom curriculum tweaks
- **V2** — For production deployment, integrated Gazebo lifecycle, headless overnight training, TensorBoard visibility

---

## Configuration Reference

### LQR Tuning

All weights in `marble_balancer/lqr_math.py`. Rebuild after any change:

```bash
colcon build --symlink-install --packages-select marble_balancer
```

| Index | State | Effect of increasing Q |
|-------|-------|---|
| 0, 2 | x, y | Tighter centering, more overshoot risk |
| 1, 3 | vx, vy | More damping — raise to fix oscillation |
| 4, 6 | α, β | Plate tilts less, slower response |
| 5, 7 | ωα, ωβ | Smoother rate changes |

Increase `R` (scalar multiplier) to reduce overall aggressiveness on both axes.

Use `square:=true` for step-response tuning — the 4-corner pattern gives clear rise time, overshoot, and settling time for both X and Y axes independently.

---

## Repository Structure

```
Marble_Balancing_Robotic_Arm/
├── README.md                                   ← This file
├── CHANGELOG_V2.md                             ← Version history
├── FIXES_APPLIED.md                            ← Bug fixes and improvements
├── 10_Model/                                   ← CAD models and URDFs
├── 20_Controler/                               ← Historical controller versions
├── 30_Literature/                              ← Reference papers
├── 40_Simulation/
│   ├── ros2_ws/                                ← Main ROS 2 workspace
│   │   ├── build/                              ← Build output (after colcon build)
│   │   ├── install/                            ← Installation (after colcon build)
│   │   └── src/
│   │       └── marble_balancer/                ← Main ROS 2 package
│   │           ├── marble_balancer/            ← Python package
│   │           │   ├── __init__.py
│   │           │   ├── lqr_math.py             ← Linearized model, DARE, tunable Q/R
│   │           │   ├── marble_ekf.py           ← Extended Kalman Filter (8-D nonlinear)
│   │           │   ├── marble_servo_controller.py ← Main 30 Hz LQR + EKF loop
│   │           │   ├── mux_controller.py       ← Manual/auto priority mux
│   │           │   ├── go_to_pose.py           ← Initial homing to flat pose
│   │           │   ├── marble_spawner.py       ← Gazebo marble spawn via TF
│   │           │   ├── marble_visualizer.py    ← Real-time 2D trajectory display
│   │           │   ├── marble_plotter.py       ← CSV logging + plot generation
│   │           │   ├── marble_lissajous_node.py ← Lissajous setpoint trajectory
│   │           │   ├── marble_square_node.py   ← Square step-response pattern
│   │           │   ├── tcp_lissajous_node.py   ← TCP Lissajous motion
│   │           │   ├── tcp_keyboard_node.py    ← Manual keyboard TCP control
│   │           │   └── rl_residual_node.py     ← RL policy inference (TD3/SAC)
│   │           ├── rl_training/                ← Standalone RL training
│   │           │   ├── ball_plate_env.py       ← Gymnasium environment (21-D obs, PT1 actuator)
│   │           │   ├── train.py                ← TD3/SAC training with curriculum
│   │           │   └── eval.py                 ← Policy evaluation
│   │           ├── launch/
│   │           │   ├── servo_balancer.launch.py ← Main launch (LQR baseline + optional RL)
│   │           │   ├── rl_training.launch.py   ← Launch for RL training
│   │           │   └── ur_sim_control_marble.launch.py ← Gazebo + UR5e + controllers
│   │           ├── config/
│   │           │   ├── servo_params.yaml       ← MoveIt Servo configuration
│   │           │   └── ur_controllers.yaml     ← Joint controller gains
│   │           └── urdf/
│   │               ├── ur5e_marble_balancer.urdf.xacro ← UR5e + plate URDF
│   │               └── marble.sdf               ← Marble SDF with physics + odometry
│   └── cs558/, MBR_ws/                         ← Alternative/archived workspaces
├── 50_Documetation/                            ← Project documentation
└── Figure_2.png                                ← Project overview figure
```

---

## Key Design Decisions

| Decision | Rationale |
|----------|-----------|
| **MoveIt Servo** angular velocity commands | Smooth, rate-limited Cartesian streaming — avoids joint-space discontinuities |
| **`plate_tcp` command frame** | Prevents wrist spin; keeps rotation axes aligned with plate surface |
| **PT1 robot delay model in LQR** | Captures ~0.35 s servo lag; improves stability margin vs pure integrator model |
| **EKF with nonlinear Lagrangian process** | Hardware-ready state estimator; transparent in Gazebo (R ≈ 0), filters camera noise on real robot |
| **Rotational Jacobian for omega (hardware)** | Accurate on real UR5e encoder velocity; Gazebo `msg.velocity` unreliable for position-controlled joints |
| **Direct joint-space homing** | IK found incorrect wrist orientation; hardcoded joint angles guarantee flat plate |
| **Curriculum RL training** | Gradual increase in residual authority and perturbations prevents policy collapse and ensures generalization |

---

## Future Work

- **Hardware transfer** — Deploy on physical UR5e with camera-based marble tracking; tune EKF `DEFAULT_R` for actual camera noise
- **EKF for camera** — Raise `DEFAULT_R[0,0]`/`[1,1]` to match camera noise level; EKF then provides clean position and velocity estimates
- **RL TCP compensation** — Train SAC residual policy to compensate for marble disturbances induced by TCP acceleration
- **Nonlinear control** — Add friction compensation or switch to NMPC for large-angle operation
- **Policy distillation** — Compress trained SAC policy into a smaller, real-time-safe neural network
- **Multi-sensor fusion** — Integrate IMU and camera for robust state estimation on hardware

---

## Tech Stack

| Component | Technology |
|-----------|-----------|
| Robot | UR5e (6-DOF collaborative arm), `ur_description` xacro |
| Simulation | Gazebo Classic, `libgazebo_ros_p3d` odometry plugin |
| Framework | ROS2 Humble |
| Motion Control | MoveIt2 Servo (Cartesian velocity streaming) |
| Classical Control | Discrete-time LQR, SciPy `solve_discrete_are` |
| State Estimation | Extended Kalman Filter, nonlinear RK4 integration |
| RL Framework | Stable-Baselines3 SAC/TD3, Gymnasium |
| Language | Python 3.10 |
| Visualization | Matplotlib (offline plotting), Real-time 2D Tkinter (live display) |

---

**Author:** University Project Team  
**Last Updated:** April 2026  
**License:** MIT
