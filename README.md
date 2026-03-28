# Marble Balancing Robotic Arm — ROS2 Simulation

Real-time LQR control of a **UR5e 6-DOF robotic arm** balancing a marble on a plate attached to the end-effector. The arm tilts the plate using angular velocity commands via MoveIt2 Servo, keeping the marble centered. Runs in **Gazebo Classic** simulation with hardware-ready architecture.

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
| `tcp_lissajous_node` | Moves TCP along a figure-eight while balancing |
| `tcp_keyboard_node` | WASD/arrow keys control TCP XYZ velocity while balancing |
| `rl_residual_node` | Loads trained SAC policy; adds residual correction to LQR output |

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

### Optional (keyboard TCP control)
```bash
pip install pynput
```

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
  rl_model:=$(pwd)/src/marble_balancer/rl_training/models/best_model.zip \
  rl_norm:=$(pwd)/src/marble_balancer/rl_training/models/vec_normalize.pkl \
  rl_stage:=3
```

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

## RL Training (Standalone)

```bash
cd src/marble_balancer/rl_training

# Train from scratch (4 parallel envs, 600 k steps)
python train.py --timesteps 600000 --envs 4

# Continue from checkpoint
python train.py --load models/best_model.zip

# Evaluate
python eval.py --model models/best_model.zip --norm models/vec_normalize.pkl

# Monitor training
tensorboard --logdir tensorboard/
```

**Curriculum stages:**

| Stage | Residual clip | Lissajous | Perturbations |
|-------|--------------|-----------|---------------|
| 0 | ±2 °/s | off | off |
| 1 | ±5 °/s | off | on |
| 2 | ±10 °/s | on | on |
| 3 | ±20 °/s | on | on |

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
│   │   ├── tcp_lissajous_node.py      # TCP figure-eight motion
│   │   ├── tcp_keyboard_node.py       # WASD/arrow-key TCP XYZ velocity control
│   │   └── rl_residual_node.py        # SAC residual policy inference
│   ├── rl_training/
│   │   ├── ball_plate_env.py          # Gymnasium env (PT1 + friction + randomisation)
│   │   ├── train.py                   # SAC training with curriculum
│   │   └── eval.py                    # Policy evaluation
│   ├── launch/
│   │   └── servo_balancer.launch.py   # Full system launch
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
- **RL TCP compensation** — Train SAC residual policy to compensate marble disturbances from TCP acceleration
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
| RL | Stable-Baselines3 SAC, Gymnasium |
| Language | Python 3 |
