# Marble Balancing Robotic Arm — ROS2 Simulation

Real-time LQR control of a **UR5e 6-DOF robotic arm** balancing a marble on a plate attached to the end-effector. The arm tilts the plate using small angular velocity commands via MoveIt2 Servo, keeping the marble centered. Runs entirely in **Gazebo Classic** simulation.

---

## Demo

| Mode | Description |
|------|-------------|
| Baseline | Marble dropped on plate; LQR stabilizes to center |
| Marble Lissajous | Marble tracks a figure-eight setpoint on the plate |
| TCP Lissajous | Robot TCP traces a figure-eight while balancing the marble |
| RL Residual | SAC policy augments LQR output for improved performance |

---

## System Overview

```
Gazebo (UR5e + marble)
        |
   /marble/odom (50 Hz)          /joint_states (100 Hz)
        |                               |
        +-----------> marble_servo_controller (30 Hz LQR) <---+
                              |                               |
                     /marble_servo/delta_twist_cmds           |
                              |                         /marble/desired_pos
                       mux_controller                  (marble_lissajous_node)
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
| `marble_servo_controller` | 30 Hz LQR controller — main control loop |
| `mux_controller` | Priority mux: manual overrides LQR; reverts after 0.5 s |
| `go_to_pose` | Homes arm to balanced start pose via IK |
| `marble_spawner` | Spawns marble above plate using TF lookup |
| `marble_visualizer` | Live 2D plot: marble trail, plate boundary, setpoint |
| `marble_plotter` | Records CSV + generates trajectory plots |
| `marble_lissajous_node` | Publishes Lissajous setpoints on `/marble/desired_pos` |
| `tcp_lissajous_node` | Moves TCP along figure-eight; publishes feedforward tilt |
| `rl_residual_node` | Loads SAC policy; adds residual to LQR output |

---

## Controller Design

### State Vector (8-D)

```
x = [x, vx, y, vy, alpha, omega_alpha, beta, omega_beta]

x, y          — marble position on plate (m)
vx, vy        — marble velocity (m/s)
alpha         — plate pitch (rad), governs X-axis ball motion
omega_alpha   — pitch rate (rad/s), PT1 actuator state
beta          — plate roll (rad), governs Y-axis ball motion
omega_beta    — roll rate (rad/s), PT1 actuator state
```

### Physics Model

Ball acceleration is coupled to plate tilt angle:

```
a_ball = -C * theta_plate
C = M*g / (M + I/r²)  ≈  7.0 m/s²
```

Robot actuator lag is modeled as a PT1 (first-order) delay with `T = 0.35 s`:

```
d(omega)/dt = -(omega - omega_cmd) / T
```

### Discrete-Time LQR

The continuous system is discretized via Zero-Order Hold (matrix exponential) at 30 Hz, then the optimal gain `K` is found by solving the Discrete Algebraic Riccati Equation (DARE):

```
P  = solve_discrete_are(Ad, Bd, Q, R)
K  = inv(R + Bd' P Bd) @ Bd' P Ad
u  = -K @ (x - x_desired)       # applied at 30 Hz
```

**Default cost weights:**

```
Q = diag([100, 100, 200, 400, 5, 0.5, 5, 1.0])
         [ x,  vx,  y,   vy,  α,  ωα, β,  ωβ ]
R = 5.0 * I₂
```

---

## Requirements

- ROS2 Humble
- MoveIt2 (`moveit_ros`, `moveit_servo`)
- Gazebo Classic (gazebo_ros_pkgs)
- `ur_description`, `ur_robot_driver`
- Python: `numpy`, `scipy`

### Optional (RL training only)
```bash
pip install gymnasium stable-baselines3[extra] tensorboard
```

---

## Build

```bash
cd ~/Marble_Balancing_Robotic_Arm/40_Simulation/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Run

```bash
# Basic balancing
ros2 launch marble_balancer servo_balancer.launch.py

# With live visualizer and data logging
ros2 launch marble_balancer servo_balancer.launch.py plot:=true

# Marble follows Lissajous curve setpoint
ros2 launch marble_balancer servo_balancer.launch.py lissajous:=true plot:=true

# TCP traces figure-eight while balancing
ros2 launch marble_balancer servo_balancer.launch.py tcp_lissajous:=true plot:=true

# SAC residual controller
ros2 launch marble_balancer servo_balancer.launch.py \
  rl:=true \
  rl_model:=$(pwd)/src/marble_balancer/rl_training/models/best_model.zip \
  rl_norm:=$(pwd)/src/marble_balancer/rl_training/models/vec_normalize.pkl \
  rl_stage:=3
```

---

## RL Training (Standalone)

```bash
cd src/marble_balancer/rl_training

# Train from scratch
python train.py --timesteps 600000 --envs 4

# Continue from checkpoint
python train.py --load models/best_model.zip

# Evaluate
python eval.py --model models/best_model.zip --norm models/vec_normalize.pkl

# Monitor
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
│   ├── marble_balancer/        # Python nodes
│   │   ├── lqr_math.py         # Physics model + ZOH + DARE
│   │   ├── marble_servo_controller.py
│   │   ├── mux_controller.py
│   │   ├── go_to_pose.py
│   │   ├── marble_spawner.py
│   │   ├── marble_visualizer.py
│   │   ├── marble_plotter.py
│   │   ├── marble_lissajous_node.py
│   │   ├── tcp_lissajous_node.py
│   │   └── rl_residual_node.py
│   ├── launch/
│   │   └── servo_balancer.launch.py
│   ├── config/
│   │   ├── servo_params.yaml
│   │   └── ur_controllers.yaml
│   ├── urdf/
│   │   ├── ur5e_marble_balancer.urdf.xacro
│   │   └── marble.sdf
│   └── rl_training/
│       ├── ball_plate_env.py
│       ├── train.py
│       └── eval.py
└── ur_moveit_config/
    └── config/
        ├── ur.srdf
        └── kinematics.yaml
```

---

## Future Work

- **Kalman Filter** — Replace finite-difference velocity estimation with a KF/EKF using the PT1 ball-plate model as the process model for optimal noise rejection
- **RL Residual Controller** — Complete SAC training and validate transfer to Gazebo simulation
- **Feedforward Compensation** — Verify and tune tilt pre-compensation for TCP Lissajous acceleration
- **Nonlinear Control** — Add friction compensation or switch to NMPC for large-angle regimes
- **Hardware Transfer** — Deploy on physical UR5e with vision-based marble tracking

---

## Tech Stack

| Component | Technology |
|-----------|-----------|
| Robot | UR5e (6-DOF), `ur_description` xacro |
| Simulation | Gazebo Classic, `libgazebo_ros_p3d` |
| Framework | ROS2 Humble |
| Motion | MoveIt2 Servo (Cartesian velocity streaming) |
| Control | Discrete-time LQR, SciPy `solve_discrete_are` |
| RL | Stable-Baselines3 SAC, Gymnasium |
| Language | Python 3 |
