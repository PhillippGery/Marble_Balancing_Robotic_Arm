# Marble Balancing Robotic Arm — ROS2 Simulation

## Project Purpose
Real-time LQR control of a UR5e robot arm balancing a marble on a plate. A marble is spawned in Gazebo, and the arm tilts the plate using MoveIt Servo to keep the marble centered. Supports Lissajous curve trajectory tracking.

## Tech Stack
- **Language:** Python 3 (all control logic)
- **Robot Framework:** ROS2 Humble + MoveIt2 (move_group, servo_node)
- **Simulation:** Gazebo Classic with `libgazebo_ros_p3d` for marble odometry
- **Control:** Discrete-time LQR via SciPy `solve_discrete_are`, PT1 robot delay model
- **Robot:** UR5e (6-DOF), described via `ur_description` xacro macros

## Key Directories
```
src/marble_balancer/
├── marble_balancer/     # All Python nodes (control, spawning, plotting)
├── launch/              # Launch files
├── config/              # YAML configs (servo, LQR, controllers)
└── urdf/                # Robot xacro + marble.sdf
src/ur_moveit_config/
└── config/              # MoveIt SRDF, kinematics, joint limits
```

## Essential Commands

### Build
```bash
cd ~/Marble_Balancing_Robotic_Arm/40_Simulation/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Run (primary launch)
```bash
ros2 launch marble_balancer servo_balancer.launch.py
# With data recording:
ros2 launch marble_balancer servo_balancer.launch.py plot:=true
# With marble Lissajous tracking:
ros2 launch marble_balancer servo_balancer.launch.py lissajous:=true plot:=true
# With TCP Lissajous (arm traces figure-eight while balancing):
ros2 launch marble_balancer servo_balancer.launch.py tcp_lissajous:=true plot:=true
```

### Test
```bash
colcon test --packages-select marble_balancer
colcon test-result --verbose
```

## Core Nodes

| Node | File | Purpose |
|------|------|---------|
| `marble_servo_controller` | `marble_servo_controller.py` | 30 Hz LQR controller; main control loop |
| `mux_controller` | `mux_controller.py` | Priority mux: manual overrides LQR; reverts after 0.5 s timeout |
| `go_to_pose` | `go_to_pose.py` | Homing to balanced start position |
| `marble_spawner` | `marble_spawner.py` | Spawns marble above plate via Gazebo service |
| `marble_plotter` | `marble_plotter.py` | Records CSV + generates trajectory plots |
| `marble_lissajous` | `marble_lissajous_node.py` | Publishes Lissajous setpoints on `/marble/desired_pos` |
| `tcp_lissajous` | `tcp_lissajous_node.py` | Moves TCP along XY Lissajous; publishes velocity + feedforward tilt |
| LQR math | `lqr_math.py` | Physics model + ZOH discretization + gain computation |

## Key Topics / Services

| Topic | Type | Direction |
|-------|------|-----------|
| `/marble/odom` | `nav_msgs/Odometry` | Gazebo → controller (50 Hz) |
| `/joint_states` | `sensor_msgs/JointState` | Gazebo → controller (100 Hz) |
| `/marble_servo/delta_twist_cmds` | `geometry_msgs/TwistStamped` | LQR controller → mux_controller (30 Hz) |
| `/manual/delta_twist_cmds` | `geometry_msgs/TwistStamped` | operator → mux_controller (any rate) |
| `/servo_node/delta_twist_cmds` | `geometry_msgs/TwistStamped` | mux_controller → MoveIt Servo (30 Hz) |
| `/mux_controller/mode` | `std_msgs/String` | mux status: `"manual"` or `"auto"` |
| `/marble/landed` / `/marble/fell_off` | `std_msgs/Empty` | spawner → plotter/lissajous (TRANSIENT_LOCAL) |
| `/marble/desired_pos` | `geometry_msgs/Point` | lissajous → controller |
| `/tcp/lissajous_vel` | `geometry_msgs/TwistStamped` | tcp_lissajous → controller (linear.x/y) |
| `/tcp/lissajous_ff_tilt` | `geometry_msgs/Vector3` | tcp_lissajous → controller (feedforward tilt) |

Services: `/spawn_entity`, `/delete_entity` (Gazebo), `/compute_ik` (MoveIt), `/servo_node/start_servo`

## Config Files
- `config/servo_params.yaml` — MoveIt Servo tuning (30 Hz, `speed_units`, `plate_tcp` EEF)
- `config/lqr_params.yaml` — LQR Q/R weights, TCP home pose, control rate
- `config/ur_controllers.yaml` — Controller manager (100 Hz), JointTrajectoryController joints
- `src/ur_moveit_config/config/kinematics.yaml` — KDL IK solver, 5 ms timeout

## State Vector (8-D)
`[x, vx, y, vy, alpha, ω_alpha, beta, ω_beta]`
`alpha`/`beta` = plate pitch/roll from TF; `ω_alpha`/`ω_beta` = from Jacobian × joint velocities
See: `marble_servo_controller.py:control_callback`, `lqr_math.py:build_system_matrices`

## Launch Sequence
Event-driven (not time-based): UR sim → MoveIt nodes → `go_to_pose` → `marble_spawner` → controller + mux + plotter/lissajous
See: `launch/servo_balancer.launch.py`

## Additional Documentation
- `.claude/docs/architectural_patterns.md` — PT1 model, Jacobian velocity, event-driven launch, LQR design, QoS patterns
