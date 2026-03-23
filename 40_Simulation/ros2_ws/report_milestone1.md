# Project Milestone 1 Report
## Marble Balancing Robotic Arm — ROS2 Simulation

---

## 1. Introduction

This project implements real-time feedback control of a UR5e 6-DOF robotic arm that balances a marble on a flat plate attached to the robot's end-effector. The arm tilts the plate using small angular velocity commands, keeping the marble centered. The full system runs in simulation using **Gazebo Classic**, **ROS2 Humble**, and **MoveIt2 Servo**.

The challenge combines two coupled dynamical systems: the rolling dynamics of the marble on the plate, and the servo dynamics of the robot arm responding to velocity commands. The ball's acceleration depends on plate tilt angle, while the arm's response to commands is subject to inherent actuator lag. A classical discrete-time **Linear Quadratic Regulator (LQR)** is used as the primary controller, designed around a physics model that explicitly captures both the ball-plate coupling and the robot's first-order (PT1) delay.

Optional modes extend the base system to trajectory tracking: the marble can follow a Lissajous curve setpoint, the robot TCP can simultaneously trace a figure-eight path, and a Soft Actor-Critic (SAC) residual controller can augment the LQR output. This report covers the simulation setup and LQR classical controller (Milestone 1).

---

## 2. Methods

### 2.1 Simulation Environment

**Robot and World:** The UR5e arm is loaded into Gazebo Classic via a URDF xacro that uses the standard `ur_description` macros. A 0.40 m × 0.40 m × 5 mm acrylic plate is rigidly attached to the robot's `tool0` link. A virtual frame `plate_tcp` sits at the plate's top surface and serves as the control reference frame. The world contains only the robot, the plate, and the marble — no external obstacles.

**Marble:** The marble is a 15 mm radius, 50 g sphere defined in SDF format. Gazebo's ODE physics engine simulates its rolling contact on the plate with a friction coefficient of 0.6. The marble is dynamically spawned above the plate after the arm reaches its home pose, using the `/spawn_entity` Gazebo service. The spawn position is computed at runtime from a TF lookup of the `plate_tcp` frame so that no coordinates are hardcoded.

**State Sensing:**
- *Marble position and velocity:* The `libgazebo_ros_p3d` Gazebo plugin publishes ground-truth odometry at 50 Hz on `/marble/odom`. Velocity is estimated by numerically differentiating successive positions and filtering with an exponential moving average (time constant `OMEGA_LPF_TC = 0.08 s`).
- *Plate orientation:* Obtained by querying the TF tree for the `world → plate_tcp` transform and converting the quaternion to roll/pitch Euler angles. This is available at the odometry rate (~50 Hz) and is consistent with the Gazebo physics state.
- *Plate angular velocity:* Computed by numerically differentiating the TF-derived Euler angles (pitch and roll) in the same 50 Hz odometry callback that updates `alpha` and `beta`. This gives `omega_alpha = d(alpha)/dt` and `omega_beta = d(beta)/dt`, which are sign-consistent with `alpha`/`beta` by construction. An exponential moving average (time constant `OMEGA_LPF_TC = 0.08 s`) smooths the derivative noise. An earlier Jacobian-based approach (`J_rot @ q_dot` from 100 Hz joint states) was abandoned because it produced a sign inconsistency relative to the TF-derived angles at the robot's home configuration, causing oscillation in the pitch channel.

**Initialization Sequence:** The system uses an event-driven launch sequence (no fixed time delays). After Gazebo and MoveIt load, a `go_to_pose` node commands the arm to its home Cartesian pose via inverse kinematics and a `JointTrajectoryController` action. When that node exits, the `marble_spawner` fires, drops the marble, and exits — triggering the controller and auxiliary nodes. This chain uses ROS2 `OnProcessExit` launch event handlers.

### 2.2 Classical Controller: Discrete-Time LQR

#### Physical Model

The ball-plate system is modeled as a planar coupled system. Ball acceleration along each axis is proportional to plate tilt angle (small-angle approximation for a rolling sphere):

```
a_ball = -C * theta_plate
C = M*g / (M + I/r^2)  ≈  7.0  m/s²
```

where `M = 0.05 kg` (marble mass), `I = (2/5)Mr^2` (solid sphere inertia), `r = 0.015 m` (radius), and `g = 9.81 m/s²`.

The robot arm's response to angular velocity commands is modeled as a **first-order (PT1) lag** with time constant `T_ROBOT = 0.35 s`:

```
d(omega)/dt = -(omega - omega_cmd) / T_ROBOT
```

This captures the combined effect of communication latency, controller bandwidth, and joint servo dynamics.

#### State Vector (8-D)

The full state is:

```
x = [x,  vx,  y,  vy,  alpha,  omega_alpha,  beta,  omega_beta]^T

x, y            : marble position on plate (m)
vx, vy          : marble velocity (m/s)
alpha (pitch)   : plate tilt about Y axis (rad) — governs X ball motion
omega_alpha     : rate of change of alpha (rad/s) — PT1 state
beta (roll)     : plate tilt about X axis (rad) — governs Y ball motion
omega_beta      : rate of change of beta (rad/s) — PT1 state
```

The two axes are decoupled in the model, forming an 8×8 system.

#### Continuous-Time System Matrices

**A matrix (8×8):**

```
[0  1   0  0    0         0      0         0   ]   dx/dt = vx
[0  0   0  0   -C         0      0         0   ]   dvx/dt = -C*alpha
[0  0   0  1    0         0      0         0   ]   dy/dt = vy
[0  0   0  0    0         0     -C         0   ]   dvy/dt = -C*beta
[0  0   0  0    0         1      0         0   ]   d(alpha)/dt = omega_alpha
[0  0   0  0  -1/T        0      0         0   ]   d(omega_alpha)/dt = -(omega_alpha)/T + u1/T
[0  0   0  0    0         0      0         1   ]   d(beta)/dt = omega_beta
[0  0   0  0    0         0    -1/T        0   ]   d(omega_beta)/dt = -(omega_beta)/T + u2/T
```

**B matrix (8×2):**

Only rows 5 and 7 are nonzero: `B[5,0] = 1/T`, `B[7,1] = 1/T`.

Control inputs `u = [omega_alpha_cmd, omega_beta_cmd]^T` are angular velocity commands sent to MoveIt Servo.

#### Discretization (Zero-Order Hold)

The continuous system is discretized using the matrix exponential method at `dt = 1/30 s` (30 Hz control rate):

```
M = [A   B]   =>   exp(M*dt)  =>  Ad = top-left 8x8,  Bd = top-right 8x2
    [0   0]
```

This is implemented using `scipy.linalg.expm` and ensures accurate discretization of the coupled dynamics.

#### LQR Gain Computation

The discrete-time LQR gain `K` is found by solving the **Discrete Algebraic Riccati Equation (DARE)**:

```
P = solve_discrete_are(Ad, Bd, Q, R)
K = inv(R + Bd^T P Bd) @ Bd^T P Ad
```

This is solved once at node startup. The gain `K` is a 2×8 matrix mapping the full state to two control outputs.

**Cost matrices:**

```
Q = diag([100.0, 100.0, 200.0, 400.0, 5.0, 0.5, 5.0, 1.0])
         [  x,    vx,    y,     vy,    a,   wa,  b,   wb ]

R = 5.0 * I_{2x2}
```

Position weights (Q[0], Q[2]) penalize centering error. Velocity weights (Q[1], Q[3]) provide damping — `Q[3] = 400` is elevated relative to Q[1] because the Y axis is driven at twice the frequency in the TCP Lissajous mode, requiring stronger damping. Angle and rate weights (Q[4]–Q[7]) are kept small to allow the plate to tilt aggressively when needed without excessive control effort cost.

#### Online Control Loop (30 Hz)

At each control tick:

1. Assemble the 8-D state from filtered odometry and TF data.
2. Compute the state error: `e = x - x_desired` (setpoint is zero by default, or a Lissajous point).
3. Apply LQR: `u = -K @ e`
4. Saturate: `u = clip(u, -45 deg/s, +45 deg/s)`
5. Publish as a `TwistStamped` angular velocity command to MoveIt Servo via the mux.

MoveIt Servo converts the Cartesian angular velocity command into joint velocity commands at 30 Hz, which are then tracked by the UR5e's `JointTrajectoryController`.

#### Mux Controller

A priority multiplexer sits between the LQR controller and Servo. Manual operator commands (e.g., from a joystick) override LQR output, reverting to automatic control after 0.5 s of inactivity. This allows safe operator intervention without restarting the system.

---

## 3. Preliminary Experiment Results

The following describes observed system behavior during initial simulation runs.

**Homing and spawning:** The arm reliably reaches the home pose (TCP at approximately `[0.233, 0.25, 0.8] m` in world frame) via inverse kinematics before the marble is dropped. The TF-based spawn calculation correctly places the marble 80 mm above the plate surface regardless of minor home pose variation. Landing detection (5 consecutive odometry ticks within ±3 cm of the plate surface) triggers servo activation without false positives.

**Stabilization — baseline LQR:** With default Q/R weights and no trajectory tracking, the LQR controller successfully stabilizes the marble near the plate center. The marble settles from an initial drop within approximately 2–3 seconds, with residual oscillation on the order of ±1–2 cm in steady state. Both X and Y axes stabilize, though Y shows slightly more oscillation in early runs.

**Y-axis oscillation:** When the TCP Lissajous mode is active (the robot TCP traces a figure-eight at amplitude 0.30 m, period 12 s, with Y at 2× frequency), the increased plate acceleration in the Y direction drives the marble further off center. Raising `Q[3]` (marble Y velocity cost) from 100 to 400 reduced this oscillation significantly. Raising `Q[2]` (marble Y position cost) to 200 further improved centering. These changes are reflected in the current default weights.

**Lissajous trajectory tracking:** With marble Lissajous setpoints enabled, the controller successfully causes the marble to follow a figure-eight pattern on the plate. Tracking error increases with setpoint amplitude and frequency, as expected from the linear model's ignoring of large-angle nonlinearities.

**Fell-off recovery:** When the marble rolls off the plate edge, the system correctly detects the fall (marble Z drops below plate surface by >5 cm), stops Servo, re-homes the arm, and re-spawns the marble autonomously. This cycle completes in approximately 8–10 seconds.

---

## 4. Future Work

**4.1 Reinforcement Learning Residual Controller**

A Soft Actor-Critic (SAC) residual policy is partially implemented. The training environment (`ball_plate_env.py`) simulates the same PT1 ball-plate physics with Coulomb/viscous friction and domain randomization (mass, friction, time constant). A curriculum trains the agent through four stages of increasing difficulty (residual authority ±2 → ±20 deg/s, then adding Lissajous and perturbations). The next step is to complete training, evaluate transfer to the Gazebo simulation, and tune the LQR + RL blending.

**4.2 Feedforward Tilt Compensation**

The TCP Lissajous node computes and publishes a feedforward tilt vector (`/tcp/lissajous_ff_tilt`) intended to pre-compensate for the pseudo-force the marble experiences during TCP acceleration. The feedforward gain (`ff_gain`) is currently set to 0.0 (disabled). The correct sign and magnitude need to be verified experimentally by enabling the feedforward and comparing marble trajectory error with and without compensation.

**4.3 Nonlinear / Adaptive Control**

The linear model assumes small plate angles and ignores friction. At higher TCP velocities or Lissajous amplitudes, these nonlinearities become significant. A planned extension is to either (a) add a friction compensation term to the LQR output using identified static and kinetic friction coefficients, or (b) switch to a nonlinear model predictive controller (NMPC) that handles large-angle kinematics.

**4.4 Hardware Transfer**

The simulation is designed to match real UR5e hardware: the PT1 delay model uses a measured actuator time constant, and `libgazebo_ros_p3d` is replaceable with a physical vision system (e.g., overhead camera with blob detection). The next milestone will involve validating the controller on a physical UR5e arm with a real marble, assessing whether the PT1 model adequately captures real actuator dynamics and whether the LQR gains transfer without retuning.

**4.5 Kalman Filter for State Estimation**

Currently, marble velocity and plate angular velocity are estimated by numerically differentiating position measurements and filtering with a simple exponential moving average. This approach is sensitive to measurement noise and introduces phase lag. A planned improvement is to replace this with a **Kalman Filter (KF)** or **Extended Kalman Filter (EKF)** that fuses odometry, joint state, and TF data using the same ball-plate PT1 model as the process model. This will provide optimal state estimates under Gaussian noise assumptions, reduce derivative noise without adding excessive lag, and improve controller performance — particularly for marble velocity terms that currently require high Q weights to compensate for estimation error.

**4.6 Quantitative Performance Metrics**

Current evaluation is qualitative. Future work will compute: RMS centering error, mean time to stabilize from drop, maximum stable Lissajous amplitude, and comparison of LQR-only vs. LQR+RL performance on identical disturbance sequences.
