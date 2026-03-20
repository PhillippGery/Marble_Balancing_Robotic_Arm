# Architectural Patterns

## 1. PT1 Robot Delay Model (Physics Augmentation)

**Pattern:** Augment the plant model with a first-order lag to represent actuator delay, then design LQR over the augmented system.

**Where:** `lqr_math.py:build_system_matrices()`, `marble_servo_controller.py` (T_ROBOT=0.35)

The continuous-time A matrix includes rows for `ω_alpha` and `ω_beta` states:
```
dω/dt = -(ω - ω_cmd) / T_ROBOT
```
This models servo response lag directly in the LQR state rather than treating it as a disturbance. The commanded angular rate becomes the control input `u`. Discretized via zero-order hold (ZOH) using matrix exponential (`scipy.linalg.expm`).

**Why it matters:** Without the delay model, LQR gains tuned in simulation are destabilizing at high bandwidth. Embedding the lag in the model lets the optimizer trade off aggressiveness vs. stability.

---

## 2. Jacobian-Based Plate Angular Velocity Estimation

**Pattern:** Compute end-effector angular velocity from joint velocities via the geometric Jacobian, avoiding noisy numerical differentiation of orientation.

**Where:** `marble_servo_controller.py:_compute_plate_angular_velocity()` (~line 180)

Steps:
1. Look up TF transforms for each UR joint frame (joint axes in world frame)
2. Build the 3×N rotational Jacobian: `J_rot[:, i] = z_i` (joint axis in world frame)
3. Multiply by joint velocity vector: `ω_plate = J_rot @ q_dot`
4. Extract `ω_alpha` (Y-component) and `ω_beta` (X-component) for LQR state

This pattern appears in both `marble_servo_controller.py` and `velocity_bridge.py` (PyKDL variant). The servo controller computes it manually from TF rather than using a library to avoid KDL chain setup overhead at 30 Hz.

---

## 3. Event-Driven Launch Sequencing

**Pattern:** Use `RegisterEventHandler(OnProcessExit(...))` to chain node startup, replacing fragile time-based `TimerAction` delays.

**Where:** `launch/servo_balancer.launch.py` (entire file structure)

Sequence:
```
UR sim launched
  → [OnProcessStart: move_group] → start servo_node
  → [OnProcessExit: go_to_pose] → launch marble_spawner
  → [OnProcessExit: marble_spawner] → launch controller + plotter + lissajous
```

Each "phase" node is a `Node` action that exits when its job is done (e.g., `go_to_pose.py` calls `rclpy.shutdown()` after trajectory completes). Contrast with `marble_balancer.launch.py` (legacy), which uses fixed `TimerAction(period=15.0)` delays — fragile when system load varies.

**Key implementation:** `go_to_pose.py` and `marble_spawner.py` intentionally exit after one-shot work; their `OnProcessExit` events trigger the next phase.

---

## 4. LQR Control Architecture

**Pattern:** Linearize nonlinear ball-on-plate dynamics around the equilibrium, solve discrete-time algebraic Riccati equation (DARE) offline, apply `u = -K(x - x_d)` online.

**Where:** `lqr_math.py:compute_dlqr()`, `marble_servo_controller.py:control_callback()`

Design decisions:
- **Q weighting** (`lqr_math.py`): Position states penalized 20× more than velocity states; angle states penalized less than position — prioritizes marble position over plate angle
- **R = 5.0 * I₂**: Symmetric, moderate control effort cost
- **Saturation:** Output clipped to `MAX_RATE = 45 deg/s` before publishing
- **Setpoint tracking:** `x_d` comes from `/marble/desired_pos` (Lissajous) or zeros (balance at center)

The gain matrix K is computed once at node startup in `__init__` and reused every control tick. Recomputing per-tick is not necessary since the linearization is valid near equilibrium.

---

## 5. TRANSIENT_LOCAL QoS for State Events

**Pattern:** Use `TRANSIENT_LOCAL` durability (latched topics) for one-shot lifecycle events so late-subscribing nodes receive the last message.

**Where:** `marble_servo_controller.py`, `marble_plotter.py`, `marble_lissajous_node.py`

Topics: `/marble/landed`, `/marble/fell_off`

```python
qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
self.pub_landed = self.create_publisher(Empty, '/marble/landed', qos)
```

This is used consistently across all nodes that publish or subscribe to lifecycle events. `marble_plotter` and `marble_lissajous` subscribe with matching TRANSIENT_LOCAL QoS so they activate correctly even if they start after the marble lands.

---

## 6. TF-Based Coordinate Frame Extraction

**Pattern:** Derive all geometric quantities (plate angles, Jacobian axes, marble relative position) from TF lookups rather than hardcoding or tracking pose state manually.

**Where:** `marble_servo_controller.py:odom_callback()`, `marble_spawner.py:spawn_marble()`

Examples:
- Plate pitch/roll: quaternion from `world → plate_tcp` TF, decomposed to Euler angles
- Marble position relative to plate: subtract plate TCP position from marble world position
- Marble spawn point: look up `world → plate_tcp`, add 0.08m vertical offset
- Jacobian joint axes: look up `world → joint_i_frame` for each UR joint

**TF lookup pattern used everywhere:**
```python
t = self.tf_buffer.lookup_transform('world', 'plate_tcp', rclpy.time.Time())
# Use t.transform.translation, t.transform.rotation
```

---

## 7. One-Shot Node Pattern

**Pattern:** Nodes that perform initialization work (homing, spawning) execute their task, then call `rclpy.shutdown()` and exit cleanly to signal completion.

**Where:** `go_to_pose.py:main()`, `marble_spawner.py:main()`

These are not long-running nodes. They:
1. Initialize, subscribe to needed topics/services
2. Wait for prerequisites (joint states, TF, services available)
3. Execute action (send trajectory, call spawn service)
4. Wait for completion
5. `rclpy.shutdown()` → process exits → `OnProcessExit` event fires in launch system

This pattern is the mechanism that makes event-driven launch sequencing (pattern #3) work.

---

## 8. Servo Command Frame Convention

**Pattern:** LQR outputs angular rates in world frame; MoveIt Servo receives them as `delta_twist_cmds` in the `plate_tcp` frame with frame_id set to `"world"`.

**Where:** `marble_servo_controller.py:control_callback()`, `config/servo_params.yaml`

The TwistStamped message has:
- `header.frame_id = "world"` — tells servo the command is in world frame
- `twist.angular.x` = `ω_beta_cmd` (roll rate)
- `twist.angular.y` = `ω_alpha_cmd` (pitch rate)
- Linear components set to zero (pure rotation)

MoveIt Servo automatically transforms to joint space using the Jacobian. The `ee_frame_name: plate_tcp` in `servo_params.yaml` specifies which frame's angular velocity is being commanded.
