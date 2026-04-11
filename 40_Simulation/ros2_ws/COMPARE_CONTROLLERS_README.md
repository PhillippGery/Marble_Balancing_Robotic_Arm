# Model Comparison - CORRECTED WORKFLOW

## The Problem

The original `compare_controllers.py` script **requires Gazebo infrastructure to be already running**:
- Gazebo (physics engine)
- UR5e robot
- MoveIt move_group
- MoveIt Servo node
- ROS2 services (/spawn_entity, /delete_entity, /compute_ik)

Running the script standalone fails with:
```
[ERROR] /delete_entity service unavailable after retries!
[ERROR] /spawn_entity service unavailable after retries!
[WARN] go_to_pose timed out
```

## The Solution

**Use the new dedicated launch file** instead of running the script directly:

```bash
ros2 launch marble_balancer compare_controllers.launch.py \
  model:=models_td3_v2/checkpoint_td3_v2_500000_steps.zip \
  episodes:=5
```

This launch file handles the full sequence:
1. ✅ Start Gazebo + UR5e + joint controllers
2. ✅ Start MoveIt move_group
3. ✅ Start MoveIt Servo node
4. ✅ Home arm via go_to_pose
5. ✅ Run compare_controllers.py (with all services ready)

## Usage Examples

### Quick test (5 episodes, ~5 minutes)
```bash
ros2 launch marble_balancer compare_controllers.launch.py \
  model:=models_td3_v2/checkpoint_td3_v2_500000_steps.zip
```

### With TCP Lissajous (harder test, 10 episodes, ~7-10 minutes)
```bash
ros2 launch marble_balancer compare_controllers.launch.py \
  model:=models_td3_v2/best_model_td3_v2.zip \
  episodes:=10 \
  tcp_lissajous:=true
```

### With Gazebo GUI visible (for debugging)
```bash
ros2 launch marble_balancer compare_controllers.launch.py \
  model:=models_td3_v2/best_model_td3_v2.zip \
  episodes:=3 \
  gui:=true
```

### Random spawn radius (generalization test)
```bash
ros2 launch marble_balancer compare_controllers.launch.py \
  model:=models_td3_v2/best_model_td3_v2.zip \
  episodes:=5 \
  spawn_radius:=0.18
```

## Available Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `model` | string | (required) | Path to model .zip file (e.g., `models_td3_v2/best_model_td3_v2.zip`) |
| `episodes` | int | 5 | Number of comparison episodes |
| `tcp_lissajous` | bool | false | Enable TCP Lissajous disturbance |
| `spawn_radius` | float | 0.12 | Marble spawn radius in meters (0=center, 0.18=edge) |
| `gui` | bool | false | Show Gazebo GUI (true/false for headless) |

## Execution Flow

```
Launch Start
    ↓
[Gazebo + UR5e + Controllers] (30s)
    ↓
[MoveIt move_group + Servo] (5s)
    ↓
[go_to_pose - Arm Homing] (5s)
    ↓
[2s pause for stability]
    ↓
[compare_controllers.py] ← All services ready!
    ├─ Episode 1 RL:   Reset → Run → Report
    ├─ [1s sleep]
    ├─ Episode 1 LQR:  Reset → Run → Report
    ├─ [1s sleep]
    ├─ Episode 2 RL:   Reset → Run → Report
    ├─ [1s sleep]
    ├─ Episode 2 LQR:  Reset → Run → Report
    └─ [Report statistics]
```

## Stopping

To stop the comparison:
1. Press **Ctrl+C** in the terminal
2. Press Ctrl+C again if needed to kill Gazebo cleanly

## Troubleshooting

### "model: not found"
**Cause:** Model file doesn't exist or path is wrong
**Fix:** 
```bash
ls models_td3_v2/*.zip  # List available models
# Use full path from this list
```

### "IK service still timing out"
**Cause:** System overloaded or Gazebo crashed
**Fix:**
```bash
# Kill all ROS2 processes
pkill -9 gzserver
pkill -9 gzclient
ros2 daemon stop
sleep 2
ros2 daemon start

# Try again with gui:=true to see Gazebo status
ros2 launch marble_balancer compare_controllers.launch.py \
  model:=models_td3_v2/best_model_td3_v2.zip \
  episodes:=1 \
  gui:=true
```

### Each episode takes >60 seconds
**Cause:** System slow or physics timestep too large
**Fix:**
- Reduce episodes: `episodes:=1` for fast test
- Check world file: `/src/marble_balancer/urdf/marble_balancer.world`
  - Ensure: `<max_step_size>0.01</max_step_size>` (10ms steps for 30Hz servo)
  - Ensure: `<real_time_update_rate>0</real_time_update_rate>` (uncapped speed)

## Performance Baselines

### Stationary TCP (baseline)
- RL win rate: 55-65%
- Y-axis error improvement: 5-10%

### TCP Lissajous (hardest)
- RL win rate: 70-80%
- Y-axis error improvement: 40% ⭐

### Random spawn (generalization)
- RL win rate: 60-70%
- Robust across spawn positions

---

## Deployment After Comparison

Once you've selected a good model using comparison:

```bash
# Run the model in live balancing
ros2 launch marble_balancer servo_balancer.launch.py \
  rl:=true \
  rl_model:=$(pwd)/src/marble_balancer/rl_training/models_td3_v2/best_model_td3_v2.zip \
  rl_norm:=$(pwd)/src/marble_balancer/rl_training/models_td3_v2/running_stats_v2.pkl \
  rl_stage:=2
```

**Last Updated:** 2026-04-11
