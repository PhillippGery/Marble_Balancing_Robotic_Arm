# Compare RL vs Pure LQR - Clean Workflow

## Quick Start

```bash
cd ~/Marble_Balancing_Robotic_Arm/40_Simulation/ros2_ws

# Test a checkpoint (5 episodes, ~5 minutes)
ros2 launch marble_balancer compare_rl_vs_lqr.launch.py \
  model:=models_td3_v2/checkpoint_td3_v2_500000_steps.zip

# Full evaluation (10 episodes with TCP movement, ~7-10 minutes)
ros2 launch marble_balancer compare_rl_vs_lqr.launch.py \
  model:=models_td3_v2/best_model_td3_v2.zip \
  episodes:=10 \
  tcp_lissajous:=true

# With Gazebo GUI (for debugging)
ros2 launch marble_balancer compare_rl_vs_lqr.launch.py \
  model:=models_td3_v2/best_model_td3_v2.zip \
  episodes:=3 \
  gui:=true
```

## What It Does

1. **Launches Infrastructure** (automatic, no manual setup)
   - Gazebo with UR5e robot
   - MoveIt motion planning
   - MoveIt Servo control
   - Arm homing

2. **Runs Comparison** (after infrastructure ready)
   - Episode N with RL enabled (stage 2, λ = 15°/s)
   - Sleep 1.0s (Gazebo settles)
   - Episode N with LQR only (stage 0, λ = 0)
   - Sleep 1.0s (Gazebo settles)
   - Repeat for all episodes

3. **Reports Metrics**
   - Episode-by-episode: reward, max error (X/Y axes), winner
   - Summary: mean ± std, improvement %, win rate

## Expected Output

```
================================================================================
  RL Model Comparison (RL vs Pure LQR)
================================================================================

Model: best_model_td3_v2.zip
Episodes: 5
TCP Lissajous: true
Spawn Radius: 0.12m

================================================================================
  Running Comparison
================================================================================

Episode    RL Reward       LQR Reward      RL Max Err       LQR Max Err      Winner    
1          +2.451          +1.893          0.0231          0.0310           RL ✓      
2          +1.923          +2.104          0.0200          0.0280           LQR ✗     
3          +2.187          +1.756          0.0215          0.0325           RL ✓      
4          +1.845          +1.612          0.0242          0.0295           RL ✓      
5          +2.034          +1.789          0.0219          0.0312           RL ✓      

================================================================================
  SUMMARY METRICS
================================================================================

REWARD:
  RL  (avg): +2.088 ± 0.272
  LQR (avg): +1.831 ± 0.201
  Improvement: +14.0%

MAX ERROR (m):
  RL  (avg): 0.0221 ± 0.0018
  LQR (avg): 0.0304 ± 0.0019
  Improvement: +27.3%

WIN RATE:
  RL:  4/5 (80%)
  LQR: 1/5 (20%)
```

## Parameters

| Param | Type | Default | Description |
|-------|------|---------|-------------|
| `model` | string | (required) | Model .zip file path |
| `episodes` | int | 5 | Episodes to compare |
| `tcp_lissajous` | bool | false | TCP movement during comparison |
| `spawn_radius` | float | 0.12 | Marble spawn spread (m) |
| `gui` | bool | false | Show Gazebo GUI |

## Interpreting Results

### Metrics Explained

- **Reward**: Sum of episode rewards. Higher = better centering.
- **Max Error**: Largest marble distance from center (meters). Lower = better.
- **Win Rate**: Percentage of episodes RL beats LQR.

### Performance Expectations

| Scenario | RL Win % | Y-Axis Improvement |
|----------|---------|------------------|
| Stationary | 55-65% | 5-10% |
| TCP Lissajous (hardest) | 70-80% | **25-40%** ⭐ |
| Random spawn (r=0.18) | 60-70% | 10-20% |

### Interpretation

- **RL Win% > 70%**: RL model is well-trained
- **Improvement > 20%**: Significant benefit on hard axis (Y with TCP movement)
- **Consistent across spawn positions**: Good generalization

## Troubleshooting

### "Model not found"
```bash
# List available models
ls src/marble_balancer/rl_training/models_td3_v2/*.zip

# Use correct path (e.g.,)
model:=models_td3_v2/best_model_td3_v2.zip
```

### "IK service timeout"
```bash
# Clean up stale processes
pkill -9 gzserver gzclient
ros2 daemon stop && sleep 2 && ros2 daemon start

# Try with 1 episode for speed
episodes:=1
```

### Each episode takes >60 seconds
- Gazebo is slow (check system load)
- Use `gui:=false` (headless is 3x faster)
- Reduce episodes: `episodes:=1` for quick test

### Process dies immediately
```bash
# Check Gazebo started
ps aux | grep gzserver

# Run with GUI to see errors
gui:=true episodes:=1
```

## Stopping

Press **Ctrl+C** to stop the comparison.

Gazebo will clean up automatically.

## Deployment After Testing

Once you find a good model:

```bash
ros2 launch marble_balancer servo_balancer.launch.py \
  rl:=true \
  rl_model:=$(pwd)/src/marble_balancer/rl_training/models_td3_v2/best_model_td3_v2.zip \
  rl_stage:=2 \
  tcp_lissajous:=true
```

This runs live marble balancing with the trained RL model.

## Files

- **compare_rl_vs_lqr.launch.py** — Infrastructure launcher
- **compare_rl_vs_lqr.py** — Comparison engine with metrics

---

**Version:** 2.0 (Clean, Simple, Metrics-Focused)
**Last Updated:** 2026-04-11
