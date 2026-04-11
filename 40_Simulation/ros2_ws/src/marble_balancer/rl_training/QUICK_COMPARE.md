# Quick Reference: Model Comparison

## TL;DR - Just Run This

```bash
cd ~/Marble_Balancing_Robotic_Arm/40_Simulation/ros2_ws

# Quick test (5 episodes, ~5 min)
ros2 launch marble_balancer compare_controllers.launch.py \
  model:=models_td3_v2/checkpoint_td3_v2_500000_steps.zip

# Full evaluation (10 episodes, ~7 min)
ros2 launch marble_balancer compare_controllers.launch.py \
  model:=models_td3_v2/best_model_td3_v2.zip \
  episodes:=10 \
  tcp_lissajous:=true
```

## What Just Happens

1. Gazebo starts (robot simulation)
2. MoveIt + Servo start (motion planning & control)
3. Arm homes to balance position
4. Comparison runs:
   - Episode 1 with RL enabled (stage 2)
   - Episode 1 with LQR only (stage 0)
   - Reports which is better
   - Repeats for N episodes

## Output Shows

```
Episode    RL Reward       LQR Reward      RL Max Err   LQR Max Err  Winner  
1          +2.451          +1.893          0.0231      0.0310       RL ✓
2          +1.923          +2.104          0.0200      0.0280       LQR ✗
3          +2.187          +1.756          0.0215      0.0325       RL ✓

[Summary statistics with win rate and improvement %]
```

## Common Commands

| Task | Command |
|------|---------|
| Test checkpoint at 500K steps | `model:=models_td3_v2/checkpoint_td3_v2_500000_steps.zip episodes:=3` |
| Test best model | `model:=models_td3_v2/best_model_td3_v2.zip episodes:=10` |
| Hard test (TCP moves) | `... tcp_lissajous:=true` |
| See robot in GUI | `... gui:=true` |
| Random spawn spots | `... spawn_radius:=0.18` |
| Just 1 episode (fast) | `... episodes:=1` |

## Expected Results

- **RL wins 50-80%** of episodes (depending on conditions)
- **Y-axis improves 5-40%** (most benefit with TCP movement)
- **Total time: ~5-10 min** (includes infrastructure startup + episodes)

## If Something Breaks

```bash
# Kill everything
pkill -9 gzserver
pkill -9 gzclient
ros2 daemon stop && sleep 2 && ros2 daemon start

# Try again with verbose output
ros2 launch marble_balancer compare_controllers.launch.py \
  model:=models_td3_v2/best_model_td3_v2.zip \
  episodes:=1 \
  gui:=true
```

## See Also

- **COMPARE_CONTROLLERS_README.md** — Full documentation
- **RESOURCE_CONFLICT_FIX.md** — Technical details on single-env fix
- **TESTING_WORKFLOWS.md** — When to use comparison
- **COMPARISON_GUIDE.md** — Architecture reference

---

**Latest:** 2026-04-11
