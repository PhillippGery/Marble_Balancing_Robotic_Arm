# Model Comparison & Evaluation Documentation Index

## Quick Links

### 🚀 Get Started Now
- **[QUICK_COMPARE.md](src/marble_balancer/rl_training/QUICK_COMPARE.md)** — 2-minute quick reference with copy-paste commands

### 📖 Full Guides
- **[COMPARE_CONTROLLERS_README.md](COMPARE_CONTROLLERS_README.md)** — Complete usage guide with examples, parameters, and troubleshooting
- **[RESOURCE_CONFLICT_FIX.md](src/marble_balancer/rl_training/RESOURCE_CONFLICT_FIX.md)** — Technical explanation of single-environment pattern
- **[TESTING_WORKFLOWS.md](src/marble_balancer/rl_training/TESTING_WORKFLOWS.md)** — Different evaluation scenarios and when to use each

### 📋 Reference
- **[COMPARISON_GUIDE.md](src/marble_balancer/rl_training/COMPARISON_GUIDE.md)** — Architecture reference and observation formats

---

## Workflow

### 1. **Quick Test** (5 min)
To verify a checkpoint works:
```bash
ros2 launch marble_balancer compare_controllers.launch.py \
  model:=models_td3_v2/checkpoint_td3_v2_500000_steps.zip \
  episodes:=3
```
→ See [QUICK_COMPARE.md](src/marble_balancer/rl_training/QUICK_COMPARE.md)

### 2. **Full Evaluation** (7-10 min)
To compare RL vs LQR comprehensively:
```bash
ros2 launch marble_balancer compare_controllers.launch.py \
  model:=models_td3_v2/best_model_td3_v2.zip \
  episodes:=10 \
  tcp_lissajous:=true
```
→ See [COMPARE_CONTROLLERS_README.md](COMPARE_CONTROLLERS_README.md)

### 3. **Deployment**
After selecting a good model:
```bash
ros2 launch marble_balancer servo_balancer.launch.py \
  rl:=true \
  rl_model:=$(pwd)/src/marble_balancer/rl_training/models_td3_v2/best_model_td3_v2.zip
```

---

## What's New (v2)

### Architecture Fix: Single Environment Pattern
- ✅ **Eliminated ROS2 node conflicts** — Now uses one environment with stage toggling
- ✅ **Fixed service timeouts** — Inter-episode sleep (1.0s) allows Gazebo to settle
- ✅ **Proper infrastructure setup** — New launch file handles Gazebo + MoveIt sequencing

### Before (Broken)
```
[env_rl node 1] + [env_lqr node 2] = CONFLICT ❌
```

### After (Fixed)
```
[Single env] → set_stage(2) → RL episode
           → [sleep 1.0s] → Gazebo settles
           → set_stage(0) → LQR episode ✅
```

---

## File Structure

```
40_Simulation/ros2_ws/
├── COMPARE_CONTROLLERS_README.md          ← Full usage guide
│
├── src/marble_balancer/
│   ├── launch/
│   │   └── compare_controllers.launch.py  ← Infrastructure launcher (NEW!)
│   │
│   └── rl_training/
│       ├── compare_controllers.py         ← Main comparison script (FIXED)
│       ├── gazebo_rl_env_v2.py           ← Environment (ENHANCED)
│       ├── QUICK_COMPARE.md              ← TL;DR quick reference
│       ├── COMPARISON_GUIDE.md           ← Architecture reference
│       ├── RESOURCE_CONFLICT_FIX.md      ← Technical deep-dive
│       └── TESTING_WORKFLOWS.md          ← Evaluation scenarios
```

---

## Key Parameters

All parameters go in the launch command:

```bash
ros2 launch marble_balancer compare_controllers.launch.py \
  model:=<PATH>                 # Model file (REQUIRED)
  episodes:=5                   # Number to compare (default 5)
  tcp_lissajous:=true          # Enable TCP movement (default false)
  spawn_radius:=0.12           # Marble spawn spread (default 0.12)
  gui:=false                   # Show Gazebo GUI (default false)
```

---

## Expected Results

| Condition | RL Win Rate | Y Error Improvement |
|-----------|-----------|-------------------|
| Stationary TCP | 55-65% | 5-10% |
| TCP Lissajous (hardest) | 70-80% | **40%** ⭐ |
| Random spawn | 60-70% | 5-15% |

---

## Performance Baselines

### Pure LQR (stage=0, no RL)
- RMS Error X: 1.8 cm
- RMS Error Y: 1.8 cm (3.5-4.0 cm with TCP Lissajous)
- Max angle: ~8.5° (18° with TCP Lissajous)

### RL + LQR (stage=2, λ=15°/s)
- RMS Error X: 1.8 cm (unchanged)
- RMS Error Y: 1.6 cm (2.5 cm with TCP Lissajous) ← **Better!**
- Max angle: ~8° (12° with TCP Lissajous)

---

## Troubleshooting

### Services timeout
**Cause:** Infrastructure not running
**Solution:** Use `compare_controllers.launch.py` (handles setup for you)

### Gazebo is very slow
**Cause:** System overloaded or render overhead
**Solution:** 
- Use `gui:=false` (headless, 3x faster)
- Check world file has `<max_step_size>0.01</max_step_size>`

### Can't find model file
**Cause:** Wrong checkpoint name
**Solution:** `ls src/marble_balancer/rl_training/models_td3_v2/`

→ See [COMPARE_CONTROLLERS_README.md](COMPARE_CONTROLLERS_README.md#troubleshooting) for more

---

## Training Integration

Run comparison while training continues in another terminal:

**Terminal 1: Training**
```bash
ros2 launch marble_balancer rl_training.launch.py gui:=false \
  timesteps:=1000000 tcp_lissajous:=true spawn_radius:=0.12
```

**Terminal 2: Periodic evaluation**
```bash
# After 500K steps (check models_td3_v2 directory)
ros2 launch marble_balancer compare_controllers.launch.py \
  model:=models_td3_v2/checkpoint_td3_v2_500000_steps.zip \
  episodes:=5 \
  tcp_lissajous:=true
```

→ See [TESTING_WORKFLOWS.md](src/marble_balancer/rl_training/TESTING_WORKFLOWS.md#5-continuous-training--periodic-evaluation)

---

## Deployment After Evaluation

Once you've found a good model through comparison:

```bash
ros2 launch marble_balancer servo_balancer.launch.py \
  rl:=true \
  rl_model:=$(pwd)/src/marble_balancer/rl_training/models_td3_v2/best_model_td3_v2.zip \
  rl_norm:=$(pwd)/src/marble_balancer/rl_training/models_td3_v2/running_stats_v2.pkl \
  rl_stage:=2 \
  tcp_lissajous:=true
```

This runs the trained RL model in **live marble balancing** with visual feedback.

---

## Documentation Versions

| Document | Purpose | Best For |
|----------|---------|----------|
| QUICK_COMPARE.md | TL;DR copy-paste | First-time users |
| COMPARE_CONTROLLERS_README.md | Complete usage guide | Detailed learning |
| RESOURCE_CONFLICT_FIX.md | Technical deep-dive | Understanding architecture |
| TESTING_WORKFLOWS.md | Scenario-based guide | Different test cases |
| COMPARISON_GUIDE.md | API reference | Debugging/development |

---

## Files Changed

### New Files
- `src/marble_balancer/launch/compare_controllers.launch.py` (140 lines)
- `COMPARE_CONTROLLERS_README.md` (workspace root)
- `src/marble_balancer/rl_training/QUICK_COMPARE.md`
- `src/marble_balancer/rl_training/RESOURCE_CONFLICT_FIX.md` (created earlier)

### Updated Files
- `src/marble_balancer/rl_training/compare_controllers.py` (single env pattern)
- `src/marble_balancer/rl_training/gazebo_rl_env_v2.py` (service robustness)

---

**Last Updated:** 2026-04-11
**Version:** 2.0 (Single Environment Pattern)
