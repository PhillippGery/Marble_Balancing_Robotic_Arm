# Testing & Evaluation Workflows

Quick reference for different evaluation scenarios.

## 1. Quick Model Validation (5-10 min)

**Goal:** Check if a new checkpoint converged or improved

**Command:**
```bash
python3 compare_controllers.py \
  --model models_td3_v2/checkpoint_td3_v2_500000_steps.zip \
  --episodes 3
```

**What it tests:**
- Basic functionality (no crashes)
- Reward trend direction
- No ROS2 timeouts

**Expected output:**
- ✅ 3 episodes complete in ~2 minutes
- ✅ RL vs LQR reward comparison table
- ✅ Win rate shown
- ❌ NOT statistically significant (only 3 episodes)

**Use when:**
- Periodic checkpoint sampling during long training runs
- Quick sanity check before full evaluation
- Debugging (model loads + runs without errors)

---

## 2. Comprehensive Model Evaluation (15-25 min)

**Goal:** Statistically significant performance metrics for model selection

**Command:**
```bash
python3 compare_controllers.py \
  --model models_td3_v2/best_model_td3_v2.zip \
  --episodes 10 \
  --tcp-lissajous true \
  --spawn-radius 0.12
```

**What it tests:**
- RL vs LQR on stationary + moving TCP
- Y-axis performance improvement (hardest case)
- Generalization to random marble spawn

**Expected output:**
- ✅ 10 episodes in ~5-7 minutes
- ✅ Clear win rate (typically 70-80% for trained models)
- ✅ Y-axis error: RL ~2.5cm vs LQR ~4.0cm (40% improvement)
- ✅ Standard deviations shown (assess variance)

**Use when:**
- Selecting best checkpoint from training
- Before deploying to servo_balancer.launch.py
- Documenting final performance
- Comparing two models head-to-head

---

## 3. Deployment Validation (20-30 min)

**Goal:** Verify model performs well under realistic conditions

**Command:**
```bash
# 1. Stationary (baseline)
python3 compare_controllers.py \
  --model models_td3_v2/best_model_td3_v2.zip \
  --episodes 5

# 2. Moving TCP (hardest)
python3 compare_controllers.py \
  --model models_td3_v2/best_model_td3_v2.zip \
  --episodes 5 \
  --tcp-lissajous true

# 3. Generalization (random spawn)
python3 compare_controllers.py \
  --model models_td3_v2/best_model_td3_v2.zip \
  --episodes 5 \
  --spawn-radius 0.18
```

**What it tests:**
- ✅ Baseline performance
- ✅ Hardest case (TCP Lissajous)
- ✅ Robustness to spawn variation

**Expected results:**
- Baseline: RL wins ~55-65%
- TCP Lissajous: RL wins ~70-80%
- Random spawn: RL wins ~60-70%

**Use when:**
- Final quality gate before production
- Deploying to real/production scenarios

---

## 4. Debugging (Varies)

**Problem:** Comparison crashes with timeout

**Diagnostic:**
```bash
# Test with single episode, verbose output
python3 compare_controllers.py \
  --model models_td3_v2/checkpoint_td3_v2_100000_steps.zip \
  --episodes 1
```

**Expected:** Should complete in ~30-40 seconds per episode
- If crashes: See error message (usually service timeout)
- If slow (>60s): Gazebo might be overloaded

**Solution:**
1. Stop any background training
2. Check no other compare_controllers running (only one can use Gazebo)
3. Restart ros2 daemon: `ros2 daemon stop && sleep 2 && ros2 daemon start`
4. Try again

---

## 5. Continuous Training + Periodic Evaluation

**Setup:**
```bash
# Terminal 1: Training
ros2 launch marble_balancer rl_training.launch.py gui:=false \
  timesteps:=1000000 tcp_lissajous:=true spawn_radius:=0.12

# Terminal 2: Periodic evaluation (while training runs)
cd src/marble_balancer/rl_training

# After 100K steps
sleep 300 && python3 compare_controllers.py \
  --model models_td3_v2/checkpoint_td3_v2_100000_steps.zip --episodes 3

# After 500K steps
sleep 1200 && python3 compare_controllers.py \
  --model models_td3_v2/checkpoint_td3_v2_500000_steps.zip --episodes 5 \
  --tcp-lissajous true

# After training completes
python3 compare_controllers.py \
  --model models_td3_v2/best_model_td3_v2.zip --episodes 10 \
  --tcp-lissajous true
```

**Advantage:** Track improvement over time, avoid waiting for full 1M steps to validate

---

## Performance Baselines

### Pure LQR (stage=0, no RL)
| Condition | RMS Error X | RMS Error Y | Max Angle | Notes |
|-----------|-----------|-----------|----------|-------|
| Stationary TCP | 1.8 cm | 1.8 cm | 8.5° | Baseline |
| TCP Lissajous | 1.8 cm | 4.0 cm | 18° | Y-axis struggles |
| Random spawn r=0.12m | 1.9 cm | 1.9 cm | 9° | Robust to spawn |

### RL + LQR (stage=2, λ=15°/s)
| Condition | RMS Error X | RMS Error Y | Max Angle | Improvement |
|-----------|-----------|-----------|----------|------------|
| Stationary TCP | 1.8 cm | 1.6 cm | 8° | 5-10% on Y |
| TCP Lissajous | 1.8 cm | 2.5 cm | 12° | **40% on Y** ⭐ |
| Random spawn r=0.12m | 1.8 cm | 1.7 cm | 8.5° | Generalizes well |

**Key insight:** RL primarily helps Y-axis when TCP accelerates (Lissajous case).

---

## Troubleshooting

### "IK service timeout" or "already registered"
**Cause:** Two environments running simultaneously or not enough sleep between episodes
**Fix:** Single environment pattern handles this — should not occur. If it does:
```bash
# Stop any other compare or training scripts
pkill -f compare_controllers.py
pkill -f train_td3

# Restart ROS2 daemon
ros2 daemon stop && sleep 2 && ros2 daemon start

# Try again with increased sleep (edit compare_controllers.py, line ~120)
# Change time.sleep(1.0) → time.sleep(2.0)
```

### "Model not found"
**Cause:** Wrong checkpoint name or training not complete
**Fix:** List available: `ls models_td3_v2/*.zip`

### "Normalization file not found"
**Cause:** File doesn't exist (OK to ignore if using raw observations)
**Info:** Only needed if model was trained with obs normalization

### "Gazebo is slow / each episode takes >60s"
**Cause:** System overloaded or physics step too large
**Fix:**
- Close other apps
- Check GPU: `nvidia-smi`
- In world file, ensure: `<max_step_size>0.01</max_step_size>`

---

## Deployment After Evaluation

Once you've selected a good model (e.g., best_model_td3_v2.zip):

```bash
# Deploy with servo_balancer.launch.py
ros2 launch marble_balancer servo_balancer.launch.py \
  rl:=true \
  rl_model:=$(pwd)/src/marble_balancer/rl_training/models_td3_v2/best_model_td3_v2.zip \
  rl_norm:=$(pwd)/src/marble_balancer/rl_training/models_td3_v2/running_stats_v2.pkl \
  rl_stage:=2 \
  tcp_lissajous:=true
```

This runs the **trained RL model** with live marble balancing in real-time.

---

**Last Updated:** 2026-04-11
