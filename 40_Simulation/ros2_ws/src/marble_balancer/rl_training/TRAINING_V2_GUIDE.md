# Training with gazebo_rl_env_v2.py — Quick Start Guide

## ✅ Files Created

### Core Environment
- **`gazebo_rl_env_v2.py`** — New 21-D observation environment with 3D Random Jitter Walk & Lissajous disturbance

### Training Scripts (V2 versions, non-destructive)
- **`train_td3_gazebo_v2.py`** — TD3 online training (uses v2 env)
- **`train_cmaes_gazebo_v2.py`** — CMA-ES validator (uses v2 env, 21→2 linear policy, 44 params)

### Original V1 Files (Unchanged)
- `gazebo_rl_env.py` — Original 32-D env (still works)
- `train_td3_gazebo.py` — TD3 v1 (still works)
- `train_cmaes_gazebo.py` — CMA-ES v1 (still works)

---

## 🚀 Quick Start Training

### Prerequisites
```bash
# Install dependencies (once)
pip install stable-baselines3[extra] tensorboard gymnasium cma

# Source ROS2 workspace
cd ~/Marble_Balancing_Robotic_Arm/40_Simulation/ros2_ws
source install/setup.bash
```

### Basic Training (V2 environment)
```bash
cd src/marble_balancer/rl_training

# IMPORTANT: Always launch with gui:=false to avoid 300% slowdown!
# In terminal 1: Start Gazebo + MoveIt + servo infrastructure (HEADLESS)
ros2 launch marble_balancer rl_training.launch.py gui:=false

# In terminal 2: Run training
python train_td3_gazebo_v2.py --timesteps 100000

# Resume training from checkpoint
python train_td3_gazebo_v2.py \
  --timesteps 1000000 \
  --load models_td3_v2/best_model_td3_v2.zip \
  --seed-steps 0
```

### Key Command-Line Options
```
--timesteps N           Total training steps (default 500 000)
--stage S               Start at curriculum stage 0-2 (default 0)
--seed-steps N          LQR pre-seeding steps (default 20 000)
--tcp-lissajous true    Enable 50% Lissajous + 50% Random Walk (default false)
--spawn-radius R        Random marble spawn radius (default 0, recommended 0.12 m)
--use-ekf              Use EKF state instead of EMA velocity
--load PATH            Load existing model to resume training
```

---

## 📊 What's Different in V2?

### Observation Space
| Feature | V1 (32-D) | V2 (21-D) |
|---------|-----------|----------|
| Marble state | 8-D | 8-D (same) |
| Action history | 10 × 2D = 20-D | removed |
| Twist command | 4-D | removed |
| **TCP velocity window (2D)** | ❌ | removed |
| **TCP 3D velocity window** | ❌ | **9-D (NEW: X, Y, Z)** |
| **Marble target** | ❌ | **2-D (NEW)** |
| **Prev action** | ❌ | **2-D (NEW)** |

### Training Data & Dynamics (V2 NEW: Full 3D Movement!)
| Feature | V1 | V2 |
|---------|----|----|
| TCP movement | Lissajous only | **50% Lissajous + 50% Random Jitter Walk** |
| **TCP Axes** | X, Y only | **X, Y, Z (full 3D)** |
| **Z Lissajous** | ❌ | **0.02 m sin oscillation** |
| **Z Jitter Range** | ❌ | **±0.05 m/s** |
| Disturbance variety | Limited | **More aggressive, varied accelerations in 3D** |

### Reward Function
- V1: Simple pos + vel + smoothness
- V2: **exp(-100.0 * r²) centering + tilt_penalty (-0.2, increased) + potential shaping**
- **Tilt penalty doubled** to prevent RL from "cheating" by using extreme angles during Z-axis jitter

---

## 🏃 Training Modes

### FAST (Recommended for Development)
```bash
# Uses offline simulator (100× faster than Gazebo, ~10s per episode)
python train_cmaes_gazebo_v2.py --offline  # default
```

### SLOW (Gazebo Real Sim — Use for Validation Only)
```bash
# Uses live Gazebo (1-2 min per episode, 100+ hours total)
# ONLY for final sim-to-real validation after training
python train_cmaes_gazebo_v2.py --no-offline --td3-model models_td3_v2/best_model_td3_v2.zip
```

---

## ⚡ Headless Gazebo Speedup (2-3× Faster) — **CRITICAL FOR TRAINING!**

**⚠️ If Gazebo GUI is visible during training, it will slow everything by ~300%. Always launch headless.**

To enable faster-than-realtime training:

1. **ALWAYS use `gui:=false`:**
```bash
ros2 launch marble_balancer rl_training.launch.py gui:=false
```

2. Optional: Edit Gazebo world `.sdf` file for 2-3× physics speedup:
```xml
<physics>
  <real_time_update_rate>0</real_time_update_rate>   <!-- Remove RT limit -->
  <max_step_size>0.01</max_step_size>                <!-- 10 ms steps (30 Hz servo) -->
</physics>
```

This allows Gazebo to run faster than wall-clock time, accelerating training significantly.

---

## 📁 Output Files

Training creates:

```
models_td3_v2/
├── best_model_td3_v2.zip           # Best policy
├── final_model_td3_v2.zip          # Final policy
├── running_stats_v2.pkl            # Obs normalization stats
├── checkpoint_td3_v2_100000.zip    # Periodic checkpoints
└── checkpoint_td3_v2_200000.zip

tensorboard_td3_v2/                 # Monitor with:
└── ...                             # tensorboard --logdir tensorboard_td3_v2/
```

---

## 🚁 Deploy Trained Model

After training completes:

```bash
ros2 launch marble_balancer servo_balancer.launch.py \
  rl:=true \
  rl_model:=$(pwd)/src/marble_balancer/rl_training/models_td3_v2/best_model_td3_v2.zip \
  rl_norm:=$(pwd)/src/marble_balancer/rl_training/models_td3_v2/running_stats_v2.pkl \
  rl_stage:=2 \
  tcp_circle:=true \
  tcp_circle_radius:=0.10 \
  tcp_circle_period:=10.0
```

---

## ⚠️ Important Notes

1. **V1 & V2 Models Are Incompatible**
   - Old v1 models (.zip files) expect 32-D obs
   - **V2 models expect 21-D obs (with 3D TCP velocity window)**
   - Train from scratch with v2 — don't try to load v1 models

2. **Observation Dimension Grew to 21-D**
   - 8-D marble state (unchanged)
   - **9-D TCP 3D velocity window** (was 6-D in v1, now includes Z-axis)
   - 2-D marble target
   - 2-D action history
   - Total: 8 + 9 + 2 + 2 = **21-D**

3. **V1 Training Still Works**
   - Original files unchanged
   - Use `train_td3_gazebo.py` (v1) if you want 32-D obs

4. **No Changes to LQR**
   - `lqr_math.py` untouched
   - Only RL observation/reward changed

5. **Curriculum Stages**
   - Auto-advances when survival fraction > thresholds
   - Stage 0: λ = 5°/s (small residuals)
   - Stage 1: λ = 10°/s (medium residuals)
   - Stage 2: λ = 15°/s (full residuals)

6. **Tilt Penalty Doubled for 3D Robustness**
   - Increased from -0.1 to -0.2 to prevent RL from "cheating" with extreme angles
   - Compensates for new Z-axis disturbance

---

## 🐛 Troubleshooting

### "gazebo_rl_env_v2 not found"
```bash
# Make sure you're in the rl_training directory
cd src/marble_balancer/rl_training
python train_td3_gazebo_v2.py
```

### "Observation dimension mismatch"
- **V2 expects 21-D obs** (was 18-D before 3D TCP window addition)
- Check that `gazebo_rl_env_v2.py` defines `observation_space` correctly
- Running stats .pkl files from v1 or old v2 won't work with updated v2 (different shape)
- If you get shape mismatch: delete old `running_stats_v2.pkl` and retrain

### Training is very slow
```bash
# 1. Use offline mode (CMA-ES only)
python train_cmaes_gazebo_v2.py --offline

# 2. Enable Gazebo headless mode (see above)
# 3. Use smaller --timesteps for testing
python train_td3_gazebo_v2.py --timesteps 10000
```

---

## 📚 Additional Resources

- **Architecture**: `.claude/docs/architectural_patterns.md`
- **LQR tuning**: See custom instructions (Q/R matrices in `lqr_math.py`)
- **RL theory**: See comments in `gazebo_rl_env_v2.py` (TCP velocity window logic, reward design)

---

**Ready to train! 🚀**
