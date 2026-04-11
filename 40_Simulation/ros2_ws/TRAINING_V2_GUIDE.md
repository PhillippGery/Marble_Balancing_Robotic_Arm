# RL Training V2 — Complete Guide

## Overview

This is the **production-ready V2 training environment** for the marble balancing task. It includes bulletproof spawning, full TensorBoard logging, headless mode, and checkpoint resumption.

---

## Installation (One-Time)

```bash
cd ~/Marble_Balancing_Robotic_Arm/40_Simulation/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# Install Python deps
pip install gymnasium stable-baselines3[extra] tensorboard
```

---
[python3-9]   File "/home/Gery/Marble_Balancing_Robotic_Arm/40_Simulation/ros2_ws/install/marble_balancer/share/marble_balancer/../../../../src/marble_balancer/rl_training/train_td3_gazebo_v2.py", line 329, in <module>
[python3-9]     train(parser.parse_args())
[python3-9]   File "/home/Gery/Marble_Balancing_Robotic_Arm/40_Simulation/ros2_ws/install/marble_balancer/share/marble_balancer/../../../../src/marble_balancer/rl_training/train_td3_gazebo_v2.py", line 267, in train
[python3-9]     model.learn(
[python3-9]   File "/home/Gery/.local/lib/python3.10/site-packages/stable_baselines3/td3/td3.py", line 227, in learn
[python3-9]     return super().learn(
[python3-9]   File "/home/Gery/.local/lib/python3.10/site-packages/stable_baselines3/common/off_policy_algorithm.py", line 335, in learn
[python3-9]     rollout = self.collect_rollouts(
[python3-9]   File "/home/Gery/.local/lib/python3.10/site-packages/stable_baselines3/common/off_policy_algorithm.py", line 576, in collect_rollouts
[python3-9]     if not callback.on_step():
[python3-9]   File "/home/Gery/.local/lib/python3.10/site-packages/stable_baselines3/common/callbacks.py", line 115, in on_step
[python3-9]     return self._on_step()
[python3-9]   File "/home/Gery/.local/lib/python3.10/site-packages/stable_baselines3/common/callbacks.py", line 224, in _on_step
[python3-9]     continue_training = callback.on_step() and continue_training
[python3-9]   File "/home/Gery/.local/lib/python3.10/site-packages/stable_baselines3/common/callbacks.py", line 115, in on_step
[python3-9]     return self._on_step()
[python3-9]   File "/home/Gery/Marble_Balancing_Robotic_Arm/40_Simulation/ros2_ws/install/marble_balancer/share/marble_balancer/../../../../src/marble_balancer/rl_training/train_td3_gazebo_v2.py", line 158, in _on_step
[python3-9]     self.env.set_stage(self._stage)
[python3-9] AttributeError: 'Monitor' object has no attribute 'set_stage'
[ERROR] [python3-9]: process has died [pid 165696, exit code 1, cmd 'python3 /home/Gery/Marble_Balancing_Robotic_Arm/40_Simulation/ros2_ws/install/marble_balancer/share/marble_balancer/../../../../src/marble_balancer/rl_training/train_td3_gazebo_v2.py --timesteps 1000000 --stage 0 --seed-steps 20000 --load --spawn-radius 0.12 --tcp-lissajous true'].
## Quick Start: First Training Run

### Terminal 1 — Start Training (Headless)

```bash
cd ~/Marble_Balancing_Robotic_Arm/40_Simulation/ros2_ws
source install/setup.bash

# Standard: 1M steps, TCP Lissajous enabled, random spawn
ros2 launch marble_balancer rl_training.launch.py gui:=false \
  timesteps:=1000000 tcp_lissajous:=true spawn_radius:=0.12
```

Expected output shows marbles being spawned and training progressing.

Let it run to completion (~2-3 hours).

### Terminal 2 (Optional) — Monitor Progress

```bash
tensorboard --logdir ~/Marble_Balancing_Robotic_Arm/40_Simulation/ros2_ws/src/marble_balancer/rl_training/tensorboard_td3_v2/
```

Open browser: `http://localhost:6006`

---

## Model Storage

Models saved to: `src/marble_balancer/rl_training/models_td3_v2/`

**Deploy best model:**
```bash
ros2 launch marble_balancer servo_balancer.launch.py \
  rl:=true \
  rl_model:=$(pwd)/src/marble_balancer/rl_training/models_td3_v2/best_model_td3_v2.zip \
  rl_norm:=$(pwd)/src/marble_balancer/rl_training/models_td3_v2/running_stats_v2.pkl \
  rl_stage:=2
```

---

## Resume Training

```bash
ros2 launch marble_balancer rl_training.launch.py gui:=false \
  timesteps:=500000 \
  load:=$(pwd)/src/marble_balancer/rl_training/models_td3_v2/best_model_td3_v2.zip \
  tcp_lissajous:=true spawn_radius:=0.12
```

---

**See README.md and CHANGELOG_V2.md for full details.**
