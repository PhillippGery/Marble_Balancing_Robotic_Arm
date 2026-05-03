#!/usr/bin/env python3
"""Real training learning curve from TD3_gazebo_v2_15 (rollout/ep_rew_mean).

This replaces the previous ``eval_reward'' figure, whose evaluations.npz
reward used a different scaling and looked artificially flat.
"""
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator, STORE_EVERYTHING_SIZE_GUIDANCE
import numpy as np, matplotlib.pyplot as plt, matplotlib as mpl

mpl.rcParams.update({"font.family":"serif","font.size":9,"axes.labelsize":9,
                     "axes.titlesize":10,"xtick.labelsize":8,"ytick.labelsize":8,
                     "legend.fontsize":8,"axes.linewidth":0.7,"savefig.bbox":"tight",
                     "savefig.dpi":300})

RUN = "/sessions/festive-adoring-bardeen/mnt/Marble_Balancing_Robotic_Arm/40_Simulation/ros2_ws/src/marble_balancer/rl_training/tensorboard_td3_v2/TD3_gazebo_v2_15"
ea = EventAccumulator(RUN, size_guidance=STORE_EVERYTHING_SIZE_GUIDANCE); ea.Reload()


def series(tag):
    s = ea.Scalars(tag)
    return np.array([x.step for x in s]), np.array([x.value for x in s])


def ema(x, alpha=0.10):
    y = np.empty_like(x, dtype=float); y[0] = x[0]
    for i in range(1, len(x)): y[i] = alpha*x[i] + (1-alpha)*y[i-1]
    return y


steps_r, rew = series("rollout/ep_rew_mean")
steps_l, eplen = series("rollout/ep_len_mean")

fig, axes = plt.subplots(1, 2, figsize=(7.0, 2.7))
SHOCK = 613_000

# (a) Episode reward
ax = axes[0]
ax.plot(steps_r/1e3, rew, color="#1f77b4", lw=0.6, alpha=0.35, label="raw")
ax.plot(steps_r/1e3, ema(rew), color="#1f77b4", lw=1.6, label="EMA ($\\alpha{=}0.10$)")
ax.axhline(0, color="black", lw=0.4, alpha=0.5)
ax.axvline(SHOCK/1e3, color="#c0392b", linestyle="--", lw=1.0, alpha=0.85)
ax.axvline(200, color="#2ca02c", linestyle=":", lw=1.0, alpha=0.85)
ax.text(SHOCK/1e3, ax.get_ylim()[1]*0.92, " Stage-2 onset (613k)",
        color="#c0392b", fontsize=7.5, va="top", ha="left")
ax.text(200, ax.get_ylim()[1]*0.78, " 200k checkpoint\n(eval point)",
        color="#2ca02c", fontsize=7.5, va="top", ha="left")
ax.set_xlabel("Training step ($\\times 10^3$)")
ax.set_ylabel("Mean episode reward")
ax.set_title("(a) Training reward (rollout)")
ax.grid(True, alpha=0.3)
ax.legend(loc="lower left", framealpha=0.95)

# (b) Episode length over training
ax = axes[1]
ax.plot(steps_l/1e3, eplen, color="#2ca02c", lw=0.6, alpha=0.35)
ax.plot(steps_l/1e3, ema(eplen), color="#2ca02c", lw=1.6, label="EMA")
ax.axhline(500, color="black", lw=0.4, ls=":", alpha=0.5, label="episode horizon $H{=}500$")
ax.axvline(SHOCK/1e3, color="#c0392b", linestyle="--", lw=1.0, alpha=0.85)
ax.axvline(200, color="#2ca02c", linestyle=":", lw=1.0, alpha=0.85)
ax.set_xlabel("Training step ($\\times 10^3$)")
ax.set_ylabel("Mean episode length (steps)")
ax.set_title("(b) Survival horizon over training")
ax.grid(True, alpha=0.3)
ax.legend(loc="lower left", framealpha=0.95)

fig.tight_layout()
out = "/sessions/festive-adoring-bardeen/mnt/outputs/paper/figures/training_curve.pdf"
fig.savefig(out); fig.savefig(out.replace(".pdf",".png"))
print("saved:", out)

# Stats for paper text
print(f"reward at 200k: {rew[steps_r==200000][0] if 200000 in steps_r else 'no exact':}")
mask = (steps_r>=180_000) & (steps_r<=220_000)
print(f"reward in 180-220k window: mean {rew[mask].mean():.1f}")
