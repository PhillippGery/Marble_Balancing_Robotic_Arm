#!/usr/bin/env python3
"""Eval-reward learning curve from evaluations.npz to corroborate training dynamics
and to flag the 200k checkpoint that produced the head-to-head results."""
import numpy as np, matplotlib.pyplot as plt, matplotlib as mpl
mpl.rcParams.update({"font.family": "serif", "font.size": 9, "axes.labelsize": 9,
                     "axes.titlesize": 10, "xtick.labelsize": 8, "ytick.labelsize": 8,
                     "legend.fontsize": 8, "axes.linewidth": 0.7, "savefig.bbox": "tight",
                     "savefig.dpi": 300})

PATH = "/sessions/festive-adoring-bardeen/mnt/Marble_Balancing_Robotic_Arm/40_Simulation/ros2_ws/src/marble_balancer/rl_training/models/eval_logs/evaluations.npz"
d = np.load(PATH)
ts = d["timesteps"]
mean = d["results"].mean(axis=1)
std  = d["results"].std(axis=1)

fig, ax = plt.subplots(figsize=(6.8, 2.6))
ax.fill_between(ts/1e3, mean-std, mean+std, alpha=0.18, color="#1f77b4", lw=0)
ax.plot(ts/1e3, mean, color="#1f77b4", lw=1.2, label="mean over 20 eval episodes")
ax.axvline(200, color="#d62728", linestyle="--", lw=0.9, label="200k checkpoint (Precision Damper)")
ax.axvspan(600, 700, alpha=0.10, color="#c0392b", lw=0, label="Stage-1 transition window")
ax.set_xlabel("Training step (×$10^3$)")
ax.set_ylabel("Eval episode reward")
ax.set_title("Evaluation reward over training (20 eval episodes per checkpoint)")
ax.grid(True, alpha=0.3)
ax.legend(loc="lower right", framealpha=0.95)
fig.tight_layout()

out = "/sessions/festive-adoring-bardeen/mnt/outputs/paper/figures/eval_reward.pdf"
fig.savefig(out); fig.savefig(out.replace(".pdf", ".png"))
print("saved:", out)
print(f"reward at 200k: {mean[ts==200000][0]:.2f} ± {std[ts==200000][0]:.2f}")
print(f"reward peak: {mean.max():.2f} at step {ts[np.argmax(mean)]}")
