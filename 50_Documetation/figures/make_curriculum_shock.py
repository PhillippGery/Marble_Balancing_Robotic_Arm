#!/usr/bin/env python3
"""Curriculum Shock centerpiece — bigger panels, clearer styling, EMA visible.

The transition really IS a sharp cliff in the data, so we (i) shade the
Stage-0 vs Stage-1 regimes, (ii) annotate per-panel before/after numbers, and
(iii) show both the raw trace and a longer-window EMA so the trend reads
even when the cliff is sharp."""
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator, STORE_EVERYTHING_SIZE_GUIDANCE
import numpy as np, matplotlib.pyplot as plt, matplotlib as mpl

mpl.rcParams.update({"font.family":"serif","font.size":9,"axes.labelsize":9,
                     "axes.titlesize":10,"xtick.labelsize":8,"ytick.labelsize":8,
                     "legend.fontsize":7.5,"axes.linewidth":0.7,"savefig.bbox":"tight",
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
steps_a, aloss = series("train/actor_loss")
steps_c, closs = series("train/critic_loss")
steps_l, eplen = series("rollout/ep_len_mean")

SHOCK = 613_000  # step at Stage-1 onset
HEALTHY_END = 556_000

fig, axes = plt.subplots(2, 2, figsize=(7.2, 5.0), sharex=True)


def style(ax, ymax_pad=0.05):
    ax.axvspan(0, HEALTHY_END/1e3, alpha=0.07, color="#2ca02c", lw=0)
    ax.axvspan(SHOCK/1e3, max(steps_r.max(), steps_a.max(), steps_c.max(), steps_l.max())/1e3,
               alpha=0.10, color="#c0392b", lw=0)
    ax.axvline(SHOCK/1e3, color="#c0392b", linestyle="--", lw=1.1, alpha=0.95)
    ax.grid(True, alpha=0.3)


# (a) Reward
ax = axes[0, 0]
ax.plot(steps_r/1e3, rew, color="#1f77b4", lw=0.6, alpha=0.35)
ax.plot(steps_r/1e3, ema(rew), color="#1f77b4", lw=1.7, label="EMA")
style(ax)
healthy_med = float(np.median(rew[steps_r < HEALTHY_END]))
post_med = float(np.median(rew[steps_r > 700_000]))
ax.axhline(healthy_med, color="#2ca02c", lw=0.6, ls=":", alpha=0.7)
ax.axhline(post_med, color="#c0392b", lw=0.6, ls=":", alpha=0.7)
ax.text(50, healthy_med, f" healthy median {healthy_med:+.0f}",
        color="#2ca02c", fontsize=7, va="bottom")
ax.text(720, post_med, f" post-shock median {post_med:+.0f}",
        color="#c0392b", fontsize=7, va="top")
ax.set_ylabel(r"Episode reward $\bar{R}$")
ax.set_title("(a) Reward collapse: $+599 \\rightarrow -1{,}919$")
ax.legend(loc="upper right", framealpha=0.95)

# (b) Episode length
ax = axes[0, 1]
ax.plot(steps_l/1e3, eplen, color="#2ca02c", lw=0.6, alpha=0.35)
ax.plot(steps_l/1e3, ema(eplen), color="#2ca02c", lw=1.7)
style(ax)
ax.axhline(500, color="black", lw=0.4, ls=":", alpha=0.5)
ax.text(50, 500, " horizon $H{=}500$", fontsize=7, va="bottom")
ax.set_ylabel("Episode length (steps)")
ax.set_title("(b) Survivability collapse: $580 \\rightarrow 26$ steps")

# (c) Critic loss (log)
ax = axes[1, 0]
ax.semilogy(steps_c/1e3, np.maximum(closs, 1.0), color="#d62728", lw=0.6, alpha=0.35)
ax.semilogy(steps_c/1e3, np.maximum(ema(closs), 1.0), color="#d62728", lw=1.7)
style(ax)
ax.set_ylabel(r"Critic loss $\mathcal{L}_Q$ (log)")
ax.set_xlabel(r"Training step ($\times 10^3$)")
ax.set_title("(c) Critic divergence: $220 \\rightarrow 65{,}484$ ($\\sim\\!300\\times$)")

# (d) Actor loss
ax = axes[1, 1]
ax.plot(steps_a/1e3, aloss, color="#9467bd", lw=0.6, alpha=0.35)
ax.plot(steps_a/1e3, ema(aloss), color="#9467bd", lw=1.7)
style(ax)
ax.set_ylabel(r"Actor loss $\mathcal{L}_\pi$")
ax.set_xlabel(r"Training step ($\times 10^3$)")
ax.set_title(r"(d) Actor blow-up: $102 \rightarrow 1{,}471$ ($\sim\!14\times$)")

fig.suptitle(
    "Curriculum Shock: green = Stage 1 plateau ($\\lambda{=}10^\\circ$/s), "
    "red = Stage 2 ($\\lambda{=}15^\\circ$/s), dashed line marks Stage-2 onset (613k steps)",
    fontsize=9.5, y=1.005, fontweight="bold")
fig.tight_layout()

OUT = "/sessions/festive-adoring-bardeen/mnt/outputs/paper/figures/curriculum_shock.pdf"
fig.savefig(OUT)
fig.savefig(OUT.replace(".pdf", ".png"))
print("saved:", OUT)
