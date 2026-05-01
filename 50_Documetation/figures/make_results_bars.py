#!/usr/bin/env python3
"""Win-rate and metric bar charts comparing pure LQR vs. 200k Hybrid (Precision Damper).
Numbers verified directly from CS558_ML2.pdf (Slide: Evaluation dLQR vs Residual)."""
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np

mpl.rcParams.update({
    "font.family": "serif",
    "font.size": 9,
    "axes.labelsize": 9,
    "axes.titlesize": 10,
    "xtick.labelsize": 8,
    "ytick.labelsize": 8,
    "legend.fontsize": 8,
    "axes.linewidth": 0.7,
    "savefig.bbox": "tight",
    "savefig.dpi": 300,
})

# Numbers verbatim from CS558_ML2.pdf Evaluation slide.
data = {
    "Scenario 1\n(0.12 m, Lissajous TCP)": {
        "win":  (40.0, 60.0),
        "rmse": (0.1920, 0.1898),
        "peak": (0.3858, 0.4098),
        "rew":  (711.99, 690.81),
    },
    "Scenario 2\n(0.18 m edge, Lissajous + jitter)": {
        "win":  (70.0, 30.0),
        "rmse": (0.2587, 0.2556),
        "peak": (0.6923, 0.6379),
        "rew":  (823.68, 778.91),
    },
}

fig, axes = plt.subplots(1, 2, figsize=(7.0, 2.6))
labels = list(data.keys())

# (a) Win rate
ax = axes[0]
x = np.arange(len(labels))
w = 0.35
lqr = [data[k]["win"][0] for k in labels]
rl = [data[k]["win"][1] for k in labels]
b1 = ax.bar(x - w/2, lqr, w, label="Pure LQR (baseline)", color="#4878d0", edgecolor="black", lw=0.5)
b2 = ax.bar(x + w/2, rl,  w, label="Hybrid (TD3 200k)",  color="#ee854a", edgecolor="black", lw=0.5)
ax.set_xticks(x)
ax.set_xticklabels(labels)
ax.set_ylabel("Episode Win-rate (%)")
ax.set_title("(a) Win-rate by scenario")
ax.set_ylim(0, 100)
ax.grid(True, axis="y", alpha=0.3)
ax.legend(loc="upper right", framealpha=0.95)
for bars, vals in [(b1, lqr), (b2, rl)]:
    for b, v in zip(bars, vals):
        ax.text(b.get_x() + b.get_width()/2, v + 1.5, f"{v:.0f}%",
                ha="center", va="bottom", fontsize=8)

# (b) Tracking error: RMSE & peak (grouped)
ax = axes[1]
metrics = ["RMSE", "Max peak err."]
x = np.arange(len(metrics) * 2)
w = 0.35
# Order: S1 RMSE, S2 RMSE, S1 Peak, S2 Peak (grouped per metric)
positions = [0, 1, 2.5, 3.5]
ax.bar([p - w/2 for p in positions],
       [data[labels[0]]["rmse"][0], data[labels[1]]["rmse"][0],
        data[labels[0]]["peak"][0], data[labels[1]]["peak"][0]],
       w, label="Pure LQR", color="#4878d0", edgecolor="black", lw=0.5)
ax.bar([p + w/2 for p in positions],
       [data[labels[0]]["rmse"][1], data[labels[1]]["rmse"][1],
        data[labels[0]]["peak"][1], data[labels[1]]["peak"][1]],
       w, label="Hybrid TD3 200k", color="#ee854a", edgecolor="black", lw=0.5)
ax.set_xticks(positions)
ax.set_xticklabels(["RMSE\nS1", "RMSE\nS2", "Peak\nS1", "Peak\nS2"])
ax.set_ylabel("Tracking error (m)")
ax.set_title("(b) RMSE and peak error")
ax.grid(True, axis="y", alpha=0.3)
ax.legend(loc="upper left", framealpha=0.95)

fig.tight_layout()
out = "/sessions/festive-adoring-bardeen/mnt/outputs/paper/figures/results_bars.pdf"
fig.savefig(out)
fig.savefig(out.replace(".pdf", ".png"))
print("saved:", out)
