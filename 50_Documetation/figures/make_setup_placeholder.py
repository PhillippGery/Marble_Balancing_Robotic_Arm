#!/usr/bin/env python3
"""Generate a placeholder PNG for the experimental-setup screenshot.
The user replaces this file with their own screenshot at the same filename
(`robot_setup.png`); the LaTeX figure environment in the paper picks it up
automatically."""
import matplotlib.pyplot as plt
import matplotlib.patches as patches

fig, ax = plt.subplots(figsize=(7.0, 4.5))
ax.set_xlim(0, 10)
ax.set_ylim(0, 6.5)
ax.set_aspect("equal")
ax.axis("off")

# Outer dashed border
ax.add_patch(patches.Rectangle((0.1, 0.1), 9.8, 6.3,
                               linewidth=1.5, edgecolor="#888888",
                               facecolor="#f4f4f4", linestyle="--"))

# Schematic icon: robot base + arm + plate + marble (tiny illustrative sketch)
ax.add_patch(patches.Rectangle((4.4, 0.8), 1.2, 0.4, color="#444"))           # base
ax.plot([5.0, 5.0], [1.2, 2.5], color="#555", lw=4)                            # link 1
ax.plot([5.0, 6.5], [2.5, 3.4], color="#555", lw=4)                            # link 2
ax.plot([6.5, 6.5], [3.4, 4.0], color="#555", lw=4)                            # wrist
ax.add_patch(patches.Rectangle((5.5, 4.0), 2.0, 0.12, color="#666"))           # plate
ax.add_patch(patches.Circle((6.7, 4.22), 0.10, color="#c0392b"))               # marble

# Text
ax.text(5.0, 5.6, "Drop your screenshot here",
        ha="center", va="center", fontsize=18, color="#333", fontweight="bold")
ax.text(5.0, 5.0,
        "Replace  figures/robot_setup.png  with the photo of\n"
        "the UR5e + plate + marble setup. The figure environment\n"
        "in the paper will use whatever image is at that path.",
        ha="center", va="center", fontsize=10, color="#555")

plt.savefig("/sessions/festive-adoring-bardeen/mnt/outputs/paper/figures/robot_setup.png",
            dpi=150, bbox_inches="tight")
print("placeholder saved")
