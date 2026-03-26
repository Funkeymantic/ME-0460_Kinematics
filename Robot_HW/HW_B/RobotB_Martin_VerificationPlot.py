# ============================================================
# STUDENT NAME: Paul Martin
# ME4640 Robot Homework B - Trajectory Verification Plot
#
# PURPOSE:
#   Simulates the programmed trajectory from RobotB_Martin_Trajectory.ino
#   and produces a plot of theta_1 through theta_4 vs. time.
#
# USAGE (two modes):
#   1. SIMULATION MODE (default):
#      Run this script directly. It replicates the Arduino trajectory
#      logic in Python so you can verify the motion BEFORE uploading.
#
#   2. LIVE DATA MODE:
#      After running the Arduino code in verification mode (servos unpowered),
#      copy the CSV lines from Serial Monitor into a file, then set:
#          USE_SERIAL_CSV = True
#          CSV_FILE = "serial_output.csv"
# ============================================================

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

# ===== CONFIGURATION =====
USE_SERIAL_CSV = False          # Set True to plot from captured Serial CSV data
CSV_FILE       = "serial_output.csv"   # CSV from Serial Monitor (time,th1,th2,th3,th4)

# ===== TRAJECTORY PARAMETERS (must match Arduino code) =====
DT = 0.010       # simulation timestep (seconds) — matches LOOP_MS = 10 ms

# Target angles (degrees from home)
TH1_A    =  45.0
TH2_A    =  40.0
TH3_A    = -40.0
TH4_OPEN =  35.0

# Phase durations (seconds)
# Phase: 1     2     3     4     5     6     7     8     9
DURATIONS = [2.0, 1.0, 2.0, 1.0, 2.0, 1.0, 1.0, 1.0, 2.0]
PHASE_NAMES = [
    "1: Base 0→45°",
    "2: Hold Base",
    "3: Shoulder 0→40°",
    "4: Hold Shoulder",
    "5: Elbow 0→-40°",
    "6: Hold Elbow",
    "7: Gripper Open",
    "8: Gripper Close",
    "9: Return Base",
]


def lerp(a, b, t):
    """Linear interpolation; t is clamped to [0,1]."""
    t = np.clip(t, 0.0, 1.0)
    return a + (b - a) * t


def simulate_trajectory():
    """
    Replicate the Arduino state machine in Python.
    Returns arrays: time, th1, th2, th3, th4
    """
    times = []
    TH = {1: [], 2: [], 3: [], 4: []}

    th1, th2, th3, th4 = 0.0, 0.0, 0.0, 0.0
    t = 0.0

    for phase_idx, (duration, name) in enumerate(zip(DURATIONS, PHASE_NAMES)):
        phase_num = phase_idx + 1
        th1_start = th1
        th2_start = th2
        th3_start = th3
        th4_start = th4

        steps = int(duration / DT)
        for step in range(steps + 1):
            t_frac = step / steps if steps > 0 else 1.0

            if phase_num == 1:    # Sweep th1: 0 → 45
                th1 = lerp(th1_start, TH1_A, t_frac)
            elif phase_num == 2:  # Hold th1 at 45
                th1 = TH1_A
            elif phase_num == 3:  # Sweep th2: 0 → 40
                th2 = lerp(th2_start, TH2_A, t_frac)
            elif phase_num == 4:  # Hold th2 at 40
                th2 = TH2_A
            elif phase_num == 5:  # Sweep th3: 0 → -40
                th3 = lerp(th3_start, TH3_A, t_frac)
            elif phase_num == 6:  # Hold th3 at -40
                th3 = TH3_A
            elif phase_num == 7:  # Open gripper: 0 → 35
                th4 = lerp(th4_start, TH4_OPEN, t_frac)
            elif phase_num == 8:  # Close gripper: 35 → 0
                th4 = lerp(th4_start, 0.0, t_frac)
            elif phase_num == 9:  # Return th1: 45 → 0
                th1 = lerp(th1_start, 0.0, t_frac)

            times.append(t)
            TH[1].append(th1)
            TH[2].append(th2)
            TH[3].append(th3)
            TH[4].append(th4)
            t += DT

    return (np.array(times),
            np.array(TH[1]),
            np.array(TH[2]),
            np.array(TH[3]),
            np.array(TH[4]))


def load_serial_csv(filepath):
    """Load CSV data captured from Arduino Serial Monitor."""
    data = []
    with open(filepath, "r") as f:
        for line in f:
            line = line.strip()
            if line.startswith("#") or line.startswith("=") or line.startswith("-"):
                continue
            if "time_s" in line.lower():
                continue
            parts = line.split(",")
            if len(parts) == 5:
                try:
                    row = [float(x) for x in parts]
                    data.append(row)
                except ValueError:
                    continue
    arr = np.array(data)
    return arr[:, 0], arr[:, 1], arr[:, 2], arr[:, 3], arr[:, 4]


# ============================================================
# MAIN
# ============================================================
if USE_SERIAL_CSV:
    print(f"Loading Serial data from: {CSV_FILE}")
    t, th1, th2, th3, th4 = load_serial_csv(CSV_FILE)
    title_suffix = "(Live Serial Data)"
else:
    print("Running trajectory simulation...")
    t, th1, th2, th3, th4 = simulate_trajectory()
    title_suffix = "(Simulated)"

# ===== COMPUTE PHASE BOUNDARIES FOR SHADING =====
phase_starts = [0.0]
for d in DURATIONS[:-1]:
    phase_starts.append(phase_starts[-1] + d)
phase_starts.append(phase_starts[-1] + DURATIONS[-1])

# ===== PLOT =====
fig, ax = plt.subplots(figsize=(12, 6))

# Joint lines
ax.plot(t, th1, linewidth=2.0, color="#1f77b4", label=r"$\theta_1$ – Base")
ax.plot(t, th2, linewidth=2.0, color="#ff7f0e", label=r"$\theta_2$ – Shoulder")
ax.plot(t, th3, linewidth=2.0, color="#2ca02c", label=r"$\theta_3$ – Elbow")
ax.plot(t, th4, linewidth=2.0, color="#d62728", label=r"$\theta_4$ – Gripper")

# Phase shading (alternating light backgrounds)
colors_bg = ["#f0f4ff", "#fff8f0"]
for i, (t0, t1) in enumerate(zip(phase_starts[:-1], phase_starts[1:])):
    ax.axvspan(t0, t1, alpha=0.25, color=colors_bg[i % 2], zorder=0)
    ax.axvline(t0, color="gray", linestyle="--", linewidth=0.7, alpha=0.6, zorder=1)

# Phase labels at top of plot
y_label = ax.get_ylim()[1] if ax.get_ylim()[1] != 0 else 50
ax.set_ylim(bottom=min(th3.min(), -50) - 8, top=max(th1.max(), th2.max(), 50) + 12)
y_top = ax.get_ylim()[1] - 5
for i, (t0, t1) in enumerate(zip(phase_starts[:-1], phase_starts[1:])):
    tmid = (t0 + t1) / 2.0
    ax.text(tmid, ax.get_ylim()[1] - 6, PHASE_NAMES[i],
            ha="center", va="top", fontsize=8.5, rotation=0,
            color="dimgray", fontstyle="italic")

# Reference line at 0
ax.axhline(0, color="black", linewidth=0.8, linestyle="-", alpha=0.5)

# Formatting
ax.set_xlabel("Time (s)", fontsize=13)
ax.set_ylabel("Joint Angle (degrees)", fontsize=13)
ax.set_title(
    f"ME4640 HW-B — Programmed Joint Trajectory {title_suffix}\n"
    r"$\theta_1$ – Base  |  $\theta_2$ – Shoulder  |  $\theta_3$ – Elbow  |  $\theta_4$ – Gripper",
    fontsize=13
)
ax.tick_params(labelsize=12)
ax.xaxis.set_minor_locator(ticker.MultipleLocator(0.5))
ax.yaxis.set_minor_locator(ticker.MultipleLocator(5))
ax.grid(True, which="major", linestyle="-",  linewidth=0.6, alpha=0.5)
ax.grid(True, which="minor", linestyle=":",  linewidth=0.4, alpha=0.35)
ax.legend(fontsize=12, loc="lower right")

plt.tight_layout()

# ===== SAVE =====
out_png = "RobotB_Martin_TrajectoryPlot.png"
out_pdf = "RobotB_Martin_TrajectoryPlot.pdf"
fig.savefig(out_png, dpi=200, bbox_inches="tight")
fig.savefig(out_pdf, bbox_inches="tight")
print(f"Saved: {out_png}")
print(f"Saved: {out_pdf}")
plt.show()
