import json
import time
import numpy as np
from scipy.interpolate import CubicSpline

# 1. Load Spline Dataset
with open("logs/rom_plant_spline_dataset.json", "r", encoding="utf-8") as f:
    spline_data = json.load(f)

pitch_knots = np.array(spline_data["pitch_rad_knots"])
muscle_names = spline_data["muscle_names"]
n_musc = len(muscle_names)

# Stack all 72 muscles into 2D matrices: shape (91, 72)
ma1_matrix = np.column_stack([spline_data["muscle_moment_arms_pitch1"][m] for m in muscle_names])
ma2_matrix = np.column_stack([spline_data["muscle_moment_arms_pitch2"][m] for m in muscle_names])
len_matrix = np.column_stack([spline_data["muscle_lengths"][m] for m in muscle_names])

# Multi-dimensional CubicSpline: ONE evaluation returns array of shape (72,)!
spline_ma1_all = CubicSpline(pitch_knots, ma1_matrix, axis=0)
spline_ma2_all = CubicSpline(pitch_knots, ma2_matrix, axis=0)
spline_len_all = CubicSpline(pitch_knots, len_matrix, axis=0)

theta = 0.0488

# Warmup
r1 = spline_ma1_all(theta)
print(f"Vectorized spline output shape: {r1.shape} (all {n_musc} muscles!)")

t0 = time.perf_counter()
n_steps = 10000
for _ in range(n_steps):
    r1 = spline_ma1_all(theta)
    r2 = spline_ma2_all(theta)
    lmt = spline_len_all(theta)
dt = time.perf_counter() - t0

print(f"Evaluated {n_steps} steps of ALL 72 muscles in {dt:.3f} s ({dt/n_steps*1e6:.2f} us/step)!")
print(f"Time for 3,000 steps (1 full frequency): {3000 * (dt/n_steps)*1000:.1f} ms!")
print(f"Time for all 6 frequencies: {6 * 3000 * (dt/n_steps)*1000:.1f} ms!")

