import json
import math
import time
import numpy as np
from scipy.interpolate import CubicSpline

# 1. Load Spline Dataset
with open("logs/rom_plant_spline_dataset.json", "r", encoding="utf-8") as f:
    spline_data = json.load(f)

# 2. Load Muscle Parameters
with open("logs/cervical_72_muscle_parameters.json", "r", encoding="utf-8") as f:
    musc_params = json.load(f)

pitch_knots = np.array(spline_data["pitch_rad_knots"])
spline_G_grav = CubicSpline(pitch_knots, spline_data["G_grav_Nm"])

muscle_names = spline_data["muscle_names"]
n_musc = len(muscle_names)

# Build Cubic Splines for all 72 muscles
splines_ma1 = [CubicSpline(pitch_knots, spline_data["muscle_moment_arms_pitch1"][m]) for m in muscle_names]
splines_ma2 = [CubicSpline(pitch_knots, spline_data["muscle_moment_arms_pitch2"][m]) for m in muscle_names]
splines_len = [CubicSpline(pitch_knots, spline_data["muscle_lengths"][m]) for m in muscle_names]

# Parameter arrays
F0_arr = np.array([musc_params[m]["F0"] for m in muscle_names])
l0m_arr = np.array([musc_params[m]["l0m"] for m in muscle_names])
lst_arr = np.array([musc_params[m]["lst"] for m in muscle_names])
alpha0_arr = np.array([musc_params[m]["alpha0"] for m in muscle_names])
vmax_arr = np.array([musc_params[m]["vmax"] for m in muscle_names])

print(f"Loaded {n_musc} muscles successfully!")

# Benchmark step execution speed
theta = 0.0488
t0 = time.perf_counter()
n_steps = 3000
for _ in range(n_steps):
    r1 = np.array([s(theta) for s in splines_ma1])
    r2 = np.array([s(theta) for s in splines_ma2])
    lmt = np.array([s(theta) for s in splines_len])
    # Array forces
    act = np.full(n_musc, 0.04)
    F = act * F0_arr
    tau1 = np.dot(r1, F)
    tau2 = np.dot(r2, F)
dt = time.perf_counter() - t0

print(f"Completed {n_steps} full 72-muscle steps in {dt:.3f} s ({dt/n_steps*1e6:.1f} us/step)!")

