import json
import numpy as np

with open("logs/rom_plant_spline_dataset.json", "r", encoding="utf-8") as f:
    spline_data = json.load(f)

with open("logs/cervical_72_muscle_parameters.json", "r", encoding="utf-8") as f:
    musc_params = json.load(f)

muscle_names = spline_data["muscle_names"]
F0 = np.array([musc_params[m]["F0"] for m in muscle_names])
r1 = np.array([spline_data["muscle_moment_arms_pitch1"][m][45] for m in muscle_names])
r2 = np.array([spline_data["muscle_moment_arms_pitch2"][m][45] for m in muscle_names])
r_tot = r1 + r2

A = r_tot * F0
A_A_T = np.sum(A**2)
print(f"Total moment arm torque capacity variance (A A^T): {A_A_T:.2f} (N m)^2")
print(f"Number of muscles: {len(muscle_names)}")
print(f"Sum of positive torque capacity (extension): {np.sum(A[A>0]):.2f} Nm")
print(f"Sum of negative torque capacity (flexion): {np.sum(A[A<0]):.2f} Nm")

