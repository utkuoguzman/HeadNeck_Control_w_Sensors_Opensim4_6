import time
import math
import numpy as np

# Load Spline Dataset
import json
DATASET_PATH = "logs/rom_plant_spline_dataset.json"
with open(DATASET_PATH, "r", encoding="utf-8") as f:
    spline_data = json.load(f)

PARAM_PATH = "logs/cervical_72_muscle_parameters.json"
with open(PARAM_PATH, "r", encoding="utf-8") as f:
    musc_params = json.load(f)

muscle_names = spline_data["muscle_names"]
n_musc = len(muscle_names)

mid_idx = 45 # index for 0 deg in 91 knots
r1_mid = np.array([spline_data["muscle_moment_arms_pitch1"][m][mid_idx] for m in muscle_names])
r2_mid = np.array([spline_data["muscle_moment_arms_pitch2"][m][mid_idx] for m in muscle_names])
r_tot_mid = r1_mid + r2_mid

F0_arr = np.array([musc_params[m]["F0"] for m in muscle_names])
A_all = r_tot_mid * F0_arr
A_A_T = np.sum(A_all**2) + 1.0

print(f"Precomputed constant vectors for 72 muscles. A_A_T = {A_A_T:.2f}")
