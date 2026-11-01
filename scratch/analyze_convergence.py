import json
import numpy as np

with open("logs/adaptive_12params_trajectory.json") as f:
    d = json.load(f)

t = np.array(d["time"])
params = d["params"]

print("=" * 90)
print("PARAMETER CONVERGENCE & STABILITY ANALYSIS ACROSS 30 SECONDS")
print("=" * 90)
print(f"{'Parameter':<20s} | {'Initial (t=0)':>13s} | {'At t=15s':>10s} | {'At t=25s':>10s} | {'Final (t=30s)':>13s} | {'Drift in last 5s (%)':>20s}")
print("-" * 90)

for name in params:
    arr = np.array(params[name])
    v_init = arr[0]
    v_15 = arr[np.searchsorted(t, 15.0)]
    v_25 = arr[np.searchsorted(t, 25.0)]
    v_30 = arr[-1]
    drift_last_5s_pct = abs(v_30 - v_25) / (abs(v_init) + 1e-9) * 100.0
    print(f"{name:<20s} | {v_init:13.4f} | {v_15:10.4f} | {v_25:10.4f} | {v_30:13.4f} | {drift_last_5s_pct:19.3f}%")

print("=" * 90)

# Check standard deviation over last 5 seconds to detect oscillations vs plateau
print("\nPARAMETER OSCILLATION (STANDARD DEVIATION OVER LAST 5 SECONDS):")
mask_last5 = t >= 25.0
for name in params:
    arr = np.array(params[name])[mask_last5]
    std = np.std(arr)
    mean = np.mean(arr)
    cv = (std / abs(mean)) * 100.0
    print(f"  {name:<20s}: Mean = {mean:10.4f}, StdDev = {std:10.6f}, CoeffOfVariation = {cv:8.4f}%")
