import os
import sys
import json
import subprocess
import numpy as np

ROOT = r"d:\Akademik\PhD_Thesis\OpenSim\HeadNeck_Control_w_Sensors_Opensim4_6"
PY_EXE = sys.executable

# 1. Run pure stationary 9.17s simulation for Run 4 (converged parameters)
# Final converged parameters of Run 4:
# K_ton=5.259843, p_ton=16.227225, K_sc=4.101869, z_sc=0.005544, p_sc=53.284141,
# K_phas=15.420487, p_phas=4.711937, kp_p=0.443511, z_ccr=0.0, p_ccr=99.919586,
# k_dyn=1.0, k_stat=1.0, Kp=3.790117, Kd=0.494580, Ki=3.721492

p15_run4 = [
    5.2598427596916935, 16.227225, 4.101868521564445, 0.0055444407969709435, 53.284141,
    15.420486774711076, 4.711937, 0.44351132582628877, 0.0, 99.919586,
    1.0, 1.0, 3.7901174594192497, 0.49457962404720823, 3.7214919179870036
]

# W04_S4_G02 parameters:
p15_w04 = [
    5.26208895, 16.227225, 4.33558867, 0.0, 53.284141,
    14.32499501, 4.711937, 0.55078102, 0.0, 99.919586,
    1.0, 1.0, 3.83566807, 0.54049415, 3.79082911
]

def run_pure(p15, name, cycles="2.0", slot=0):
    env = os.environ.copy()
    env["MULTISINE_CYCLES"] = str(cycles)
    env["MULTISINE_EXP"] = "1.0"
    env["MULTISINE_MIN_AMP_M"] = "0.0"
    env["NOISE_FLOOR_7927_DEG"] = "0.0"
    env["SAVE_TS"] = "1"
    env["PCORE_ONLY"] = "1"
    env["PCORE_SLOT"] = str(slot)
    env.pop("Z_PHAS_TAU", None)
    env.pop("TAU_MUSCLE_ACT", None)
    
    snippet = f"""
import os, sys, json
sys.path.insert(0, os.path.abspath("scripts"))
sys.path.insert(0, os.path.abspath("util"))
from evaluate_pure_converged_simulation import evaluate_pure_candidate
p15 = {json.dumps(p15)}
evaluate_pure_candidate(p15, "{name}", "{name}")
"""
    print(f"Running {name} (cycles={cycles})...", flush=True)
    subprocess.run([PY_EXE, "-c", snippet], cwd=ROOT, env=env, check=True)

if __name__ == "__main__":
    from concurrent.futures import ProcessPoolExecutor
    print("Launching paired evaluations in parallel on P-core slots 0 and 1...", flush=True)
    with ProcessPoolExecutor(max_workers=2) as ex:
        f1 = ex.submit(run_pure, p15_run4, "pure_converged_adaptation_run4", "2.0", 0)
        f2 = ex.submit(run_pure, p15_w04, "pre_adaptive_champion_w04_21s", "6.0", 1)
        f1.result()
        f2.result()
    print("ALL SIMULATIONS COMPLETE!", flush=True)
