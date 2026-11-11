import os
import sys
import json
import subprocess

PYTHON_EXE = sys.executable

def main():
    env_adapt = os.environ.copy()
    env_adapt["MULTISINE_EXP"] = "1.0"
    env_adapt["MULTISINE_MIN_AMP_M"] = "0.0"
    env_adapt["NOISE_FLOOR_7927_DEG"] = "0.0"
    env_adapt["ADAPT_SEED_JSON"] = "logs/candidates_multisine/W04_S4_G02.json"
    env_adapt["ADAPT_MRAC_MODE"] = "2"
    env_adapt["ADAPT_START_TIME"] = "1.8"
    env_adapt["ADAPT_FREEZE_TIME"] = "3.0"
    env_adapt["PCORE_ONLY"] = "1"
    env_adapt["PCORE_SLOT"] = "0"
    env_adapt.pop("Z_PHAS_TAU", None)
    env_adapt.pop("TAU_MUSCLE_ACT", None)

    print("=== [STAGE 1/2] Running 9.17s Continuous Adaptation Run 4 ===", flush=True)
    subprocess.run([PYTHON_EXE, "scripts/run_adaptive_controller_updated.py"], env=env_adapt, check=True)

    with open("logs/adaptive_updated_15params_trajectory.json", "r", encoding="utf-8") as f:
        traj_data = json.load(f)
    fp = traj_data["final_parameters_15d"]
    names = [
        "adapt_K_ton", "adapt_p_ton", "adapt_K_sc", "adapt_z_sc", "adapt_p_sc",
        "adapt_K_phas", "adapt_p_phas", "adapt_kp_prop", "adapt_z_ccr", "adapt_p_ccr",
        "adapt_K_gamma_dyn", "adapt_K_gamma_stat",
        "adapt_Kp_task_pitch", "adapt_Kd_task_pitch", "adapt_Ki_task_pitch"
    ]
    p15 = [float(fp[k]) for k in names]
    print("\n=== Converged 15 Parameters from Adaptation Run 4 ===", flush=True)
    for k, v in zip(names, p15):
        print(f"  {k:22s} = {v:.6f}", flush=True)

    eval_code = f"""
import os, sys
os.environ["MULTISINE_CYCLES"] = "6.0"
os.environ["MULTISINE_EXP"] = "1.0"
os.environ["MULTISINE_MIN_AMP_M"] = "0.0"
os.environ["NOISE_FLOOR_7927_DEG"] = "0.0"
os.environ["SAVE_TS"] = "1"
sys.path.insert(0, os.path.abspath("scripts"))
from evaluate_pure_converged_simulation import evaluate_pure_candidate
p15 = {json.dumps(p15)}
evaluate_pure_candidate(
    p15,
    "pure_converged_adaptation_run4_21s",
    "Adaptation Run 4 (Pure 21.5s Simulation — Zero Time-Constant & 1.099Hz Tuned)",
    color_main="#059669"
)
"""
    env_eval = os.environ.copy()
    env_eval["PCORE_ONLY"] = "1"
    env_eval["PCORE_SLOT"] = "0"
    env_eval.pop("Z_PHAS_TAU", None)
    env_eval.pop("TAU_MUSCLE_ACT", None)
    print("\n=== [STAGE 2/2] Running Pure 21.52s Stationary Evaluation (6 Cycles) ===", flush=True)
    subprocess.run([PYTHON_EXE, "-c", eval_code], env=env_eval, check=True)

if __name__ == "__main__":
    main()
