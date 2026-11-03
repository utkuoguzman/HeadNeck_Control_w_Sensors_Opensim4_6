import os
import sys
import json
import time
import subprocess
from concurrent.futures import ThreadPoolExecutor

sys.path.append(os.getcwd())
from scripts.run_master_training_until_1810 import run_single_adaptive_stage
from scripts.run_adaptive_controller_updated import ADAPT_15_NAMES


def run_worker_mode(mode_id):
    with open("logs/candidates_multisine/W16_MS_G22.json", "r", encoding="utf-8") as f:
        bc_data = json.load(f)

    bc_p15 = [float(x) for x in bc_data["params"]]
    bc_freqs = bc_data["frequencies"]
    bc_gains = [float(bc_freqs[fk]["sim_gain"]) for fk in ["0.324", "0.783", "1.099", "2.446", "5.583", "7.927"]]
    bc_phases = [float(bc_freqs[fk]["sim_phase"]) for fk in ["0.324", "0.783", "1.099", "2.446", "5.583", "7.927"]]
    base_score = float(bc_data["total_score"])

    os.environ["ADAPT_MRAC_MODE"] = str(mode_id)
    if mode_id == 1:
        label = "Option1_RealPlant_6Freq_Bode_MRAC"
        out_json = "logs/adaptive_option1_realplant_bode.json"
        out_png = "logs/adaptive_option1_realplant_bode.png"
        adapt_mult = 0.25
        sigma_leak = 0.020
    else:
        label = "Option2_RealPlant_TimeDomain_Regressor_MRAC"
        out_json = "logs/adaptive_option2_realplant_timedomain.json"
        out_png = "logs/adaptive_option2_realplant_timedomain.png"
        adapt_mult = 0.25
        sigma_leak = 0.020

    summary = run_single_adaptive_stage(
        seed_p15=bc_p15,
        base_gains=bc_gains,
        base_phases=bc_phases,
        base_ms_score=base_score,
        pre_freq_results=bc_freqs,
        stage_label=label,
        out_json_path=out_json,
        out_png_path=out_png,
        adapt_mult=adapt_mult,
        sigma_leak=sigma_leak,
        t_freeze=14.0,
    )

    # Also evaluate the converged frozen 15 parameters on the exact standalone 9.17s multisine benchmark
    fp = summary["final_parameters_15d"]
    p15_conv = [float(fp[k]) for k in ADAPT_15_NAMES]
    # Convert z_sc (idx 3) and z_ccr (idx 8) to r_sc = z_sc / p_sc and r_ccr = z_ccr / p_ccr for cmaes_worker_multisine.py CLI
    cma_args = list(p15_conv)
    cma_args[3] = p15_conv[3] / max(1e-4, p15_conv[4])
    cma_args[8] = p15_conv[8] / max(1e-4, p15_conv[9])

    worker_tag = f"MRAC_OPT{mode_id}_FROZEN"
    cmd = [
        sys.executable, "cmaes_worker_multisine.py",
        "22.0", "0.707", "40.2786",
        *[str(v) for v in cma_args],
        worker_tag
    ]
    res = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
    frozen_score = None
    if res.returncode == 0:
        lines = [l.strip() for l in res.stdout.strip().split("\n") if l.strip()]
        frozen_score = float(lines[-1])
    summary["standalone_frozen_multisine_score"] = frozen_score
    with open(out_json, "w", encoding="utf-8") as f_out:
        json.dump(summary, f_out, indent=2)
    print(f"[{label}] Post-Adapt Window Score = {summary['post_adaptation_window_score']:,.2f} pts | Standalone Frozen Score = {frozen_score}", flush=True)


def main():
    if len(sys.argv) > 1 and sys.argv[1] in ("1", "2"):
        run_worker_mode(int(sys.argv[1]))
        return

    print("=" * 96, flush=True)
    print("LAUNCHING PARALLEL COMPARISON: OPTION 1 (REAL-PLANT BODE MRAC) vs OPTION 2 (TIME-DOMAIN MRAC)", flush=True)
    print("=" * 96, flush=True)
    t0 = time.time()

    def launch_sub(m):
        p = subprocess.run(
            [sys.executable, "scripts/compare_realplant_mrac_modes.py", str(m)],
            capture_output=False
        )
        return p.returncode

    with ThreadPoolExecutor(max_workers=2) as ex:
        f1 = ex.submit(launch_sub, 1)
        f2 = ex.submit(launch_sub, 2)
        f1.result()
        f2.result()

    print(f"\nBoth MRAC experiments completed in {(time.time() - t0)/60.0:.2f} min!", flush=True)


if __name__ == "__main__":
    main()

