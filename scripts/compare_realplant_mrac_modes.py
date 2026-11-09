import os, sys
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_UTIL_PATH = os.path.join(_REPO_ROOT, 'util')
for _p in (_UTIL_PATH, _REPO_ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import os
import sys
import json
import time
import subprocess
from concurrent.futures import ThreadPoolExecutor

sys.path.append(os.getcwd())
from scripts.run_master_training_until_1810 import run_single_adaptive_stage
from scripts.run_adaptive_controller_updated import ADAPT_15_NAMES


import glob
import shutil


def get_global_best_standalone_champion():
    """
    Scans all standalone benchmark candidates in logs/candidates_multisine/*.json
    (including Multisine CMA-ES W*_MS_G*.json, Option 1 MRAC_OPT1_*_FROZEN.json,
    and Option 2 MRAC_OPT2_*_FROZEN.json) and returns the all-time best standalone
    champion (path, worker_name, score, data_dict), syncing it to logs/multisine_best_champion.json.
    """
    best_score = float("inf")
    best_path = None
    best_data = None
    best_worker = None

    for path in glob.glob("logs/candidates_multisine/*.json"):
        try:
            with open(path, "r", encoding="utf-8") as f:
                d = json.load(f)
            sc = float(d.get("total_score", float("inf")))
            if "params" in d and "frequencies" in d and len(d["params"]) == 15 and sc < best_score:
                best_score = sc
                best_path = path
                best_data = d
                best_worker = d.get("worker", os.path.splitext(os.path.basename(path))[0])
        except Exception:
            continue

    if best_data is not None:
        with open("logs/multisine_best_champion.json", "w", encoding="utf-8") as fb:
            json.dump(best_data, fb, indent=2)

    return best_path, best_worker, best_score, best_data


def run_single_mode_from_global_best(mode_id, st_num, adapt_mult, sigma_leak):
    best_path, best_worker, base_score, bc_data = get_global_best_standalone_champion()

    if mode_id == 1:
        label = f"Option1_Stage{st_num}_from_{best_worker}_Bode_MRAC"
        out_json = f"logs/adaptive_option1_stage{st_num}_realplant_bode.json"
        out_png = f"logs/adaptive_option1_stage{st_num}_realplant_bode.png"
    else:
        label = f"Option2_Stage{st_num}_from_{best_worker}_TimeDomain_MRAC"
        out_json = f"logs/adaptive_option2_stage{st_num}_realplant_timedomain.json"
        out_png = f"logs/adaptive_option2_stage{st_num}_realplant_timedomain.png"

    bc_p15 = [float(x) for x in bc_data["params"]]
    bc_freqs = bc_data["frequencies"]
    bc_gains = [float(bc_freqs[fk]["sim_gain"]) for fk in ["0.324", "0.783", "1.099", "2.446", "5.583", "7.927"]]
    bc_phases = [float(bc_freqs[fk]["sim_phase"]) for fk in ["0.324", "0.783", "1.099", "2.446", "5.583", "7.927"]]

    os.environ["ADAPT_MRAC_MODE"] = str(mode_id)
    g_str = ",".join(f"{g:.4f}" for g in bc_gains)
    p_str = ",".join(f"{p:.4f}" for p in bc_phases)
    os.environ["OSIM_BASE_GAINS"] = g_str
    os.environ["OSIM_BASE_PHASES"] = p_str

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

    # Evaluate converged frozen 15 parameters on standalone 9.17s multisine benchmark
    fp = summary["final_parameters_15d"]
    p15_conv = [float(fp[k]) for k in ADAPT_15_NAMES]
    cma_args = list(p15_conv)
    cma_args[3] = p15_conv[3] / max(1e-4, p15_conv[4])
    cma_args[8] = p15_conv[8] / max(1e-4, p15_conv[9])

    worker_tag = f"MRAC_OPT{mode_id}_S{st_num}_GLOBAL_FROZEN"
    cmd = [
        sys.executable, os.path.join("util", "cmaes_worker_multisine.py"),
        "22.0", "0.707", "40.2786",
        *[str(v) for v in cma_args],
        worker_tag
    ]
    res = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
    frozen_score = None
    if res.returncode == 0:
        lines = [l.strip() for l in res.stdout.strip().split("\n") if l.strip()]
        frozen_score = float(lines[-1])
    summary["seed_champion_worker"] = best_worker
    summary["seed_champion_path"] = best_path
    summary["prev_standalone_frozen_score"] = base_score
    summary["standalone_frozen_multisine_score"] = frozen_score
    with open(out_json, "w", encoding="utf-8") as f_out:
        json.dump(summary, f_out, indent=2)
    print(
        f"[{label}] Seed ({best_worker}): {base_score:,.2f} pts -> "
        f"Standalone Frozen ({worker_tag}): {frozen_score:,.2f} pts "
        f"(Late Window: {summary['post_adaptation_window_score']:,.2f} pts)",
        flush=True
    )


def main():
    if len(sys.argv) == 5 and sys.argv[1] in ("1", "2"):
        mode_id = int(sys.argv[1])
        st_num = int(sys.argv[2])
        adapt_mult = float(sys.argv[3])
        sigma_leak = float(sys.argv[4])
        run_single_mode_from_global_best(mode_id, st_num, adapt_mult, sigma_leak)
        return

    stages = [
        (3, 0.08, 0.025),
        (4, 0.04, 0.030),
    ]
    t0 = time.time()
    for st_num, adapt_mult, sigma_leak in stages:
        _, best_worker, best_score, _ = get_global_best_standalone_champion()
        print("=" * 96, flush=True)
        print(
            f"STAGE {st_num} (GLOBAL ELITIST SEED): Starting BOTH Option 1 & Option 2 from "
            f"{best_worker} = {best_score:,.2f} pts (mult={adapt_mult}, leak={sigma_leak})",
            flush=True
        )
        print("=" * 96, flush=True)

        def launch_sub(m):
            if m == 2:
                time.sleep(3.0)
            p = subprocess.run(
                [
                    sys.executable,
                    "scripts/compare_realplant_mrac_modes.py",
                    str(m),
                    str(st_num),
                    str(adapt_mult),
                    str(sigma_leak),
                ],
                capture_output=False,
            )
            return p.returncode

        with ThreadPoolExecutor(max_workers=2) as ex:
            f1 = ex.submit(launch_sub, 1)
            f2 = ex.submit(launch_sub, 2)
            f1.result()
            f2.result()

    _, final_worker, final_score, _ = get_global_best_standalone_champion()
    print(
        f"\nAll Global-Best Elitist MRAC Stages completed in {(time.time() - t0)/60.0:.2f} min! "
        f"Global Best Standalone Champion: {final_worker} = {final_score:,.2f} pts",
        flush=True
    )


if __name__ == "__main__":
    main()
