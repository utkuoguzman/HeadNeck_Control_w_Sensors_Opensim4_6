#!/usr/bin/env python3
"""Local A/B benchmark testing Mileusnic06Spindle C(Lp) continuity vs 1e-9 Roll chaos."""

import argparse
import json
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from datetime import datetime

for _env_k in (
    "OMP_NUM_THREADS",
    "OPENBLAS_NUM_THREADS",
    "MKL_NUM_THREADS",
    "NUMEXPR_NUM_THREADS",
    "SIMBODY_NUM_THREADS",
):
    os.environ.setdefault(_env_k, "1")

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_SCRIPT_DIR)
for _p in (_ROOT, _SCRIPT_DIR, os.path.join(_ROOT, "util")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from scripts.generate_standard_plot import run_axis_worker

LOG_DIR = os.path.join(_ROOT, "logs")
OUT_JSON = os.path.join(LOG_DIR, "spindle_smooth_ab.json")


def _to_overrides(pdict):
    return [(k, f"{float(v):.15g}") for k, v in pdict.items()]


def _run_case(job):
    name, mode, pdict = job
    t0 = time.time()
    ovr = _to_overrides(pdict)
    res = run_axis_worker((mode, ovr))
    freq_rows = {}
    for fk, fv in (res.get("freq_results") or {}).items():
        freq_rows[str(fk)] = {
            "freq_hz": float(fv["freq_hz"]),
            "sim_gain": float(fv["sim_gain"]),
            "target_gain": float(fv["target_gain"]),
            "gain_db_diff": float(fv["gain_db_diff"]),
            "gain_loss_pts": float(fv["gain_loss_pts"]),
            "sim_phase": float(fv["sim_phase"]),
            "target_phase": float(fv["target_phase"]),
            "phase_err": float(fv["phase_err"]),
            "phase_loss_pts": float(fv["phase_loss_pts"]),
        }
    return name, {
        "mode": mode,
        "elapsed_s": round(time.time() - t0, 2),
        "aborted": bool(res["aborted"]),
        "abort_time_s": float(res["abort_time_s"]),
        "false_booleans": list(res["false_booleans"]),
        "bool_banner": res["bool_banner"],
        "classic_bode_score": float(res["classic_bode_score"]),
        "gain_loss_pts": float(res["gain_loss_pts"]),
        "phase_loss_pts": float(res["phase_loss_pts"]),
        "pitch_crosstalk_penalty": float(res["pitch_crosstalk_penalty"]),
        "hf_penalty": float(res["hf_penalty"]),
        "total_score": float(res["total_score"]),
        "hf_noise_rms": float(res["hf_noise_rms"]),
        "corr_vel": float(res["corr_vel"]),
        "freq_results": freq_rows,
    }


def load_champions():
    g35 = json.load(open(os.path.join(LOG_DIR, "cmaes_3axis_18p_roll_champion.json"), "r", encoding="utf-8"))["params"]
    best_r = json.load(open(os.path.join(LOG_DIR, "coop_best_roll18.json"), "r", encoding="utf-8"))
    best_p = json.load(open(os.path.join(LOG_DIR, "coop_best_pitch18.json"), "r", encoding="utf-8"))
    best_y = json.load(open(os.path.join(LOG_DIR, "coop_best_yaw15.json"), "r", encoding="utf-8"))
    v3ax = json.load(open(os.path.join(LOG_DIR, "coop_3axis_champion.json"), "r", encoding="utf-8"))

    champ_merged = {
        "K_gamma_dyn": float(g35["K_gamma_dyn"]),
        "K_gamma_stat": float(g35["K_gamma_stat"]),
    }
    champ_merged.update({k: float(v) for k, v in best_r["params_roll18"].items()})
    champ_merged.update({k: float(v) for k, v in best_p["params_pitch18"].items()})
    champ_merged.update({k: float(v) for k, v in best_y["params_yaw15"].items()})
    assert len(champ_merged) == 53, f"CHAMP_MERGED_LIVE has {len(champ_merged)} != 53"

    champ_v3ax = {k: float(v) for k, v in v3ax["params"].items()}
    assert len(champ_v3ax) == 53, f"CHAMP_3AXIS_VERIFIED has {len(champ_v3ax)} != 53"

    return champ_merged, champ_v3ax, {
        "roll_id": best_r.get("roll_id"),
        "pitch_gen": best_p.get("gen"),
        "yaw_gen": best_y.get("gen"),
        "v3ax_gen": v3ax.get("gen"),
        "v3ax_source_tag": v3ax.get("source_tag"),
    }


def main():
    parser = argparse.ArgumentParser(description="A/B benchmark for Mileusnic06Spindle continuity vs Roll chaos")
    parser.add_argument("--label", required=True, help="Condition label (e.g. A_orig, B_tanh1000, C_tanh200)")
    args = parser.parse_args()

    champ_merged, champ_v3ax, meta = load_champions()
    champ_merged_eps = dict(champ_merged)
    champ_merged_eps["K_ton_roll"] = champ_merged["K_ton_roll"] + 1e-9
    assert f"{champ_merged['K_ton_roll']:.15g}" != f"{champ_merged_eps['K_ton_roll']:.15g}"

    champ_v3ax_eps = dict(champ_v3ax)
    champ_v3ax_eps["K_ton_roll"] = champ_v3ax["K_ton_roll"] + 1e-9
    assert f"{champ_v3ax['K_ton_roll']:.15g}" != f"{champ_v3ax_eps['K_ton_roll']:.15g}"

    jobs = [
        ("MERGED_APEO", "APEO", champ_merged),
        ("MERGED_LatEC_base", "LatEC", champ_merged),
        ("MERGED_LatEC_eps1e9", "LatEC", champ_merged_eps),
        ("MERGED_YawMA", "YawMA", champ_merged),
        ("V3AX_LatEC_base", "LatEC", champ_v3ax),
        ("V3AX_LatEC_eps1e9", "LatEC", champ_v3ax_eps),
    ]

    t_all = time.time()
    out = {}
    with ProcessPoolExecutor(max_workers=6) as ex:
        futs = [ex.submit(_run_case, j) for j in jobs]
        for fut in as_completed(futs):
            name, rec = fut.result()
            out[name] = rec

    ordered = {name: out[name] for name, _m, _p in jobs}
    delta_merged = abs(ordered["MERGED_LatEC_base"]["classic_bode_score"] - ordered["MERGED_LatEC_eps1e9"]["classic_bode_score"])
    delta_v3ax = abs(ordered["V3AX_LatEC_base"]["classic_bode_score"] - ordered["V3AX_LatEC_eps1e9"]["classic_bode_score"])
    delta_merged_hf = abs(ordered["MERGED_LatEC_base"]["hf_noise_rms"] - ordered["MERGED_LatEC_eps1e9"]["hf_noise_rms"])
    delta_v3ax_hf = abs(ordered["V3AX_LatEC_base"]["hf_noise_rms"] - ordered["V3AX_LatEC_eps1e9"]["hf_noise_rms"])

    summary = {
        "label": args.label,
        "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "wall_s": round(time.time() - t_all, 2),
        "meta": meta,
        "delta_merged_bode": round(delta_merged, 4),
        "delta_v3ax_bode": round(delta_v3ax, 4),
        "delta_merged_hf": round(delta_merged_hf, 4),
        "delta_v3ax_hf": round(delta_v3ax_hf, 4),
        "cases": ordered,
    }

    all_data = {}
    if os.path.exists(OUT_JSON):
        try:
            all_data = json.load(open(OUT_JSON, "r", encoding="utf-8"))
        except Exception:
            all_data = {}
    all_data[args.label] = summary
    tmp = f"{OUT_JSON}.tmp.{os.getpid()}"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(all_data, f, indent=2)
    os.replace(tmp, OUT_JSON)

    print(f"=== [{args.label}] Finished in {summary['wall_s']}s ===")
    for name in [j[0] for j in jobs]:
        r = ordered[name]
        print(
            f"  {name:22s} | abort={r['aborted']} | bode={r['classic_bode_score']:9.2f} "
            f"(G={r['gain_loss_pts']:8.1f}, P={r['phase_loss_pts']:8.1f}) "
            f"| xtalk={r['pitch_crosstalk_penalty']:6.2f} | hf={r['hf_noise_rms']:.3f} | tot={r['total_score']:9.2f} | {r['bool_banner']}"
        )
    print(f"  delta_merged_bode = {delta_merged:.4f} (HF delta = {delta_merged_hf:.4f})")
    print(f"  delta_v3ax_bode   = {delta_v3ax:.4f} (HF delta = {delta_v3ax_hf:.4f})")


if __name__ == "__main__":
    main()
