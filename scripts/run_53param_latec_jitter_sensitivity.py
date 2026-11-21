#!/usr/bin/env python3
"""All-53-Parameter LatEC Roll Velocity Jitter Sensitivity Sweep (2 waves on TRUBA 112 cores).

Wave 1 (110 parallel LatEC sims):
  - BASE_G35: All 53 parameters at Roll18P Gen 35 Champion.
  - GROUP_OSIM_ROLL18: 18 active Roll parameters reverted to .osim baseline; 35 frozen kept at Gen 35.
  - GROUP_OSIM_FROZEN35: 35 frozen Pitch/Yaw/Fusimotor parameters reverted to .osim baseline; 18 Roll kept at Gen 35.
  - GROUP_ALL_OSIM53: All 53 parameters at .osim baseline.
  - PROBE_A_<k> & PROBE_B_<k> for each of the 53 parameters (106 sims).

Wave 2 (20 parallel LatEC sims):
  - Ranks all 53 parameters by best reduction in hf_rms_latec relative to BASE_G35.
  - Tests cumulative combinations Top-K for K in (1, 2, 3, 4, 5, 6, 8, 10, 12, 15),
    both at 100% winning probe values and 50% blend with Gen 35.
"""

import argparse
import json
import os
import re
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed

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

from scripts.run_3axis_42param_cmaes import LOG_DIR, PARAM_SPECS, _sim_job
from scripts.run_3axis_36param_v2_cmaes import NEW_AXIS_SPECS
from scripts.run_3axis_18param_roll_polish_cmaes import ACTIVE_ROLL_18_SPECS

CHAMP_ROLL18_JSON = os.path.join(LOG_DIR, "cmaes_3axis_18p_roll_champion.json")
OSIM_PATH = os.path.join(_ROOT, "osim_files", "HYOID_HeadNeckNeural_Model_Detailed.osim")
OUT_JSON = os.path.join(LOG_DIR, "latec_53param_jitter_sensitivity.json")
OUT_MD = os.path.join(LOG_DIR, "latec_53param_jitter_sensitivity.md")


def load_bounds_map():
    """Build (lb, ub) map across all 53 optimizable parameters."""
    bmap = {}
    for name, _x0, lb, ub, _std in PARAM_SPECS:
        bmap[name] = (float(lb), float(ub))
    for name, _x0, lb, ub, _std in NEW_AXIS_SPECS:
        bmap[name] = (float(lb), float(ub))
    for name, _parent, lb, ub, _std in ACTIVE_ROLL_18_SPECS:
        bmap[name] = (float(lb), float(ub))
    bmap["Kp_proprioception_yaw"] = (0.003, 0.120)
    bmap["Kp_buckle_yaw"] = (0.20, 4.00)
    return bmap


def extract_osim_baseline(param_names):
    """Extract current XML tag values from HYOID_HeadNeckNeural_Model_Detailed.osim."""
    txt = open(OSIM_PATH, "r", encoding="utf-8").read()
    osim_vals = {}
    for name in param_names:
        m = re.search(rf"<{re.escape(name)}>([^<]+)</{re.escape(name)}>", txt)
        if not m:
            raise KeyError(f"Tag <{name}> not found in {OSIM_PATH}")
        osim_vals[name] = float(m.group(1).strip())
    return osim_vals


def build_probes_for_param(name, v_g35, v_osim, lb, ub):
    """Return (val_a, label_a, val_b, label_b) for a single parameter."""
    rel_diff = abs(v_osim - v_g35) / max(1e-6, abs(v_g35))
    if rel_diff > 0.08:
        val_a = float(v_osim)
        lbl_a = "osim"
    elif name.startswith("z_"):
        val_a = 0.0
        lbl_a = "zero"
    else:
        val_a = max(lb, 0.60 * v_g35)
        lbl_a = "0.60x"

    if name.startswith("tau_lead_pole"):
        val_b = min(ub, 1.50 * v_g35)
        lbl_b = "1.50x"
    elif name.startswith("z_"):
        val_b = 0.0
        lbl_b = "zero"
    else:
        val_b = max(lb, 0.50 * v_g35)
        lbl_b = "0.50x"

    return float(val_a), lbl_a, float(val_b), lbl_b


def _to_overrides(pdict):
    return [(k, f"{float(v):.10g}") for k, v in pdict.items()]


def run_wave(cases, workers):
    """Run parallel single-axis LatEC simulations for (case_name, param_dict) list."""
    jobs = [(idx, "LatEC", _to_overrides(pdict)) for idx, (_name, pdict) in enumerate(cases)]
    out = [None] * len(cases)
    with ProcessPoolExecutor(max_workers=workers) as ex:
        fut_map = {ex.submit(_sim_job, job): job[0] for job in jobs}
        for fut in as_completed(fut_map):
            idx = fut_map[fut]
            out[idx] = fut.result()
    return {cases[i][0]: out[i] for i in range(len(cases))}


def main():
    parser = argparse.ArgumentParser(description="All-53-Parameter LatEC Roll Velocity Jitter Sensitivity Sweep")
    parser.add_argument("--workers", type=int, default=112, help="Parallel OpenSim worker processes")
    args = parser.parse_args()

    t_start = time.time()
    os.makedirs(LOG_DIR, exist_ok=True)

    champ = json.load(open(CHAMP_ROLL18_JSON, "r", encoding="utf-8"))
    g35 = {k: float(v) for k, v in champ["params"].items()}
    param_names = list(g35.keys())
    roll18_set = {s[0] for s in ACTIVE_ROLL_18_SPECS}
    bmap = load_bounds_map()
    osim_base = extract_osim_baseline(param_names)

    missing_bounds = [k for k in param_names if k not in bmap]
    if missing_bounds:
        raise KeyError(f"Missing bounds for parameters: {missing_bounds}")

    # --- Wave 1: Baselines + 2 probes for each of the 53 parameters (110 LatEC sims) ---
    wave1_cases = []
    wave1_cases.append(("BASE_G35", dict(g35)))

    p_osim_roll18 = dict(g35)
    for k in roll18_set:
        p_osim_roll18[k] = osim_base[k]
    wave1_cases.append(("GROUP_OSIM_ROLL18", p_osim_roll18))

    p_osim_frozen35 = dict(g35)
    for k in param_names:
        if k not in roll18_set:
            p_osim_frozen35[k] = osim_base[k]
    wave1_cases.append(("GROUP_OSIM_FROZEN35", p_osim_frozen35))

    wave1_cases.append(("GROUP_ALL_OSIM53", dict(osim_base)))

    probe_meta = {}
    for k in param_names:
        lb, ub = bmap[k]
        val_a, lbl_a, val_b, lbl_b = build_probes_for_param(k, g35[k], osim_base[k], lb, ub)
        probe_meta[k] = {
            "group": "Roll18" if k in roll18_set else "Frozen35",
            "g35": g35[k],
            "osim": osim_base[k],
            "lb": lb,
            "ub": ub,
            "val_a": val_a,
            "lbl_a": lbl_a,
            "val_b": val_b,
            "lbl_b": lbl_b,
        }
        pa = dict(g35)
        pa[k] = val_a
        wave1_cases.append((f"PROBE_A__{k}", pa))

        pb = dict(g35)
        pb[k] = val_b
        wave1_cases.append((f"PROBE_B__{k}", pb))

    print(f"=== Wave 1: Running {len(wave1_cases)} LatEC sims on {args.workers} workers ===", flush=True)
    w1_res = run_wave(wave1_cases, args.workers)
    base_res = w1_res["BASE_G35"]
    base_rms = float(base_res["hf_noise_rms"])
    base_bode = float(base_res["classic_bode_score"])
    print(
        f"BASE_G35: hf_rms_latec={base_rms:.3f} deg/s (hf_pen={base_res['hf_penalty']:.1f}), "
        f"bode_latec={base_bode:.1f}, aborted={base_res['aborted']}",
        flush=True,
    )
    for gname in ("GROUP_OSIM_ROLL18", "GROUP_OSIM_FROZEN35", "GROUP_ALL_OSIM53"):
        gr = w1_res[gname]
        print(
            f"{gname:20s}: hf_rms={gr['hf_noise_rms']:.3f} deg/s (delta={gr['hf_noise_rms'] - base_rms:+.3f}), "
            f"bode_latec={gr['classic_bode_score']:.1f}, aborted={gr['aborted']}",
            flush=True,
        )

    # Rank all 53 parameters by most negative delta hf_rms_latec (non-aborted winning probe preferred)
    param_rows = []
    for k in param_names:
        m = probe_meta[k]
        ra = w1_res[f"PROBE_A__{k}"]
        rb = w1_res[f"PROBE_B__{k}"]

        def _score_probe(r):
            # Prefer non-aborted lowest hf_noise_rms
            return (1 if r["aborted"] else 0, float(r["hf_noise_rms"]))

        if _score_probe(ra) <= _score_probe(rb):
            win_tag, win_val, win_lbl, win_r = "A", m["val_a"], m["lbl_a"], ra
        else:
            win_tag, win_val, win_lbl, win_r = "B", m["val_b"], m["lbl_b"], rb

        delta_hf = float(win_r["hf_noise_rms"]) - base_rms if not win_r["aborted"] else 999.0
        delta_bode = float(win_r["classic_bode_score"]) - base_bode if not win_r["aborted"] else 999999.0
        param_rows.append({
            "name": k,
            "group": m["group"],
            "g35": m["g35"],
            "osim": m["osim"],
            "lb": m["lb"],
            "ub": m["ub"],
            "val_a": m["val_a"],
            "lbl_a": m["lbl_a"],
            "hf_rms_a": float(ra["hf_noise_rms"]),
            "bode_a": float(ra["classic_bode_score"]),
            "aborted_a": bool(ra["aborted"]),
            "val_b": m["val_b"],
            "lbl_b": m["lbl_b"],
            "hf_rms_b": float(rb["hf_noise_rms"]),
            "bode_b": float(rb["classic_bode_score"]),
            "aborted_b": bool(rb["aborted"]),
            "best_probe": win_tag,
            "best_val": win_val,
            "best_lbl": win_lbl,
            "best_hf_rms": float(win_r["hf_noise_rms"]),
            "best_bode": float(win_r["classic_bode_score"]),
            "best_aborted": bool(win_r["aborted"]),
            "delta_hf_rms": delta_hf,
            "delta_bode": delta_bode,
        })

    param_rows.sort(key=lambda r: (r["delta_hf_rms"], r["delta_bode"]))

    group_summary = {
        gname: {
            "hf_rms_latec": float(w1_res[gname]["hf_noise_rms"]),
            "delta_hf_rms": float(w1_res[gname]["hf_noise_rms"]) - base_rms,
            "bode_latec": float(w1_res[gname]["classic_bode_score"]),
            "hf_penalty": float(w1_res[gname]["hf_penalty"]),
            "total_latec": float(w1_res[gname]["total_score"]),
            "aborted": bool(w1_res[gname]["aborted"]),
        }
        for gname in ("BASE_G35", "GROUP_OSIM_ROLL18", "GROUP_OSIM_FROZEN35", "GROUP_ALL_OSIM53")
    }

    def _save_outputs(wave2_summary):
        payload = {
            "elapsed_s": round(time.time() - t_start, 2),
            "groups": group_summary,
            "ranked_53": param_rows,
            "wave2_cumulative": wave2_summary,
        }
        with open(OUT_JSON, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2)

        lines = [
            "# All-53-Parameter LatEC Roll Velocity Jitter Sensitivity Sweep",
            "",
            f"Elapsed: {payload['elapsed_s']:.1f}s on {args.workers} workers | Baseline Gen 35 `hf_rms_latec = {base_rms:.3f} deg/s`, `bode_latec = {base_bode:.2f}`",
            "",
            "## Group Baselines",
            "",
            "| Group | `hf_rms_latec` (deg/s) | Delta `hf_rms` | `hf_latec` (pts) | `bode_latec` (pts) | Aborted |",
            "|---|---:|---:|---:|---:|---|",
        ]
        for gname, gval in group_summary.items():
            lines.append(
                f"| `{gname}` | {gval['hf_rms_latec']:.3f} | {gval['delta_hf_rms']:+.3f} | "
                f"{gval['hf_penalty']:.1f} | {gval['bode_latec']:.1f} | {gval['aborted']} |"
            )

        if wave2_summary:
            lines.extend([
                "",
                "## Wave 2: Cumulative Top-K Culprit Combinations",
                "",
                "| Top-K | 100% Probe `hf_rms` | Delta `hf_rms` | 100% `bode_latec` | Aborted | 50% Blend `hf_rms` | Delta `hf_rms` | 50% `bode_latec` | Aborted | Last Added Param |",
                "|---:|---:|---:|---:|---|---:|---:|---:|---|---|",
            ])
            for w in wave2_summary:
                f_r = w["full"]
                b_r = w["half_blend"]
                lines.append(
                    f"| {w['k']} | {f_r['hf_rms_latec']:.3f} | {f_r['delta_hf_rms']:+.3f} | {f_r['bode_latec']:.1f} | {f_r['aborted']} | "
                    f"{b_r['hf_rms_latec']:.3f} | {b_r['delta_hf_rms']:+.3f} | {b_r['bode_latec']:.1f} | {b_r['aborted']} | `{w['params'][-1]}` |"
                )

        lines.extend([
            "",
            "## All 53 Parameters Ranked by Roll Jitter Reduction (`Delta hf_rms_latec`)",
            "",
            "| Rank | Parameter | Group | Gen 35 | .osim | Best Probe | Best Val | `hf_rms` | `Delta hf_rms` | `bode_latec` | `Delta bode_latec` | Probe A `hf_rms` / `bode` | Probe B `hf_rms` / `bode` |",
            "|---:|---|---|---:|---:|---|---:|---:|---:|---:|---:|---|---|",
        ])
        for idx, r in enumerate(param_rows, 1):
            pa_str = "ABORT" if r["aborted_a"] else f"{r['hf_rms_a']:.3f} / {r['bode_a']:.0f} ({r['lbl_a']})"
            pb_str = "ABORT" if r["aborted_b"] else f"{r['hf_rms_b']:.3f} / {r['bode_b']:.0f} ({r['lbl_b']})"
            d_hf = "ABORT" if r["best_aborted"] else f"{r['delta_hf_rms']:+.3f}"
            d_bo = "ABORT" if r["best_aborted"] else f"{r['delta_bode']:+.1f}"
            lines.append(
                f"| {idx} | `{r['name']}` | {r['group']} | {r['g35']:.5g} | {r['osim']:.5g} | "
                f"{r['best_probe']} ({r['best_lbl']}) | {r['best_val']:.5g} | "
                f"{r['best_hf_rms']:.3f} | {d_hf} | {r['best_bode']:.1f} | {d_bo} | {pa_str} | {pb_str} |"
            )

        with open(OUT_MD, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")

        print(f"[OK] Wrote {OUT_JSON} and {OUT_MD} in {payload['elapsed_s']:.1f}s", flush=True)

    _save_outputs([])

    # --- Wave 2: Cumulative Top-K Verification (K in 1,2,3,4,5,6,8,10,12,15; full & 50% blend) ---
    ks = [1, 2, 3, 4, 5, 6, 8, 10, 12, 15]
    wave2_cases = []
    for k_top in ks:
        top_subset = param_rows[:k_top]
        p_full = dict(g35)
        p_blend = dict(g35)
        for row in top_subset:
            pname = row["name"]
            p_full[pname] = row["best_val"]
            p_blend[pname] = 0.5 * (g35[pname] + row["best_val"])
        wave2_cases.append((f"TOP{k_top:02d}_FULL", p_full))
        wave2_cases.append((f"TOP{k_top:02d}_HALF", p_blend))

    print(f"=== Wave 2: Running {len(wave2_cases)} cumulative Top-K LatEC sims ===", flush=True)
    w2_res = run_wave(wave2_cases, args.workers)

    wave2_summary = []
    for k_top in ks:
        rf = w2_res[f"TOP{k_top:02d}_FULL"]
        rb = w2_res[f"TOP{k_top:02d}_HALF"]
        wave2_summary.append({
            "k": k_top,
            "params": [r["name"] for r in param_rows[:k_top]],
            "full": {
                "hf_rms_latec": float(rf["hf_noise_rms"]),
                "delta_hf_rms": float(rf["hf_noise_rms"]) - base_rms,
                "bode_latec": float(rf["classic_bode_score"]),
                "hf_penalty": float(rf["hf_penalty"]),
                "total_latec": float(rf["total_score"]),
                "aborted": bool(rf["aborted"]),
            },
            "half_blend": {
                "hf_rms_latec": float(rb["hf_noise_rms"]),
                "delta_hf_rms": float(rb["hf_noise_rms"]) - base_rms,
                "bode_latec": float(rb["classic_bode_score"]),
                "hf_penalty": float(rb["hf_penalty"]),
                "total_latec": float(rb["total_score"]),
                "aborted": bool(rb["aborted"]),
            },
        })

    _save_outputs(wave2_summary)


if __name__ == "__main__":
    main()
