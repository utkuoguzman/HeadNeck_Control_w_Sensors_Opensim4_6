#!/usr/bin/env python3
"""Single-wave 112-core TRUBA joint evaluation & plot generator for 10 C1-Pos 2x2 IK SE2(3) candidates."""

import argparse
import copy
import json
import os
import pickle
import sys
import time
from concurrent.futures import ProcessPoolExecutor

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

from scripts.generate_standard_plot import (
    MODEL_PATH,
    render_standard_4panel,
    run_axis_worker,
    write_html_viewer,
)
from scripts.tune_c1pos_se2_3axis import (
    CHAMP_OUT_JSON,
    COOP_JSON,
    _populate_verified_windows,
    to_ov,
)
from scripts.validate_5dof_voluntary_and_se2_shift import (
    OUT_JSON as VAL_OUT_JSON,
    build_validation_specs,
    render_validation_figure,
    run_validation_case,
)

SWEEP_OUT_JSON = os.path.join(_ROOT, "logs", "c1pos_joint_final_sweep.json")
PLOT_SUM_OUT_JSON = os.path.join(_ROOT, "logs", "standard_plot_summary_c1pos_final.json")

# (tag, pz_obs, (kp_lat, kd_lat, ki_lat), tau_wash, (kp_ap, kd_ap, ki_ap), (sway_px, sway_vx, sway_ki))
CANDIDATES = [
    ("C0", -0.045, (4.5, 0.30, 3.0), 5.0, (5.5, 0.85, 2.5), (0.00, -0.16, 0.0)),
    ("C1", -0.045, (4.5, 0.38, 2.2), 5.0, (5.5, 0.90, 2.2), (-0.04, -0.15, 0.0)),
    ("C2", -0.044, (4.5, 0.42, 2.0), 5.0, (6.0, 1.00, 2.0), (-0.05, -0.16, 0.0)),
    ("C3", -0.042, (4.2, 0.36, 2.0), 5.0, (5.5, 0.85, 2.5), (-0.04, -0.15, 0.0)),
    ("C4", -0.040, (4.0, 0.35, 1.8), 5.0, (6.0, 0.95, 2.2), (0.00, -0.16, 0.0)),
    ("C5", -0.046, (4.6, 0.45, 1.8), 5.0, (5.0, 0.80, 2.5), (-0.03, -0.15, 0.0)),
    ("C6", -0.045, (4.8, 0.50, 1.5), 5.0, (6.5, 1.10, 2.0), (0.00, -0.18, 0.0)),
    ("C7", -0.043, (4.4, 0.38, 2.2), 4.5, (5.5, 0.90, 2.2), (-0.04, -0.16, 0.0)),
    ("C8", -0.045, (4.5, 0.35, 2.4), 4.5, (5.0, 0.75, 2.8), (0.00, -0.15, 0.0)),
    ("C9", -0.041, (4.3, 0.40, 2.0), 4.5, (6.0, 1.00, 2.0), (-0.04, -0.15, 0.0)),
]


def _build_candidate_params(base_raw, spec):
    tag, pz_obs, lat_pid, tau_wash, ap_pid, _ = spec
    p = copy.deepcopy(base_raw)
    p.update(
        {
            "Kp_trans_roll": 0.0,
            "Kv_trans_roll": 0.0,
            "Kp_trans_pitch": 0.0,
            "Kv_trans_pitch": 0.0,
            "Kp_shift_obs_px": 0.0,
            "Kv_shift_obs_vx": 0.0,
            "Kp_shift_obs_pz": pz_obs,
            "Kv_shift_obs_vz": 0.0,
            "Kp_shift_lat": lat_pid[0],
            "Kd_shift_lat": lat_pid[1],
            "Ki_shift_lat": lat_pid[2],
            "Kp_shift_ap": ap_pid[0],
            "Kd_shift_ap": ap_pid[1],
            "Ki_shift_ap": ap_pid[2],
            "w_pos_ap": 1.0,
            "w_pos_lat": 1.0,
            "shift_station_blend": 0.0,
            "tau_vel_leak": 3.0,
            "tau_pos_leak": 2.0,
            "tau_obs_washout": tau_wash,
        }
    )
    return tag, p


def _dispatch(task):
    kind = task[0]
    if kind == "apeo":
        _, tw, params = task
        return ("apeo", tw, run_axis_worker(("APEO", to_ov(params))))
    if kind == "axis":
        _, tag, mode, params = task
        return ("axis", tag, mode, run_axis_worker((mode, to_ov(params))))
    _, tag, idx, val_spec = task
    sm, tr = run_validation_case(val_spec)
    return ("val", tag, idx, sm, tr)


def _evaluate_gates(res_3, val_by_name):
    failures = []
    for m in ("APEO", "LatEC", "YawMA"):
        r = res_3[m]
        if r["aborted"] or r["false_booleans"] or round(float(r["quiet_main_max_deg"]), 4) > 0.001:
            failures.append(f"{m}_invalid")
    for cname, rec in val_by_name.items():
        sm = rec["summary"]
        if sm["aborted"] or sm["false_booleans"] or float(sm["quiet_max_att_deg"]) > 0.01:
            failures.append(f"{cname}_invalid")

    c1 = val_by_name["Case1_AP_Protraction"]["summary"]
    c2 = val_by_name["Case2_AP_Retraction"]["summary"]
    c3 = val_by_name["Case3_Lat_Glide"]["summary"]
    c4a = val_by_name["Case4A_Combined_wPos1"]["summary"]
    c4b = val_by_name["Case4B_AttitudeOnly_wPos0"]["summary"]
    c5a = val_by_name["Case5A_Sway_SE2Off"]["summary"]
    c5b = val_by_name["Case5B_Sway_SE2On"]["summary"]

    if c1["ss_c1_dx_rel_cm"] < 1.65:
        failures.append(f"C1_dx={c1['ss_c1_dx_rel_cm']:.2f}<1.65")
    if abs(c1["ss_pitch_deg"]) > 0.7:
        failures.append(f"C1_p={c1['ss_pitch_deg']:.2f}")
    if c2["ss_c1_dx_rel_cm"] > -1.30:
        failures.append(f"C2_dx={c2['ss_c1_dx_rel_cm']:.2f}>-1.30")
    if abs(c2["ss_pitch_deg"]) > 0.7:
        failures.append(f"C2_p={c2['ss_pitch_deg']:.2f}")
    if c2["ss_std_cun_ap_cm"] > 0.08:
        failures.append(f"C2_std={c2['ss_std_cun_ap_cm']:.4f}>0.08")
    if c3["ss_c1_dz_rel_cm"] < 0.43:
        failures.append(f"C3_dz={c3['ss_c1_dz_rel_cm']:.2f}<0.43")
    if abs(c3["ss_roll_deg"]) > 0.15:
        failures.append(f"C3_r={c3['ss_roll_deg']:.2f}")
    if c3["ss_std_cun_lat_cm"] > 0.015:
        failures.append(f"C3_std={c3['ss_std_cun_lat_cm']:.4f}>0.015")
    if not (9.2 <= c4a["ss_pitch_deg"] <= 10.8):
        failures.append(f"C4A_p={c4a['ss_pitch_deg']:.2f}")
    if c4a["max_abs_pitch_deg"] > 12.5:
        failures.append(f"C4A_pmax={c4a['max_abs_pitch_deg']:.2f}>12.5")
    if c4a["ss_std_pitch_deg"] > 0.35:
        failures.append(f"C4A_pstd={c4a['ss_std_pitch_deg']:.3f}>0.35")
    if c4a["ss_c1_dx_rel_cm"] < 0.55:
        failures.append(f"C4A_dx={c4a['ss_c1_dx_rel_cm']:.2f}<0.55")
    if not (9.4 <= c4b["ss_pitch_deg"] <= 10.6):
        failures.append(f"C4B_p={c4b['ss_pitch_deg']:.2f}")
    if c4b["ss_c1_dx_rel_cm"] > -0.80:
        failures.append(f"C4B_dx={c4b['ss_c1_dx_rel_cm']:.2f}>-0.80")
    if c5b["sway_rms_c1_glob_cm"] > c5a["sway_rms_c1_glob_cm"] - 0.12:
        failures.append(f"C5B_sway={c5b['sway_rms_c1_glob_cm']:.3f} vs {c5a['sway_rms_c1_glob_cm']:.3f}")
    if abs(c5b["sway_mean_c1_rel_cm"]) > 0.10:
        failures.append(f"C5B_mean={c5b['sway_mean_c1_rel_cm']:+.3f}")

    combined_bode = sum(float(res_3[m]["classic_bode_score"]) for m in ("APEO", "LatEC", "YawMA"))
    combined_xtalk = sum(float(res_3[m]["pitch_crosstalk_penalty"]) for m in ("APEO", "LatEC", "YawMA"))
    f0 = res_3["LatEC"]["freq_results"]["0.1209"]
    obj = (
        combined_bode
        + 0.35 * combined_xtalk
        + 6.0 * abs(float(f0["phase_err"]))
        + 60.0 * abs(float(f0["gain_db_diff"]))
        + 200.0 * c5b["sway_rms_c1_glob_cm"]
        + 500.0 * c3["ss_std_cun_lat_cm"]
        + 200.0 * c4a["ss_std_pitch_deg"]
    )
    soft_penalty = (
        max(0.0, 0.55 - c4a["ss_c1_dx_rel_cm"]) * 3000.0
        + max(0.0, c4a["max_abs_pitch_deg"] - 12.5) * 2000.0
        + max(0.0, c4a["ss_std_pitch_deg"] - 0.35) * 4000.0
        + max(0.0, c3["ss_std_cun_lat_cm"] - 0.015) * 30000.0
        + max(0.0, c5b["sway_rms_c1_glob_cm"] - (c5a["sway_rms_c1_glob_cm"] - 0.12)) * 4000.0
        + max(0.0, abs(c5b["sway_mean_c1_rel_cm"]) - 0.10) * 3000.0
        + max(0.0, 0.43 - c3["ss_c1_dz_rel_cm"]) * 3000.0
        + max(0.0, 1.65 - c1["ss_c1_dx_rel_cm"]) * 2000.0
        + max(0.0, c2["ss_c1_dx_rel_cm"] + 1.30) * 2000.0
    )
    hard_invalid = any(f.endswith("_invalid") for f in failures)
    return failures, hard_invalid, obj, soft_penalty


def _save_standard_plot_summary(params, res_3, out_files):
    modes = ("APEO", "LatEC", "YawMA")
    summary = {
        "model_path": MODEL_PATH,
        "cli_overrides": [{"key": k, "val": v} for k, v in to_ov(params)],
        "false_booleans": res_3["APEO"]["false_booleans"],
        "bool_banner": res_3["APEO"]["bool_banner"],
        "bool_state": res_3["APEO"]["bool_state"],
        "axes": {},
    }
    for m in modes:
        r = res_3[m]
        summary["axes"][m] = {
            "output_png": out_files[m],
            "aborted": r["aborted"],
            "abort_time_s": r["abort_time_s"],
            "abort_reason": r["abort_reason"],
            "classic_bode_score": r["classic_bode_score"],
            "pitch_crosstalk_penalty": r["pitch_crosstalk_penalty"],
            "total_score": r["total_score"],
            "gain_loss_pts": r["gain_loss_pts"],
            "phase_loss_pts": r["phase_loss_pts"],
            "hf_penalty": r["hf_penalty"],
            "pitch_hf_10hz_rms_deg": r["pitch_hf_10hz_rms_deg"],
            "hf_noise_rms": r["hf_noise_rms"],
            "corr_vel": r["corr_vel"],
            "quiet_main_max_deg": r["quiet_main_max_deg"],
            "quiet_pitch_dev_max_deg": r["quiet_pitch_dev_max_deg"],
            "quiet_roll_max_deg": r["quiet_roll_max_deg"],
            "quiet_yaw_rel_max_deg": r["quiet_yaw_rel_max_deg"],
            "full_pitch_dev_max_deg": r["full_pitch_dev_max_deg"],
            "full_roll_max_deg": r["full_roll_max_deg"],
            "full_yaw_rel_max_deg": r["full_yaw_rel_max_deg"],
            "post_pitch_dev_max_deg": r["post_pitch_dev_max_deg"],
            "post_roll_max_deg": r["post_roll_max_deg"],
            "post_yaw_rel_max_deg": r["post_yaw_rel_max_deg"],
            "frequencies": r["freq_results"],
        }
    summary["combined_bode_score"] = round(sum(summary["axes"][m]["classic_bode_score"] for m in modes), 2)
    summary["combined_pitch_crosstalk_penalty"] = round(sum(summary["axes"][m]["pitch_crosstalk_penalty"] for m in modes), 2)
    summary["combined_hf_penalty"] = round(sum(summary["axes"][m]["hf_penalty"] for m in modes), 2)
    summary["combined_total_score"] = round(sum(summary["axes"][m]["total_score"] for m in modes), 2)
    with open(PLOT_SUM_OUT_JSON, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)
    print(f"[OK] Saved summary JSON: {PLOT_SUM_OUT_JSON}", flush=True)
    return summary


def _enqueue_and_run_slice(cand_slice, base_raw, workers):
    """Build Bode-first task queue for cand_slice and execute in parallel."""
    cand_map = {}
    apeo_tasks = []
    other_axis_tasks = []
    val_tasks = []
    seen_apeo_tw = set()
    for spec in cand_slice:
        tag, params = _build_candidate_params(base_raw, spec)
        _, _, _, tw, _, sway_cfg = spec
        tw_k = round(tw, 4)
        cand_map[tag] = {
            "spec": spec,
            "params": params,
            "tw": tw_k,
            "sway_cfg": sway_cfg,
            "axes": {},
            "val": [None] * 7,
        }
        if tw_k not in seen_apeo_tw:
            seen_apeo_tw.add(tw_k)
            apeo_tasks.append(("apeo", tw_k, params))
        other_axis_tasks.append(("axis", tag, "LatEC", params))
        other_axis_tasks.append(("axis", tag, "YawMA", params))
        for idx, val_sp in enumerate(build_validation_specs(params, sway_cfg[0], sway_cfg[1], sway_cfg[2])):
            val_tasks.append(("val", tag, idx, val_sp))

    # Schedule long 15.8s Bode tasks before shorter 6-8s validation tasks to minimize tail latency.
    tasks = apeo_tasks + other_axis_tasks + val_tasks
    print(f"=== PARALLEL EXECUTION ON {workers} WORKERS: {len(tasks)} tasks ({len(cand_slice)} candidates) ===", flush=True)
    with ProcessPoolExecutor(max_workers=workers) as ex:
        raw_out = list(ex.map(_dispatch, tasks))

    apeo_by_tw = {}
    for item in raw_out:
        if item[0] == "apeo":
            _, tw_k, r = item
            apeo_by_tw[tw_k] = r
        elif item[0] == "axis":
            _, tag, mode, r = item
            cand_map[tag]["axes"][mode] = r
        else:
            _, tag, idx, sm, tr = item
            cand_map[tag]["val"][idx] = {"summary": sm, "traces": tr}

    sweep_entries = []
    ranked_entries = []
    for spec in cand_slice:
        tag = spec[0]
        cinfo = cand_map[tag]
        res_3 = {"APEO": apeo_by_tw[cinfo["tw"]], "LatEC": cinfo["axes"]["LatEC"], "YawMA": cinfo["axes"]["YawMA"]}
        val_records = cinfo["val"]
        val_by_name = {rec["summary"]["name"]: rec for rec in val_records}
        failures, hard_invalid, obj, soft_penalty = _evaluate_gates(res_3, val_by_name)
        f0 = res_3["LatEC"]["freq_results"].get("0.1209", {"gain_db_diff": 0.0, "phase_err": 0.0, "sim_phase": 0.0})
        comb_bode = sum(float(res_3[m]["classic_bode_score"]) for m in ("APEO", "LatEC", "YawMA"))
        sweep_entries.append(
            {
                "tag": tag,
                "combined_bode_score": round(comb_bode, 2),
                "axes_bode": {m: res_3[m]["classic_bode_score"] for m in ("APEO", "LatEC", "YawMA")},
                "latec_0p1209": {"db": round(float(f0["gain_db_diff"]), 3), "ph_err": round(float(f0["phase_err"]), 2)},
                "validation": {rec["summary"]["name"]: rec["summary"] for rec in val_records},
                "gate_failures": failures,
                "hard_invalid": hard_invalid,
                "obj": round(obj, 2),
                "soft_penalty": round(soft_penalty, 2),
            }
        )
        ranked_entries.append(
            (
                hard_invalid,
                len(failures) > 0,
                obj + soft_penalty,
                obj,
                tag,
                cinfo["params"],
                cinfo["sway_cfg"],
                res_3,
                val_records,
                failures,
            )
        )
    return sweep_entries, ranked_entries


def _print_candidate_row(tag, res_3, val_records, failures, obj):
    val_by_name = {rec["summary"]["name"]: rec["summary"] for rec in val_records}
    c3 = val_by_name["Case3_Lat_Glide"]
    c4a = val_by_name["Case4A_Combined_wPos1"]
    c4b = val_by_name["Case4B_AttitudeOnly_wPos0"]
    c5a = val_by_name["Case5A_Sway_SE2Off"]
    c5b = val_by_name["Case5B_Sway_SE2On"]
    f0 = res_3["LatEC"]["freq_results"].get("0.1209", {"gain_db_diff": 0.0, "phase_err": 0.0})
    comb_bode = sum(float(res_3[m]["classic_bode_score"]) for m in ("APEO", "LatEC", "YawMA"))
    gate_str = "PASS" if not failures else f"FAIL({','.join(failures[:3])})"
    print(
        f"  {tag}: Lat={res_3['LatEC']['classic_bode_score']:7.2f}(0.12={f0['gain_db_diff']:+5.2f}dB/{f0['phase_err']:+5.2f}deg) "
        f"Yaw={res_3['YawMA']['classic_bode_score']:6.2f} AP={res_3['APEO']['classic_bode_score']:7.2f} Comb={comb_bode:7.2f} | "
        f"C3_dz={c3['ss_c1_dz_rel_cm']:+.2f}(std={c3['ss_std_cun_lat_cm']:.4f}) | "
        f"C4A[p={c4a['ss_pitch_deg']:+.2f},max={c4a['max_abs_pitch_deg']:.2f},dx={c4a['ss_c1_dx_rel_cm']:+.2f}] "
        f"C4B[dx={c4b['ss_c1_dx_rel_cm']:+.2f}] | "
        f"C5={c5a['sway_rms_c1_glob_cm']:.3f}->{c5b['sway_rms_c1_glob_cm']:.3f}(m={c5b['sway_mean_c1_rel_cm']:+.2f}) | "
        f"obj={obj:7.1f} [{gate_str}]",
        flush=True,
    )


def _finalize_outputs(sweep_entries, ranked_entries, elapsed_s):
    cand_order = {spec[0]: idx for idx, spec in enumerate(CANDIDATES)}
    sweep_sorted = sorted(sweep_entries, key=lambda e: cand_order.get(e["tag"], 999))
    with open(SWEEP_OUT_JSON, "w", encoding="utf-8") as f:
        json.dump(sweep_sorted, f, indent=2)
    print(f"[SAVED] {SWEEP_OUT_JSON}", flush=True)

    ranked_by_tag = {item[4]: item for item in ranked_entries}
    print("\n=== JOINT CANDIDATE MATRIX (3-AXIS BODE + 7-CASE VALIDATION) ===", flush=True)
    for spec in CANDIDATES:
        tag = spec[0]
        item = ranked_by_tag[tag]
        _print_candidate_row(tag, item[7], item[8], item[9], item[3])

    ranked_sorted = sorted(ranked_entries, key=lambda x: (x[0], x[1], x[2], x[3]))
    hard_invalid, has_gate_fail, _, win_obj, win_tag, win_params, sway_cfg, win_res_3, win_val_records, win_failures = ranked_sorted[0]
    if hard_invalid:
        raise RuntimeError(f"All candidates hard-invalid! Best={win_tag}, failures={win_failures}")
    print(f"\n=== WINNER: {win_tag} (gates={'ALL PASSED' if not has_gate_fail else win_failures}, obj={win_obj:.2f}) ===", flush=True)

    out_files = {
        "APEO": "standard_plot_apeo_pitch_c1pos_final.png",
        "LatEC": "standard_plot_latec_roll_c1pos_final.png",
        "YawMA": "standard_plot_yawma_yaw_c1pos_final.png",
    }
    for m in ("APEO", "LatEC", "YawMA"):
        render_standard_4panel(win_res_3[m], out_files[m])
    plot_summary = _save_standard_plot_summary(win_params, win_res_3, out_files)
    write_html_viewer(plot_summary["bool_banner"], plot_summary["axes"], out_files, suffix="_c1pos_final")

    render_validation_figure(win_val_records)
    val_json = {"elapsed_s": round(elapsed_s, 1), "candidate": win_tag, "cases": [r["summary"] for r in win_val_records]}
    with open(VAL_OUT_JSON, "w", encoding="utf-8") as f:
        json.dump(val_json, f, indent=2)
    print(f"[OK] Saved validation JSON: {VAL_OUT_JSON}", flush=True)

    win_c5b = next(r["summary"] for r in win_val_records if r["summary"]["name"] == "Case5B_Sway_SE2On")
    payload = {
        "source": f"C1-Pos 2x2 IK SE2(3) Joint ({win_tag})",
        "se2_c1pos_params": {
            k: win_params[k]
            for k in (
                "Kp_trans_roll", "Kv_trans_roll", "Kp_trans_pitch", "Kv_trans_pitch",
                "Kp_shift_obs_pz", "Kv_shift_obs_vz", "Kp_shift_obs_px", "Kv_shift_obs_vx",
                "Kp_shift_lat", "Kd_shift_lat", "Ki_shift_lat",
                "Kp_shift_ap", "Kd_shift_ap", "Ki_shift_ap",
                "tau_vel_leak", "tau_pos_leak", "tau_obs_washout",
                "shift_station_blend", "w_pos_ap", "w_pos_lat",
            )
        },
        "best_sway_ap_config": {
            "tag": f"{win_tag}_sway",
            "Kp_shift_obs_px": sway_cfg[0],
            "Kv_shift_obs_vx": sway_cfg[1],
            "Ki_shift_ap": sway_cfg[2],
            "sway_rms_c1_glob_cm": win_c5b["sway_rms_c1_glob_cm"],
        },
        "latec_0p1209_hz": {},
        "apeo_5p583_hz": {},
        "verified_3axis_windows": {},
        "params": win_params,
    }
    _populate_verified_windows(payload, win_res_3, win_res_3["APEO"]["false_booleans"])
    with open(CHAMP_OUT_JSON, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)
    print(f"[SAVED] {CHAMP_OUT_JSON} (elapsed={elapsed_s:.1f}s)", flush=True)


def main():
    parser = argparse.ArgumentParser(description="Joint C1-Pos 2x2 IK SE2(3) evaluation (full, sharded, or finalize-shards).")
    parser.add_argument("--shard-idx", type=int, default=None, help="Shard index 0..4 (evaluates 2 candidates on 16 workers).")
    parser.add_argument("--finalize-shards", action="store_true", help="Merge logs/c1pos_shard_0..4.pkl and render final outputs.")
    args = parser.parse_args()

    t0 = time.time()
    if args.finalize_shards:
        sweep_entries = []
        ranked_entries = []
        seen_tags = set()
        for idx in range(5):
            sp = os.path.join(_ROOT, "logs", f"c1pos_shard_{idx}.pkl")
            assert os.path.exists(sp), f"Missing shard pickle: {sp}"
            with open(sp, "rb") as pf:
                shard_data = pickle.load(pf)
            sweep_entries.extend(shard_data["sweep_entries"])
            ranked_entries.extend(shard_data["ranked_entries"])
            seen_tags.update(item[4] for item in shard_data["ranked_entries"])
        expected_tags = {spec[0] for spec in CANDIDATES}
        assert seen_tags == expected_tags, f"Missing candidates: {expected_tags - seen_tags}"
        _finalize_outputs(sweep_entries, ranked_entries, time.time() - t0)
        return

    base_raw = json.load(open(COOP_JSON, "r", encoding="utf-8"))["params"]
    if args.shard_idx is not None:
        idx = args.shard_idx
        assert 0 <= idx <= 4, f"Expected shard-idx in 0..4, got {idx}"
        cand_slice = CANDIDATES[2 * idx : 2 * idx + 2]
        slurm_n = os.environ.get("SLURM_NTASKS")
        workers = min(16, int(slurm_n) if slurm_n else (os.cpu_count() or 16))
        sweep_entries, ranked_entries = _enqueue_and_run_slice(cand_slice, base_raw, workers)
        for item in ranked_entries:
            _print_candidate_row(item[4], item[7], item[8], item[9], item[3])
        sp = os.path.join(_ROOT, "logs", f"c1pos_shard_{idx}.pkl")
        os.makedirs(os.path.dirname(sp), exist_ok=True)
        with open(sp, "wb") as pf:
            pickle.dump(
                {
                    "shard_idx": idx,
                    "elapsed_s": round(time.time() - t0, 1),
                    "sweep_entries": sweep_entries,
                    "ranked_entries": ranked_entries,
                },
                pf,
                protocol=pickle.HIGHEST_PROTOCOL,
            )
        print(f"[SAVED SHARD {idx}] {sp} ({len(ranked_entries)} candidates, elapsed={time.time() - t0:.1f}s)", flush=True)
        return

    workers = min(96, max(4, (os.cpu_count() or 16) - 4))
    sweep_entries, ranked_entries = _enqueue_and_run_slice(CANDIDATES, base_raw, workers)
    _finalize_outputs(sweep_entries, ranked_entries, time.time() - t0)


if __name__ == "__main__":
    main()
