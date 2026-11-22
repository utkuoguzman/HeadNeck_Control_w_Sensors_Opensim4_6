#!/usr/bin/env python3
"""TRUBA 112-core Wave 2+3 runner (resuming from Wave 1 best Roll C1-Pos config pz=-0.045, Ki_lat=3, Kd_lat=0.3, w=5)."""

import argparse
import copy
import json
import os
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

COOP_JSON = os.path.join(_ROOT, "logs", "coop_3axis_champion.json")
WAVE2_JSON = os.path.join(_ROOT, "logs", "c1pos_wave2_results.json")
PLOT_SUM_JSON = os.path.join(_ROOT, "logs", "standard_plot_summary_c1pos_final.json")
CHAMP_OUT_JSON = os.path.join(_ROOT, "logs", "se2_c1pos_3axis_champion.json")


def to_ov(params):
    return [(k, f"{float(v):.15g}") for k, v in params.items()]


def _dispatch(job):
    kind, tag, mode_or_spec, params = job
    if kind == "axis":
        from scripts.generate_standard_plot import run_axis_worker
        res = run_axis_worker((mode_or_spec, to_ov(params)))
        return (kind, tag, params, res)
    from scripts.validate_5dof_voluntary_and_se2_shift import run_validation_case
    sm, _ = run_validation_case(mode_or_spec)
    return (kind, tag, params, sm)


def latec_obj(r):
    if r["aborted"] or r["false_booleans"] or round(float(r["quiet_main_max_deg"]), 4) > 0.001:
        return 1e9
    f0 = r["freq_results"]["0.1209"]
    return (
        float(r["classic_bode_score"])
        + float(r["pitch_crosstalk_penalty"])
        + 25.0 * float(r["hf_noise_rms"])
        + 4.0 * abs(float(f0["phase_err"]))
        + 80.0 * abs(float(f0["gain_db_diff"]))
    )


def apeo_obj(r):
    if r["aborted"] or r["false_booleans"] or round(float(r["quiet_main_max_deg"]), 4) > 0.001:
        return 1e9
    f_hi = r["freq_results"]["5.583"]
    db_excess = max(0.0, float(f_hi["gain_db_diff"]) - 1.25)
    return (
        float(r["classic_bode_score"])
        + float(r["pitch_crosstalk_penalty"])
        + 25.0 * float(r["hf_noise_rms"])
        + 300.0 * db_excess
    )


def _build_base_params():
    base = dict(json.load(open(COOP_JSON, "r", encoding="utf-8"))["params"])
    base.update(
        {
            "Kp_trans_roll": 0.0,
            "Kv_trans_roll": 0.0,
            "Kp_trans_pitch": 0.0,
            "Kv_trans_pitch": 0.0,
            "Kp_shift_obs_px": 0.0,
            "Kv_shift_obs_vx": 0.0,
            "Kp_shift_obs_pz": -0.045,
            "Kv_shift_obs_vz": 0.0,
            "Kp_shift_ap": 4.5,
            "Kd_shift_ap": 0.45,
            "Ki_shift_ap": 6.0,
            "Kp_shift_lat": 4.5,
            "Kd_shift_lat": 0.30,
            "Ki_shift_lat": 3.0,
            "w_pos_ap": 1.0,
            "w_pos_lat": 1.0,
            "shift_station_blend": 0.0,
            "tau_vel_leak": 3.0,
            "tau_pos_leak": 2.0,
            "tau_obs_washout": 5.0,
        }
    )
    return base


def _select_and_report_wave2(roll_res, apeo_res, sway_map):
    roll_res.sort(key=lambda x: latec_obj(x[2]))
    print("\n  Top Roll Fine Candidates:", flush=True)
    for t, _, r in roll_res:
        f0 = r["freq_results"]["0.1209"]
        print(
            f"    {t:18s} | LatEC Bode={r['classic_bode_score']:7.2f} | "
            f"0.12Hz: dB={f0['gain_db_diff']:+6.3f}, ph={f0['phase_err']:+6.2f} | "
            f"HF={r['hf_noise_rms']:.3f} | XT={r['pitch_crosstalk_penalty']:5.1f} | "
            f"corr={r['corr_vel']:+.4f} | obj={latec_obj(r):8.2f}",
            flush=True,
        )
    best_r_lbl, best_r_params, _ = roll_res[0]

    apeo_res.sort(key=lambda x: apeo_obj(x[2]))
    print("\n  Top 12 Pitch APEO Candidates (+ Case 5B Sway C1 Global RMS):", flush=True)
    for t, _, r in apeo_res[:12]:
        if r.get("aborted") or "5.583" not in r.get("freq_results", {}):
            continue
        f_hi = r["freq_results"]["5.583"]
        f_lo = r["freq_results"].get("0.324") or next(iter(r["freq_results"].values()))
        sw = sway_map.get(t)
        sw_str = f"SwayGlob={sw['sway_rms_c1_glob_cm']:.3f}cm (mean={sw['sway_mean_c1_rel_cm']:+.2f})" if sw else "Sway=n/a"
        print(
            f"    {t:22s} | APEO Bode={r['classic_bode_score']:7.2f} | "
            f"0.32Hz={f_lo['gain_db_diff']:+5.2f}dB | 5.58Hz={f_hi['gain_db_diff']:+5.3f}dB | "
            f"HF={r['hf_noise_rms']:.3f} | corr={r['corr_vel']:+.4f} | {sw_str}",
            flush=True,
        )

    apeo_by_tag = {t: r for (t, _, r) in apeo_res}
    apeo_params_by_tag = {t: p for (t, p, _) in apeo_res}
    sway_valid = [
        (t, sw)
        for t, sw in sway_map.items()
        if not sw.get("aborted")
        and not sw.get("false_booleans")
        and t in apeo_by_tag
        and not apeo_by_tag[t].get("aborted")
        and not apeo_by_tag[t].get("false_booleans")
        and "5.583" in apeo_by_tag[t].get("freq_results", {})
    ]
    assert sway_valid, "No valid non-aborted Case 5B sway candidate found with valid APEO frequency results"
    sway_sorted = sorted(sway_valid, key=lambda kv: kv[1]["sway_rms_c1_glob_cm"])
    print("\n  Best Case 5B Trunk-Sway Attenuation Candidates (Non-Aborted Only):", flush=True)
    for t, sw in sway_sorted[:6]:
        ar = apeo_by_tag[t]
        f_hi = ar["freq_results"]["5.583"]
        print(
            f"    {t:22s} | SwayC1Glob={sw['sway_rms_c1_glob_cm']:.3f}cm "
            f"(COM={sw['sway_rms_com_glob_cm']:.3f}cm, mean={sw['sway_mean_c1_rel_cm']:+.2f}) | "
            f"APEO={ar['classic_bode_score']:7.2f} (5.58Hz={f_hi['gain_db_diff']:+5.3f}dB)",
            flush=True,
        )

    best_p_lbl, best_p_params, _ = apeo_res[0]
    best_sw_lbl, best_sw_sm = sway_sorted[0]
    best_sw_params = apeo_params_by_tag[best_sw_lbl]
    return best_r_lbl, best_r_params, best_p_lbl, best_p_params, best_sw_lbl, best_sw_params, best_sw_sm


def _build_base_payload(best_r_lbl, best_r_params, best_p_lbl, best_p_params, best_sw_lbl, best_sw_params, best_sw_sm):
    final_params = copy.deepcopy(best_r_params)
    final_params["Kp_shift_obs_px"] = best_p_params["Kp_shift_obs_px"]
    final_params["Kv_shift_obs_vx"] = best_p_params["Kv_shift_obs_vx"]
    final_params["Ki_shift_ap"] = best_p_params["Ki_shift_ap"]
    return final_params, {
        "source": f"C1-Pos 2x2 IK SE2(3) ({best_r_lbl} + {best_p_lbl})",
        "se2_c1pos_params": {
            k: final_params[k]
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
            "tag": best_sw_lbl,
            "Kp_shift_obs_px": best_sw_params["Kp_shift_obs_px"],
            "Kv_shift_obs_vx": best_sw_params["Kv_shift_obs_vx"],
            "Ki_shift_ap": best_sw_params["Ki_shift_ap"],
            "sway_rms_c1_glob_cm": best_sw_sm["sway_rms_c1_glob_cm"],
        },
        "latec_0p1209_hz": {},
        "apeo_5p583_hz": {},
        "verified_3axis_windows": {},
        "params": final_params,
    }


def _populate_verified_windows(payload, axes_by_mode, false_booleans):
    assert not false_booleans, f"False booleans detected: {false_booleans}"
    tot_bode = tot_xt = tot_hf = 0.0
    axes_summary = {}
    for m in ("APEO", "LatEC", "YawMA"):
        r = axes_by_mode[m]
        assert not r.get("false_booleans") and not r["aborted"], f"{m} failed verification: aborted={r['aborted']}"
        b = float(r["classic_bode_score"])
        xt = float(r["pitch_crosstalk_penalty"])
        hf = float(r["hf_penalty"])
        tot_bode += b
        tot_xt += xt
        tot_hf += hf
        axes_summary[m] = {
            "classic_bode_score": round(b, 2),
            "gain_loss_pts": round(float(r["gain_loss_pts"]), 2),
            "phase_loss_pts": round(float(r["phase_loss_pts"]), 2),
            "pitch_crosstalk_penalty": round(xt, 2),
            "hf_penalty": round(hf, 2),
            "hf_noise_rms": round(float(r["hf_noise_rms"]), 3),
            "total_score": round(b + xt + hf, 2),
            "corr_vel": round(float(r["corr_vel"]), 4),
            "quiet_main_max_deg": round(float(r["quiet_main_max_deg"]), 4),
            "aborted": bool(r["aborted"]),
        }
        print(
            f"  [{m:5s}] Bode={b:7.2f} | XT={xt:5.1f} | HF={r['hf_noise_rms']:.3f} | "
            f"corr={r['corr_vel']:+.4f} | Q={r['quiet_main_max_deg']:.4f}",
            flush=True,
        )

    lat_freqs = axes_by_mode["LatEC"].get("frequencies", axes_by_mode["LatEC"].get("freq_results", {}))
    ap_freqs = axes_by_mode["APEO"].get("frequencies", axes_by_mode["APEO"].get("freq_results", {}))
    f0 = lat_freqs["0.1209"]
    f5 = ap_freqs["5.583"]
    print(f"  [LatEC 0.1209 Hz] dB={f0['gain_db_diff']:+6.3f}, ph={f0['sim_phase']:.2f} (err={f0['phase_err']:+.2f} deg)", flush=True)
    print(f"  [APEO   5.583 Hz] gain={f5['sim_gain']:.3f}, dB={f5['gain_db_diff']:+6.3f}", flush=True)
    print(f"  Combined Bode={tot_bode:.2f}, XT={tot_xt:.2f}, Total={tot_bode + tot_xt + tot_hf:.2f}", flush=True)

    payload["latec_0p1209_hz"] = {
        "ph_0p12": round(float(f0["sim_phase"]), 2),
        "ph_err_0p12": round(float(f0["phase_err"]), 2),
        "db_0p12": round(float(f0["gain_db_diff"]), 3),
    }
    payload["apeo_5p583_hz"] = {
        "sim_gain": round(float(f5["sim_gain"]), 3),
        "db_5p583": round(float(f5["gain_db_diff"]), 3),
    }
    payload["verified_3axis_windows"] = {
        "combined_bode_score": round(tot_bode, 2),
        "combined_xtalk_penalty": round(tot_xt, 2),
        "combined_hf_penalty": round(tot_hf, 2),
        "combined_total_score": round(tot_bode + tot_xt + tot_hf, 2),
        "false_booleans": [],
        "axes": axes_summary,
    }


def prepare_from_wave2():
    """Reconstruct Wave 2 winner & non-aborted Case 5B config into logs/se2_c1pos_3axis_champion.json."""
    base = _build_base_params()
    wave2_log = json.load(open(WAVE2_JSON, "r", encoding="utf-8"))
    roll_res, apeo_res, sway_map = [], [], {}
    for entry in wave2_log:
        t = entry["tag"]
        p = copy.deepcopy(base)
        p.update(entry.get("key_params", {}))
        if entry["kind"] == "axis":
            if entry.get("mode") == "LatEC" or t.startswith("R_"):
                roll_res.append((t, p, entry))
            else:
                apeo_res.append((t, p, entry))
        else:
            sway_map[t] = entry["summary"]

    best_r_lbl, best_r_params, best_p_lbl, best_p_params, best_sw_lbl, best_sw_params, best_sw_sm = _select_and_report_wave2(
        roll_res, apeo_res, sway_map
    )
    _, payload = _build_base_payload(
        best_r_lbl, best_r_params, best_p_lbl, best_p_params, best_sw_lbl, best_sw_params, best_sw_sm
    )
    with open(CHAMP_OUT_JSON, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)
    print(f"[PREPARED FROM WAVE2] {CHAMP_OUT_JSON} (source={payload['source']}, best_sway={best_sw_lbl})", flush=True)


def finalize_from_plot_summary():
    """Populate verified_3axis_windows in logs/se2_c1pos_3axis_champion.json from standard_plot_summary_c1pos_final.json."""
    payload = json.load(open(CHAMP_OUT_JSON, "r", encoding="utf-8"))
    plot_sum = json.load(open(PLOT_SUM_JSON, "r", encoding="utf-8"))
    print("\n=== FINALIZING 3-AXIS SUMMARY FROM STANDARD PLOT RUN ===", flush=True)
    _populate_verified_windows(payload, plot_sum["axes"], plot_sum["false_booleans"])
    with open(CHAMP_OUT_JSON, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)
    print(f"[FINALIZED] {CHAMP_OUT_JSON}", flush=True)


def main():
    parser = argparse.ArgumentParser(description="Tune/prepare/finalize C1-Position 2x2 IK SE2(3) champion.")
    parser.add_argument("--prepare-from-wave2", action="store_true", help="Select winners from saved logs/c1pos_wave2_results.json without re-running Wave 2.")
    parser.add_argument("--finalize-from-plot-summary", action="store_true", help="Populate verified_3axis_windows from logs/standard_plot_summary_c1pos_final.json.")
    args = parser.parse_args()
    if args.prepare_from_wave2:
        prepare_from_wave2()
        return
    if args.finalize_from_plot_summary:
        finalize_from_plot_summary()
        return

    t0 = time.time()
    workers = min(96, max(4, (os.cpu_count() or 16) - 4))
    base = _build_base_params()

    jobs = []
    # Fine Roll polish around Wave 1 winner (LatEC Bode=1926.24, 0.12Hz +0.17dB/+0.70deg)
    roll_cands = [
        ("R_pz-0.0435", -0.0435,  0.000, 3.0, 0.30, 5.0),
        ("R_pz-0.0445", -0.0445,  0.000, 3.0, 0.30, 5.0),
        ("R_pz-0.0450", -0.0450,  0.000, 3.0, 0.30, 5.0),
        ("R_pz-0.0455", -0.0455,  0.000, 3.0, 0.30, 5.0),
        ("R_pz-0.0450_vzm", -0.0450, -0.001, 3.0, 0.30, 5.0),
        ("R_pz-0.0450_vzp", -0.0450, +0.001, 3.0, 0.30, 5.0),
    ]
    for lbl, kpz, kvz, kil, kdl, tw in roll_cands:
        p = copy.deepcopy(base)
        p.update({"Kp_shift_obs_pz": kpz, "Kv_shift_obs_vz": kvz, "Ki_shift_lat": kil, "Kd_shift_lat": kdl, "tau_obs_washout": tw})
        jobs.append(("axis", lbl, "LatEC", p))

    # Simultaneous Pitch APEO + Case5B 0.5 Hz Sway grid
    kp_x_grid = [0.0, -0.04, -0.10, -0.20, -0.35, -0.50]
    kv_x_grid = [0.0, -0.06, -0.15]
    ki_x_grid = [6.0, 0.0]
    for kpx in kp_x_grid:
        for kvx in kv_x_grid:
            for kia in ki_x_grid:
                lbl = f"px{kpx:+.2f}_vx{kvx:+.2f}_ki{int(kia)}"
                p = copy.deepcopy(base)
                p["Kp_shift_obs_px"] = kpx
                p["Kv_shift_obs_vx"] = kvx
                p["Ki_shift_ap"] = kia
                jobs.append(("axis", lbl, "APEO", p))
                if kia == 6.0 or (kpx in (-0.35, -0.50) and kvx in (-0.06, -0.15)):
                    sway_spec = {
                        "name": f"Sway_{lbl}",
                        "label": lbl,
                        "base_params": p,
                        "init_overrides": {"w_pos_ap": 1.0, "Kp_shift_obs_px": kpx, "Kv_shift_obs_vx": kvx, "Ki_shift_ap": kia, "tau_obs_washout": 4.0},
                        "step_overrides": {},
                        "step_time": 0.3,
                        "duration": 8.0,
                        "sway_amp_m": 0.015,
                        "sway_freq_hz": 0.5,
                    }
                    jobs.append(("sway", lbl, sway_spec, p))

    print(f"\n=== PARALLEL SWEEP ON {workers} WORKERS: {len(jobs)} jobs (Roll fine + APEO grid + Case5B sway) ===", flush=True)
    with ProcessPoolExecutor(max_workers=workers) as ex:
        results = list(ex.map(_dispatch, jobs))

    # Persist lightweight Wave 2 summaries before printing/ranking so no compute is ever lost.
    wave2_log = []
    for k, t, p, r in results:
        if k == "axis":
            fr_brief = {
                fk: {
                    "sim_gain": round(float(fv["sim_gain"]), 4),
                    "gain_db_diff": round(float(fv["gain_db_diff"]), 4),
                    "sim_phase": round(float(fv["sim_phase"]), 2),
                    "phase_err": round(float(fv["phase_err"]), 2),
                }
                for fk, fv in r["freq_results"].items()
            }
            wave2_log.append({
                "kind": k, "tag": t, "mode": r["mode"],
                "aborted": bool(r["aborted"]), "false_booleans": list(r["false_booleans"]),
                "classic_bode_score": float(r["classic_bode_score"]),
                "pitch_crosstalk_penalty": float(r["pitch_crosstalk_penalty"]),
                "hf_noise_rms": float(r["hf_noise_rms"]),
                "corr_vel": float(r["corr_vel"]),
                "quiet_main_max_deg": float(r["quiet_main_max_deg"]),
                "freq_results": fr_brief,
                "key_params": {kp_k: p[kp_k] for kp_k in ("Kp_shift_obs_pz", "Kv_shift_obs_vz", "Kp_shift_obs_px", "Kv_shift_obs_vx", "Ki_shift_ap", "Ki_shift_lat", "Kd_shift_lat", "tau_obs_washout")},
            })
        else:
            wave2_log.append({
                "kind": k, "tag": t,
                "summary": r,
                "key_params": {kp_k: p[kp_k] for kp_k in ("Kp_shift_obs_px", "Kv_shift_obs_vx", "Ki_shift_ap")},
            })
    wave2_path = os.path.join(_ROOT, "logs", "c1pos_wave2_results.json")
    with open(wave2_path, "w", encoding="utf-8") as wf:
        json.dump(wave2_log, wf, indent=2)
    print(f"[SAVED] {wave2_path} ({len(wave2_log)} entries)", flush=True)

    roll_res = [(t, p, r) for (k, t, p, r) in results if k == "axis" and t.startswith("R_")]
    sway_map = {t: r for (k, t, _, r) in results if k == "sway"}
    apeo_res = [(t, p, r) for (k, t, p, r) in results if k == "axis" and not t.startswith("R_")]
    best_r_lbl, best_r_params, best_p_lbl, best_p_params, best_sw_lbl, best_sw_params, best_sw_sm = _select_and_report_wave2(
        roll_res, apeo_res, sway_map
    )
    final_params, payload = _build_base_payload(
        best_r_lbl, best_r_params, best_p_lbl, best_p_params, best_sw_lbl, best_sw_params, best_sw_sm
    )

    # Simultaneous 3-axis verification
    print("\n=== FINAL 3-AXIS VERIFICATION ===", flush=True)
    from scripts.generate_standard_plot import run_axis_worker
    ov_final = to_ov(final_params)
    with ProcessPoolExecutor(max_workers=3) as ex:
        futs = [ex.submit(run_axis_worker, (m, ov_final)) for m in ("APEO", "LatEC", "YawMA")]
        res_3 = {r["mode"]: r for r in (f.result() for f in futs)}

    _populate_verified_windows(payload, res_3, res_3["APEO"]["false_booleans"])
    print(f"  Elapsed: {time.time() - t0:.1f}s", flush=True)
    with open(CHAMP_OUT_JSON, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)
    print(f"[SAVED] {CHAMP_OUT_JSON}", flush=True)


if __name__ == "__main__":
    main()
