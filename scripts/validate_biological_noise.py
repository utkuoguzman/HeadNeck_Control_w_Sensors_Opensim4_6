#!/usr/bin/env python3
"""Tier-1 Physiological Sensory & Motor Noise Verification Suite (Noise-OFF vs 2x2 Ablation vs 3-Seed Bode & C1 Shift)."""

import cmath
import json
import math
import os
import shutil
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

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_SCRIPT_DIR)
for _p in (_ROOT, _SCRIPT_DIR, os.path.join(_ROOT, "util")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import opensim as osim
from cmaes_worker_p22 import PER_MUSCLE_72_U0
from scripts.generate_standard_plot import apply_cli_overrides_and_inspect, run_axis_worker
from validate_5dof_voluntary_and_se2_shift import (
    CHAMP_C1_JSON,
    MODEL_PATH,
    build_validation_specs,
    ensure_libraries_loaded,
    run_validation_case,
)

ART_DIR = r"C:\Users\User\.gemini\antigravity\brain\89fb8d56-9d12-4051-a256-02497ef8d6ae"
OUT_JSON = os.path.join(_ROOT, "logs", "biological_noise_validation.json")
OUT_PNG = os.path.join(_ROOT, "results", "biological_noise_validation.png")

SEEDS = [1, 2, 3]
BODE_MODES = ["APEO", "LatEC", "YawMA"]


def run_quiet_stance_worker(spec: dict) -> dict:
    """5.0 s quiet upright stance hold under a specific (sensor_noise, motor_noise, seed) config."""
    t0 = time.time()
    ensure_libraries_loaded()
    pdict = json.load(open(CHAMP_C1_JSON, "r", encoding="utf-8"))["params"]

    s_on = bool(spec["sensor_noise"])
    m_on = bool(spec["motor_noise"])
    seed = int(spec["seed"])
    scale = float(spec.get("noise_scale", 1.0))
    duration = float(spec.get("duration", 5.0))

    overrides = [(k, f"{float(v):.15g}") for k, v in pdict.items()] + [
        ("urgency", "0.0"),
        ("desired_pitch", "0.0"),
        ("desired_roll", "0.0"),
        ("desired_yaw", "0.0"),
        ("desired_ap_shift", "0.0"),
        ("desired_lat_shift", "0.0"),
        ("enable_sensor_noise", "true" if s_on else "false"),
        ("enable_motor_noise", "true" if m_on else "false"),
        ("noise_seed", str(seed)),
        ("noise_scale", f"{scale:.6g}"),
    ]

    model = osim.Model(MODEL_PATH)
    controller = None
    for i in range(model.getControllerSet().getSize()):
        c = model.updControllerSet().get(i)
        if c.getConcreteClassName() == "FixationController":
            c.set_enabled(False)
        elif c.getConcreteClassName() == "HeadNeckNeuralController":
            controller = c
    apply_cli_overrides_and_inspect(controller, overrides)

    for eye_c in ["r_eye_add_abd", "r_eye_sup_inf", "r_eye_inc_exc", "l_eye_add_abd", "l_eye_sup_inf", "l_eye_inc_exc"]:
        if model.getCoordinateSet().contains(eye_c):
            c = model.updCoordinateSet().get(eye_c)
            c.setDefaultValue(0.0)
            c.setDefaultLocked(True)

    zero_func = osim.Constant(0.0)
    for coord_name in ["gndpitch", "gndroll", "gndyaw", "spine_tx", "spine_ty", "spine_tz"]:
        c = model.updCoordinateSet().get(coord_name)
        c.set_locked(False)
        c.setDefaultIsPrescribed(True)
        c.setPrescribedFunction(zero_func)

    c_yaw1 = model.updCoordinateSet().get("yaw1")
    c_yaw1.set_locked(False)
    c_yaw1.setDefaultIsPrescribed(False)

    state = model.initSystem()
    c_yaw1.setIsPrescribed(state, False)
    model.realizePosition(state)

    skull = model.getBodySet().get("skull")
    q0 = osim.Rotation(skull.getTransformInGround(state).R()).convertRotationToQuaternion()
    for idx in range(4):
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/obs_q{idx}", q0.get(idx))
    for sv in ("pitch_error_integral", "roll_error_integral", "yaw_error_integral", "vol_filt_pitch", "vol_filt_roll", "vol_filt_yaw"):
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/{sv}", 0.0)

    k_gdyn = float(pdict.get("K_gamma_dyn", 0.857744))
    k_gstat = float(pdict.get("K_gamma_stat", 0.69312344))
    muscles = model.updMuscles()
    neck_idx = []
    for i in range(muscles.getSize()):
        m = muscles.get(i)
        mname = m.getName()
        if mname.startswith("r_") or mname.startswith("l_"):
            m.setAppliesForce(state, False)
            continue
        neck_idx.append(i)
        a0 = PER_MUSCLE_72_U0.get(mname, 0.030)
        m.setActivation(state, a0)
        gd0 = 100.0 * k_gdyn * (0.25 + 0.75 * a0)
        gs0 = 100.0 * k_gstat * a0
        fdyn0 = (gd0 * gd0) / (gd0 * gd0 + 3600.0)
        fstat0 = (gs0 * gs0) / (gs0 * gs0 + 3600.0)
        m.setStateVariableValue(state, "spindle/dynamic_activation", fdyn0)
        m.setStateVariableValue(state, "spindle/static_activation", fstat0)
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/sp_fdyn_{mname}", fdyn0)
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/sp_fstat_{mname}", fstat0)
    model.equilibrateMuscles(state)

    mgr = osim.Manager(model)
    mgr.setWriteToStorage(False)
    mgr.setPerformAnalyses(False)
    mgr.setIntegratorMethod(5)
    mgr.setIntegratorAccuracy(1e-4)
    mgr.setIntegratorMinimumStepSize(1e-4)
    mgr.setIntegratorMaximumStepSize(0.001)
    state.setTime(0.0)
    mgr.initialize(state)

    cs = model.getCoordinateSet()
    c_p1, c_p2 = cs.get("pitch1"), cs.get("pitch2")
    c_r1, c_r2 = cs.get("roll1"), cs.get("roll2")
    c_y1, c_y2 = cs.get("yaw1"), cs.get("yaw2")

    ts, ps, rs, ys, macts = [], [], [], [], []
    dt = 0.01
    t_now = 0.0
    stop_reason = None
    while t_now < duration - 1e-9:
        t_next = round(min(t_now + dt, duration), 5)
        try:
            state = mgr.integrate(t_next)
            model.realizeDynamics(state)
        except Exception as e:
            stop_reason = f"Stopped at t={t_next:.3f}s: {str(e).splitlines()[0]}"
            break
        t_now = t_next
        ts.append(t_now)
        ps.append(math.degrees(c_p1.getValue(state) + c_p2.getValue(state)))
        rs.append(math.degrees(c_r1.getValue(state) + c_r2.getValue(state)))
        ys.append(math.degrees(c_y1.getValue(state) + c_y2.getValue(state)))
        macts.append(max(float(muscles.get(idx).getActivation(state)) for idx in neck_idx))

    t_arr = np.asarray(ts)
    mask = t_arr >= 1.0 - 1e-9
    p_w, r_w, y_w = np.asarray(ps)[mask], np.asarray(rs)[mask], np.asarray(ys)[mask]

    def rms_demean(arr):
        return float(np.sqrt(np.mean((arr - np.mean(arr)) ** 2))) if len(arr) else 0.0

    return {
        "kind": "quiet",
        "label": spec["label"],
        "sensor_noise": s_on,
        "motor_noise": m_on,
        "seed": seed,
        "stop_reason": stop_reason,
        "wall_s": round(time.time() - t0, 2),
        "pitch_rms_deg": round(rms_demean(p_w), 4),
        "roll_rms_deg": round(rms_demean(r_w), 4),
        "yaw_rms_deg": round(rms_demean(y_w), 4),
        "max_exc_deg": round(float(max(np.max(np.abs(p_w)), np.max(np.abs(r_w)), np.max(np.abs(y_w)))), 4),
        "max_act": round(float(np.max(np.asarray(macts)[mask])), 4),
        "t": [round(float(v), 3) for v in ts],
        "pitch_deg": [round(float(v), 4) for v in ps],
        "roll_deg": [round(float(v), 4) for v in rs],
        "yaw_deg": [round(float(v), 4) for v in ys],
    }


def run_bode_worker(spec: dict) -> dict:
    pdict = json.load(open(CHAMP_C1_JSON, "r", encoding="utf-8"))["params"]
    overrides = [(k, f"{float(v):.15g}") for k, v in pdict.items()] + [
        ("enable_sensor_noise", "true" if spec["sensor_noise"] else "false"),
        ("enable_motor_noise", "true" if spec["motor_noise"] else "false"),
        ("noise_seed", str(int(spec["seed"]))),
        ("noise_scale", f"{float(spec.get('noise_scale', 1.0)):.6g}"),
    ]
    res = run_axis_worker((spec["mode"], overrides))
    return {
        "kind": "bode",
        "tag": spec["tag"],
        "mode": spec["mode"],
        "seed": int(spec["seed"]),
        "sensor_noise": bool(spec["sensor_noise"]),
        "motor_noise": bool(spec["motor_noise"]),
        "classic_bode_score": res["classic_bode_score"],
        "total_score": res["total_score"],
        "gain_loss_pts": res["gain_loss_pts"],
        "phase_loss_pts": res["phase_loss_pts"],
        "pitch_crosstalk_penalty": res["pitch_crosstalk_penalty"],
        "hf_penalty": res["hf_penalty"],
        "hf_noise_rms": res["hf_noise_rms"],
        "post_pitch_dev_max_deg": res["post_pitch_dev_max_deg"],
        "post_roll_max_deg": res["post_roll_max_deg"],
        "post_yaw_rel_max_deg": res["post_yaw_rel_max_deg"],
        "corr_vel": res["corr_vel"],
        "quiet_main_max_deg": res["quiet_main_max_deg"],
        "aborted": res["aborted"],
        "freq_frf": {
            k: {
                "sim_gain": float(v["sim_gain"]),
                "sim_phase": float(v["sim_phase"]),
                "target_gain": float(v["target_gain"]),
                "target_phase": float(v["target_phase"]),
            }
            for k, v in res.get("freq_results", {}).items()
        },
    }


def run_shift_worker(spec: dict) -> dict:
    pdict = json.load(open(CHAMP_C1_JSON, "r", encoding="utf-8"))["params"]
    specs = {s["name"]: s for s in build_validation_specs(pdict, 0.0, -0.16, 3.0)}
    sp = dict(specs[spec["case_name"]])
    init_ov = dict(sp["init_overrides"])
    init_ov["enable_sensor_noise"] = "true" if spec["sensor_noise"] else "false"
    init_ov["enable_motor_noise"] = "true" if spec["motor_noise"] else "false"
    init_ov["noise_seed"] = int(spec["seed"])
    init_ov["noise_scale"] = float(spec.get("noise_scale", 1.0))
    sp["init_overrides"] = init_ov
    sm, tr = run_validation_case(sp)
    return {
        "kind": "shift",
        "tag": spec["tag"],
        "case_name": spec["case_name"],
        "sensor_noise": bool(spec["sensor_noise"]),
        "motor_noise": bool(spec["motor_noise"]),
        "seed": int(spec["seed"]),
        "summary": sm,
        "traces": tr,
    }


def make_noise_plot(quiet_res: list, bode_res: list, shift_res: list, ens_frf: dict):
    fig, axes = plt.subplots(2, 2, figsize=(14.5, 9.2), dpi=180)

    # Panel A: Quiet Stance 2x2 RMS Sway
    ax = axes[0, 0]
    labels = ["Off (0,0)", "Sensor (1,0)", "Motor (0,1)", "Both (1,1)"]
    axes_keys = [("pitch_rms_deg", "Pitch RMS", "#059669"), ("roll_rms_deg", "Roll RMS", "#2563eb"), ("yaw_rms_deg", "Yaw RMS", "#7c3aed")]
    x = np.arange(len(labels))
    w = 0.25
    for idx, (k, name, col) in enumerate(axes_keys):
        means, sds = [], []
        for lbl in labels:
            vals = [r[k] for r in quiet_res if r["label"] == lbl]
            means.append(np.mean(vals))
            sds.append(np.std(vals, ddof=1) if len(vals) > 1 else 0.0)
        ax.bar(x + (idx - 1) * w, means, w, yerr=sds, capsize=3, color=col, alpha=0.88, label=name)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=9)
    ax.set_ylabel("Demeaned RMS Sway (1–5 s) [deg]", fontweight="bold")
    ax.set_title("A. Quiet Upright Stance 2x2 Noise Ablation (Seeds 1–3)", fontweight="bold")
    ax.grid(True, axis="y", linestyle=":", alpha=0.55)
    ax.legend(fontsize=8.5, loc="upper left")

    # Panel B: Quiet Stance Time-Series (Both vs Off)
    ax = axes[0, 1]
    q_off = [r for r in quiet_res if r["label"] == "Off (0,0)"][0]
    q_both = [r for r in quiet_res if r["label"] == "Both (1,1)" and r["seed"] == 1][0]
    ax.plot(q_off["t"], q_off["pitch_deg"], "#94a3b8", ls="--", lw=1.4, label=f"Off Pitch (RMS={q_off['pitch_rms_deg']:.3f}°)")
    ax.plot(q_both["t"], q_both["pitch_deg"], "#059669", lw=1.6, label=f"Both Pitch s1 (RMS={q_both['pitch_rms_deg']:.3f}°)")
    ax.plot(q_both["t"], q_both["roll_deg"], "#2563eb", lw=1.4, alpha=0.85, label=f"Both Roll s1 (RMS={q_both['roll_rms_deg']:.3f}°)")
    ax.plot(q_both["t"], q_both["yaw_deg"], "#7c3aed", lw=1.4, alpha=0.85, label=f"Both Yaw s1 (RMS={q_both['yaw_rms_deg']:.3f}°)")
    ax.set_xlabel("Time [s]", fontweight="bold")
    ax.set_ylabel("Head Attitude [deg]", fontweight="bold")
    ax.set_title("B. Quiet Sway Trace: Tier-1 Sensory+Motor Noise vs OFF", fontweight="bold")
    ax.grid(True, linestyle=":", alpha=0.55)
    ax.legend(fontsize=8.0, loc="upper right")

    # Panel C: 3-Axis Bode Score OFF vs ON (mean +- SD)
    ax = axes[1, 0]
    xm = np.arange(len(BODE_MODES))
    off_vals = [next(r["total_score"] for r in bode_res if r["tag"] == "OFF" and r["mode"] == m) for m in BODE_MODES]
    on_means, on_sds = [], []
    for m in BODE_MODES:
        vs = [r["total_score"] for r in bode_res if r["tag"] == "ON" and r["mode"] == m]
        on_means.append(np.mean(vs))
        on_sds.append(np.std(vs, ddof=1) if len(vs) > 1 else 0.0)
    ax.bar(xm - 0.18, off_vals, 0.34, color="#64748b", label="Noise OFF (Deterministic)")
    ax.bar(xm + 0.18, on_means, 0.34, yerr=on_sds, capsize=4, color="#059669", label="Sensor+Motor ON (Seeds 1–3)")
    y_top = max(max(off_vals), max(m + s for m, s in zip(on_means, on_sds))) * 1.24
    for i, (m_name, ov, om, osd) in enumerate(zip(BODE_MODES, off_vals, on_means, on_sds)):
        ax.text(i - 0.18, ov + 0.02 * y_top, f"{ov:.0f}", ha="center", va="bottom", fontsize=8)
        ef = ens_frf[m_name]
        ax.text(i + 0.18, om + osd + 0.02 * y_top, f"{om:.0f}±{osd:.0f}\n(EnsC:{ef['ens_classic']:.0f})", ha="center", va="bottom", fontsize=7.6)
    ax.set_xticks(xm)
    ax.set_xticklabels(BODE_MODES, fontsize=9.5, fontweight="bold")
    ax.set_ylabel("Total Score [pts] (Lower = Better)", fontweight="bold")
    ax.set_title("C. 3-Axis Passive Bode Robustness Under Tier-1 Noise", fontweight="bold")
    ax.grid(True, axis="y", linestyle=":", alpha=0.55)
    ax.set_ylim(0, y_top)
    ax.legend(fontsize=8.5, loc="upper right")

    # Panel D: 5-DOF C1 Protraction (+2.0 cm) OFF vs ON
    ax = axes[1, 1]
    s_off = next(r for r in shift_res if r["case_name"] == "Case1_AP_Protraction" and not r["sensor_noise"])
    s_on = next(r for r in shift_res if r["case_name"] == "Case1_AP_Protraction" and r["sensor_noise"])
    ax.plot(s_off["traces"]["t"], s_off["traces"]["c1_dx_rel_cm"], "#64748b", ls="--", lw=1.8, label=f"C1 dx OFF ({s_off['summary']['ss_c1_dx_rel_cm']:+.2f} cm)")
    ax.plot(s_on["traces"]["t"], s_on["traces"]["c1_dx_rel_cm"], "#059669", lw=2.0, label=f"C1 dx ON  ({s_on['summary']['ss_c1_dx_rel_cm']:+.2f} cm)")
    ax_r = ax.twinx()
    ax_r.plot(s_off["traces"]["t"], s_off["traces"]["pitch_deg"], "#94a3b8", ls=":", lw=1.4, label=f"Pitch OFF ({s_off['summary']['ss_pitch_deg']:+.2f}°)")
    ax_r.plot(s_on["traces"]["t"], s_on["traces"]["pitch_deg"], "#dc2626", alpha=0.8, lw=1.3, label=f"Pitch ON  ({s_on['summary']['ss_pitch_deg']:+.2f}°)")
    ax_r.set_ylim(-3.0, 3.0)
    ax.set_xlabel("Time [s]", fontweight="bold")
    ax.set_ylabel("C1 AP Shift [cm]", color="#059669", fontweight="bold")
    ax_r.set_ylabel("Gaze Pitch Error [deg]", color="#dc2626", fontweight="bold")
    ax.set_title("D. 5-DOF C1 Protraction (+2.0 cm) & Gaze Lock (seed=42)", fontweight="bold")
    ax.grid(True, linestyle=":", alpha=0.55)
    l1, lb1 = ax.get_legend_handles_labels()
    l2, lb2 = ax_r.get_legend_handles_labels()
    ax.legend(l1 + l2, lb1 + lb2, fontsize=7.8, loc="lower right")

    fig.suptitle(
        "Tier-1 Physiological Sensory + Motor Noise Validation (C^2 200 Hz Cubic B-Spline OU + SDN Bank | enable_sensor_noise & enable_motor_noise)",
        fontsize=11.5,
        fontweight="bold",
        y=0.985,
    )
    plt.tight_layout(rect=[0, 0, 1, 0.955])
    os.makedirs(os.path.dirname(OUT_PNG), exist_ok=True)
    fig.savefig(OUT_PNG, dpi=180)
    plt.close(fig)
    if os.path.isdir(ART_DIR):
        shutil.copy2(OUT_PNG, os.path.join(ART_DIR, os.path.basename(OUT_PNG)))


def main():
    t0 = time.time()
    # Build task batches
    quiet_specs = [{"label": "Off (0,0)", "sensor_noise": False, "motor_noise": False, "seed": 42}]
    for lbl, sn, mn in [("Sensor (1,0)", True, False), ("Motor (0,1)", False, True), ("Both (1,1)", True, True)]:
        for sd in SEEDS:
            quiet_specs.append({"label": lbl, "sensor_noise": sn, "motor_noise": mn, "seed": sd})

    bode_specs = [{"tag": "OFF", "mode": m, "sensor_noise": False, "motor_noise": False, "seed": 42} for m in BODE_MODES]
    for sd in SEEDS:
        for m in BODE_MODES:
            bode_specs.append({"tag": "ON", "mode": m, "sensor_noise": True, "motor_noise": True, "seed": sd})

    shift_specs = []
    for cname in ("Case1_AP_Protraction", "Case4A_Combined_wPos1"):
        shift_specs.append({"tag": "OFF", "case_name": cname, "sensor_noise": False, "motor_noise": False, "seed": 42})
        shift_specs.append({"tag": "ON", "case_name": cname, "sensor_noise": True, "motor_noise": True, "seed": 42})

    print(f"Running {len(quiet_specs)} quiet + {len(bode_specs)} Bode + {len(shift_specs)} shift runs across 6 cores...", flush=True)
    with ProcessPoolExecutor(max_workers=6) as pool:
        quiet_res = list(pool.map(run_quiet_stance_worker, quiet_specs))
        bode_res = list(pool.map(run_bode_worker, bode_specs))
        shift_res = list(pool.map(run_shift_worker, shift_specs))

    # Zero-regression check on Noise OFF Bode
    comb_off = round(sum(r["total_score"] for r in bode_res if r["tag"] == "OFF"), 2)
    print(f"\n[CHECK] Noise-OFF Combined Total = {comb_off:.2f} pts (prior 5498.73, delta={comb_off - 5498.73:+.2f}, gate <= 5500.0)")
    assert comb_off <= 5500.0, f"Noise-OFF regression detected: {comb_off}"

    print("\n=== QUIET STANCE 2x2 NOISE ABLATION (t=1..5 s) ===")
    for lbl in ["Off (0,0)", "Sensor (1,0)", "Motor (0,1)", "Both (1,1)"]:
        sub = [r for r in quiet_res if r["label"] == lbl]
        p_m, p_s = np.mean([r["pitch_rms_deg"] for r in sub]), np.std([r["pitch_rms_deg"] for r in sub], ddof=1) if len(sub) > 1 else 0.0
        r_m, r_s = np.mean([r["roll_rms_deg"] for r in sub]), np.std([r["roll_rms_deg"] for r in sub], ddof=1) if len(sub) > 1 else 0.0
        y_m, y_s = np.mean([r["yaw_rms_deg"] for r in sub]), np.std([r["yaw_rms_deg"] for r in sub], ddof=1) if len(sub) > 1 else 0.0
        mx = np.max([r["max_exc_deg"] for r in sub])
        ma = np.max([r["max_act"] for r in sub])
        print(f"  {lbl:12s} | Pitch={p_m:.4f}±{p_s:.4f}° | Roll={r_m:.4f}±{r_s:.4f}° | Yaw={y_m:.4f}±{y_s:.4f}° | MaxExc={mx:.3f}° | MaxAct={ma:.4f}")

    ens_frf = {}
    print("\n=== 3-AXIS PASSIVE BODE: NOISE OFF vs SENSOR+MOTOR ON (Seeds 1-3 + Ensemble FRF) ===")
    for m in BODE_MODES:
        off_r = next(r for r in bode_res if r["tag"] == "OFF" and r["mode"] == m)
        on_rs = [r for r in bode_res if r["tag"] == "ON" and r["mode"] == m]
        c_m, c_s = np.mean([r["classic_bode_score"] for r in on_rs]), np.std([r["classic_bode_score"] for r in on_rs], ddof=1)
        t_m, t_s = np.mean([r["total_score"] for r in on_rs]), np.std([r["total_score"] for r in on_rs], ddof=1)
        rv_m = np.mean([r["corr_vel"] for r in on_rs])
        ens_classic = 0.0
        for fk, fv in off_r["freq_frf"].items():
            tgt_g, tgt_p = fv["target_gain"], fv["target_phase"]
            H_mean = np.mean([r["freq_frf"][fk]["sim_gain"] * cmath.exp(1j * math.radians(r["freq_frf"][fk]["sim_phase"])) for r in on_rs])
            ens_g = abs(H_mean)
            ens_p = math.degrees(cmath.phase(H_mean))
            db_err = 20.0 * math.log10(max(ens_g, 1e-6) / tgt_g)
            ph_err = ((ens_p - tgt_p + 180.0) % 360.0) - 180.0
            ens_classic += 100.0 * (db_err ** 2) + (ph_err ** 2)
        ens_delta = 100.0 * (ens_classic - off_r["classic_bode_score"]) / off_r["classic_bode_score"]
        ens_frf[m] = {
            "off_classic": off_r["classic_bode_score"],
            "ens_classic": round(float(ens_classic), 2),
            "delta_pct": round(float(ens_delta), 2),
        }
        print(f"  {m:5s} | OFF Total={off_r['total_score']:7.2f} (Classic={off_r['classic_bode_score']:7.2f}, r_v={off_r['corr_vel']:+.3f}) | "
              f"ON Total={t_m:7.2f}±{t_s:4.2f} (Classic={c_m:7.2f}±{c_s:4.2f}, r_v={rv_m:+.3f}) | delta={100*(t_m-off_r['total_score'])/off_r['total_score']:+.2f}% | "
              f"EnsFRF Classic={ens_classic:7.2f} ({ens_delta:+.2f}%)")

    comb_on_seeds = [sum(r["total_score"] for r in bode_res if r["tag"] == "ON" and r["seed"] == sd) for sd in SEEDS]
    comb_ens_classic = round(sum(v["ens_classic"] for v in ens_frf.values()), 2)
    comb_off_classic = round(sum(v["off_classic"] for v in ens_frf.values()), 2)
    print(f"  TOTAL | OFF={comb_off:.2f} (Classic={comb_off_classic:.2f}) | ON={np.mean(comb_on_seeds):.2f}±{np.std(comb_on_seeds, ddof=1):.2f} | EnsFRF Classic={comb_ens_classic:.2f} ({100*(comb_ens_classic-comb_off_classic)/comb_off_classic:+.2f}%)")

    print("\n=== 5-DOF VOLUNTARY C1 SHIFT: NOISE OFF vs SENSOR+MOTOR ON (seed=42) ===")
    for r in shift_res:
        sm = r["summary"]
        print(f"  {r['case_name']:22s} [{r['tag']:3s}] | C1_dx={sm['ss_c1_dx_rel_cm']:+6.2f}cm | att=[p={sm['ss_pitch_deg']:+5.2f}°, r={sm['ss_roll_deg']:+5.2f}°, y={sm['ss_yaw_deg']:+5.2f}°] | Q={sm['quiet_max_att_deg']:.3f}°")

    make_noise_plot(quiet_res, bode_res, shift_res, ens_frf)

    os.makedirs(os.path.dirname(OUT_JSON), exist_ok=True)
    payload = {
        "meta": {"seeds": SEEDS, "wall_s": round(time.time() - t0, 2)},
        "quiet_stance": [{k: v for k, v in r.items() if k not in ("t", "pitch_deg", "roll_deg", "yaw_deg")} for r in quiet_res],
        "bode": bode_res,
        "ensemble_frf_classic": ens_frf,
        "voluntary_shift": [r["summary"] | {"tag": r["tag"]} for r in shift_res],
    }
    with open(OUT_JSON, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)
    print(f"\n[OK] Saved {OUT_JSON} and {OUT_PNG} in {time.time() - t0:.1f}s")


if __name__ == "__main__":
    main()
