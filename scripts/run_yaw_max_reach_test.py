import json
import math
import os
import shutil
import sys
import time
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import opensim as osim

REPO_ROOT = r"D:\Akademik\PhD_Thesis\OpenSim\HeadNeck_Control_w_Sensors_Opensim4_6"
ART_DIR = r"C:\Users\User\.gemini\antigravity\brain\89fb8d56-9d12-4051-a256-02497ef8d6ae"
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

from validate_5dof_voluntary_and_se2_shift import (
    CHAMP_C1_JSON,
    MODEL_PATH,
    ensure_libraries_loaded,
)

OUT_PNG = os.path.join(REPO_ROOT, "results", "yaw_max_reach_test.png")
OUT_JSON = os.path.join(REPO_ROOT, "logs", "yaw_max_reach_test.json")

PHASES = [
    {"id": 0, "name": "0_Neutral",  "t_start":  0.0, "t_end":  0.5, "yaw_deg":  0.0, "urgency": 0.00},
    {"id": 1, "name": "1_Yaw_30",   "t_start":  0.5, "t_end":  2.5, "yaw_deg": 30.0, "urgency": 0.35},
    {"id": 2, "name": "2_Yaw_40",   "t_start":  2.5, "t_end":  4.5, "yaw_deg": 40.0, "urgency": 0.35},
    {"id": 3, "name": "3_Yaw_45",   "t_start":  4.5, "t_end":  6.5, "yaw_deg": 45.0, "urgency": 0.35},
    {"id": 4, "name": "4_Yaw_50",   "t_start":  6.5, "t_end":  8.5, "yaw_deg": 50.0, "urgency": 0.35},
    {"id": 5, "name": "5_Yaw_55",   "t_start":  8.5, "t_end": 10.5, "yaw_deg": 55.0, "urgency": 0.35},
    {"id": 6, "name": "6_Yaw_60",   "t_start": 10.5, "t_end": 12.5, "yaw_deg": 60.0, "urgency": 0.35},
    {"id": 7, "name": "7_Return_0", "t_start": 12.5, "t_end": 14.5, "yaw_deg":  0.0, "urgency": 0.35},
]


def set_prop(controller, name, val):
    prop = controller.updPropertyByName(name)
    osim.PropertyHelper.setValueDouble(float(val), prop)


def main():
    t0 = time.time()
    ensure_libraries_loaded()

    champ_obj = json.load(open(CHAMP_C1_JSON, "r", encoding="utf-8"))
    base_params = dict(champ_obj["params"])
    base_params.update({
        "w_pos_ap": 0.0,
        "w_pos_lat": 0.0,
        "Kp_trans_roll": 0.0,
        "Kp_trans_pitch": 0.0,
        "Kp_shift_obs_px": 0.0,
        "Kv_shift_obs_vx": 0.0,
        "Kp_shift_obs_pz": 0.0,
        "Kv_shift_obs_vz": 0.0,
        "shift_station_blend": 0.0,
        "trajectory_tau": 0.28,
        "urgency": 0.0,
        "desired_pitch": 0.0,
        "desired_roll": 0.0,
        "desired_yaw": 0.0,
        "desired_ap_shift": 0.0,
        "desired_lat_shift": 0.0,
    })

    model = osim.Model(MODEL_PATH)
    controller = model.updControllerSet().get("VCR_CCR_Controller")
    for k, v in base_params.items():
        if isinstance(v, bool):
            continue
        try:
            set_prop(controller, k, v)
        except Exception:
            pass

    c_yaw1 = model.updCoordinateSet().get("yaw1")
    c_yaw1.set_locked(False)
    c_yaw1.setDefaultIsPrescribed(False)

    state = model.initSystem()
    c_yaw1.setIsPrescribed(state, False)
    model.realizePosition(state)

    skull = model.getBodySet().get("skull")
    R_head_to_world = skull.getTransformInGround(state).R()
    q0 = osim.Rotation(R_head_to_world).convertRotationToQuaternion()
    for idx in range(4):
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/obs_q{idx}", q0.get(idx))

    for sv in (
        "pitch_error_integral",
        "roll_error_integral",
        "yaw_error_integral",
        "vol_filt_pitch",
        "vol_filt_roll",
        "vol_filt_yaw",
    ):
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/{sv}", 0.0)

    k_gdyn = float(base_params.get("K_gamma_dyn", 0.857744))
    k_gstat = float(base_params.get("K_gamma_stat", 0.69312344))
    muscles = model.updMuscles()
    n_mus = muscles.getSize()
    for i in range(n_mus):
        m = muscles.get(i)
        mname = m.getName()
        if mname.startswith("r_") or mname.startswith("l_"):
            continue
        u0 = 0.03
        gd0 = 100.0 * k_gdyn * (0.25 + 0.75 * u0)
        gs0 = 100.0 * k_gstat * u0
        fdyn0 = (gd0 * gd0) / (gd0 * gd0 + 3600.0)
        fstat0 = (gs0 * gs0) / (gs0 * gs0 + 3600.0)
        m.setStateVariableValue(state, "spindle/dynamic_activation", fdyn0)
        m.setStateVariableValue(state, "spindle/static_activation", fstat0)
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/sp_fdyn_{mname}", fdyn0)
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/sp_fstat_{mname}", fstat0)

    model.equilibrateMuscles(state)

    manager = osim.Manager(model)
    manager.setWriteToStorage(False)
    manager.setPerformAnalyses(False)
    manager.setIntegratorMethod(5)
    manager.setIntegratorAccuracy(1e-4)
    manager.setIntegratorMinimumStepSize(1e-4)
    manager.setIntegratorMaximumStepSize(0.001)
    state.setTime(0.0)
    manager.initialize(state)

    cs = model.getCoordinateSet()
    c_p1, c_p2 = cs.get("pitch1"), cs.get("pitch2")
    c_r1, c_r2 = cs.get("roll1"), cs.get("roll2")
    c_y1, c_y2 = cs.get("yaw1"), cs.get("yaw2")

    dt = 0.01
    duration = 14.5
    t_now = 0.0
    curr_phase_id = -1

    tr = {
        "t": [], "phase": [], "target_yaw": [],
        "pitch": [], "roll": [], "yaw": [], "ref_yaw": [],
        "y1": [], "y2": [], "max_act": [],
    }
    stop_reason = None

    while t_now < duration - 1e-9:
        for ph in PHASES:
            if ph["t_start"] - 1e-9 <= t_now < ph["t_end"] - 1e-9:
                if ph["id"] != curr_phase_id:
                    curr_phase_id = ph["id"]
                    set_prop(controller, "urgency", ph["urgency"])
                    set_prop(controller, "desired_pitch", 0.0)
                    set_prop(controller, "desired_roll", 0.0)
                    set_prop(controller, "desired_yaw", math.radians(ph["yaw_deg"]))
                    print(
                        f"[PHASE {ph['id']}] t={t_now:5.2f}s | {ph['name']:10s} | target_yaw={ph['yaw_deg']:+5.1f} deg | u={ph['urgency']:.2f}",
                        flush=True,
                    )
                break

        t_next = min(t_now + dt, duration)
        try:
            state = manager.integrate(t_next)
            model.realizeVelocity(state)
        except Exception as e:
            stop_reason = f"Stopped at t={t_next:.3f}s (phase {curr_phase_id}): {str(e).splitlines()[0]}"
            print(f"[WARN] {stop_reason}", flush=True)
            break
        t_now = t_next

        p1 = math.degrees(c_p1.getValue(state)); p2 = math.degrees(c_p2.getValue(state))
        r1 = math.degrees(c_r1.getValue(state)); r2 = math.degrees(c_r2.getValue(state))
        y1 = math.degrees(c_y1.getValue(state)); y2 = math.degrees(c_y2.getValue(state))
        ry = math.degrees(model.getStateVariableValue(state, "/controllerset/VCR_CCR_Controller/traj_pos_yaw"))

        m_act = 0.0
        for m_i in range(n_mus):
            m = muscles.get(m_i)
            mname = m.getName()
            if mname.startswith("r_") or mname.startswith("l_"):
                continue
            a = float(m.getActivation(state))
            if a > m_act:
                m_act = a

        ph_obj = PHASES[curr_phase_id]
        tr["t"].append(t_now)
        tr["phase"].append(curr_phase_id)
        tr["target_yaw"].append(ph_obj["yaw_deg"])
        tr["pitch"].append(p1 + p2)
        tr["roll"].append(r1 + r2)
        tr["yaw"].append(y1 + y2)
        tr["ref_yaw"].append(ry)
        tr["y1"].append(y1)
        tr["y2"].append(y2)
        tr["max_act"].append(m_act)

    t_np = np.array(tr["t"])
    phase_summaries = []
    for ph in PHASES:
        mask = (t_np >= ph["t_start"]) & (t_np <= ph["t_end"])
        if not np.any(mask):
            continue
        t_ph = t_np[mask]
        end_mask = (t_ph >= max(ph["t_start"], t_ph[-1] - 0.35)) & (t_ph <= t_ph[-1])
        idx_end = np.where(mask)[0][-len(t_ph[end_mask]):]

        end_yaw = float(np.mean(np.array(tr["yaw"])[idx_end]))
        end_y1 = float(np.mean(np.array(tr["y1"])[idx_end]))
        end_y2 = float(np.mean(np.array(tr["y2"])[idx_end]))
        end_pitch = float(np.mean(np.array(tr["pitch"])[idx_end]))
        end_roll = float(np.mean(np.array(tr["roll"])[idx_end]))
        end_act = float(np.mean(np.array(tr["max_act"])[idx_end]))
        peak_act = float(np.max(np.array(tr["max_act"])[mask]))
        max_off_axis = float(max(
            np.max(np.abs(np.array(tr["pitch"])[mask])),
            np.max(np.abs(np.array(tr["roll"])[mask])),
        ))
        y1_share = (100.0 * end_y1 / (end_y1 + end_y2)) if abs(end_y1 + end_y2) > 1.0 else None

        sm = {
            "id": ph["id"],
            "name": ph["name"],
            "window_s": [ph["t_start"], round(float(t_ph[-1]), 2)],
            "urgency": ph["urgency"],
            "target_yaw_deg": ph["yaw_deg"],
            "end_yaw_deg": round(end_yaw, 2),
            "end_y1_deg": round(end_y1, 2),
            "end_y2_deg": round(end_y2, 2),
            "y1_share_pct": round(y1_share, 1) if y1_share is not None else None,
            "end_pitch_deg": round(end_pitch, 2),
            "end_roll_deg": round(end_roll, 2),
            "max_off_axis_deg": round(max_off_axis, 2),
            "peak_muscle_act": round(peak_act, 3),
            "end_muscle_act": round(end_act, 3),
        }
        phase_summaries.append(sm)
        share_str = f"{y1_share:.1f}%" if y1_share is not None else "n/a"
        print(
            f"  -> {ph['name']:10s} | tgt={ph['yaw_deg']:+5.1f} -> yaw={end_yaw:+5.2f}° "
            f"(y1={end_y1:+5.2f}°, y2={end_y2:+5.2f}°, y1%={share_str}) | "
            f"xtalk=(p:{end_pitch:+5.2f}°, r:{end_roll:+5.2f}°, max:{max_off_axis:4.2f}°) | "
            f"act(pk={peak_act:.3f}, end={end_act:.3f})",
            flush=True,
        )

    os.makedirs(os.path.dirname(OUT_JSON), exist_ok=True)
    with open(OUT_JSON, "w", encoding="utf-8") as f:
        json.dump({
            "elapsed_s": round(time.time() - t0, 1),
            "t_end_s": round(float(t_now), 3),
            "stop_reason": stop_reason,
            "phases": phase_summaries,
        }, f, indent=2)

    fig, axes = plt.subplots(2, 1, figsize=(15.5, 9.2), dpi=160, sharex=True)
    fig.suptitle(
        "14.5 s Quasi-Static Yaw Staircase Reach Test (+30, +40, +45, +50, +55, +60, 0 deg | u = 0.35)\n"
        "Exact KKT F_eff QP | Widened LimitForces (+/-68 deg) | Unchanged yaw2 +/-32.4 deg <range> & Bushings | kRomDiffRatio[2]=+0.08",
        fontsize=12.0,
        fontweight="bold",
        y=0.97,
    )

    ax = axes[0]
    ax.step(t_np, tr["target_yaw"], where="post", color="#4c1d95", ls="--", lw=1.6, label="Commanded Target Yaw")
    ax.plot(t_np, tr["ref_yaw"], "#6d28d9", ls=":", lw=1.6, label="3rd-Order Ref Yaw")
    ax.plot(t_np, tr["yaw"], "#7c3aed", lw=2.4, label="Realized Total Yaw (y1+y2)")
    ax.plot(t_np, tr["pitch"], "#059669", lw=1.8, label="Off-Axis Pitch (p1+p2)")
    ax.plot(t_np, tr["roll"], "#2563eb", lw=1.8, label="Off-Axis Roll (r1+r2)")
    for ph in PHASES[1:]:
        ax.axvline(ph["t_start"], color="#94a3b8", ls="--", lw=0.9, alpha=0.7)
        ax.text(ph["t_start"] + 0.08, 62.5, f"{ph['yaw_deg']:+.0f}°", fontsize=8.0, fontweight="bold", color="#334155")
    ax.set_ylim(-20, 68)
    ax.set_ylabel("Head Attitude [deg]", fontweight="bold")
    ax.set_title("Panel A: Staircase Yaw Reach vs. Commanded Target & Off-Axis Pitch/Roll Crosstalk", fontweight="bold", fontsize=10.8)
    ax.grid(True, alpha=0.3); ax.legend(fontsize=8.5, ncol=3, loc="upper left")

    ax = axes[1]
    ax_r = ax.twinx()
    ax.plot(t_np, tr["y1"], "#7c3aed", lw=2.2, label="Lower Cervical Yaw y1 [deg]")
    ax.plot(t_np, tr["y2"], "#c084fc", ls="--", lw=2.2, label="Upper Cervical Yaw y2 [deg]")
    ax_r.plot(t_np, tr["max_act"], "#dc2626", lw=1.8, alpha=0.85, label="Peak Muscle Activation [0-1]")
    ax_r.set_ylim(0.0, 1.12)
    for ph in PHASES[1:]:
        ax.axvline(ph["t_start"], color="#94a3b8", ls="--", lw=0.9, alpha=0.7)
    ax.set_xlabel("Time [s]", fontweight="bold")
    ax.set_ylabel("Segment Angles y1, y2 [deg]", fontweight="bold")
    ax_r.set_ylabel("Peak Muscle Activation", color="#dc2626", fontweight="bold")
    ax.set_title("Panel B: Lower (y1) vs. Upper (y2) Yaw Partitioning & Muscle Saturation Onset", fontweight="bold", fontsize=10.8)
    l1, lb1 = ax.get_legend_handles_labels(); l2, lb2 = ax_r.get_legend_handles_labels()
    ax.legend(l1 + l2, lb1 + lb2, fontsize=8.5, ncol=3, loc="upper left")
    ax.grid(True, alpha=0.3)

    fig.tight_layout(rect=[0, 0, 1, 0.94])
    os.makedirs(os.path.dirname(OUT_PNG), exist_ok=True)
    fig.savefig(OUT_PNG, bbox_inches="tight")
    shutil.copyfile(OUT_PNG, os.path.join(ART_DIR, "yaw_max_reach_test.png"))
    plt.close(fig)
    print(f"[OK] Saved {OUT_PNG} and {OUT_JSON} in {time.time()-t0:.1f}s", flush=True)


if __name__ == "__main__":
    main()
