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

OUT_PNG = os.path.join(REPO_ROOT, "results", "max_velocity_overshoot_u1_test.png")
OUT_JSON = os.path.join(REPO_ROOT, "logs", "max_velocity_overshoot_u1_test.json")

PHASES = [
    {"id": 0, "name": "0_Quiet_Neutral",   "t_start": 0.0,  "t_end": 1.0,  "pitch_deg":   0.0, "roll_deg":   0.0, "yaw_deg":   0.0, "urgency": 0.00},
    {"id": 1, "name": "1_Pitch_Ext_u1",    "t_start": 1.0,  "t_end": 3.5,  "pitch_deg": -35.0, "roll_deg":   0.0, "yaw_deg":   0.0, "urgency": 1.00},
    {"id": 2, "name": "2_Pitch_Flex_u1",   "t_start": 3.5,  "t_end": 6.0,  "pitch_deg": +50.0, "roll_deg":   0.0, "yaw_deg":   0.0, "urgency": 1.00},
    {"id": 3, "name": "3_Pitch_Neutral",   "t_start": 6.0,  "t_end": 8.0,  "pitch_deg":   0.0, "roll_deg":   0.0, "yaw_deg":   0.0, "urgency": 0.50},
    {"id": 4, "name": "4_LeftRoll_u1",     "t_start": 8.0,  "t_end": 10.0, "pitch_deg":   0.0, "roll_deg": +28.0, "yaw_deg":   0.0, "urgency": 1.00},
    {"id": 5, "name": "5_RightRoll_u1",    "t_start": 10.0, "t_end": 12.0, "pitch_deg":   0.0, "roll_deg": -28.0, "yaw_deg":   0.0, "urgency": 1.00},
    {"id": 6, "name": "6_Roll_Neutral",    "t_start": 12.0, "t_end": 13.5, "pitch_deg":   0.0, "roll_deg":   0.0, "yaw_deg":   0.0, "urgency": 0.50},
    {"id": 7, "name": "7_LeftYaw_u1",      "t_start": 13.5, "t_end": 15.5, "pitch_deg":   0.0, "roll_deg":   0.0, "yaw_deg": +45.0, "urgency": 1.00},
    {"id": 8, "name": "8_RightYaw_u1",     "t_start": 15.5, "t_end": 18.0, "pitch_deg":   0.0, "roll_deg":   0.0, "yaw_deg": -45.0, "urgency": 1.00},
]


def set_prop(controller, name, val):
    prop = controller.updPropertyByName(name)
    osim.PropertyHelper.setValueDouble(float(val), prop)


def compute_overshoot(vals, target_deg, start_deg):
    arr = np.asarray(vals, dtype=float)
    if abs(target_deg - start_deg) < 1e-6:
        return round(float(np.max(np.abs(arr - target_deg))), 2)
    direction = 1.0 if target_deg > start_deg else -1.0
    overshoot = direction * (arr - target_deg)
    return round(float(max(0.0, np.max(overshoot))), 2)


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
    duration = 18.0
    t_now = 0.0
    curr_phase_id = -1

    tr = {
        "t": [], "phase": [], "urgency": [],
        "pitch": [], "roll": [], "yaw": [],
        "ref_pitch": [], "ref_roll": [], "ref_yaw": [],
        "vel_pitch": [], "vel_roll": [], "vel_yaw": [],
        "p1": [], "p2": [], "r1": [], "r2": [], "y1": [], "y2": [],
        "max_act": [],
    }
    stop_reason = None

    while t_now < duration - 1e-9:
        for ph in PHASES:
            if ph["t_start"] - 1e-9 <= t_now < ph["t_end"] - 1e-9:
                if ph["id"] != curr_phase_id:
                    curr_phase_id = ph["id"]
                    set_prop(controller, "urgency", ph["urgency"])
                    set_prop(controller, "desired_pitch", math.radians(ph["pitch_deg"]))
                    set_prop(controller, "desired_roll", math.radians(ph["roll_deg"]))
                    set_prop(controller, "desired_yaw", math.radians(ph["yaw_deg"]))
                    print(
                        f"[PHASE {ph['id']}] t={t_now:5.2f}s | {ph['name']:16s} | "
                        f"target=({ph['pitch_deg']:+6.1f}, {ph['roll_deg']:+6.1f}, {ph['yaw_deg']:+6.1f}) deg | u={ph['urgency']:.2f}",
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
        vp = math.degrees(c_p1.getSpeedValue(state) + c_p2.getSpeedValue(state))
        vr = math.degrees(c_r1.getSpeedValue(state) + c_r2.getSpeedValue(state))
        vy = math.degrees(c_y1.getSpeedValue(state) + c_y2.getSpeedValue(state))

        rp = math.degrees(model.getStateVariableValue(state, "/controllerset/VCR_CCR_Controller/traj_pos_pitch"))
        rr = math.degrees(model.getStateVariableValue(state, "/controllerset/VCR_CCR_Controller/traj_pos_roll"))
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
        tr["urgency"].append(ph_obj["urgency"])
        tr["pitch"].append(p1 + p2); tr["roll"].append(r1 + r2); tr["yaw"].append(y1 + y2)
        tr["ref_pitch"].append(rp); tr["ref_roll"].append(rr); tr["ref_yaw"].append(ry)
        tr["vel_pitch"].append(vp); tr["vel_roll"].append(vr); tr["vel_yaw"].append(vy)
        tr["p1"].append(p1); tr["p2"].append(p2)
        tr["r1"].append(r1); tr["r2"].append(r2)
        tr["y1"].append(y1); tr["y2"].append(y2)
        tr["max_act"].append(m_act)

    t_np = np.array(tr["t"])
    phase_summaries = []
    prev_end = {"pitch": 0.0, "roll": 0.0, "yaw": 0.0}
    for ph in PHASES:
        mask = (t_np >= ph["t_start"]) & (t_np <= ph["t_end"])
        if not np.any(mask):
            continue
        t_ph = t_np[mask]
        end_mask = (t_ph >= max(ph["t_start"], t_ph[-1] - 0.25)) & (t_ph <= t_ph[-1])
        idx_end = np.where(mask)[0][-len(t_ph[end_mask]):]

        p_ph = np.array(tr["pitch"])[mask]
        r_ph = np.array(tr["roll"])[mask]
        y_ph = np.array(tr["yaw"])[mask]
        end_p = float(np.mean(np.array(tr["pitch"])[idx_end]))
        end_r = float(np.mean(np.array(tr["roll"])[idx_end]))
        end_y = float(np.mean(np.array(tr["yaw"])[idx_end]))

        ovs_p = compute_overshoot(p_ph, ph["pitch_deg"], prev_end["pitch"])
        ovs_r = compute_overshoot(r_ph, ph["roll_deg"], prev_end["roll"])
        ovs_y = compute_overshoot(y_ph, ph["yaw_deg"], prev_end["yaw"])

        # Off-axis crosstalk relative to commanded axes (use delta from prev_end so neutral returns measure off-axis properly)
        if abs(ph["pitch_deg"] - prev_end["pitch"]) > 1.0:
            max_xtalk = round(float(max(np.max(np.abs(r_ph)), np.max(np.abs(y_ph)))), 2)
        elif abs(ph["roll_deg"] - prev_end["roll"]) > 1.0:
            max_xtalk = round(float(max(np.max(np.abs(p_ph)), np.max(np.abs(y_ph)))), 2)
        elif abs(ph["yaw_deg"] - prev_end["yaw"]) > 1.0:
            max_xtalk = round(float(max(np.max(np.abs(p_ph)), np.max(np.abs(r_ph)))), 2)
        else:
            max_xtalk = round(float(max(np.max(np.abs(p_ph)), np.max(np.abs(r_ph)), np.max(np.abs(y_ph)))), 2)

        sm = {
            "id": ph["id"],
            "name": ph["name"],
            "window_s": [ph["t_start"], round(float(t_ph[-1]), 2)],
            "urgency": ph["urgency"],
            "target_deg": [ph["pitch_deg"], ph["roll_deg"], ph["yaw_deg"]],
            "end_deg": [round(end_p, 2), round(end_r, 2), round(end_y, 2)],
            "overshoot_deg": [ovs_p, ovs_r, ovs_y],
            "peak_vel_deg_s": [
                round(float(np.max(np.abs(np.array(tr["vel_pitch"])[mask]))), 1),
                round(float(np.max(np.abs(np.array(tr["vel_roll"])[mask]))), 1),
                round(float(np.max(np.abs(np.array(tr["vel_yaw"])[mask]))), 1),
            ],
            "max_off_axis_crosstalk_deg": max_xtalk,
            "peak_muscle_act": round(float(np.max(np.array(tr["max_act"])[mask])), 3),
        }
        phase_summaries.append(sm)
        prev_end = {"pitch": end_p, "roll": end_r, "yaw": end_y}
        print(
            f"  -> {ph['name']:16s} | end={sm['end_deg']} | ovs={sm['overshoot_deg']} deg | "
            f"Vmax={sm['peak_vel_deg_s']} deg/s | xtalk={max_xtalk:.2f} deg | act_max={sm['peak_muscle_act']:.3f}",
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

    fig, axes = plt.subplots(3, 1, figsize=(16.5, 12.2), dpi=160, sharex=True)
    fig.suptitle(
        "18.0 s Max-Velocity (u = 1.0, fc_vol = 5.0 Hz) & Overshoot Test Inside Physical Limit Envelope\n"
        "3rd-Order Trajectory | Exact Jacobian Above 8 deg | ROM-Proportional tau_buckle | w_pos = 0",
        fontsize=12.6,
        fontweight="bold",
        y=0.98,
    )

    # Panel A: 3-Axis Head Attitude vs 3rd-Order Reference
    ax = axes[0]
    ax.plot(t_np, tr["pitch"], "#059669", lw=2.2, label="True Head Pitch (p1+p2)")
    ax.plot(t_np, tr["ref_pitch"], "#065f46", ls=":", lw=1.5, label="3rd-Order Pitch Ref")
    ax.plot(t_np, tr["roll"], "#2563eb", lw=2.2, label="True Head Roll (r1+r2)")
    ax.plot(t_np, tr["ref_roll"], "#1e3a8a", ls=":", lw=1.5, label="3rd-Order Roll Ref")
    ax.plot(t_np, tr["yaw"], "#7c3aed", lw=2.2, label="True Head Yaw (y1+y2)")
    ax.plot(t_np, tr["ref_yaw"], "#4c1d95", ls=":", lw=1.5, label="3rd-Order Yaw Ref")
    for ph in PHASES[1:]:
        ax.axvline(ph["t_start"], color="#94a3b8", ls="--", lw=0.9, alpha=0.7)
        ax.text(ph["t_start"] + 0.08, 49.0, f"P{ph['id']}\nu={ph['urgency']:.2f}", fontsize=7.5, fontweight="bold", color="#334155")
    ax.set_ylim(-55, 62)
    ax.set_ylabel("Head Attitude [deg]", fontweight="bold")
    ax.set_title("Panel A: 3-Axis Ballistic Head Attitude Tracking vs. 3rd-Order Reference Trajectory", fontweight="bold", fontsize=10.8)
    ax.grid(True, alpha=0.3); ax.legend(fontsize=8.2, ncol=3, loc="lower left")

    # Panel B: Angular Velocity Profiles & Peak Muscle Activation
    ax = axes[1]
    ax_r = ax.twinx()
    ax.plot(t_np, tr["vel_pitch"], "#059669", lw=2.0, label="Pitch Velocity [deg/s]")
    ax.plot(t_np, tr["vel_roll"], "#2563eb", lw=2.0, label="Roll Velocity [deg/s]")
    ax.plot(t_np, tr["vel_yaw"], "#7c3aed", lw=2.0, label="Yaw Velocity [deg/s]")
    ax_r.plot(t_np, tr["max_act"], "#dc2626", ls="--", lw=1.4, alpha=0.85, label="Max Muscle Activation [0-1]")
    ax_r.set_ylim(0.0, 1.15)
    for ph in PHASES[1:]:
        ax.axvline(ph["t_start"], color="#94a3b8", ls="--", lw=0.9, alpha=0.7)
    ax.set_ylabel("Angular Velocity [deg/s]", fontweight="bold")
    ax_r.set_ylabel("Peak Muscle Activation", color="#dc2626", fontweight="bold")
    ax.set_title("Panel B: Max Angular Velocity Bells (u = 1.0) & Peak Muscle Excitation", fontweight="bold", fontsize=10.8)
    l1, lb1 = ax.get_legend_handles_labels(); l2, lb2 = ax_r.get_legend_handles_labels()
    ax.legend(l1 + l2, lb1 + lb2, fontsize=8.2, ncol=4, loc="upper left")
    ax.grid(True, alpha=0.3)

    # Panel C: Upper vs Lower Cervical Joint Split (q1, q2) & Off-Axis Margins
    ax = axes[2]
    ax.plot(t_np, tr["p1"], "#059669", lw=1.8, label="Lower Pitch p1")
    ax.plot(t_np, tr["p2"], "#34d399", ls="--", lw=1.8, label="Upper Pitch p2")
    ax.plot(t_np, tr["r1"], "#2563eb", lw=1.8, label="Lower Roll r1")
    ax.plot(t_np, tr["r2"], "#60a5fa", ls="--", lw=1.8, label="Upper Roll r2")
    ax.plot(t_np, tr["y1"], "#7c3aed", lw=1.8, label="Lower Yaw y1")
    ax.plot(t_np, tr["y2"], "#c084fc", ls="--", lw=1.8, label="Upper Yaw y2")
    for ph in PHASES[1:]:
        ax.axvline(ph["t_start"], color="#94a3b8", ls="--", lw=0.9, alpha=0.7)
    ax.set_xlabel("Time [s]", fontweight="bold")
    ax.set_ylabel("Segment Angles q1, q2 [deg]", fontweight="bold")
    ax.set_title("Panel C: Lower (q1) vs. Upper (q2) Cervical Segment Partitioning & Off-Axis Crosstalk", fontweight="bold", fontsize=10.8)
    ax.grid(True, alpha=0.3); ax.legend(fontsize=8.0, ncol=3, loc="upper left")

    fig.tight_layout(rect=[0, 0, 1, 0.95])
    os.makedirs(os.path.dirname(OUT_PNG), exist_ok=True)
    fig.savefig(OUT_PNG, bbox_inches="tight")
    shutil.copyfile(OUT_PNG, os.path.join(ART_DIR, "max_velocity_overshoot_u1_test.png"))
    plt.close(fig)
    print(f"[OK] Saved {OUT_PNG} and {OUT_JSON} in {time.time()-t0:.1f}s", flush=True)


if __name__ == "__main__":
    main()
