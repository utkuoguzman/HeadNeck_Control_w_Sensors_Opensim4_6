import json
import math
import os
import shutil
import time
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import opensim as osim

REPO_ROOT = r"D:\Akademik\PhD_Thesis\OpenSim\HeadNeck_Control_w_Sensors_Opensim4_6"
ART_DIR = r"C:\Users\User\.gemini\antigravity\brain\89fb8d56-9d12-4051-a256-02497ef8d6ae"
MODEL_PATH = os.path.join(REPO_ROOT, "osim_files", "HYOID_HeadNeckNeural_Model_Detailed.osim")
PLUGIN_REL = os.path.join(REPO_ROOT, "HeadNeckNeuralController_plugin", "build", "Release", "osimHeadNeckNeural.dll")
MILLARD_REL = os.path.join(REPO_ROOT, "proprioception_plugin", "build", "Release", "osimMillard12EqWithAff.dll")
VEST_REL = os.path.join(REPO_ROOT, "vestibular_plugin", "build", "Release", "osimVestibular.dll")
FIX_REL = os.path.join(REPO_ROOT, "FixationController_plugin", "build", "Release", "osimFixationController.dll")
CHAMP_JSON = os.path.join(REPO_ROOT, "logs", "se2_c1pos_3axis_champion.json")
OUT_PNG = os.path.join(REPO_ROOT, "results", "extreme_rom_multi_urgency_test.png")
OUT_JSON = os.path.join(REPO_ROOT, "logs", "extreme_rom_multi_urgency_test.json")

PHASES = [
    {"id": 0, "name": "0_Quiet_Neutral",   "t_start": 0.0,  "t_end": 1.0,  "pitch_deg":   0.0, "roll_deg":   0.0, "yaw_deg":   0.0, "urgency": 0.00},
    {"id": 1, "name": "1_90pct_Extension", "t_start": 1.0,  "t_end": 3.0,  "pitch_deg": -52.9, "roll_deg":   0.0, "yaw_deg":   0.0, "urgency": 0.50},
    {"id": 2, "name": "2_90pct_Flexion",   "t_start": 3.0,  "t_end": 5.0,  "pitch_deg": +77.8, "roll_deg":   0.0, "yaw_deg":   0.0, "urgency": 0.75},
    {"id": 3, "name": "3_Pitch_Neutral",   "t_start": 5.0,  "t_end": 7.0,  "pitch_deg":   0.0, "roll_deg":   0.0, "yaw_deg":   0.0, "urgency": 0.50},
    {"id": 4, "name": "4_90pct_LeftRoll",  "t_start": 7.0,  "t_end": 9.0,  "pitch_deg":   0.0, "roll_deg": +42.1, "yaw_deg":   0.0, "urgency": 0.50},
    {"id": 5, "name": "5_90pct_RightRoll", "t_start": 9.0,  "t_end": 11.0, "pitch_deg":   0.0, "roll_deg": -42.1, "yaw_deg":   0.0, "urgency": 0.75},
    {"id": 6, "name": "6_Roll_Neutral",    "t_start": 11.0, "t_end": 13.0, "pitch_deg":   0.0, "roll_deg":   0.0, "yaw_deg":   0.0, "urgency": 0.50},
    {"id": 7, "name": "7_90pct_LeftYaw",   "t_start": 13.0, "t_end": 15.0, "pitch_deg":   0.0, "roll_deg":   0.0, "yaw_deg": +70.2, "urgency": 0.50},
    {"id": 8, "name": "8_90pct_RightYaw",  "t_start": 15.0, "t_end": 17.0, "pitch_deg":   0.0, "roll_deg":   0.0, "yaw_deg": -70.2, "urgency": 0.75},
    {"id": 9, "name": "9_Final_Neutral",   "t_start": 17.0, "t_end": 20.5, "pitch_deg":   0.0, "roll_deg":   0.0, "yaw_deg":   0.0, "urgency": 0.25},
]


def set_prop(controller, name, val):
    prop = controller.updPropertyByName(name)
    osim.PropertyHelper.setValueDouble(float(val), prop)


def main():
    t0 = time.time()
    for dll in [MILLARD_REL, VEST_REL, FIX_REL, PLUGIN_REL]:
        if os.path.exists(dll):
            osim.LoadOpenSimLibraryExact(dll)

    champ_obj = json.load(open(CHAMP_JSON, "r", encoding="utf-8"))
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
    controller = osim.HeadNeckNeuralController.safeDownCast(model.updComponent("HeadNeckNeuralController"))
    for k, v in base_params.items():
        if isinstance(v, bool):
            continue
        try:
            set_prop(controller, k, v)
        except Exception:
            pass

    state = model.initSystem()
    model.equilibrateMuscles(state)

    cs = model.getCoordinateSet()
    c_p1, c_p2 = cs.get("pitch1"), cs.get("pitch2")
    c_r1, c_r2 = cs.get("roll1"), cs.get("roll2")
    c_y1, c_y2 = cs.get("yaw1"), cs.get("yaw2")
    muscles = model.getMuscles()
    n_mus = muscles.getSize()

    manager = osim.Manager(model)
    manager.setIntegratorAccuracy(1e-4)
    manager.initialize(state)

    dt = 0.01
    duration = 20.5
    t_now = 0.0
    curr_phase_id = -1

    tr = {
        "t": [], "phase": [], "urgency": [],
        "pitch": [], "roll": [], "yaw": [],
        "ref_pitch": [], "ref_roll": [], "ref_yaw": [],
        "target_pitch": [], "target_roll": [], "target_yaw": [],
        "vel_pitch": [], "vel_roll": [], "vel_yaw": [],
        "ref_vel_pitch": [], "ref_vel_roll": [], "ref_vel_yaw": [],
        "p1": [], "p2": [], "r1": [], "r2": [], "y1": [], "y2": [],
        "max_act": [],
    }
    stop_reason = None

    while t_now < duration - 1e-9:
        # Check phase transition at current t_now
        for ph in PHASES:
            if ph["t_start"] - 1e-9 <= t_now < ph["t_end"] - 1e-9:
                if ph["id"] != curr_phase_id:
                    curr_phase_id = ph["id"]
                    set_prop(controller, "urgency", ph["urgency"])
                    set_prop(controller, "desired_pitch", math.radians(ph["pitch_deg"]))
                    set_prop(controller, "desired_roll", math.radians(ph["roll_deg"]))
                    set_prop(controller, "desired_yaw", math.radians(ph["yaw_deg"]))
                    print(
                        f"[PHASE {ph['id']}] t={t_now:5.2f}s | {ph['name']:18s} | "
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

        rp = math.degrees(controller.getStateVariableValue(state, "traj_pos_pitch"))
        rr = math.degrees(controller.getStateVariableValue(state, "traj_pos_roll"))
        ry = math.degrees(controller.getStateVariableValue(state, "traj_pos_yaw"))
        rvp = math.degrees(controller.getStateVariableValue(state, "traj_vel_pitch"))
        rvr = math.degrees(controller.getStateVariableValue(state, "traj_vel_roll"))
        rvy = math.degrees(controller.getStateVariableValue(state, "traj_vel_yaw"))

        m_act = 0.0
        for m_i in range(n_mus):
            a = float(muscles.get(m_i).getActivation(state))
            if a > m_act:
                m_act = a

        ph_obj = PHASES[curr_phase_id]
        tr["t"].append(t_now)
        tr["phase"].append(curr_phase_id)
        tr["urgency"].append(ph_obj["urgency"])
        tr["pitch"].append(p1 + p2); tr["roll"].append(r1 + r2); tr["yaw"].append(y1 + y2)
        tr["ref_pitch"].append(rp); tr["ref_roll"].append(rr); tr["ref_yaw"].append(ry)
        tr["target_pitch"].append(ph_obj["pitch_deg"]); tr["target_roll"].append(ph_obj["roll_deg"]); tr["target_yaw"].append(ph_obj["yaw_deg"])
        tr["vel_pitch"].append(vp); tr["vel_roll"].append(vr); tr["vel_yaw"].append(vy)
        tr["ref_vel_pitch"].append(rvp); tr["ref_vel_roll"].append(rvr); tr["ref_vel_yaw"].append(rvy)
        tr["p1"].append(p1); tr["p2"].append(p2)
        tr["r1"].append(r1); tr["r2"].append(r2)
        tr["y1"].append(y1); tr["y2"].append(y2)
        tr["max_act"].append(m_act)

    # Summarize each phase
    t_np = np.array(tr["t"])
    phase_summaries = []
    for ph in PHASES:
        mask = (t_np >= ph["t_start"]) & (t_np <= ph["t_end"])
        if not np.any(mask):
            continue
        t_ph = t_np[mask]
        end_mask = (t_ph >= max(ph["t_start"], t_ph[-1] - 0.25)) & (t_ph <= t_ph[-1])
        idx_end = np.where(mask)[0][-len(t_ph[end_mask]):]
        sm = {
            "id": ph["id"],
            "name": ph["name"],
            "window_s": [ph["t_start"], round(float(t_ph[-1]), 2)],
            "urgency": ph["urgency"],
            "target_deg": [ph["pitch_deg"], ph["roll_deg"], ph["yaw_deg"]],
            "end_pitch_deg": round(float(np.mean(np.array(tr["pitch"])[idx_end])), 2),
            "end_roll_deg": round(float(np.mean(np.array(tr["roll"])[idx_end])), 2),
            "end_yaw_deg": round(float(np.mean(np.array(tr["yaw"])[idx_end])), 2),
            "end_q1q2_deg": {
                "p1_p2": [round(float(np.mean(np.array(tr["p1"])[idx_end])), 2), round(float(np.mean(np.array(tr["p2"])[idx_end])), 2)],
                "r1_r2": [round(float(np.mean(np.array(tr["r1"])[idx_end])), 2), round(float(np.mean(np.array(tr["r2"])[idx_end])), 2)],
                "y1_y2": [round(float(np.mean(np.array(tr["y1"])[idx_end])), 2), round(float(np.mean(np.array(tr["y2"])[idx_end])), 2)],
            },
            "peak_vel_deg_s": [
                round(float(np.max(np.abs(np.array(tr["vel_pitch"])[mask]))), 1),
                round(float(np.max(np.abs(np.array(tr["vel_roll"])[mask]))), 1),
                round(float(np.max(np.abs(np.array(tr["vel_yaw"])[mask]))), 1),
            ],
            "peak_muscle_act": round(float(np.max(np.array(tr["max_act"])[mask])), 3),
        }
        phase_summaries.append(sm)
        print(
            f"  -> {ph['name']:18s} | end=({sm['end_pitch_deg']:+6.2f}, {sm['end_roll_deg']:+6.2f}, {sm['end_yaw_deg']:+6.2f}) deg | "
            f"q1/q2 p={sm['end_q1q2_deg']['p1_p2']} r={sm['end_q1q2_deg']['r1_r2']} y={sm['end_q1q2_deg']['y1_y2']} | "
            f"Vmax={sm['peak_vel_deg_s']} deg/s | act_max={sm['peak_muscle_act']:.3f}",
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

    # Render 3-Panel Diagnostic Figure
    fig, axes = plt.subplots(3, 1, figsize=(16.5, 12.2), dpi=160, sharex=True)
    fig.suptitle(
        "20.5 s Extreme-ROM (90% Joint Limits) Multi-Urgency Voluntary Tracking Test (u in [0.0, 0.75])\n"
        "3rd-Order Right-Skewed Bell Trajectory Planner | 72-Muscle HYOID Model | w_pos = 0 (Attitude Mode)",
        fontsize=12.8,
        fontweight="bold",
        y=0.98,
    )

    # Panel A: 3-Axis Head Attitude Tracking vs Smooth 3rd-Order Reference
    ax = axes[0]
    ax.plot(t_np, tr["pitch"], "#059669", lw=2.2, label="True Head Pitch (p1+p2)")
    ax.plot(t_np, tr["ref_pitch"], "#065f46", ls=":", lw=1.5, label="3rd-Order Pitch Ref")
    ax.plot(t_np, tr["roll"], "#2563eb", lw=2.2, label="True Head Roll (r1+r2)")
    ax.plot(t_np, tr["ref_roll"], "#1e3a8a", ls=":", lw=1.5, label="3rd-Order Roll Ref")
    ax.plot(t_np, tr["yaw"], "#7c3aed", lw=2.2, label="True Head Yaw (y1+y2)")
    ax.plot(t_np, tr["ref_yaw"], "#4c1d95", ls=":", lw=1.5, label="3rd-Order Yaw Ref")
    for ph in PHASES[1:]:
        ax.axvline(ph["t_start"], color="#94a3b8", ls="--", lw=0.9, alpha=0.7)
        ax.text(ph["t_start"] + 0.08, 78.0, f"P{ph['id']}\nu={ph['urgency']:.2f}", fontsize=7.5, fontweight="bold", color="#334155")
    ax.set_ylim(-85, 95)
    ax.set_ylabel("Head Attitude [deg]", fontweight="bold")
    ax.set_title("Panel A: 3-Axis Extreme-ROM Head Attitude vs. 3rd-Order Reference Trajectory", fontweight="bold", fontsize=10.8)
    ax.grid(True, alpha=0.3); ax.legend(fontsize=8.2, ncol=3, loc="lower left")

    # Panel B: Upper (q2) vs Lower (q1) Cervical Joint Split & Hard Limits
    ax = axes[1]
    ax.plot(t_np, tr["p1"], "#059669", lw=1.9, label="Lower Pitch p1 [-19.2, +28.8]")
    ax.plot(t_np, tr["p2"], "#34d399", ls="--", lw=1.9, label="Upper Pitch p2 [-39.6, +57.6]")
    ax.plot(t_np, tr["r1"], "#2563eb", lw=1.9, label="Lower Roll r1 [-7.2, +7.2]")
    ax.plot(t_np, tr["r2"], "#60a5fa", ls="--", lw=1.9, label="Upper Roll r2 [-39.6, +39.6]")
    ax.plot(t_np, tr["y1"], "#7c3aed", lw=1.9, label="Lower Yaw y1 [-45.6, +45.6]")
    ax.plot(t_np, tr["y2"], "#c084fc", ls="--", lw=1.9, label="Upper Yaw y2 [-32.4, +32.4]")
    for ph in PHASES[1:]:
        ax.axvline(ph["t_start"], color="#94a3b8", ls="--", lw=0.9, alpha=0.7)
    ax.axhline(7.2, color="#2563eb", ls=":", lw=0.9, alpha=0.6); ax.axhline(-7.2, color="#2563eb", ls=":", lw=0.9, alpha=0.6)
    ax.set_ylabel("Segment Angles q1, q2 [deg]", fontweight="bold")
    ax.set_title("Panel B: Lower (q1) vs. Upper (q2) Cervical Joint Partitioning & Hard-Stop Margins", fontweight="bold", fontsize=10.8)
    ax.grid(True, alpha=0.3); ax.legend(fontsize=8.0, ncol=3, loc="upper left")

    # Panel C: Angular Velocity Profiles & Peak Muscle Activation
    ax = axes[2]
    ax_r = ax.twinx()
    ax.plot(t_np, tr["vel_pitch"], "#059669", lw=1.9, label="Pitch Velocity [deg/s]")
    ax.plot(t_np, tr["vel_roll"], "#2563eb", lw=1.9, label="Roll Velocity [deg/s]")
    ax.plot(t_np, tr["vel_yaw"], "#7c3aed", lw=1.9, label="Yaw Velocity [deg/s]")
    ax_r.plot(t_np, tr["max_act"], "#dc2626", ls="--", lw=1.4, alpha=0.85, label="Max Muscle Activation [0-1]")
    ax_r.set_ylim(0.0, 1.15)
    for ph in PHASES[1:]:
        ax.axvline(ph["t_start"], color="#94a3b8", ls="--", lw=0.9, alpha=0.7)
    ax.set_xlabel("Time [s]", fontweight="bold")
    ax.set_ylabel("Angular Velocity [deg/s]", fontweight="bold")
    ax_r.set_ylabel("Peak Muscle Activation", color="#dc2626", fontweight="bold")
    ax.set_title("Panel C: Skewed Bell Velocity Profiles Across Urgency Regimes & Peak Muscle Excitation", fontweight="bold", fontsize=10.8)
    l1, lb1 = ax.get_legend_handles_labels(); l2, lb2 = ax_r.get_legend_handles_labels()
    ax.legend(l1 + l2, lb1 + lb2, fontsize=8.2, ncol=4, loc="upper left")
    ax.grid(True, alpha=0.3)

    fig.tight_layout(rect=[0, 0, 1, 0.95])
    os.makedirs(os.path.dirname(OUT_PNG), exist_ok=True)
    fig.savefig(OUT_PNG, bbox_inches="tight")
    shutil.copyfile(OUT_PNG, os.path.join(ART_DIR, "extreme_rom_multi_urgency_test.png"))
    plt.close(fig)
    print(f"[OK] Saved {OUT_PNG} and {OUT_JSON} in {time.time()-t0:.1f}s", flush=True)


if __name__ == "__main__":
    main()
