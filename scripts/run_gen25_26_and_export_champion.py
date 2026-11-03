import os, sys
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_UTIL_PATH = os.path.join(_REPO_ROOT, 'util')
for _p in (_UTIL_PATH, _REPO_ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import os
import sys
import glob
import json
import math
import shutil
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import opensim as osim

sys.path.append(os.getcwd())
from cmaes_worker_p22 import ensure_libraries_loaded, PER_MUSCLE_72_U0
from scripts.run_multisine_cmaes_campaign import main as run_cmaes_campaign
from scripts.run_adaptive_controller_updated import (
    TARGETS_MULTISINE,
    build_multisine_spline,
    compute_reference_pitch,
)


def get_global_best_champion():
    best_score = float("inf")
    best_path = None
    best_data = None
    for cand_path in glob.glob(os.path.join("logs", "candidates_multisine", "*.json")):
        try:
            with open(cand_path, "r", encoding="utf-8") as f:
                d = json.load(f)
            sc = float(d.get("total_score", float("inf")))
            if "params" in d and "frequencies" in d and len(d["params"]) == 15 and sc < best_score:
                best_score = sc
                best_path = cand_path
                best_data = d
        except Exception:
            continue
    return best_path, best_data


def simulate_and_export_champion(champ_data):
    worker_name = champ_data.get("worker", "Global_Champion")
    total_score = float(champ_data["total_score"])
    params = [float(x) for x in champ_data["params"]]
    (
        K_ton, p_ton, K_sc, z_sc, p_sc, K_phas, p_phas,
        kp_prop, z_ccr, p_ccr, k_gamma_dyn, k_gamma_stat,
        kp_task_pitch, kd_task_pitch, ki_task_pitch
    ) = params

    # Ensure z_sc and z_ccr are actual zero locations (not ratios)
    if z_sc < 0.99:
        z_sc = z_sc * p_sc
    if z_ccr < 0.99:
        z_ccr = z_ccr * p_ccr

    print("\n" + "=" * 92, flush=True)
    print(f"[EXPORT] Simulating Global Champion {worker_name} ({total_score:,.2f} pts) for .mot/.mto & 3-Panel Plot...", flush=True)
    print("=" * 92, flush=True)

    ensure_libraries_loaded()
    model = osim.Model("osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim")

    controller = None
    for i in range(model.getControllerSet().getSize()):
        ctrl = model.getControllerSet().get(i)
        if ctrl.getConcreteClassName() == "FixationController":
            ctrl.set_enabled(False)
        elif ctrl.getConcreteClassName() == "HeadNeckNeuralController":
            controller = ctrl

    osim.PropertyHelper.setValueDouble(K_ton, controller.updPropertyByName("K_ton"))
    osim.PropertyHelper.setValueDouble(p_ton, controller.updPropertyByName("p_ton"))
    osim.PropertyHelper.setValueDouble(K_sc, controller.updPropertyByName("K_sc"))
    osim.PropertyHelper.setValueDouble(z_sc, controller.updPropertyByName("z_sc"))
    osim.PropertyHelper.setValueDouble(p_sc, controller.updPropertyByName("p_sc"))
    osim.PropertyHelper.setValueDouble(K_phas, controller.updPropertyByName("K_phas"))
    osim.PropertyHelper.setValueDouble(-1.0, controller.updPropertyByName("z_phas"))
    osim.PropertyHelper.setValueDouble(p_phas, controller.updPropertyByName("p_phas"))
    osim.PropertyHelper.setValueDouble(-1.0, controller.updPropertyByName("p_phas2"))
    osim.PropertyHelper.setValueDouble(kp_prop, controller.updPropertyByName("k_p"))

    osim.PropertyHelper.setValueDouble(kp_task_pitch, controller.updPropertyByName("Kp_task_pitch"))
    osim.PropertyHelper.setValueDouble(kd_task_pitch, controller.updPropertyByName("Kd_task_pitch"))
    osim.PropertyHelper.setValueDouble(10.21, controller.updPropertyByName("Kp_task_roll"))
    osim.PropertyHelper.setValueDouble(1.29, controller.updPropertyByName("Kd_task_roll"))
    osim.PropertyHelper.setValueDouble(0.24, controller.updPropertyByName("Kp_task_yaw"))
    osim.PropertyHelper.setValueDouble(0.12, controller.updPropertyByName("Kd_task_yaw"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("k_vigor"))
    osim.PropertyHelper.setValueDouble(0.1, controller.updPropertyByName("trajectory_tau"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("K_eso"))

    osim.PropertyHelper.setValueDouble(ki_task_pitch, controller.updPropertyByName("Ki_task"))
    osim.PropertyHelper.setValueDouble(ki_task_pitch, controller.updPropertyByName("Ki_task_pitch"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("Ki_task_roll"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("Ki_task_yaw"))

    osim.PropertyHelper.setValueDouble(kp_prop, controller.updPropertyByName("Kp_proprioception"))
    osim.PropertyHelper.setValueDouble(z_ccr, controller.updPropertyByName("z_ccr"))
    osim.PropertyHelper.setValueDouble(p_ccr, controller.updPropertyByName("p_ccr"))
    osim.PropertyHelper.setValueDouble(k_gamma_dyn, controller.updPropertyByName("K_gamma_dyn"))
    osim.PropertyHelper.setValueDouble(k_gamma_stat, controller.updPropertyByName("K_gamma_stat"))
    osim.PropertyHelper.setValueDouble(0.010, controller.updPropertyByName("delay_vcr"))
    osim.PropertyHelper.setValueDouble(0.015, controller.updPropertyByName("delay_ccr"))
    osim.PropertyHelper.setValueBool(True, controller.updPropertyByName("use_smith_predictor"))
    osim.PropertyHelper.setValueDouble(0.007, controller.updPropertyByName("delay_smith_proprio"))
    osim.PropertyHelper.setValueDouble(0.007, controller.updPropertyByName("delay_afferent_proprio"))

    zero_func = osim.Constant(0.0)
    for coord_name in ["gndpitch", "gndroll", "gndyaw", "spine_ty", "spine_tz"]:
        c = model.updCoordinateSet().get(coord_name)
        c.set_locked(False)
        c.setDefaultIsPrescribed(True)
        c.setPrescribedFunction(zero_func)

    c_yaw1 = model.updCoordinateSet().get("yaw1")
    c_yaw1.set_locked(False)
    c_yaw1.setDefaultIsPrescribed(False)

    for eye_c in ["gaze_r_x", "gaze_r_y", "gaze_r_z", "gaze_l_x", "gaze_l_y", "gaze_l_z"]:
        if model.getCoordinateSet().contains(eye_c):
            c = model.updCoordinateSet().get(eye_c)
            c.setDefaultValue(0.0)
            c.setDefaultLocked(True)

    delay = 1.5
    ramp = 1.5
    eval_window = 2.0 / 0.324
    sim_duration = delay + ramp + eval_window  # 9.1728395s

    spline, freqs, amps, phases, scale_factor = build_multisine_spline(
        TARGETS_MULTISINE, duration=sim_duration, delay=delay, ramp=ramp, dt_spline=0.0025
    )

    coord_tx = model.updCoordinateSet().get("spine_tx")
    coord_tx.set_locked(False)
    coord_tx.setDefaultIsPrescribed(True)
    coord_tx.setPrescribedFunction(spline)

    state = model.initSystem()
    c_yaw1.setIsPrescribed(state, False)
    coord_tx.setSpeedValue(state, 0.0)

    model.realizePosition(state)
    skull = model.getBodySet().get("skull")
    R_head_to_world = skull.getTransformInGround(state).R()
    q = osim.Rotation(R_head_to_world).convertRotationToQuaternion()
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q0", q.get(0))
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q1", q.get(1))
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q2", q.get(2))
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q3", q.get(3))
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/pitch_error_integral", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/roll_error_integral", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/yaw_error_integral", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_pitch", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_roll", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_yaw", 0.0)

    muscles = model.updMuscles()
    active_muscle_names = []
    for i in range(muscles.getSize()):
        m = muscles.get(i)
        name = m.getName()
        if name.startswith("r_") or name.startswith("l_"):
            m.setAppliesForce(state, False)
            continue
        active_muscle_names.append(name)
        a0 = PER_MUSCLE_72_U0.get(name, 0.030)
        m.setActivation(state, a0)
        gd0 = 100.0 * k_gamma_dyn * a0
        gs0 = 100.0 * k_gamma_stat * a0
        fdyn0 = (gd0 * gd0) / (gd0 * gd0 + 3600.0)
        fstat0 = (gs0 * gs0) / (gs0 * gs0 + 3600.0)
        m.setStateVariableValue(state, "spindle/dynamic_activation", fdyn0)
        m.setStateVariableValue(state, "spindle/static_activation", fstat0)
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/sp_fdyn_{name}", fdyn0)
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/sp_fstat_{name}", fstat0)

    model.equilibrateMuscles(state)

    manager = osim.Manager(model)
    manager.setWriteToStorage(False)
    manager.setPerformAnalyses(False)
    manager.setIntegratorMethod(5)  # SemiExplicitEuler2
    manager.setIntegratorAccuracy(1e-4)
    manager.setIntegratorMinimumStepSize(1e-4)
    manager.setIntegratorMaximumStepSize(0.001)
    state.setTime(0.0)
    manager.initialize(state)

    coord_set = model.getCoordinateSet()
    n_coords = coord_set.getSize()
    coord_names = [coord_set.get(i).getName() for i in range(n_coords)]
    translational_coords = {"spine_tx", "spine_ty", "spine_tz"}

    c_p1 = coord_set.get("pitch1")
    c_p2 = coord_set.get("pitch2")
    c_r1 = coord_set.get("roll1")
    c_r2 = coord_set.get("roll2")
    c_y1 = coord_set.get("yaw1")
    c_y2 = coord_set.get("yaw2")

    dt_step = 0.005  # 200 Hz
    n_steps = int(round(sim_duration / dt_step))

    times = []
    pitch_deg = []
    ref_pitch_deg = []
    roll_deg = []
    yaw_deg = []
    rows_rad = []
    rows_deg = []
    rows_deg_with_muscles = []

    def record_frame(t_val, st):
        p_val = math.degrees(c_p1.getValue(st) + c_p2.getValue(st))
        r_val = math.degrees(c_r1.getValue(st) + c_r2.getValue(st))
        y_val = math.degrees(c_y1.getValue(st) + c_y2.getValue(st))
        p_ref = compute_reference_pitch(t_val, freqs, amps, phases, TARGETS_MULTISINE, delay=delay, ramp=ramp)

        times.append(t_val)
        pitch_deg.append(p_val)
        ref_pitch_deg.append(p_ref)
        roll_deg.append(r_val)
        yaw_deg.append(y_val)

        r_rad = [t_val]
        r_deg = [t_val]
        for idx_c in range(n_coords):
            c_obj = coord_set.get(idx_c)
            c_name = coord_names[idx_c]
            v_raw = c_obj.getValue(st)
            r_rad.append(v_raw)
            if c_name in translational_coords:
                r_deg.append(v_raw)
            else:
                r_deg.append(math.degrees(v_raw))
        rows_rad.append(r_rad)
        rows_deg.append(r_deg)

        r_mus = list(r_deg)
        for m_name in active_muscle_names:
            try:
                act_v = model.getStateVariableValue(st, f"/forceset/{m_name}/activation")
            except Exception:
                act_v = 0.03
            r_mus.append(act_v)
        rows_deg_with_muscles.append(r_mus)

    # Record t = 0.0 initial frame
    record_frame(0.0, state)

    for step_idx in range(1, n_steps + 1):
        t_target = min(step_idx * dt_step, sim_duration)
        state = manager.integrate(t_target)
        record_frame(state.getTime(), state)

    os.makedirs("state_files", exist_ok=True)
    os.makedirs("results", exist_ok=True)

    # 1. Write standard radians/meters .mot (identical header structure to state_files/happee/happee_motion_*.mot)
    mot_rad_path = os.path.join("state_files", f"champion_{worker_name}_multisine.mot")
    mot_deg_path = os.path.join("state_files", f"champion_{worker_name}_multisine_degrees.mot")
    mot_mus_path = os.path.join("state_files", f"champion_{worker_name}_multisine_with_muscles.mot")

    def write_mot_file(filepath, header_title, in_degrees_str, col_names, data_rows):
        with open(filepath, "w", encoding="utf-8") as f_out:
            f_out.write(f"{header_title}\n")
            f_out.write("version=1\n")
            f_out.write(f"nRows={len(data_rows)}\n")
            f_out.write(f"nColumns={len(col_names)}\n")
            f_out.write(f"inDegrees={in_degrees_str}\n")
            f_out.write("endheader\n")
            f_out.write("\t".join(col_names) + "\n")
            for row in data_rows:
                f_out.write("\t".join(f"{val:.8f}" for val in row) + "\n")

    cols_base = ["time"] + coord_names
    cols_mus = ["time"] + coord_names + [f"/forceset/{m}/activation" for m in active_muscle_names]

    write_mot_file(mot_rad_path, f"champion_{worker_name}_multisine", "no", cols_base, rows_rad)
    write_mot_file(mot_deg_path, f"champion_{worker_name}_multisine_degrees", "yes", cols_base, rows_deg)
    write_mot_file(mot_mus_path, f"champion_{worker_name}_multisine_with_muscles", "yes", cols_mus, rows_deg_with_muscles)

    # Also copy canonical names into state_files/ for quick GUI access
    shutil.copyfile(mot_rad_path, os.path.join("state_files", "champion_multisine_motion.mot"))
    shutil.copyfile(mot_deg_path, os.path.join("state_files", "champion_multisine_motion_degrees.mot"))
    shutil.copyfile(mot_mus_path, os.path.join("state_files", "champion_multisine_states_with_muscles.mot"))

    print(f"[EXPORT COMPLETE] Motion files written:", flush=True)
    print(f"  -> {mot_rad_path}", flush=True)
    print(f"  -> {mot_deg_path}", flush=True)
    print(f"  -> {mot_mus_path}", flush=True)

    # 2. Compute live 6-harmonic least-squares demodulation over steady-state window [delay+ramp, sim_duration]
    times_arr = np.array(times)
    pitch_arr = np.array(pitch_deg)
    ref_arr = np.array(ref_pitch_deg)
    roll_arr = np.array(roll_deg)
    yaw_arr = np.array(yaw_deg)

    m_post = times_arr >= (delay + ramp)
    t_fit = times_arr[m_post]
    y_fit_deg = pitch_arr[m_post]
    t_rel = t_fit - delay

    cols = [np.ones_like(t_fit), t_fit - (delay + ramp)]
    for k in range(len(freqs)):
        omega_k = 2.0 * math.pi * freqs[k]
        psi_k = omega_k * t_rel + phases[k]
        cols.append(np.sin(psi_k))
        cols.append(np.cos(psi_k))
    A_mat = np.column_stack(cols)
    coeffs, _, _, _ = np.linalg.lstsq(A_mat, y_fit_deg, rcond=None)

    freqs_dict = {}
    live_total_score = 0.0
    for k, (f_val, tgt_gain, tgt_phase) in enumerate(TARGETS_MULTISINE):
        S_k = float(coeffs[2 + 2 * k])
        C_k = float(coeffs[2 + 2 * k + 1])
        amp_deg = math.sqrt(S_k * S_k + C_k * C_k)
        sim_gain = amp_deg / amps[k]
        sim_phase = math.degrees(math.atan2(C_k, S_k))
        gain_db_diff = 20.0 * math.log10(max(sim_gain, 1e-6) / tgt_gain)
        gain_loss = 100.0 * (gain_db_diff ** 2)
        raw_p_diff = (sim_phase - tgt_phase + 180.0) % 360.0 - 180.0
        phase_err_deg = abs(raw_p_diff)
        phase_loss = 1.0 * (phase_err_deg ** 2)
        f_score = gain_loss + phase_loss
        live_total_score += f_score
        freqs_dict[f"{f_val:.3f}"] = {
            "target_gain": tgt_gain,
            "sim_gain": sim_gain,
            "gain_db_diff": gain_db_diff,
            "target_phase": tgt_phase,
            "sim_phase": sim_phase,
            "phase_err_deg": raw_p_diff,
            "score": f_score,
        }
    total_score = live_total_score
    print(f"[LIVE EVAL] Updated Champion {worker_name} Live Score = {total_score:,.2f} pts", flush=True)

    f_keys = ["0.324", "0.783", "1.099", "2.446", "5.583", "7.927"]
    f_vals = np.array([float(k) for k in f_keys])
    tgt_gains = np.array([float(freqs_dict[k]["target_gain"]) for k in f_keys])
    sim_gains = np.array([float(freqs_dict[k]["sim_gain"]) for k in f_keys])
    tgt_phases = np.array([float(freqs_dict[k]["target_phase"]) for k in f_keys])
    sim_phases_raw = np.array([float(freqs_dict[k]["sim_phase"]) for k in f_keys])
    phase_errs = np.array([float(freqs_dict[k]["phase_err_deg"]) for k in f_keys])
    # Unwrap sim_phase relative to target_phase so visual comparison on the phase axis matches the unwrapped phase error
    sim_phases_unwrapped = tgt_phases + phase_errs

    fig, axes = plt.subplots(3, 1, figsize=(12, 11.5))

    # --- Panel 1: Pitch Target vs Achieved ---
    ax0 = axes[0]
    ax0.plot(times_arr, ref_arr, "k--", lw=1.3, label="Happee Target Reference Pitch")
    ax0.plot(times_arr, pitch_arr, "b-", lw=1.6, label=f"OpenSim 6-DOF Head Pitch (p1+p2) [{worker_name}]")
    ax0.axvline(delay, color="#64748b", ls=":", lw=1.2, label="Ramp Start (t=1.5s)")
    ax0.axvline(delay + ramp, color="#dc2626", ls="--", lw=1.3, label="Steady-State Eval Window (t=3.0s..9.17s)")
    ax0.set_xlim(0.0, sim_duration)
    ax0.set_ylabel("Pitch (deg)", fontweight="bold")
    ax0.set_title(
        f"Global Champion {worker_name} — 6-Frequency Multisine Tracking & Bode Response (Total Score: {total_score:,.2f} pts)",
        fontsize=12,
        fontweight="bold",
    )
    ax0.legend(loc="upper right", fontsize=9)
    ax0.grid(True, alpha=0.3)

    # --- Panel 2: Roll & Yaw Errors ---
    ax1 = axes[1]
    ax1.plot(times_arr, roll_arr, "r-", lw=1.3, label=f"Head Roll Error (r1+r2) [Max |Roll| = {np.max(np.abs(roll_arr)):.2f}°]")
    ax1.plot(times_arr, yaw_arr, "g-", lw=1.3, label=f"Head Yaw Error (y1+y2) [Max |Yaw| = {np.max(np.abs(yaw_arr)):.2f}°]")
    ax1.axvline(delay, color="#64748b", ls=":", lw=1.2)
    ax1.axvline(delay + ramp, color="#dc2626", ls="--", lw=1.3)
    ax1.set_xlim(0.0, sim_duration)
    ax1.set_ylabel("Roll / Yaw (deg)", fontweight="bold")
    ax1.set_xlabel("Simulation Time (s)", fontweight="bold")
    ax1.legend(loc="upper right", fontsize=9)
    ax1.grid(True, alpha=0.3)

    # --- Panel 3: Combined Bode Gain & Phase (Target vs Got) ---
    ax2_gain = axes[2]
    ax2_phase = ax2_gain.twinx()

    l1 = ax2_gain.semilogx(
        f_vals, tgt_gains, "k--o", lw=1.8, ms=6, label="Target Bode Gain (Happee 2017) [deg/m]"
    )
    l2 = ax2_gain.semilogx(
        f_vals, sim_gains, "b-s", lw=2.2, ms=6.5, label=f"Achieved Bode Gain ({worker_name}) [deg/m]"
    )

    l3 = ax2_phase.semilogx(
        f_vals, tgt_phases, color="#b45309", ls="--", marker="^", lw=1.8, ms=6, label="Target Bode Phase (Happee 2017) [deg]"
    )
    l4 = ax2_phase.semilogx(
        f_vals, sim_phases_unwrapped, color="#dc2626", ls="-", marker="D", lw=2.2, ms=6, label=f"Achieved Bode Phase ({worker_name}) [deg]"
    )

    for idx_f, f_v in enumerate(f_vals):
        fk = f_keys[idx_f]
        db_diff = float(freqs_dict[fk]["gain_db_diff"])
        p_err = float(freqs_dict[fk]["phase_err_deg"])
        ax2_gain.annotate(
            f"{db_diff:+.1f} dB",
            (f_v, sim_gains[idx_f]),
            textcoords="offset points",
            xytext=(0, 8),
            ha="center",
            fontsize=8,
            color="#1d4ed8",
            fontweight="bold",
        )
        ax2_phase.annotate(
            f"{p_err:+.1f}°",
            (f_v, sim_phases_unwrapped[idx_f]),
            textcoords="offset points",
            xytext=(0, -13),
            ha="center",
            fontsize=8,
            color="#b91c1c",
            fontweight="bold",
        )

    ax2_gain.set_xticks(f_vals)
    ax2_gain.set_xticklabels([f"{fv:.3f} Hz" for fv in f_vals], fontweight="bold")
    ax2_gain.set_xlabel("Multisine Excitation Frequency (Hz, Log Scale)", fontweight="bold")
    ax2_gain.set_ylabel("Bode Gain (deg/m)", color="#1d4ed8", fontweight="bold")
    ax2_phase.set_ylabel("Bode Phase (deg)", color="#b91c1c", fontweight="bold")
    ax2_gain.tick_params(axis="y", labelcolor="#1d4ed8")
    ax2_phase.tick_params(axis="y", labelcolor="#b91c1c")
    ax2_gain.set_ylim(0.0, max(np.max(tgt_gains), np.max(sim_gains)) * 1.18)
    ax2_phase.set_ylim(min(np.min(tgt_phases), np.min(sim_phases_unwrapped)) - 35.0, 45.0)
    ax2_gain.grid(True, which="both", alpha=0.3)

    lines_all = l1 + l2 + l3 + l4
    labels_all = [l.get_label() for l in lines_all]
    ax2_gain.legend(lines_all, labels_all, loc="center left", fontsize=8.5, framealpha=0.92)

    plt.tight_layout()
    out_plot_path = os.environ.get("CHAMPION_PLOT_PATH", "logs/champion_multisine_tracking_and_bode.png")
    plt.savefig(out_plot_path, dpi=200)
    plt.close(fig)
    print(f"  -> 3-Panel Plot saved to {out_plot_path}", flush=True)
    return out_plot_path, mot_rad_path, mot_deg_path, mot_mus_path


if __name__ == "__main__":
    if "--export-only" not in sys.argv:
        os.environ["MULTISINE_STOP_GEN"] = "26"
        os.environ["PHASE2_4_WORKERS"] = "14"
        run_cmaes_campaign()

    best_path, best_data = get_global_best_champion()
    print(f"\n[CHAMPION SELECTED] {best_data['worker']} ({best_data['total_score']:,.2f} pts) from {best_path}", flush=True)
    simulate_and_export_champion(best_data)

