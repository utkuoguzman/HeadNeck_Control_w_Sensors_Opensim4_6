import os
import sys
import json
import math
import time
import shutil
import random
import subprocess
from datetime import datetime
import numpy as np
import cma
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import opensim as osim
from concurrent.futures import ThreadPoolExecutor, as_completed

sys.path.append(os.getcwd())
from cmaes_worker_p22 import PER_MUSCLE_72_U0, _load_plugin_cross_platform
from scripts.run_multisine_cmaes_campaign import (
    FIXED_W_N, FIXED_ZETA, FIXED_KI, TARGETS_ORDERED, PARAM_NAMES,
    evaluate_multisine_candidate, update_multisine_ledger
)
from scripts.run_adaptive_controller_updated import (
    TARGETS_MULTISINE, ADAPT_15_NAMES, build_multisine_spline, compute_reference_pitch
)


def time_left_until(target_hour=18, target_minute=6):
    now = datetime.now()
    target = now.replace(hour=target_hour, minute=target_minute, second=0, microsecond=0)
    return (target - now).total_seconds()


def run_single_adaptive_stage(
    seed_p15,
    base_gains,
    base_phases,
    base_ms_score,
    pre_freq_results,
    stage_label,
    out_json_path,
    out_png_path,
    adapt_mult=0.65,
    sigma_leak=0.010,
    t_freeze=14.0
):
    print("\n" + "=" * 96, flush=True)
    print(f"[ADAPTIVE STAGE: {stage_label}] Starting from {base_ms_score:,.2f} pts (mult={adapt_mult}, leak={sigma_leak})", flush=True)
    print("=" * 96, flush=True)

    (
        K_ton, p_ton, K_sc, r_lead_sc, p_sc, K_phas, p_phas,
        kp_prop, r_lead_ccr, p_ccr, k_gamma_dyn, k_gamma_stat,
        kp_task_pitch, kd_task_pitch, ki_task_pitch
    ) = [float(x) for x in seed_p15]

    z_sc = r_lead_sc if r_lead_sc > 0.99 else max(0.001, min(0.95, r_lead_sc)) * p_sc
    z_ccr = r_lead_ccr if r_lead_ccr > 0.99 else max(0.001, min(0.95, r_lead_ccr)) * p_ccr

    g_str = ",".join(f"{g:.4f}" for g in base_gains)
    p_str = ",".join(f"{p:.4f}" for p in base_phases)
    os.environ["OSIM_BASE_GAINS"] = g_str
    os.environ["OSIM_BASE_PHASES"] = p_str
    os.makedirs("logs", exist_ok=True)
    with open("logs/current_adaptive_bode_anchor.txt", "w", encoding="utf-8") as f_anc:
        f_anc.write(g_str + "\n" + p_str + "\n")

    sim_duration = t_freeze + 2.0 / 0.324
    delay = 1.5
    ramp = 1.5
    spline, freqs, amps, phases, scale_factor = build_multisine_spline(
        TARGETS_MULTISINE, duration=sim_duration, delay=delay, ramp=ramp, dt_spline=0.0025
    )

    os.environ["SIMB_NUM_THREADS"] = "1"
    os.environ["OMP_NUM_THREADS"] = "1"
    osim.Logger.setLevelString("Warn")

    if not sys.platform.startswith("win"):
        _load_plugin_cross_platform("vestibular_plugin", "osimVestibular")
    _load_plugin_cross_platform("proprioception_plugin", "osimMillard12EqWithAff")
    _load_plugin_cross_platform("FixationController_plugin", "osimFixation")
    _load_plugin_cross_platform("HeadNeckNeuralControllerAdaptive_plugin", "osimHeadNeckNeuralAdaptive")

    model = osim.Model("osim_files/HYOID_HeadNeckNeural_Model_Adaptive.osim")
    controller = None
    for i in range(model.getControllerSet().getSize()):
        ctrl = model.getControllerSet().get(i)
        if ctrl.getConcreteClassName() == "FixationController":
            ctrl.set_enabled(False)
        elif ctrl.getConcreteClassName() == "HeadNeckNeuralControllerAdaptive":
            controller = ctrl

    osim.PropertyHelper.setValueDouble(K_ton, controller.updPropertyByName("K_ton"))
    osim.PropertyHelper.setValueDouble(p_ton, controller.updPropertyByName("p_ton"))
    osim.PropertyHelper.setValueDouble(K_sc, controller.updPropertyByName("K_sc"))
    osim.PropertyHelper.setValueDouble(z_sc, controller.updPropertyByName("z_sc"))
    osim.PropertyHelper.setValueDouble(p_sc, controller.updPropertyByName("p_sc"))
    osim.PropertyHelper.setValueDouble(K_phas, controller.updPropertyByName("K_phas"))
    osim.PropertyHelper.setValueDouble(p_phas, controller.updPropertyByName("p_phas"))
    osim.PropertyHelper.setValueDouble(kp_prop, controller.updPropertyByName("k_p"))
    osim.PropertyHelper.setValueDouble(kp_prop, controller.updPropertyByName("Kp_proprioception"))
    osim.PropertyHelper.setValueDouble(z_ccr, controller.updPropertyByName("z_ccr"))
    osim.PropertyHelper.setValueDouble(p_ccr, controller.updPropertyByName("p_ccr"))
    osim.PropertyHelper.setValueDouble(k_gamma_dyn, controller.updPropertyByName("K_gamma_dyn"))
    osim.PropertyHelper.setValueDouble(k_gamma_stat, controller.updPropertyByName("K_gamma_stat"))

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

    osim.PropertyHelper.setValueDouble(0.010, controller.updPropertyByName("delay_vcr"))
    osim.PropertyHelper.setValueDouble(0.015, controller.updPropertyByName("delay_ccr"))
    osim.PropertyHelper.setValueBool(True, controller.updPropertyByName("use_smith_predictor"))
    osim.PropertyHelper.setValueDouble(0.007, controller.updPropertyByName("delay_smith_proprio"))
    osim.PropertyHelper.setValueDouble(0.007, controller.updPropertyByName("delay_afferent_proprio"))
    osim.PropertyHelper.setValueBool(True, controller.updPropertyByName("enable_adaptation"))
    osim.PropertyHelper.setValueDouble(adapt_mult, controller.updPropertyByName("adaptation_rate_multiplier"))
    osim.PropertyHelper.setValueDouble(sigma_leak, controller.updPropertyByName("sigma_leakage"))
    osim.PropertyHelper.setValueDouble(3.0, controller.updPropertyByName("adaptation_start_time"))
    osim.PropertyHelper.setValueDouble(scale_factor, controller.updPropertyByName("multisine_scale"))

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
    for i in range(muscles.getSize()):
        m = muscles.get(i)
        name = m.getName()
        if name.startswith("r_") or name.startswith("l_"):
            m.setAppliesForce(state, False)
            continue
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

    c_p1 = model.getCoordinateSet().get("pitch1")
    c_p2 = model.getCoordinateSet().get("pitch2")
    c_r1 = model.getCoordinateSet().get("roll1")
    c_r2 = model.getCoordinateSet().get("roll2")
    c_y1 = model.getCoordinateSet().get("yaw1")
    c_y2 = model.getCoordinateSet().get("yaw2")

    dt_chunk = 0.005
    n_steps = int(round(sim_duration / dt_chunk))
    t_wall0 = time.time()

    times, pitch_deg, ref_pitch_deg, roll_deg, yaw_deg = [], [], [], [], []
    theta_hist = {name: [] for name in ADAPT_15_NAMES}
    theta_init = [
        K_ton, p_ton, K_sc, z_sc, p_sc, K_phas, p_phas,
        kp_prop, z_ccr, p_ccr, k_gamma_dyn, k_gamma_stat,
        kp_task_pitch, kd_task_pitch, ki_task_pitch
    ]

    for step in range(1, n_steps + 1):
        t_target = min(step * dt_chunk, sim_duration)
        state = manager.integrate(t_target)
        t_cur = state.getTime()

        p_val = math.degrees(c_p1.getValue(state) + c_p2.getValue(state))
        r_val = math.degrees(c_r1.getValue(state) + c_r2.getValue(state))
        y_val = math.degrees(c_y1.getValue(state) + c_y2.getValue(state))
        p_ref = compute_reference_pitch(t_cur, freqs, amps, phases, TARGETS_MULTISINE, delay=delay, ramp=ramp)

        times.append(t_cur)
        pitch_deg.append(p_val)
        ref_pitch_deg.append(p_ref)
        roll_deg.append(r_val)
        yaw_deg.append(y_val)

        for idx_p, name in enumerate(ADAPT_15_NAMES):
            sv_path = f"/controllerset/VCR_CCR_Controller/{name}"
            try:
                val = model.getStateVariableValue(state, sv_path)
            except Exception:
                val = theta_init[idx_p]
            theta_hist[name].append(val)

        if step % 400 == 0 or step == n_steps:
            el = time.time() - t_wall0
            print(
                f"  {t_cur:5.2f}s | {el:6.1f}s | Pitch={p_val:+6.2f}° | R/Y={r_val:+5.2f}°/{y_val:+5.2f}° | "
                f"K_sc={theta_hist['adapt_K_sc'][-1]:.4f} | K_phas={theta_hist['adapt_K_phas'][-1]:.2f} | "
                f"z_ccr={theta_hist['adapt_z_ccr'][-1]:.2f} | Ki={theta_hist['adapt_Ki_task_pitch'][-1]:.2f}",
                flush=True
            )

    total_wall = time.time() - t_wall0
    times = np.array(times)
    pitch_deg = np.array(pitch_deg)
    ref_pitch_deg = np.array(ref_pitch_deg)
    roll_deg = np.array(roll_deg)
    yaw_deg = np.array(yaw_deg)

    mask_stat = times >= t_freeze
    t_stat = times[mask_stat]
    y_stat = pitch_deg[mask_stat]
    t_rel = t_stat - delay
    K_freq = len(TARGETS_MULTISINE)
    X_cols = [np.ones_like(t_rel), t_rel]
    for k in range(K_freq):
        psi_k = 2.0 * math.pi * freqs[k] * t_rel + phases[k]
        X_cols.append(np.sin(psi_k))
        X_cols.append(np.cos(psi_k))
    X_mat = np.column_stack(X_cols)
    beta, _, _, _ = np.linalg.lstsq(X_mat, y_stat, rcond=None)

    post_score = 0.0
    post_freq_results = {}
    for k in range(K_freq):
        f_val, t_gain, t_phase = TARGETS_MULTISINE[k]
        b_sin = beta[2 + 2 * k]
        b_cos = beta[2 + 2 * k + 1]
        resp_amp = math.hypot(b_sin, b_cos)
        sim_gain = resp_amp / amps[k]
        sim_phase = math.degrees(math.atan2(b_cos, b_sin))
        while sim_phase - t_phase > 180.0:
            sim_phase -= 360.0
        while sim_phase - t_phase < -180.0:
            sim_phase += 360.0
        gain_err_db = 20.0 * math.log10(max(sim_gain, 1e-6) / t_gain)
        phase_err = sim_phase - t_phase
        f_loss = 100.0 * (gain_err_db ** 2) + 1.0 * (phase_err ** 2)
        post_score += f_loss
        fk = f"{f_val:.3f}"
        post_freq_results[fk] = {
            "freq": f_val,
            "sim_gain": round(sim_gain, 2),
            "target_gain": t_gain,
            "gain_err_db": round(gain_err_db, 2),
            "gain_db_diff": round(gain_err_db, 2),
            "sim_phase": round(sim_phase, 2),
            "target_phase": t_phase,
            "phase_err": round(phase_err, 2),
            "phase_err_deg": round(phase_err, 2),
            "score": round(f_loss, 1)
        }

    print(f"[ADAPTIVE STAGE {stage_label} COMPLETE] Pre={base_ms_score:,.2f} -> Post={post_score:,.2f} pts (in {total_wall/60.0:.2f} min)", flush=True)

    summary = {
        "anchor_label": stage_label,
        "total_wall_time_sec": total_wall,
        "pre_adaptation_multisine_score": base_ms_score,
        "post_adaptation_window_score": post_score,
        "max_roll_deg": float(np.max(np.abs(roll_deg))),
        "max_yaw_deg": float(np.max(np.abs(yaw_deg))),
        "initial_parameters_15d": theta_init,
        "final_parameters_15d": {name: float(theta_hist[name][-1]) for name in ADAPT_15_NAMES},
        "per_freq_pre": pre_freq_results,
        "per_freq_post": post_freq_results,
    }
    with open(out_json_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)

    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    axes[0].plot(times, ref_pitch_deg, 'k--', lw=1.2, label='Happee Target Reference Pitch')
    axes[0].plot(times, pitch_deg, 'b-', lw=1.5, label='OpenSim 6-DOF Head Pitch (p1+p2)')
    axes[0].axvline(t_freeze, color='gray', ls=':', lw=1.5, label=f'Adaptation Frozen (t={t_freeze}s)')
    axes[0].set_ylabel('Pitch (deg)')
    axes[0].set_title(f'15-Parameter Adaptive Controller ({stage_label}: {base_ms_score:,.1f} -> {post_score:,.1f} pts)')
    axes[0].legend(loc='upper right')
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(times, roll_deg, 'r-', lw=1.2, label='Head Roll (r1+r2)')
    axes[1].plot(times, yaw_deg, 'g-', lw=1.2, label='Head Yaw (y1+y2)')
    axes[1].axvline(t_freeze, color='gray', ls=':', lw=1.5)
    axes[1].set_ylabel('Roll / Yaw (deg)')
    axes[1].legend(loc='upper right')
    axes[1].grid(True, alpha=0.3)

    for idx_p, name in enumerate(ADAPT_15_NAMES):
        norm_curve = np.array(theta_hist[name]) / max(1e-6, theta_init[idx_p]) * 100.0
        axes[2].plot(times, norm_curve, lw=1.4, label=name.replace('adapt_', ''))
    axes[2].axhline(100.0, color='k', ls='--', lw=0.8)
    axes[2].axvline(t_freeze, color='gray', ls=':', lw=1.5)
    axes[2].set_ylabel('Normalized Param (%)')
    axes[2].set_xlabel('Simulation Time (s)')
    axes[2].legend(loc='upper left', ncol=5, fontsize=8)
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(out_png_path, dpi=180)
    plt.close(fig)
    return summary


def main():
    print("=" * 96, flush=True)
    print(f"MASTER AUTONOMOUS TRAINING PIPELINE UNTIL 18:10 (Started at {datetime.now().strftime('%H:%M:%S')})", flush=True)
    print("=" * 96, flush=True)

    # -------------------------------------------------------------------------
    # PHASE 1: Adaptive Stage 3 Continuation (from Stage 2 = 26,758.73 pts)
    # -------------------------------------------------------------------------
    if not os.path.exists("logs/adaptive_stage2_15params_trajectory.json"):
        shutil.copyfile("logs/adaptive_updated_15params_trajectory.json", "logs/adaptive_stage2_15params_trajectory.json")
        shutil.copyfile("logs/adaptive_updated_15params_convergence.png", "logs/adaptive_stage2_15params_convergence.png")

    with open("logs/adaptive_stage2_15params_trajectory.json", "r", encoding="utf-8") as f:
        s2_data = json.load(f)

    best_adapt_score = float(s2_data["post_adaptation_window_score"])
    best_adapt_summary = s2_data

    s2_fp = s2_data["final_parameters_15d"]
    s2_p15 = [float(s2_fp[k]) for k in ADAPT_15_NAMES]
    s2_freqs = s2_data["per_freq_post"]
    s2_gains = [float(s2_freqs[fk]["sim_gain"]) for fk in ["0.324", "0.783", "1.099", "2.446", "5.583", "7.927"]]
    s2_phases = [float(s2_freqs[fk]["sim_phase"]) for fk in ["0.324", "0.783", "1.099", "2.446", "5.583", "7.927"]]

    if os.path.exists("logs/adaptive_stage3_15params_trajectory.json"):
        with open("logs/adaptive_stage3_15params_trajectory.json", "r", encoding="utf-8") as f_s3c:
            s3_summary = json.load(f_s3c)
    else:
        s3_summary = run_single_adaptive_stage(
            seed_p15=s2_p15,
            base_gains=s2_gains,
            base_phases=s2_phases,
            base_ms_score=float(s2_data["post_adaptation_window_score"]),
            pre_freq_results=s2_freqs,
            stage_label="Stage3_RestoredStage2Surrogate",
            out_json_path="logs/adaptive_stage3_15params_trajectory.json",
            out_png_path="logs/adaptive_stage3_15params_convergence.png",
            adapt_mult=0.40,
            sigma_leak=0.012
        )
    if s3_summary["post_adaptation_window_score"] < best_adapt_score:
        best_adapt_score = s3_summary["post_adaptation_window_score"]
        best_adapt_summary = s3_summary
        shutil.copyfile("logs/adaptive_stage3_15params_trajectory.json", "logs/adaptive_updated_15params_trajectory.json")
        shutil.copyfile("logs/adaptive_stage3_15params_convergence.png", "logs/adaptive_updated_15params_convergence.png")

    # -------------------------------------------------------------------------
    # PHASE 2: Continuous Multisine CMA-ES Campaign (Generations 23+) until 17:55
    # -------------------------------------------------------------------------
    n_workers = 14
    with open("data/prescreen_rational_champion.json", "r", encoding="utf-8") as f:
        seed_data = json.load(f)

    x0 = np.array(seed_data["x0"], dtype=float)
    sigma0 = np.array(seed_data["sigma0"], dtype=float)
    lower_bounds = [
        0.010,  5.000,  0.010,  0.001, 15.000,  0.200,
        1.000,  0.030,  0.001, 20.000,  0.050,  0.050,
        0.000,  0.000,  0.500
    ]
    upper_bounds = [
        3.000, 150.000,  1.500,  0.600, 160.000, 28.000,
       65.000,   2.000,  0.500, 240.000,  1.500,  1.500,
       30.000,  12.000, 65.000
    ]

    opts = {
        "bounds": [lower_bounds, upper_bounds],
        "CMA_stds": list(sigma0),
        "popsize": 28,
        "seed": 2026,
        "verbose": -1,
    }
    es = cma.CMAEvolutionStrategy(x0, 1.0, opts)

    gen_records = []
    all_time_best_score = float("inf")
    all_time_best_worker = None
    all_time_best_params = None

    for gen in range(1, 23):
        # Check if all 28 candidates for this generation are already cached on disk
        all_cached = all(
            os.path.exists(os.path.join("logs", "candidates_multisine", f"W{i+1}_MS_G{gen}.json"))
            for i in range(28)
        )

        t_gen_start = time.time()
        print(f"\n==================== STARTING MULTISINE GENERATION {gen} ====================", flush=True)
        solutions = es.ask()

        # In Generation 5, inject the best Adaptive Controller parameter vector into slot 0 if not cached
        if gen == 5 and not all_cached:
            afp = best_adapt_summary["final_parameters_15d"]
            cand_adapt = [
                float(afp["adapt_K_ton"]),
                float(afp["adapt_p_ton"]),
                float(afp["adapt_K_sc"]),
                float(np.clip(afp["adapt_z_sc"] / max(1.0, afp["adapt_p_sc"]), 0.002, 0.59)),
                float(afp["adapt_p_sc"]),
                float(afp["adapt_K_phas"]),
                float(afp["adapt_p_phas"]),
                float(afp["adapt_kp_prop"]),
                float(np.clip(afp["adapt_z_ccr"] / max(1.0, afp["adapt_p_ccr"]), 0.002, 0.49)),
                float(afp["adapt_p_ccr"]),
                float(afp["adapt_K_gamma_dyn"]),
                float(afp["adapt_K_gamma_stat"]),
                float(afp["adapt_Kp_task_pitch"]),
                float(afp["adapt_Kd_task_pitch"]),
                float(afp["adapt_Ki_task_pitch"]),
            ]
            solutions[0] = np.array(cand_adapt, dtype=float)

        # In Generation 23, inject convex & frequency-selective hybrids of W16_MS_G22 and Final_Adapt_S1_from_W16_MS_G22
        if gen == 23 and not all_cached and os.path.exists("logs/adaptive_final_polish_s1.json"):
            with open("logs/adaptive_final_polish_s1.json", "r", encoding="utf-8") as f_s1:
                s1_data = json.load(f_s1)
            afp = s1_data["final_parameters_15d"]
            p_adapt = np.array([
                float(afp["adapt_K_ton"]),
                float(afp["adapt_p_ton"]),
                float(afp["adapt_K_sc"]),
                float(np.clip(afp["adapt_z_sc"] / max(1.0, afp["adapt_p_sc"]), 0.002, 0.59)),
                float(afp["adapt_p_sc"]),
                float(afp["adapt_K_phas"]),
                float(afp["adapt_p_phas"]),
                float(afp["adapt_kp_prop"]),
                float(np.clip(afp["adapt_z_ccr"] / max(1.0, afp["adapt_p_ccr"]), 0.002, 0.49)),
                float(afp["adapt_p_ccr"]),
                float(afp["adapt_K_gamma_dyn"]),
                float(afp["adapt_K_gamma_stat"]),
                float(afp["adapt_Kp_task_pitch"]),
                float(afp["adapt_Kd_task_pitch"]),
                float(afp["adapt_Ki_task_pitch"]),
            ], dtype=float)
            p_champ = np.array(all_time_best_params, dtype=float)
            alphas = [0.08, 0.14, 0.20, 0.26, 0.32, 0.40, 0.50]
            for s_idx, a_val in enumerate(alphas):
                solutions[s_idx] = (1.0 - a_val) * p_champ + a_val * p_adapt
            # Selective hybrid 1: Keep W16_MS_G22's K_phas & Ki (to preserve 2.446 Hz = 63.3 pts), take 30% of Adapt S1's reflex/postural shaping
            hyb1 = 0.70 * p_champ + 0.30 * p_adapt
            hyb1[5] = p_champ[5]   # K_phas from W16_MS_G22
            hyb1[14] = p_champ[14] # Ki from W16_MS_G22
            solutions[7] = hyb1
            # Selective hybrid 2: 15% K_phas/Ki shift + 35% reflex shaping
            hyb2 = 0.65 * p_champ + 0.35 * p_adapt
            hyb2[5] = 0.85 * p_champ[5] + 0.15 * p_adapt[5]
            hyb2[14] = 0.85 * p_champ[14] + 0.15 * p_adapt[14]
            solutions[8] = hyb2

        scores = [0.0] * len(solutions)
        if all_cached:
            for i in range(len(solutions)):
                fn = os.path.join("logs", "candidates_multisine", f"W{i+1}_MS_G{gen}.json")
                with open(fn, "r", encoding="utf-8") as f_res:
                    d_res = json.load(f_res)
                scores[i] = float(d_res["total_score"])
        else:
            with ThreadPoolExecutor(max_workers=n_workers) as executor:
                fut_to_idx = {
                    executor.submit(evaluate_multisine_candidate, solutions[i], i, gen): i
                    for i in range(len(solutions))
                }
                for fut in as_completed(fut_to_idx):
                    idx = fut_to_idx[fut]
                    scores[idx] = fut.result()

        es.tell(solutions, scores)

        best_idx = int(np.argmin(scores))
        best_score = float(scores[best_idx])
        best_worker = f"W{best_idx+1}_MS_G{gen}"
        dur_min = (time.time() - t_gen_start) / 60.0

        pass_scores = [s for s in scores if s < 150000.0]
        n_pass = len(pass_scores)
        mean_pass = float(np.mean(pass_scores)) if pass_scores else float("nan")
        med_pass = float(np.median(pass_scores)) if pass_scores else float("nan")

        if best_score < all_time_best_score:
            all_time_best_score = best_score
            all_time_best_worker = best_worker
            all_time_best_params = [float(v) for v in solutions[best_idx]]

        champ_file = os.path.join("logs", "candidates_multisine", f"{best_worker}.json")
        with open(champ_file, "r", encoding="utf-8") as f_c:
            champ_json = json.load(f_c)

        if best_score <= all_time_best_score:
            with open("logs/multisine_best_champion.json", "w", encoding="utf-8") as f_best:
                json.dump(champ_json, f_best, indent=2)

        gen_records.append({
            "gen": gen,
            "best_worker": best_worker,
            "best_score": best_score,
            "duration_min": dur_min,
            "n_pass": n_pass,
            "mean_pass": mean_pass,
            "median_pass": med_pass,
            "champ_json": champ_json,
        })
        update_multisine_ledger(gen_records)

        print(
            f"[MULTISINE GEN {gen} COMPLETE] Best: {best_worker} = {best_score:,.2f} pts (All-Time Best: {all_time_best_worker} = {all_time_best_score:,.2f} pts) | "
            f"Pass: {n_pass}/28 | Mean Pass: {mean_pass:,.2f} | Median: {med_pass:,.2f} | Duration: {dur_min:.2f} min",
            flush=True
        )

    # -------------------------------------------------------------------------
    # PHASE 3: Final Adaptive Polish on All-Time Multisine Champion & Final Summary (until 18:06)
    # -------------------------------------------------------------------------
    print("\n" + "=" * 96, flush=True)
    print(f"[PHASE 3] Final Multi-Stage Adaptive Polish on All-Time CMA-ES Champion {all_time_best_worker} ({all_time_best_score:,.2f} pts)", flush=True)
    print("=" * 96, flush=True)

    best_champ_path = os.path.join("logs", "candidates_multisine", f"{all_time_best_worker}.json")
    with open(best_champ_path, "r", encoding="utf-8") as f_bc:
        bc_data = json.load(f_bc)

    bc_p15 = [float(x) for x in bc_data["params"]]
    bc_freqs = bc_data["frequencies"]
    bc_gains = [float(bc_freqs[fk]["sim_gain"]) for fk in ["0.324", "0.783", "1.099", "2.446", "5.583", "7.927"]]
    bc_phases = [float(bc_freqs[fk]["sim_phase"]) for fk in ["0.324", "0.783", "1.099", "2.446", "5.583", "7.927"]]

    if os.path.exists("logs/adaptive_final_polish_s1.json"):
        with open("logs/adaptive_final_polish_s1.json", "r", encoding="utf-8") as f_s1c:
            final_adapt_s1 = json.load(f_s1c)
        if final_adapt_s1["post_adaptation_window_score"] < best_adapt_score:
            best_adapt_score = final_adapt_s1["post_adaptation_window_score"]
            best_adapt_summary = final_adapt_s1
            shutil.copyfile("logs/adaptive_final_polish_s1.json", "logs/adaptive_updated_15params_trajectory.json")
            shutil.copyfile("logs/adaptive_final_polish_s1.png", "logs/adaptive_updated_15params_convergence.png")
    elif time_left_until(18, 6) > 8.0 * 60.0:
        final_adapt_s1 = run_single_adaptive_stage(
            seed_p15=bc_p15,
            base_gains=bc_gains,
            base_phases=bc_phases,
            base_ms_score=float(bc_data["total_score"]),
            pre_freq_results=bc_freqs,
            stage_label=f"Final_Adapt_S1_from_{all_time_best_worker}",
            out_json_path="logs/adaptive_final_polish_s1.json",
            out_png_path="logs/adaptive_final_polish_s1.png",
            adapt_mult=0.35,
            sigma_leak=0.018
        )
        if final_adapt_s1["post_adaptation_window_score"] < best_adapt_score:
            best_adapt_score = final_adapt_s1["post_adaptation_window_score"]
            best_adapt_summary = final_adapt_s1
            shutil.copyfile("logs/adaptive_final_polish_s1.json", "logs/adaptive_updated_15params_trajectory.json")
            shutil.copyfile("logs/adaptive_final_polish_s1.png", "logs/adaptive_updated_15params_convergence.png")

    if os.path.exists("logs/adaptive_final_polish_s2.json"):
        with open("logs/adaptive_final_polish_s2.json", "r", encoding="utf-8") as f_s2c:
            final_adapt_s2 = json.load(f_s2c)
        if final_adapt_s2["post_adaptation_window_score"] < best_adapt_score:
            best_adapt_score = final_adapt_s2["post_adaptation_window_score"]
            best_adapt_summary = final_adapt_s2
            shutil.copyfile("logs/adaptive_final_polish_s2.json", "logs/adaptive_updated_15params_trajectory.json")
            shutil.copyfile("logs/adaptive_final_polish_s2.png", "logs/adaptive_updated_15params_convergence.png")

    master_report = {
        "completed_at": datetime.now().isoformat(),
        "all_time_best_multisine_cmaes_worker": all_time_best_worker,
        "all_time_best_multisine_cmaes_score": all_time_best_score,
        "all_time_best_adaptive_stage": best_adapt_summary["anchor_label"],
        "all_time_best_adaptive_score": best_adapt_score,
    }
    with open("logs/final_1810_master_summary.json", "w", encoding="utf-8") as f_m:
        json.dump(master_report, f_m, indent=2)

    print("\n" + "=" * 96, flush=True)
    print(f"ALL TRAINING COMPLETE AT {datetime.now().strftime('%H:%M:%S')}!", flush=True)
    print(f"  -> Best Multisine CMA-ES Champion: {all_time_best_worker} = {all_time_best_score:,.2f} pts", flush=True)
    print(f"  -> Best 15-Param Adaptive Controller: {best_adapt_summary['anchor_label']} = {best_adapt_score:,.2f} pts", flush=True)
    print("=" * 96, flush=True)


if __name__ == "__main__":
    main()
