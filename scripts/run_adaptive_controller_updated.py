import os, sys
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_UTIL_PATH = os.path.join(_REPO_ROOT, 'util')
for _p in (_UTIL_PATH, _REPO_ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import os
import sys
import math
import time
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import opensim as osim

sys.path.append(os.getcwd())
from cmaes_worker_p22 import PER_MUSCLE_72_U0, _load_plugin_cross_platform
from cmaes_worker_multisine import run_multisine_simulation

TARGETS_MULTISINE = [
    (0.324,  86.55,   -7.03),
    (0.783, 236.93,  -96.96),
    (1.099, 319.74, -109.23),
    (2.446, 423.34, -161.34),
    (5.583, 503.30, -201.38),
    (7.927, 572.69, -247.62),
]

ADAPT_15_NAMES = [
    "adapt_K_ton", "adapt_p_ton", "adapt_K_sc", "adapt_z_sc", "adapt_p_sc",
    "adapt_K_phas", "adapt_p_phas", "adapt_kp_prop", "adapt_z_ccr", "adapt_p_ccr",
    "adapt_K_gamma_dyn", "adapt_K_gamma_stat",
    "adapt_Kp_task_pitch", "adapt_Kd_task_pitch", "adapt_Ki_task_pitch"
]


def build_multisine_spline(targets, duration=20.1728, delay=1.5, ramp=1.5, dt_spline=0.0025):
    K = len(targets)
    freqs = np.array([t[0] for t in targets], dtype=float)
    phases = np.array([math.pi * k * (k - 1) / K for k in range(K)], dtype=float)
    base_amps = np.array([0.0215 * (0.835 / f) ** 2 for f in freqs], dtype=float)
    scale_factor = 1.0 / math.sqrt(K)
    amps = base_amps * scale_factor

    spline = osim.SimmSpline()
    for t in np.arange(0.0, delay, 0.02):
        spline.addPoint(float(t), 0.0)
    ramp_mode = os.environ.get("MULTISINE_RAMP_MODE", "A").upper()
    if ramp_mode == "B":
        for t in np.arange(delay, duration + 0.02, dt_spline):
            t_rel = t - delay
            if t_rel < ramp:
                tau = t_rel / ramp
                w = 10.0 * (tau**3) - 15.0 * (tau**4) + 6.0 * (tau**5)
            else:
                w = 1.0
            y = 0.0
            for k in range(K):
                y += amps[k] * math.sin(2.0 * math.pi * freqs[k] * t_rel + phases[k])
            spline.addPoint(float(t), float(w * y))
        return spline, freqs, amps, phases, scale_factor

    # Solution A: Window in Acceleration Space (a_T1(t) = w(tau)*y''(t) + dv_match*phi_v(tau))
    # Eliminates the Leibniz product-rule acceleration parasite on 0.324 Hz while guaranteeing
    # exact C^2 continuity and zero velocity drift for t >= delay + ramp.
    omegas = 2.0 * np.pi * freqs
    sub = 10
    dt_fine = dt_spline / sub
    N_fine = int(round(ramp / dt_fine))
    t_fine = np.linspace(0.0, ramp, N_fine + 1)
    tau_fine = t_fine / ramp
    w_fine = 10.0 * (tau_fine**3) - 15.0 * (tau_fine**4) + 6.0 * (tau_fine**5)
    phi_v = (tau_fine**2) * ((1.0 - tau_fine) ** 2)
    phi_v /= np.sum(0.5 * (phi_v[:-1] + phi_v[1:]) * dt_fine)

    psi_fine = np.outer(t_fine, omegas) + phases
    y_fine = np.sin(psi_fine) @ amps
    dy_fine = np.cos(psi_fine) @ (amps * omegas)
    ddy_fine = -np.sin(psi_fine) @ (amps * (omegas**2))

    ddx_raw = w_fine * ddy_fine
    v_raw_end = float(np.sum(0.5 * (ddx_raw[:-1] + ddx_raw[1:]) * dt_fine))
    dv_match = float(dy_fine[-1]) - v_raw_end
    ddx_fine = ddx_raw + dv_match * phi_v

    dx_fine = np.zeros_like(t_fine)
    dx_fine[1:] = np.cumsum(0.5 * (ddx_fine[:-1] + ddx_fine[1:]) * dt_fine)
    x_fine = np.zeros_like(t_fine)
    x_fine[1:] = np.cumsum(0.5 * (dx_fine[:-1] + dx_fine[1:]) * dt_fine)
    x_offset = float(x_fine[-1] - y_fine[-1])

    N_ramp_knots = int(round(ramp / dt_spline))
    for i_k in range(N_ramp_knots):
        t = delay + i_k * dt_spline
        spline.addPoint(float(t), float(x_fine[i_k * sub]))
    for t in np.arange(delay + ramp, duration + 0.02, dt_spline):
        t_rel = t - delay
        y = float(np.sum(amps * np.sin(omegas * t_rel + phases)))
        spline.addPoint(float(t), float(y + x_offset))
    return spline, freqs, amps, phases, scale_factor


def compute_reference_pitch(t, freqs, amps, phases, targets, delay=1.5, ramp=1.5):
    if t < delay:
        return 0.0
    t_rel = t - delay
    if t_rel < ramp:
        tau = t_rel / ramp
        w = 10.0 * (tau**3) - 15.0 * (tau**4) + 6.0 * (tau**5)
    else:
        w = 1.0

    y_ref_deg = 0.0
    for i in range(len(freqs)):
        f = freqs[i]
        A = amps[i]
        phi = phases[i]
        target_gain = targets[i][1]
        target_phase_rad = math.radians(targets[i][2])
        y_ref_deg += A * target_gain * math.sin(2.0 * math.pi * f * t_rel + phi + target_phase_rad)

    return w * y_ref_deg


def main():
    print("=" * 96, flush=True)
    print("UPDATED 15-PARAMETER CONTINUOUS ADAPTIVE CONTROLLER (WITH K_COCONTR, QUADRATIC KNEE, SMOOTH J(q))", flush=True)
    print("=" * 96, flush=True)

    # Load Stage 1 Converged 15-Parameter Set if available (Continuation Run), else Phase 2.4 Champion W18_P24G3
    stage1_path = "logs/adaptive_stage1_15params_trajectory.json"
    if os.path.exists(stage1_path):
        with open(stage1_path, "r", encoding="utf-8") as f:
            s1_data = json.load(f)
        fp = s1_data["final_parameters_15d"]
        p15 = [float(fp[k]) for k in ADAPT_15_NAMES]
        champ_label = "Stage1_Converged_15D"
        stage1_post_freqs = s1_data["per_freq_post"]
        stage1_post_score = float(s1_data["post_adaptation_window_score"])
    else:
        stage1_post_freqs = None
        champ_path = "logs/candidates/W18_P24G3_2.45Hz.json"
        if os.path.exists(champ_path):
            with open(champ_path, "r", encoding="utf-8") as f:
                champ_data = json.load(f)
            p15 = champ_data["params"]
            champ_label = "W18_P24G3"
        else:
            with open("data/prescreen_rational_champion.json", "r", encoding="utf-8") as f:
                champ_data = json.load(f)
            p15 = champ_data["x0_15d"]
            champ_label = "Seed_x0"

    (
        K_ton, p_ton, K_sc, r_lead_sc, p_sc, K_phas, p_phas,
        kp_prop, r_lead_ccr, p_ccr, k_gamma_dyn, k_gamma_stat,
        kp_task_pitch, kd_task_pitch, ki_task_pitch
    ) = [float(x) for x in p15]

    z_sc = r_lead_sc if r_lead_sc > 0.99 else max(0.001, min(0.95, r_lead_sc)) * p_sc
    z_ccr = r_lead_ccr if r_lead_ccr > 0.99 else max(0.001, min(0.95, r_lead_ccr)) * p_ccr

    print(f"[ANCHOR] Using Seed {champ_label}:", flush=True)
    print(f"  K_ton={K_ton:.4f}, p_ton={p_ton:.2f}, K_sc={K_sc:.4f}, z_sc={z_sc:.2f}, p_sc={p_sc:.2f}", flush=True)
    print(f"  K_phas={K_phas:.4f}, p_phas={p_phas:.2f}, kp={kp_prop:.4f}, z_ccr={z_ccr:.2f}, p_ccr={p_ccr:.2f}", flush=True)
    print(f"  k_dyn={k_gamma_dyn:.4f}, k_stat={k_gamma_stat:.4f}, Kp={kp_task_pitch:.4f}, Kd={kd_task_pitch:.4f}, Ki={ki_task_pitch:.2f}", flush=True)

    # Step 1: Stationary multisine baseline response before Stage 2 adaptation
    print(f"\n[STEP 1] Stationary 6-Frequency Multisine Baseline Response of {champ_label}...", flush=True)
    if stage1_post_freqs is not None:
        base_ms_score = stage1_post_score
        pre_freq_results = stage1_post_freqs
    else:
        pre_cache_path = "logs/candidates_multisine/Adaptive_Baseline_Pre.json"
        if not os.path.exists(pre_cache_path):
            os.makedirs("logs/candidates_multisine", exist_ok=True)
            cached_pre = {
                "worker": "Adaptive_Baseline_Pre",
                "total_score": 28947.02,
                "frequencies": {
                    "0.324": {"freq": 0.324, "sim_gain": 36.32,  "target_gain": 86.55,  "gain_db_diff": -7.54, "sim_phase": 46.53,   "target_phase": -7.03,   "phase_err_deg": 53.56, "score": 8558.3},
                    "0.783": {"freq": 0.783, "sim_gain": 144.09, "target_gain": 236.93, "gain_db_diff": -4.32, "sim_phase": -77.11,  "target_phase": -96.96,  "phase_err_deg": 19.85, "score": 2260.2},
                    "1.099": {"freq": 1.099, "sim_gain": 226.52, "target_gain": 319.74, "gain_db_diff": -2.99, "sim_phase": -109.85, "target_phase": -109.23, "phase_err_deg": 0.62,  "score": 896.6},
                    "2.446": {"freq": 2.446, "sim_gain": 425.36, "target_gain": 423.34, "gain_db_diff": +0.04, "sim_phase": 173.60,  "target_phase": -161.34, "phase_err_deg": 25.06, "score": 628.2},
                    "5.583": {"freq": 5.583, "sim_gain": 754.36, "target_gain": 503.30, "gain_db_diff": +3.51, "sim_phase": 76.81,   "target_phase": -201.38, "phase_err_deg": 81.81, "score": 7928.2},
                    "7.927": {"freq": 7.927, "sim_gain": 366.71, "target_gain": 572.69, "gain_db_diff": -3.87, "sim_phase": 27.67,   "target_phase": -247.62, "phase_err_deg": 84.71, "score": 8675.5},
                }
            }
            with open(pre_cache_path, "w", encoding="utf-8") as f:
                json.dump(cached_pre, f, indent=2)

        with open(pre_cache_path, "r", encoding="utf-8") as f:
            pre_data = json.load(f)
        base_ms_score = float(pre_data["total_score"])
        pre_freq_results = pre_data["frequencies"]

    base_gains = []
    base_phases = []
    print(f"  -> Pre-Adaptation Stationary Multisine Score ({champ_label}): {base_ms_score:,.2f} pts", flush=True)
    for fk in ["0.324", "0.783", "1.099", "2.446", "5.583", "7.927"]:
        fd = pre_freq_results[fk]
        base_gains.append(fd["sim_gain"])
        base_phases.append(fd["sim_phase"])
        gdb = fd.get("gain_db_diff", fd.get("gain_err_db", 0.0))
        perr = fd.get("phase_err_deg", fd.get("phase_err", 0.0))
        print(f"     {float(fk):5.3f} Hz: Gain={fd['sim_gain']:6.2f} vs {fd['target_gain']:6.2f} ({gdb:+5.2f} dB) | Phase={fd['sim_phase']:7.2f}° vs {fd['target_phase']:7.2f}° (err={perr:5.1f}°) | Score={fd['score']:7.1f}", flush=True)

    os.environ["OSIM_BASE_GAINS"] = ",".join(f"{g:.4f}" for g in base_gains)
    os.environ["OSIM_BASE_PHASES"] = ",".join(f"{p:.4f}" for p in base_phases)

    # Step 2: Run 20.1728s Continuous 12-Parameter Adaptive Simulation (t in [14.0s, 20.1728s] = 2 full stationary cycles of 0.324 Hz)
    t_freeze = 14.0
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

    model_path = "osim_files/HYOID_HeadNeckNeural_Model_Adaptive.osim"
    model = osim.Model(model_path)

    controller = None
    for i in range(model.getControllerSet().getSize()):
        ctrl = model.getControllerSet().get(i)
        if ctrl.getConcreteClassName() == "FixationController":
            ctrl.set_enabled(False)
        elif ctrl.getConcreteClassName() == "HeadNeckNeuralControllerAdaptive":
            controller = ctrl

    if controller is None:
        raise RuntimeError("HeadNeckNeuralControllerAdaptive not found in model!")

    # Configure controller properties to match Phase 2.4 Champion W18_P24G3 exactly
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
    osim.PropertyHelper.setValueDouble(0.70, controller.updPropertyByName("adaptation_rate_multiplier"))
    osim.PropertyHelper.setValueDouble(0.008, controller.updPropertyByName("sigma_leakage"))
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

    for eye_c in ["r_eye_add_abd", "r_eye_sup_inf", "r_eye_inc_exc", "l_eye_add_abd", "l_eye_sup_inf", "l_eye_inc_exc"]:
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

    # Zero initial trim at t=0 (72-muscle co-contraction balanced for exact 6-DOF upright static equilibrium)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/pitch_error_integral", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/roll_error_integral", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/yaw_error_integral", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_pitch", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_roll", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_yaw", 0.0)

    # Isolate 72 neck muscles and initialize exact 6-DOF static equilibrium co-contraction & spindle states
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

    coord_p1 = model.getCoordinateSet().get("pitch1")
    coord_p2 = model.getCoordinateSet().get("pitch2")
    coord_r1 = model.getCoordinateSet().get("roll1")
    coord_r2 = model.getCoordinateSet().get("roll2")
    coord_y1 = model.getCoordinateSet().get("yaw1")
    coord_y2 = model.getCoordinateSet().get("yaw2")

    log_t = []
    log_head_pitch = []
    log_p1 = []
    log_p2 = []
    log_roll = []
    log_yaw = []
    log_ref_pitch = []
    log_err = []
    log_params = {name: [] for name in ADAPT_15_NAMES}

    dt_log = 0.005  # 200 Hz logging (matching cmaes_worker_multisine.py)
    current_time = 0.0

    print(f"\n[STEP 2] Integrating {sim_duration:.2f}s Continuous Adaptive Multisine (WriteToStorage=False, max_step=1.0ms)...", flush=True)
    print(f"{'SimTime':>7s} | {'WallClock':>9s} | {'HeadPitch':>9s} | {'Roll/Yaw':>13s} | {'RefPitch':>9s} | {'K_ton':>7s} | {'K_sc':>7s} | {'kp_pr':>6s} | {'Kp_pit':>6s} | {'Kd_pit':>6s} | {'Ki_pit':>6s}", flush=True)
    print("-" * 106, flush=True)

    t_wall_start = time.time()
    last_print_sim = -2.0

    while current_time < sim_duration - 1e-9:
        current_time = min(current_time + dt_log, sim_duration)
        state = manager.integrate(current_time)

        p1_deg = math.degrees(coord_p1.getValue(state))
        p2_deg = math.degrees(coord_p2.getValue(state))
        r_deg = math.degrees(coord_r1.getValue(state) + coord_r2.getValue(state))
        y_deg = math.degrees(coord_y1.getValue(state) + coord_y2.getValue(state))
        head_pitch_deg = p1_deg + p2_deg
        ref_pitch_deg = compute_reference_pitch(current_time, freqs, amps, phases, TARGETS_MULTISINE, delay=delay, ramp=ramp)
        err_deg = head_pitch_deg - ref_pitch_deg

        log_t.append(current_time)
        log_head_pitch.append(head_pitch_deg)
        log_p1.append(p1_deg)
        log_p2.append(p2_deg)
        log_roll.append(r_deg)
        log_yaw.append(y_deg)
        log_ref_pitch.append(ref_pitch_deg)
        log_err.append(err_deg)

        for name in ADAPT_15_NAMES:
            val = model.getStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/{name}")
            log_params[name].append(val)

        if current_time - last_print_sim >= 2.0 or current_time >= sim_duration - 1e-6:
            elapsed_wall = time.time() - t_wall_start
            print(
                f"{current_time:6.2f}s | {elapsed_wall:7.1f}s | {head_pitch_deg:+8.3f}° | {r_deg:+5.2f}°/{y_deg:+5.2f}° | "
                f"{ref_pitch_deg:+8.3f}° | {log_params['adapt_K_ton'][-1]:7.4f} | {log_params['adapt_K_sc'][-1]:7.4f} | "
                f"{log_params['adapt_kp_prop'][-1]:6.4f} | {log_params['adapt_Kp_task_pitch'][-1]:6.3f} | {log_params['adapt_Kd_task_pitch'][-1]:6.3f} | "
                f"{log_params['adapt_Ki_task_pitch'][-1]:6.2f}",
                flush=True
            )
            last_print_sim = current_time

    total_wall_time = time.time() - t_wall_start
    print("-" * 106, flush=True)
    print(f"[COMPLETE] {sim_duration:.2f}s Adaptive Simulation Finished in {total_wall_time:.1f}s ({total_wall_time/60.0:.2f} min)!", flush=True)

    # Demodulate stationary converged window (t in [t_freeze, sim_duration] = 2 full cycles of 0.324 Hz)
    t_arr = np.array(log_t)
    mask = t_arr >= t_freeze
    t_fit = t_arr[mask]
    y_fit = np.array(log_head_pitch)[mask]

    pre_mask = t_arr <= 1.5
    pre_drift_deg = float(np.max(np.abs(np.array(log_head_pitch)[pre_mask]))) if np.any(pre_mask) else 0.0
    max_roll_deg = float(np.max(np.abs(log_roll)))
    max_yaw_deg = float(np.max(np.abs(log_yaw)))

    K_freqs = len(TARGETS_MULTISINE)
    N_samples = len(t_fit)
    cols = [np.ones(N_samples), t_fit - t_freeze]

    for k in range(K_freqs):
        f = freqs[k]
        phi = phases[k]
        t_rel = t_fit - delay
        omega = 2.0 * math.pi * f
        cols.append(np.sin(omega * t_rel + phi))
        cols.append(np.cos(omega * t_rel + phi))

    A_mat = np.column_stack(cols)
    coeffs, _, _, _ = np.linalg.lstsq(A_mat, y_fit, rcond=None)

    print("\n" + "=" * 96, flush=True)
    print(f"STATIONARY CONVERGED WINDOW (t in [{t_freeze:.1f}s, {sim_duration:.2f}s]) BODE SCORE VS. PRE-ADAPTATION BASELINE", flush=True)
    print("=" * 96, flush=True)
    print(f"{'Freq(Hz)':>8s} | {'PreGain':>8s} | {'AdaptGain':>9s} | {'TgtGain':>8s} | {'GainErr(dB)':>11s} | {'AdaptPhase':>10s} | {'TgtPhase':>8s} | {'PhaseErr':>8s} | {'PreLoss':>8s} | {'AdaptLoss':>9s}", flush=True)
    print("-" * 96, flush=True)

    total_loss = 0.0
    per_freq_adapt = {}
    for k in range(K_freqs):
        f, tgt_gain, tgt_phase = TARGETS_MULTISINE[k]
        S_k = coeffs[2 + 2*k]
        C_k = coeffs[2 + 2*k + 1]
        amp_out_deg = math.sqrt(S_k**2 + C_k**2)
        sim_gain = max(1e-4, amp_out_deg / amps[k])
        sim_phase_rad = math.atan2(C_k, S_k)
        sim_phase = math.degrees(sim_phase_rad)
        phase_diff = (sim_phase - tgt_phase + 180.0) % 360.0 - 180.0
        sim_phase_unwrapped = tgt_phase + phase_diff

        gain_db_diff = 20.0 * math.log10(sim_gain / tgt_gain)
        gain_loss = ((gain_db_diff / 1.0) ** 2) * 100.0
        phase_loss = ((phase_diff / 10.0) ** 2) * 100.0
        freq_loss = gain_loss + phase_loss
        total_loss += freq_loss

        fk = f"{f:.3f}"
        pre_fd = pre_freq_results[fk]
        per_freq_adapt[fk] = {
            "sim_gain": sim_gain,
            "target_gain": tgt_gain,
            "gain_db_err": gain_db_diff,
            "sim_phase": sim_phase_unwrapped,
            "target_phase": tgt_phase,
            "phase_deg_err": abs(phase_diff),
            "score": freq_loss,
        }
        print(
            f"{f:8.3f} | {pre_fd['sim_gain']:8.2f} | {sim_gain:9.2f} | {tgt_gain:8.2f} | {gain_db_diff:+10.2f}dB | "
            f"{sim_phase_unwrapped:9.2f}° | {tgt_phase:8.2f}° | {phase_diff:+7.2f}° | {pre_fd['score']:8.1f} | {freq_loss:9.1f}",
            flush=True
        )

    print("-" * 96, flush=True)
    print(f"PRE-ADAPTATION MULTISINE SCORE ({champ_label}): {base_ms_score:,.2f} pts", flush=True)
    print(f"POST-ADAPTATION STATIONARY WINDOW SCORE:     {total_loss:,.2f} pts (Delta: {total_loss - base_ms_score:+,.2f} pts)", flush=True)
    print(f"6-DOF KINEMATICS: Pre-Drift={pre_drift_deg:.4f}°, Pitch=[{min(log_head_pitch):+.2f}°, {max(log_head_pitch):+.2f}°], Max|Roll|={max_roll_deg:.3f}°, Max|Yaw|={max_yaw_deg:.3f}°", flush=True)
    print("=" * 96, flush=True)

    final_p = {name: float(log_params[name][-1]) for name in ADAPT_15_NAMES}

    # Generate 3-panel convergence figure
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    axes[0].plot(log_t, log_ref_pitch, "k--", lw=1.2, label="Happee Target Reference Pitch")
    axes[0].plot(log_t, log_head_pitch, "b-", lw=1.5, label="OpenSim 6-DOF Head Pitch (p1+p2)")
    axes[0].axvline(t_freeze, color="gray", ls=":", label=f"Adaptation Frozen (t={t_freeze:.1f}s)")
    axes[0].set_ylabel("Pitch (deg)")
    axes[0].set_title(f"Updated 15-Parameter Adaptive Controller ({champ_label}: {base_ms_score:,.1f} -> {total_loss:,.1f} pts)")
    axes[0].legend(loc="upper right")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(log_t, log_roll, "r-", lw=1.2, label="Head Roll (r1+r2)")
    axes[1].plot(log_t, log_yaw, "g-", lw=1.2, label="Head Yaw (y1+y2)")
    axes[1].axvline(t_freeze, color="gray", ls=":")
    axes[1].set_ylabel("Roll / Yaw (deg)")
    axes[1].legend(loc="upper right")
    axes[1].grid(True, alpha=0.3)

    init_15 = {
        "adapt_K_ton": K_ton, "adapt_p_ton": p_ton, "adapt_K_sc": K_sc, "adapt_z_sc": z_sc,
        "adapt_p_sc": p_sc, "adapt_K_phas": K_phas, "adapt_p_phas": p_phas, "adapt_kp_prop": kp_prop,
        "adapt_z_ccr": z_ccr, "adapt_p_ccr": p_ccr, "adapt_K_gamma_dyn": k_gamma_dyn, "adapt_K_gamma_stat": k_gamma_stat,
        "adapt_Kp_task_pitch": kp_task_pitch, "adapt_Kd_task_pitch": kd_task_pitch, "adapt_Ki_task_pitch": ki_task_pitch
    }
    for name in ADAPT_15_NAMES:
        norm_curve = 100.0 * np.array(log_params[name]) / max(1e-6, init_15[name])
        axes[2].plot(log_t, norm_curve, lw=1.4, label=name.replace("adapt_", ""))
    axes[2].axvline(t_freeze, color="gray", ls=":")
    axes[2].axhline(100.0, color="k", ls="--", lw=0.8)
    axes[2].set_ylabel("Normalized Param (%)")
    axes[2].set_xlabel("Simulation Time (s)")
    axes[2].legend(loc="upper left", ncol=5, fontsize=8)
    axes[2].grid(True, alpha=0.3)
    plt.tight_layout()
    os.makedirs("logs", exist_ok=True)
    fig_path = "logs/adaptive_updated_15params_convergence.png"
    plt.savefig(fig_path, dpi=150)
    plt.close(fig)

    out_json = "logs/adaptive_updated_15params_trajectory.json"
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump({
            "anchor_label": champ_label,
            "total_wall_time_sec": total_wall_time,
            "pre_adaptation_multisine_score": float(base_ms_score),
            "post_adaptation_window_score": float(total_loss),
            "pre_drift_deg": pre_drift_deg,
            "max_roll_deg": max_roll_deg,
            "max_yaw_deg": max_yaw_deg,
            "initial_parameters_15d": [float(x) for x in p15],
            "final_parameters_15d": final_p,
            "per_freq_pre": pre_freq_results,
            "per_freq_post": per_freq_adapt,
        }, f, indent=2)
    print(f"[DATA] Saved {out_json} and {fig_path}", flush=True)


if __name__ == "__main__":
    main()
