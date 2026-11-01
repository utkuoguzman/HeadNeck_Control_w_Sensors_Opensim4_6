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

ADAPT_12_NAMES = [
    "adapt_K_ton", "adapt_p_ton", "adapt_K_sc", "adapt_z_sc", "adapt_p_sc",
    "adapt_K_phas", "adapt_p_phas", "adapt_kp_prop", "adapt_z_ccr", "adapt_p_ccr",
    "adapt_K_gamma_dyn", "adapt_K_gamma_stat"
]

def create_multisine_spline(targets, duration=30.0, delay=1.5, ramp=1.0):
    K = len(targets)
    freqs = np.array([t[0] for t in targets])
    phases = np.array([math.pi * k * (k - 1) / K for k in range(K)])
    base_amps = np.array([0.0215 * (0.835 / f)**2 for f in freqs])
    scale_factor = 0.35 / math.sqrt(K) # 0.142887
    amps = base_amps * scale_factor

    dt = 0.005 # 200 Hz grid for spline
    t_arr = np.arange(0.0, duration + 0.1, dt)
    spline = osim.SimmSpline()
    
    for t in t_arr:
        if t < delay:
            spline.addPoint(t, 0.0)
        else:
            t_rel = t - delay
            if t_rel < ramp:
                tau = t_rel / ramp
                w = 10.0 * (tau**3) - 15.0 * (tau**4) + 6.0 * (tau**5)
            else:
                w = 1.0
            
            y = 0.0
            for i in range(K):
                y += amps[i] * math.sin(2.0 * math.pi * freqs[i] * t_rel + phases[i])
            spline.addPoint(t, float(w * y))
            
    return spline, freqs, amps, phases, scale_factor

def compute_reference_pitch(t, freqs, amps, phases, targets, delay=1.5, ramp=1.0):
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
    print("=" * 88, flush=True)
    print("PATH 1: FULL 12-PARAMETER CONTINUOUS STATE-VARIABLE ADAPTIVE CONTROL", flush=True)
    print("Exact 1-to-1 Mirror of CMA-ES Parameter Vector in Native Simbody State Space", flush=True)
    print("=" * 88, flush=True)

    TARGETS = [
        (0.324,  86.55,   -7.03),
        (0.783, 236.93,  -96.96),
        (1.099, 319.74, -109.23),
        (2.446, 423.34, -161.34),
        (5.583, 503.30, -201.38),
        (7.927, 572.69, -247.62)
    ]

    sim_duration = 30.0
    delay = 1.5
    ramp = 1.0
    spline, freqs, amps, phases, scale_factor = create_multisine_spline(
        TARGETS, duration=sim_duration, delay=delay, ramp=ramp
    )
    print(f"[EXCITATION] 6-frequency Schroeder multisine prepared (Scale: {scale_factor:.4f})", flush=True)

    os.environ["SIMB_NUM_THREADS"] = "1"
    os.environ["OMP_NUM_THREADS"] = "1"
    osim.Logger.setLevelString("Warn")

    fix_dll = os.path.abspath("FixationController_plugin/build/Release/osimFixation.dll")
    vest_dll = os.path.abspath("vestibular_plugin/build/Release/osimVestibular.dll")
    prop_dll = os.path.abspath("proprioception_plugin/build/Release/osimMillard12EqWithAff.dll")
    adapt_dll = os.path.abspath("HeadNeckNeuralControllerAdaptive_plugin/build/Release/osimHeadNeckNeuralAdaptive.dll")

    osim.LoadOpenSimLibraryExact(fix_dll)
    osim.LoadOpenSimLibraryExact(vest_dll)
    osim.LoadOpenSimLibraryExact(prop_dll)
    osim.LoadOpenSimLibraryExact(adapt_dll)

    model_path = "osim_files/HYOID_HeadNeckNeural_Model_Adaptive.osim"
    model = osim.Model(model_path)

    # Disable FixationController if present
    for i in range(model.getControllerSet().getSize()):
        ctrl = model.getControllerSet().get(i)
        if ctrl.getConcreteClassName() == "FixationController":
            ctrl.set_enabled(False)

    # Prescribe coordinates
    zero_func = osim.Constant(0.0)
    for coord_name in ["gndpitch", "gndroll", "gndyaw", "spine_ty", "spine_tz"]:
        c = model.updCoordinateSet().get(coord_name)
        c.set_locked(False)
        c.setDefaultIsPrescribed(True)
        c.setPrescribedFunction(zero_func)

    c_yaw1 = model.updCoordinateSet().get("yaw1")
    c_yaw1.set_locked(False)
    c_yaw1.setDefaultIsPrescribed(False)

    coord_tx = model.updCoordinateSet().get("spine_tx")
    coord_tx.set_locked(False)
    coord_tx.setDefaultIsPrescribed(True)
    coord_tx.setPrescribedFunction(spline)

    state = model.initSystem()
    c_yaw1.setIsPrescribed(state, False)
    coord_tx.setSpeedValue(state, 0.0)

    # Orientation initialization
    skull = model.getBodySet().get("skull")
    R_head_to_world = skull.getTransformInGround(state).R()
    q = osim.Rotation(R_head_to_world).convertRotationToQuaternion()
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q0", q.get(0))
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q1", q.get(1))
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q2", q.get(2))
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q3", q.get(3))
    
    ki = 40.2786
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/pitch_error_integral", 0.10528)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/roll_error_integral", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/yaw_error_integral", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_pitch", ki * 0.10528)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_roll", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_yaw", 0.0)

    model.equilibrateMuscles(state)

    manager = osim.Manager(model)
    manager.setIntegratorMethod(5) # RungeKuttaMerson
    manager.setIntegratorAccuracy(1e-4)
    manager.setIntegratorMinimumStepSize(1e-4)
    manager.setIntegratorMaximumStepSize(0.01)

    state.setTime(0.0)
    manager.initialize(state)

    coord_p1 = model.getCoordinateSet().get("pitch1")
    coord_p2 = model.getCoordinateSet().get("pitch2")

    # Logging structures
    log_t = []
    log_head_pitch = []
    log_ref_pitch = []
    log_err = []
    log_params = {name: [] for name in ADAPT_12_NAMES}

    dt_log = 0.01
    current_time = 0.0

    print(f"\n[EXECUTION STARTED] Integrating 30.0s multisine with all 12 continuous parameters...", flush=True)
    print(f"{'SimTime':>7s} | {'WallClock':>9s} | {'HeadPitch':>9s} | {'RefPitch':>9s} | {'Err(deg)':>8s} | {'K_sc':>7s} | {'z_sc':>6s} | {'p_sc':>6s} | {'K_ton':>7s} | {'k_gdyn':>6s}", flush=True)
    print("-" * 92, flush=True)

    t_wall_start = time.time()
    last_print_sim = -1.0

    while current_time < sim_duration:
        current_time = min(current_time + dt_log, sim_duration)
        state = manager.integrate(current_time)

        p1_deg = math.degrees(coord_p1.getValue(state))
        p2_deg = math.degrees(coord_p2.getValue(state))
        head_pitch_deg = p1_deg + p2_deg
        ref_pitch_deg = compute_reference_pitch(current_time, freqs, amps, phases, TARGETS, delay=delay, ramp=ramp)
        err_deg = head_pitch_deg - ref_pitch_deg

        log_t.append(current_time)
        log_head_pitch.append(head_pitch_deg)
        log_ref_pitch.append(ref_pitch_deg)
        log_err.append(err_deg)

        for name in ADAPT_12_NAMES:
            val = model.getStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/{name}")
            log_params[name].append(val)

        # Print progress every 1.0 second of simulation
        if current_time - last_print_sim >= 1.0 or current_time >= sim_duration:
            elapsed_wall = time.time() - t_wall_start
            k_sc_v = log_params["adapt_K_sc"][-1]
            z_sc_v = log_params["adapt_z_sc"][-1]
            p_sc_v = log_params["adapt_p_sc"][-1]
            k_ton_v = log_params["adapt_K_ton"][-1]
            kg_dyn_v = log_params["adapt_K_gamma_dyn"][-1]
            print(f"{current_time:6.2f}s | {elapsed_wall:7.1f}s | {head_pitch_deg:8.3f}° | {ref_pitch_deg:8.3f}° | {err_deg:7.3f}° | {k_sc_v:7.4f} | {z_sc_v:6.2f} | {p_sc_v:6.2f} | {k_ton_v:7.4f} | {kg_dyn_v:6.3f}", flush=True)
            last_print_sim = current_time

    total_wall_time = time.time() - t_wall_start
    print("-" * 92, flush=True)
    print(f"\n[COMPLETE] 30.0s Simulation Finished in {total_wall_time:.1f}s ({total_wall_time/60.0:.2f} min)!", flush=True)

    # Steady-state evaluation over stationary converged window (t >= 18.0s)
    mask = np.array(log_t) >= 18.0
    t_fit = np.array(log_t)[mask]
    y_fit = np.array(log_head_pitch)[mask]

    K_freqs = len(TARGETS)
    N_samples = len(t_fit)
    cols = [np.ones(N_samples), t_fit - 18.0]

    for k in range(K_freqs):
        f = freqs[k]
        phi = phases[k]
        t_rel = t_fit - 1.5
        omega = 2.0 * math.pi * f
        cols.append(np.sin(omega * t_rel + phi))
        cols.append(np.cos(omega * t_rel + phi))

    A_mat = np.column_stack(cols)
    coeffs, _, _, _ = np.linalg.lstsq(A_mat, y_fit, rcond=None)

    print("\n" + "=" * 88, flush=True)
    print("12-PARAMETER CONTINUOUS ADAPTIVE BODE SCORE (PHASE 2.4 ISOTROPIC CORRIDOR)", flush=True)
    print("=" * 88, flush=True)
    print(f"{'Freq(Hz)':>8s} | {'SimGain(deg/m)':>14s} | {'TgtGain':>8s} | {'GainErr(dB)':>11s} | {'SimPhase':>8s} | {'TgtPhase':>8s} | {'PhaseErr':>8s} | {'Loss(pts)':>9s}", flush=True)
    print("-" * 88, flush=True)

    total_loss = 0.0
    for k in range(K_freqs):
        f, tgt_gain, tgt_phase = TARGETS[k]
        S_k = coeffs[2 + 2*k]
        C_k = coeffs[2 + 2*k + 1]
        amp_out_deg = math.sqrt(S_k**2 + C_k**2)
        sim_gain = amp_out_deg / amps[k]
        sim_phase_rad = math.atan2(C_k, S_k)
        sim_phase = math.degrees(sim_phase_rad)
        phase_diff = (sim_phase - tgt_phase + 180.0) % 360.0 - 180.0
        sim_phase_unwrapped = tgt_phase + phase_diff

        gain_db_diff = 20.0 * math.log10(sim_gain / tgt_gain)
        gain_loss = ((gain_db_diff / 1.0) ** 2) * 100.0
        phase_loss = ((phase_diff / 10.0) ** 2) * 100.0
        freq_loss = gain_loss + phase_loss
        total_loss += freq_loss

        print(f"{f:8.3f} | {sim_gain:14.2f} | {tgt_gain:8.2f} | {gain_db_diff:+10.2f}dB | {sim_phase_unwrapped:8.2f}° | {tgt_phase:8.2f}° | {phase_diff:+7.2f}° | {freq_loss:9.1f}", flush=True)

    print("-" * 88, flush=True)
    print(f"TOTAL PHASE 2.4 ISOTROPIC SCORE: {total_loss:,.2f} pts", flush=True)
    print("=" * 88, flush=True)

    # Save JSON log
    os.makedirs("logs", exist_ok=True)
    out_json = "logs/adaptive_12params_trajectory.json"
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump({
            "total_wall_time_sec": total_wall_time,
            "phase24_score": float(total_loss),
            "final_parameters": {name: float(log_params[name][-1]) for name in ADAPT_12_NAMES},
            "time": log_t,
            "head_pitch": log_head_pitch,
            "ref_pitch": log_ref_pitch,
            "error": log_err,
            "params": log_params
        }, f, indent=2)
    print(f"[DATA] Saved {out_json}", flush=True)

    # Generate 5-panel convergence plot
    fig, axes = plt.subplots(5, 1, figsize=(14, 16), sharex=True)

    axes[0].plot(log_t, log_ref_pitch, 'k--', label='Happee Target', linewidth=1.5)
    axes[0].plot(log_t, log_head_pitch, 'b-', label='Actual Head Pitch', linewidth=1.5, alpha=0.85)
    axes[0].axvline(2.5, color='gray', linestyle=':', label='Adaptation Start')
    axes[0].set_ylabel('Pitch (deg)')
    axes[0].set_title(f'12-Parameter Normalized Continuous Adaptive Control (Score: {total_loss:,.1f} pts, Sim: 30s in {total_wall_time/60.0:.1f} min)')
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc='upper right')

    axes[1].plot(log_t, log_err, 'r-', label='Tracking Error e(t)', linewidth=1.2)
    axes[1].axhline(0.0, color='k', linestyle=':', alpha=0.5)
    axes[1].set_ylabel('Error (deg)')
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc='upper right')

    # Panel 2: Normalized Relative Evolution (% of baseline) for all 12 parameters
    theta_0_map = {
        "adapt_K_ton": 0.5800, "adapt_p_ton": 78.0001, "adapt_K_sc": 0.0974, "adapt_z_sc": 21.5985,
        "adapt_p_sc": 90.0050, "adapt_K_phas": 4.5320, "adapt_p_phas": 15.0046, "adapt_kp_prop": 0.3082,
        "adapt_z_ccr": 8.4129, "adapt_p_ccr": 170.6536, "adapt_K_gamma_dyn": 0.7339, "adapt_K_gamma_stat": 1.1482
    }
    short_names = {
        "adapt_K_ton": "K_ton", "adapt_p_ton": "p_ton", "adapt_K_sc": "K_sc", "adapt_z_sc": "z_sc",
        "adapt_p_sc": "p_sc", "adapt_K_phas": "K_phas", "adapt_p_phas": "p_phas", "adapt_kp_prop": "kp_prop",
        "adapt_z_ccr": "z_ccr", "adapt_p_ccr": "p_ccr", "adapt_K_gamma_dyn": "K_γ_dyn", "adapt_K_gamma_stat": "K_γ_stat"
    }
    axes[2].axhline(100.0, color='k', linestyle='--', alpha=0.6, label='Nominal 100% Baseline')
    for name in ADAPT_12_NAMES:
        th0 = theta_0_map[name]
        pct = np.array(log_params[name]) / th0 * 100.0
        final_pct = pct[-1]
        axes[2].plot(log_t, pct, label=f'{short_names[name]} ({final_pct:.1f}%)', linewidth=1.2)
    axes[2].set_ylabel('Normalized (% th0)')
    axes[2].set_title('Normalized Parameter Trajectories (% of Baseline Centroid)')
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(loc='upper right', ncol=4, fontsize=8)

    # Panel 3: Gains
    axes[3].plot(log_t, log_params["adapt_K_ton"], label=f'K_ton -> {log_params["adapt_K_ton"][-1]:.3f}', color='coral')
    axes[3].plot(log_t, log_params["adapt_K_sc"], label=f'K_sc -> {log_params["adapt_K_sc"][-1]:.3f}', color='teal')
    axes[3].plot(log_t, log_params["adapt_K_phas"], label=f'K_phas -> {log_params["adapt_K_phas"][-1]:.3f}', color='purple')
    axes[3].plot(log_t, log_params["adapt_kp_prop"], label=f'kp_prop -> {log_params["adapt_kp_prop"][-1]:.3f}', color='green')
    axes[3].plot(log_t, log_params["adapt_K_gamma_dyn"], label=f'K_gamma_dyn -> {log_params["adapt_K_gamma_dyn"][-1]:.3f}', color='blue')
    axes[3].plot(log_t, log_params["adapt_K_gamma_stat"], label=f'K_gamma_stat -> {log_params["adapt_K_gamma_stat"][-1]:.3f}', color='orange')
    axes[3].set_ylabel('Adapted Gains')
    axes[3].grid(True, alpha=0.3)
    axes[3].legend(loc='upper right', ncol=3)

    # Panel 4: Poles and zeros in rad/s
    axes[4].plot(log_t, log_params["adapt_p_ton"], label=f'p_ton -> {log_params["adapt_p_ton"][-1]:.1f}', color='coral', linestyle='--')
    axes[4].plot(log_t, log_params["adapt_p_sc"], label=f'p_sc -> {log_params["adapt_p_sc"][-1]:.1f}', color='teal', linestyle='--')
    axes[4].plot(log_t, log_params["adapt_z_sc"], label=f'z_sc -> {log_params["adapt_z_sc"][-1]:.1f}', color='cyan')
    axes[4].plot(log_t, log_params["adapt_p_phas"], label=f'p_phas -> {log_params["adapt_p_phas"][-1]:.1f}', color='purple', linestyle='--')
    axes[4].plot(log_t, log_params["adapt_p_ccr"], label=f'p_ccr -> {log_params["adapt_p_ccr"][-1]:.1f}', color='darkgreen', linestyle='--')
    axes[4].plot(log_t, log_params["adapt_z_ccr"], label=f'z_ccr -> {log_params["adapt_z_ccr"][-1]:.1f}', color='lime')
    axes[4].set_ylabel('Poles & Zeros (rad/s)')
    axes[4].set_xlabel('Time (seconds)')
    axes[4].grid(True, alpha=0.3)
    axes[4].legend(loc='upper right', ncol=3)

    plt.tight_layout()

    plt.tight_layout()
    plot_path = "logs/adaptive_12params_convergence.png"
    plt.savefig(plot_path, dpi=300)
    plt.close()
    print(f"[PLOT] Saved {plot_path}", flush=True)

if __name__ == "__main__":
    main()

