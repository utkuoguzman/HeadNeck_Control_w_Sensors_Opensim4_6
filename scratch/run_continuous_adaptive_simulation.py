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
    print("=" * 80)
    print("PATH 1: CONTINUOUS STATE-VARIABLE ADAPTIVE CONTROL VALIDATION")
    print("Model: HYOID_HeadNeckNeural_Model_Adaptive (C++ State-Variable Integration)")
    print("=" * 80)

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
    print(f"[EXCITATION] 6-frequency Schroeder multisine prepared (Scale: {scale_factor:.4f})")

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
    log_K_sc = []
    log_K_ton = []
    log_K_phas = []
    log_kp_prop = []

    dt_log = 0.05 # Sample telemetry at 20 Hz
    current_time = 0.0

    print(f"\n[EXECUTION STARTED] Integrating 30.0s multisine simulation via continuous state variables...")
    print(f"{'SimTime':>7s} | {'WallClock':>9s} | {'HeadPitch':>9s} | {'RefPitch':>9s} | {'Err(deg)':>8s} | {'K_sc':>7s} | {'K_ton':>7s} | {'K_phas':>7s} | {'kp':>7s}")
    print("-" * 88)

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

        k_sc_val = model.getStateVariableValue(state, "/controllerset/VCR_CCR_Controller/adapt_K_sc")
        k_ton_val = model.getStateVariableValue(state, "/controllerset/VCR_CCR_Controller/adapt_K_ton")
        k_phas_val = model.getStateVariableValue(state, "/controllerset/VCR_CCR_Controller/adapt_K_phas")
        kp_val = model.getStateVariableValue(state, "/controllerset/VCR_CCR_Controller/adapt_kp_prop")

        log_t.append(current_time)
        log_head_pitch.append(head_pitch_deg)
        log_ref_pitch.append(ref_pitch_deg)
        log_err.append(err_deg)
        log_K_sc.append(k_sc_val)
        log_K_ton.append(k_ton_val)
        log_K_phas.append(k_phas_val)
        log_kp_prop.append(kp_val)

        # Print progress every 1.0 second of simulation
        if current_time - last_print_sim >= 1.0 or current_time >= sim_duration:
            elapsed_wall = time.time() - t_wall_start
            print(f"{current_time:6.2f}s | {elapsed_wall:7.1f}s | {head_pitch_deg:8.3f}° | {ref_pitch_deg:8.3f}° | {err_deg:7.3f}° | {k_sc_val:7.4f} | {k_ton_val:7.4f} | {k_phas_val:7.4f} | {kp_val:7.4f}")
            last_print_sim = current_time

    total_wall_time = time.time() - t_wall_start
    print("-" * 88)
    print(f"\n[COMPLETE] 30.0s Simulation Finished in {total_wall_time:.1f}s ({total_wall_time/60.0:.2f} min)!")

    # Evaluation metrics
    steady_mask = np.array(log_t) >= 5.0
    steady_err = np.array(log_err)[steady_mask]
    steady_ref = np.array(log_ref_pitch)[steady_mask]
    steady_head = np.array(log_head_pitch)[steady_mask]

    rmse = np.sqrt(np.mean(steady_err**2))
    corr = np.corrcoef(steady_head, steady_ref)[0, 1]

    print("\n" + "=" * 60)
    print("CONTINUOUS ADAPTIVE CONVERGENCE SUMMARY")
    print("=" * 60)
    print(f"Tracking RMSE (t >= 5s): {rmse:.4f}°")
    print(f"Target Correlation (r):  {corr:.5f}")
    print(f"Initial Gains: K_sc={log_K_sc[0]:.4f}, K_ton={log_K_ton[0]:.4f}, K_phas={log_K_phas[0]:.4f}, kp={log_kp_prop[0]:.4f}")
    print(f"Converged Gains: K_sc={log_K_sc[-1]:.4f}, K_ton={log_K_ton[-1]:.4f}, K_phas={log_K_phas[-1]:.4f}, kp={log_kp_prop[-1]:.4f}")
    print("=" * 60)

    # Save JSON log
    os.makedirs("logs", exist_ok=True)
    out_json = "logs/adaptive_continuous_trajectory.json"
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump({
            "total_wall_time_sec": total_wall_time,
            "rmse_deg": float(rmse),
            "correlation": float(corr),
            "final_gains": {
                "K_sc": float(log_K_sc[-1]),
                "K_ton": float(log_K_ton[-1]),
                "K_phas": float(log_K_phas[-1]),
                "kp_prop": float(log_kp_prop[-1])
            },
            "time": log_t,
            "head_pitch": log_head_pitch,
            "ref_pitch": log_ref_pitch,
            "error": log_err,
            "K_sc": log_K_sc,
            "K_ton": log_K_ton,
            "K_phas": log_K_phas,
            "kp_prop": log_kp_prop
        }, f, indent=2)
    print(f"[DATA] Telemetry saved to {out_json}")

    # Generate convergence plot
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    
    # 1. Pitch angle tracking
    axes[0].plot(log_t, log_ref_pitch, 'k--', label='Happee (2017) Bode Target', linewidth=1.5)
    axes[0].plot(log_t, log_head_pitch, 'b-', label='Actual Head Pitch (Adaptive Model)', linewidth=1.5, alpha=0.85)
    axes[0].axvline(2.5, color='gray', linestyle=':', label='Adaptation Start (t=2.5s)')
    axes[0].set_ylabel('Pitch (deg)')
    axes[0].set_title(f'Continuous State-Variable Adaptive Control under Schroeder Multisine\nRMSE = {rmse:.3f}°, r = {corr:.4f}, Sim Time = 30s in {total_wall_time:.1f}s wall clock')
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc='upper right')

    # 2. Tracking Error
    axes[1].plot(log_t, log_err, 'r-', label='Tracking Error e(t)', linewidth=1.2)
    axes[1].axhline(0.0, color='k', linestyle=':', alpha=0.6)
    axes[1].axvline(2.5, color='gray', linestyle=':')
    axes[1].set_ylabel('Error (deg)')
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc='upper right')

    # 3. Continuous Parameter Adaptation
    axes[2].plot(log_t, log_K_sc, label=f'K_sc (canal) -> {log_K_sc[-1]:.3f}', color='teal', linewidth=1.5)
    axes[2].plot(log_t, log_K_ton, label=f'K_ton (otolith tilt) -> {log_K_ton[-1]:.3f}', color='darkorange', linewidth=1.5)
    axes[2].plot(log_t, log_K_phas, label=f'K_phas (phasic) -> {log_K_phas[-1]:.3f}', color='purple', linewidth=1.5)
    axes[2].plot(log_t, log_kp_prop, label=f'kp_prop (CCR spindle) -> {log_kp_prop[-1]:.3f}', color='forestgreen', linewidth=1.5)
    axes[2].axvline(2.5, color='gray', linestyle=':')
    axes[2].set_xlabel('Time (seconds)')
    axes[2].set_ylabel('Continuous Reflex Gain')
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(loc='upper right')

    plt.tight_layout()
    plot_path = "logs/adaptive_continuous_convergence.png"
    plt.savefig(plot_path, dpi=300)
    plt.close()
    print(f"[PLOT] Figure saved to {plot_path}")

if __name__ == "__main__":
    main()

