import os
import sys
import time
import math
import json
import numpy as np
import opensim as osim

# Load custom plugins
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'proprioception_plugin/build/Release/osimMillard12EqWithAff.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'FixationController_plugin/build/Release/osimFixation.dll'))

FIXED_W_N = 3.398
FIXED_ZETA = 0.551
FIXED_KI = 40.28

# W3_P23G15 Parameters
with open('logs/candidates/W3_P23G15.json', 'r') as f:
    cand_info = json.load(f)
p = cand_info['params']

K_ton, p_ton, K_sc, z_sc, p_sc, K_phas, p_phas, kp_prop, z_ccr, p_ccr, k_gamma_dyn, k_gamma_stat = p

TARGETS = [
    (2.446, 423.34, -161.34),
    (5.583, 503.30, -201.38),
    (7.927, 572.69, -247.62)
]

def extract_bode(t_array, pitch_array, freq, discard_time, delay):
    valid_indices = [i for i, t in enumerate(t_array) if t >= discard_time]
    t_steady = np.array([t_array[i] for i in valid_indices])
    pitch_steady = np.array([pitch_array[i] for i in valid_indices])
    if len(t_steady) < 10:
        return 0.0, 0.0
    omega = 2.0 * math.pi * freq
    t_shifted = t_steady - delay
    coeffs = np.polyfit(t_steady, pitch_steady, 1)
    pitch_detrended = pitch_steady - (coeffs[0] * t_steady + coeffs[1])
    Is = 2.0 * np.mean(pitch_detrended * np.sin(omega * t_shifted))
    Ic = 2.0 * np.mean(pitch_detrended * np.cos(omega * t_shifted))
    amplitude_rad = np.sqrt(Is**2 + Ic**2)
    phase_rad = math.atan2(Ic, Is)
    return amplitude_rad, phase_rad

def simulate_freq(model_path, freq, target_gain, target_phase):
    os.environ["SIMB_NUM_THREADS"] = "1"
    os.environ["OMP_NUM_THREADS"] = "1"
    osim.Logger.setLevelString("Warn")

    t_start = time.perf_counter()
    model = osim.Model(model_path)
    
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
    osim.PropertyHelper.setValueDouble(p_phas, controller.updPropertyByName("p_phas"))
    osim.PropertyHelper.setValueDouble(kp_prop, controller.updPropertyByName("k_p"))

    wp_pitch = float(os.environ.get("WP_BASE_PITCH", "0.0"))
    wv_pitch = float(os.environ.get("WV_BASE_PITCH", str(2.0 * math.sqrt(wp_pitch) if wp_pitch > 0 else 0.0)))
    osim.PropertyHelper.setValueDouble(wp_pitch, controller.updPropertyByName("Wp_base_pitch"))
    osim.PropertyHelper.setValueDouble(wv_pitch, controller.updPropertyByName("Wv_base_pitch"))
    for ax in ['roll', 'yaw']:
        osim.PropertyHelper.setValueDouble(400.0, controller.updPropertyByName(f"Wp_base_{ax}"))
        osim.PropertyHelper.setValueDouble(40.0, controller.updPropertyByName(f"Wv_base_{ax}"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("k_vigor"))
    osim.PropertyHelper.setValueDouble(0.1, controller.updPropertyByName("trajectory_tau"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("K_eso"))

    osim.PropertyHelper.setValueDouble(FIXED_KI, controller.updPropertyByName("Ki_task"))
    osim.PropertyHelper.setValueDouble(kp_prop, controller.updPropertyByName("Kp_proprioception"))
    osim.PropertyHelper.setValueDouble(z_ccr, controller.updPropertyByName("z_ccr"))
    osim.PropertyHelper.setValueDouble(p_ccr, controller.updPropertyByName("p_ccr"))
    osim.PropertyHelper.setValueDouble(k_gamma_dyn, controller.updPropertyByName("K_gamma_dyn"))
    osim.PropertyHelper.setValueDouble(k_gamma_stat, controller.updPropertyByName("K_gamma_stat"))
    osim.PropertyHelper.setValueDouble(0.010, controller.updPropertyByName("delay_vcr"))
    osim.PropertyHelper.setValueDouble(0.015, controller.updPropertyByName("delay_ccr"))

    zero_func = osim.Constant(0.0)
    for coord_name in ["gndpitch", "gndroll", "gndyaw", "spine_ty", "spine_tz"]:
        c = model.updCoordinateSet().get(coord_name)
        c.set_locked(False)
        c.setDefaultIsPrescribed(True)
        c.setPrescribedFunction(zero_func)

    c_yaw1 = model.updCoordinateSet().get("yaw1")
    c_yaw1.set_locked(False)
    c_yaw1.setDefaultIsPrescribed(False)

    amplitude = 0.0215 * (0.835 / freq)**2
    delay = 1.5
    period = 1.0 / freq
    ramp_duration = period
    min_phase3_time = 2.0 / 1.8
    n_eval = max(1, math.ceil(min_phase3_time / period))
    target_time = delay + ramp_duration + (n_eval * period)

    sine_function = osim.SimmSpline()
    for t in np.arange(0.0, delay, 0.02):
        sine_function.addPoint(t, 0.0)
    spline_dt = min(0.01, period / 40.0)
    for t in np.arange(delay, target_time + 0.05, spline_dt):
        dt = t - delay
        if dt < ramp_duration:
            tau = dt / ramp_duration
            w = 10.0 * (tau**3) - 15.0 * (tau**4) + 6.0 * (tau**5)
        else:
            w = 1.0
        y = w * amplitude * math.sin(2.0 * math.pi * freq * dt)
        sine_function.addPoint(t, y)

    coord_tx = model.updCoordinateSet().get("spine_tx")
    coord_tx.set_locked(False)
    coord_tx.setDefaultIsPrescribed(True)
    coord_tx.setPrescribedFunction(sine_function)

    state = model.initSystem()
    c_yaw1.setIsPrescribed(state, False)
    coord_tx.setSpeedValue(state, 0.0)
    skull = model.getBodySet().get("skull")
    model.realizePosition(state)
    R_head_to_world = skull.getTransformInGround(state).R()
    q = osim.Rotation(R_head_to_world).convertRotationToQuaternion()
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q0", q.get(0))
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q1", q.get(1))
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q2", q.get(2))
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q3", q.get(3))
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/pitch_error_integral", 0.10528)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/roll_error_integral", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/yaw_error_integral", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_pitch", FIXED_KI * 0.10528)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_roll", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_yaw", 0.0)
    model.equilibrateMuscles(state)

    manager = osim.Manager(model)
    manager.setIntegratorMethod(5) # SemiExplicitEuler2
    manager.setIntegratorAccuracy(1e-4)
    manager.setIntegratorMinimumStepSize(1e-4)
    manager.setIntegratorMaximumStepSize(0.01)

    state.setTime(0.0)
    manager.initialize(state)

    coord_p1 = model.getCoordinateSet().get("pitch1")
    coord_p2 = model.getCoordinateSet().get("pitch2")
    coord_r1 = model.getCoordinateSet().get("roll1")
    coord_r2 = model.getCoordinateSet().get("roll2")
    coord_y1 = model.getCoordinateSet().get("yaw1")

    time_hist = []
    pitch_hist = []
    roll_hist = []
    yaw_hist = []

    step = min(0.01, (1.0/freq) / 40.0)
    current_time = 0.0

    while current_time < target_time:
        current_time = min(current_time + step, target_time)
        state = manager.integrate(current_time)

        head_pitch_deg = math.degrees(coord_p1.getValue(state) + coord_p2.getValue(state))
        head_roll_deg = math.degrees(coord_r1.getValue(state) + coord_r2.getValue(state))
        head_yaw_deg = math.degrees(coord_y1.getValue(state))

        time_hist.append(current_time)
        pitch_hist.append(head_pitch_deg)
        roll_hist.append(head_roll_deg)
        yaw_hist.append(head_yaw_deg)

    elapsed = time.perf_counter() - t_start
    discard_time = delay + ramp_duration
    amp_deg, phase_rad = extract_bode(time_hist, pitch_hist, freq, discard_time, delay)

    sim_gain = amp_deg / amplitude
    sim_phase = math.degrees(phase_rad)
    while sim_phase > 90.0: sim_phase -= 360.0
    while sim_phase < -270.0: sim_phase += 360.0

    sim_gain_safe = max(1e-6, sim_gain)
    gain_db_diff = 20.0 * math.log10(sim_gain_safe / target_gain)
    gain_loss = ((gain_db_diff / 1.0) ** 2) * 100.0
    phase_diff = (sim_phase - target_phase + 180.0) % 360.0 - 180.0
    phase_loss = ((phase_diff / 10.0) ** 2) * 100.0
    final_score = gain_loss + phase_loss

    return {
        "freq": freq,
        "elapsed": elapsed,
        "gain": sim_gain,
        "gain_db_diff": gain_db_diff,
        "phase": sim_phase,
        "phase_diff": phase_diff,
        "score": final_score,
        "time": np.array(time_hist),
        "pitch": np.array(pitch_hist),
        "roll": np.array(roll_hist),
        "yaw": np.array(yaw_hist)
    }

def main():
    normal_model = 'osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim'
    fast_model = 'osim_files/HYOID_HeadNeckNeural_Model_FastGeometry.osim'

    print("=" * 80)
    print("BENCHMARK: NORMAL MODEL (GeometryPath) vs FAST MODEL (FunctionBasedPath)")
    print(f"Candidate: W3_P23G15 (All-Time Champion, Score 21,059.90)")
    print("=" * 80)

    results = []
    for freq, t_gain, t_phase in TARGETS:
        print(f"\n>>> Running Frequency {freq:.3f} Hz (Target Gain: {t_gain:.1f}, Phase: {t_phase:.1f}°)...")
        
        # 1. Normal Model
        print(f"  [1/2] Running Normal Model (GeometryPath)...", flush=True)
        res_norm = simulate_freq(normal_model, freq, t_gain, t_phase)
        print(f"        -> Time: {res_norm['elapsed']:.2f} s | Gain: {res_norm['gain']:.2f} | Phase: {res_norm['phase']:.2f}° | Score: {res_norm['score']:.1f}")

        # 2. Fast Model
        print(f"  [2/2] Running Fast Model (FunctionBasedPath)...", flush=True)
        res_fast = simulate_freq(fast_model, freq, t_gain, t_phase)
        print(f"        -> Time: {res_fast['elapsed']:.2f} s | Gain: {res_fast['gain']:.2f} | Phase: {res_fast['phase']:.2f}° | Score: {res_fast['score']:.1f}")

        # Compare trajectory errors
        # Interpolate onto identical time points
        t_common = np.linspace(1.5, min(res_norm['time'][-1], res_fast['time'][-1]), 500)
        pitch_norm = np.interp(t_common, res_norm['time'], res_norm['pitch'])
        pitch_fast = np.interp(t_common, res_fast['time'], res_fast['pitch'])
        roll_norm = np.interp(t_common, res_norm['time'], res_norm['roll'])
        roll_fast = np.interp(t_common, res_fast['time'], res_fast['roll'])
        yaw_norm = np.interp(t_common, res_norm['time'], res_norm['yaw'])
        yaw_fast = np.interp(t_common, res_fast['time'], res_fast['yaw'])

        pitch_rmse = np.sqrt(np.mean((pitch_norm - pitch_fast)**2))
        pitch_max_err = np.max(np.abs(pitch_norm - pitch_fast))
        roll_rmse = np.sqrt(np.mean((roll_norm - roll_fast)**2))
        yaw_rmse = np.sqrt(np.mean((yaw_norm - yaw_fast)**2))

        speedup = res_norm['elapsed'] / max(0.01, res_fast['elapsed'])

        results.append({
            "freq": freq,
            "norm_time": res_norm['elapsed'],
            "fast_time": res_fast['elapsed'],
            "speedup": speedup,
            "norm_gain": res_norm['gain'],
            "fast_gain": res_fast['gain'],
            "norm_phase": res_norm['phase'],
            "fast_phase": res_fast['phase'],
            "norm_score": res_norm['score'],
            "fast_score": res_fast['score'],
            "pitch_rmse": pitch_rmse,
            "pitch_max_err": pitch_max_err,
            "roll_rmse": roll_rmse,
            "yaw_rmse": yaw_rmse
        })

    # Summary table
    print("\n" + "=" * 90)
    print("FINAL HEAD-TO-HEAD COMPARISON SUMMARY")
    print("=" * 90)
    print(f"{'Freq (Hz)':<10} {'Normal Time':<12} {'Fast Time':<12} {'Speedup':<10} {'Score (Norm)':<14} {'Score (Fast)':<14} {'Score Diff'}")
    print("-" * 90)
    for r in results:
        score_diff = r['fast_score'] - r['norm_score']
        print(f"{r['freq']:<10.3f} {r['norm_time']:<12.2f}s {r['fast_time']:<12.2f}s {r['speedup']:<10.2f}x {r['norm_score']:<14.2f} {r['fast_score']:<14.2f} {score_diff:+.2f}")

    print("\n" + "=" * 90)
    print("KINEMATIC ANGLE AGREEMENT (Degrees)")
    print("=" * 90)
    print(f"{'Freq (Hz)':<10} {'Pitch RMSE':<14} {'Pitch Max Err':<16} {'Roll RMSE':<14} {'Yaw RMSE':<14}")
    print("-" * 90)
    for r in results:
        print(f"{r['freq']:<10.3f} {r['pitch_rmse']:<14.4f}° {r['pitch_max_err']:<16.4f}° {r['roll_rmse']:<14.4f}° {r['yaw_rmse']:<14.4f}°")

    # Save results to json
    with open('scratch/fast_geometry_benchmark_results.json', 'w') as f:
        json.dump(results, f, indent=2)
    print("\nSaved detailed benchmark data to scratch/fast_geometry_benchmark_results.json")

if __name__ == '__main__':
    main()
