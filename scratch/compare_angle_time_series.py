import os
import sys
import math
import cmath
import json
import numpy as np

# Load refined plant parameters from test_offline_surrogate_refined
from test_offline_surrogate_refined import opt_plant, compute_surrogate_bode, TARGETS

def run_opensim_angle_test(freq=1.099, cand_name="W15_P23G10"):
    import opensim as osim
    
    os.environ["SIMB_NUM_THREADS"] = "1"
    os.environ["OMP_NUM_THREADS"] = "1"
    osim.Logger.setLevelString("Warn")
    osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'proprioception_plugin/build/Release/osimMillard12EqWithAff.dll'))
    osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll'))
    osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'FixationController_plugin/build/Release/osimFixation.dll'))

    with open("logs/phase2_3_master_ledger.json", "r", encoding="utf-8") as f:
        ledger = json.load(f)
        
    g10 = [g for g in ledger["generations"] if g["candidate"] == cand_name][0]
    p = g10["params"]
    
    model = osim.Model("osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim")
    controller = None
    for i in range(model.getControllerSet().getSize()):
        ctrl = model.getControllerSet().get(i)
        if ctrl.getConcreteClassName() == "FixationController":
            ctrl.set_enabled(False)
        elif ctrl.getConcreteClassName() == "HeadNeckNeuralController":
            controller = ctrl

    # Set parameters
    osim.PropertyHelper.setValueDouble(p["K_ton"], controller.updPropertyByName("K_ton"))
    osim.PropertyHelper.setValueDouble(p["p_ton"], controller.updPropertyByName("p_ton"))
    osim.PropertyHelper.setValueDouble(p["K_sc"], controller.updPropertyByName("K_sc"))
    osim.PropertyHelper.setValueDouble(p["z_sc"], controller.updPropertyByName("z_sc"))
    osim.PropertyHelper.setValueDouble(p["p_sc"], controller.updPropertyByName("p_sc"))
    osim.PropertyHelper.setValueDouble(p["K_phas"], controller.updPropertyByName("K_phas"))
    osim.PropertyHelper.setValueDouble(p["p_phas"], controller.updPropertyByName("p_phas"))
    osim.PropertyHelper.setValueDouble(p["kp"], controller.updPropertyByName("k_p"))
    osim.PropertyHelper.setValueDouble(p["kp"], controller.updPropertyByName("Kp_proprioception"))
    osim.PropertyHelper.setValueDouble(p["z_ccr"], controller.updPropertyByName("z_ccr"))
    osim.PropertyHelper.setValueDouble(p["p_ccr"], controller.updPropertyByName("p_ccr"))
    osim.PropertyHelper.setValueDouble(p["k_dyn"], controller.updPropertyByName("K_gamma_dyn"))
    osim.PropertyHelper.setValueDouble(p["k_stat"], controller.updPropertyByName("K_gamma_stat"))
    osim.PropertyHelper.setValueDouble(0.010, controller.updPropertyByName("delay_vcr"))
    osim.PropertyHelper.setValueDouble(0.015, controller.updPropertyByName("delay_ccr"))
    osim.PropertyHelper.setValueDouble(40.2786, controller.updPropertyByName("Ki_task"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("Wp_base_pitch"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("Wv_base_pitch"))
    for ax in ['roll', 'yaw']:
        osim.PropertyHelper.setValueDouble(400.0, controller.updPropertyByName(f"Wp_base_{ax}"))
        osim.PropertyHelper.setValueDouble(40.0, controller.updPropertyByName(f"Wv_base_{ax}"))

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
    R_head_to_world = skull.getTransformInGround(state).R()
    q = osim.Rotation(R_head_to_world).convertRotationToQuaternion()
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q0", q.get(0))
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q1", q.get(1))
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q2", q.get(2))
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q3", q.get(3))
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/pitch_error_integral", 0.10528)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_pitch", 40.2786 * 0.10528)
    model.equilibrateMuscles(state)

    manager = osim.Manager(model)
    manager.setIntegratorMethod(5) # SemiExplicitEuler2
    manager.setIntegratorAccuracy(1e-4)
    manager.setIntegratorMinimumStepSize(1e-4)
    manager.setIntegratorMaximumStepSize(0.01)

    state.setTime(0.0)
    manager.initialize(state)

    step = min(0.01, (1.0/freq) / 40.0)
    current_time = 0.0
    coord_p1 = model.getCoordinateSet().get("pitch1")
    coord_p2 = model.getCoordinateSet().get("pitch2")

    print(f"Integrating OpenSim model from t=0.0 to t={target_time:.2f}s (step={step:.4f}s)...", flush=True)

    t_hist, pitch_hist = [], []
    while current_time < target_time:
        current_time = min(current_time + step, target_time)
        state = manager.integrate(current_time)
        p_deg = math.degrees(coord_p1.getValue(state) + coord_p2.getValue(state))
        t_hist.append(current_time)
        pitch_hist.append(p_deg)

    print("OpenSim forward integration completed successfully! Computing surrogate comparison...", flush=True)

    # Steady state interval
    discard_time = delay + ramp_duration
    steady_mask = [i for i, t in enumerate(t_hist) if t >= discard_time]
    t_steady = np.array([t_hist[i] for i in steady_mask])
    pitch_steady = np.array([pitch_hist[i] for i in steady_mask])

    # Detrend
    coeffs = np.polyfit(t_steady, pitch_steady, 1)
    pitch_detrended = pitch_steady - (coeffs[0] * t_steady + coeffs[1])

    # Surrogate prediction
    sim_gain, sim_phase = compute_surrogate_bode(p, opt_plant, freq)
    surr_amplitude_deg = sim_gain * amplitude
    surr_phase_rad = math.radians(sim_phase)
    
    # Compute surrogate sinusoidal angle time-series
    # dt_steady = t_steady - delay
    dt_steady = t_steady - delay
    omega = 2.0 * math.pi * freq
    pitch_surrogate = surr_amplitude_deg * np.sin(omega * dt_steady + surr_phase_rad)

    # Compute correlation and RMSE
    rmse = np.sqrt(np.mean((pitch_detrended - pitch_surrogate)**2))
    corr = np.corrcoef(pitch_detrended, pitch_surrogate)[0, 1]

    print("="*75)
    print(f"OpenSim vs. Surrogate Angle Trajectory Comparison at {freq:.3f} Hz ({cand_name}):")
    print("="*75)
    print(f"  Sled Input Amplitude:            {amplitude*1000.0:.2f} mm")
    print(f"  OpenSim Peak-to-Peak Angle:      {np.ptp(pitch_detrended):.4f} deg")
    print(f"  Surrogate Peak-to-Peak Angle:    {2.0 * surr_amplitude_deg:.4f} deg")
    print(f"  Amplitude Error:                 {abs(np.ptp(pitch_detrended)/2.0 - surr_amplitude_deg):.4f} deg")
    print(f"  Time-Series RMSE:                {rmse:.4f} deg")
    print(f"  Pearson Correlation Coefficient: {corr:.4f} ({corr*100:.2f}%)")
    print("="*75)

    return t_steady, pitch_detrended, pitch_surrogate, corr, rmse

if __name__ == "__main__":
    run_opensim_angle_test(1.099, "W15_P23G10")

