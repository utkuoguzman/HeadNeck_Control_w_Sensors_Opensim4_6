import os
import sys
import math
import numpy as np
import opensim as osim

def run_simulation_record(use_smith=False, tau_aff=0.007, freq=1.099, duration=4.5):
    os.environ["SIMB_NUM_THREADS"] = "1"
    os.environ["OMP_NUM_THREADS"] = "1"
    osim.Logger.setLevelString("Warn")
    osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'proprioception_plugin/build/Release/osimMillard12EqWithAff.dll'))
    
    # Load our custom split-delay neural controller DLL
    split_dll = os.path.join(os.getcwd(), 'scratch/HeadNeckNeuralController_plugin_split/build/Release/osimHeadNeckNeural_split.dll')
    if not os.path.exists(split_dll):
        raise FileNotFoundError(f"DLL not found: {split_dll}")
    osim.LoadOpenSimLibrary(split_dll)
    osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'FixationController_plugin/build/Release/osimFixation.dll'))

    model = osim.Model("osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim")
    controller = None
    for i in range(model.getControllerSet().getSize()):
        ctrl = model.getControllerSet().get(i)
        if ctrl.getConcreteClassName() == "FixationController":
            ctrl.set_enabled(False)
        elif ctrl.getConcreteClassName() == "HeadNeckNeuralController":
            controller = ctrl

    # Best parameters from rational champion / W3
    r_ton = 0.577
    K_sc = 0.530
    K_ton = r_ton * K_sc
    p_ton = 76.50
    r_lead_sc = 0.246
    p_sc = 96.05
    z_sc = r_lead_sc * p_sc
    K_phas = 4.294
    p_phas = 17.53
    kp_prop = 1.284
    r_lead_ccr = 0.069
    p_ccr = 177.07
    z_ccr = r_lead_ccr * p_ccr
    k_dyn = 0.719
    k_stat = 1.177

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
    osim.PropertyHelper.setValueDouble(k_dyn, controller.updPropertyByName("K_gamma_dyn"))
    osim.PropertyHelper.setValueDouble(k_stat, controller.updPropertyByName("K_gamma_stat"))
    osim.PropertyHelper.setValueDouble(0.010, controller.updPropertyByName("delay_vcr"))
    osim.PropertyHelper.setValueDouble(0.015, controller.updPropertyByName("delay_ccr"))

    # Configure Anatomical Split Delay
    osim.PropertyHelper.setValueDouble(tau_aff, controller.updPropertyByName("delay_afferent_proprio"))
    osim.PropertyHelper.setValueBool(use_smith, controller.updPropertyByName("use_smith_predictor"))
    osim.PropertyHelper.setValueDouble(tau_aff, controller.updPropertyByName("delay_smith_proprio"))

    # Direct task space gains
    osim.PropertyHelper.setValueDouble(10.24, controller.updPropertyByName("Kp_task_pitch"))
    osim.PropertyHelper.setValueDouble(1.31, controller.updPropertyByName("Kd_task_pitch"))
    osim.PropertyHelper.setValueDouble(10.21, controller.updPropertyByName("Kp_task_roll"))
    osim.PropertyHelper.setValueDouble(1.29, controller.updPropertyByName("Kd_task_roll"))
    osim.PropertyHelper.setValueDouble(0.24, controller.updPropertyByName("Kp_task_yaw"))
    osim.PropertyHelper.setValueDouble(0.12, controller.updPropertyByName("Kd_task_yaw"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("k_vigor"))
    osim.PropertyHelper.setValueDouble(0.1, controller.updPropertyByName("trajectory_tau"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("K_eso"))

    # Decoupled gravity integrator
    ki = 40.2786
    osim.PropertyHelper.setValueDouble(ki, controller.updPropertyByName("Ki_task"))
    osim.PropertyHelper.setValueDouble(ki, controller.updPropertyByName("Ki_task_pitch"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("Ki_task_roll"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("Ki_task_yaw"))

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
    target_time = delay + ramp_duration + (2.0 * period)

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
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/roll_error_integral", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/yaw_error_integral", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_pitch", ki * 0.10528)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_roll", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_yaw", 0.0)
    model.equilibrateMuscles(state)

    manager = osim.Manager(model)
    manager.setIntegratorMethod(5)
    manager.setIntegratorAccuracy(1e-4)
    manager.setIntegratorMinimumStepSize(1e-4)
    manager.setIntegratorMaximumStepSize(0.01)

    state.setTime(0.0)
    manager.initialize(state)

    step = 0.01
    current_time = 0.0
    coord_p1 = model.getCoordinateSet().get("pitch1")
    coord_p2 = model.getCoordinateSet().get("pitch2")

    t_arr = []
    gt_pitch_arr = []
    mahony_pitch_arr = []

    while current_time < target_time:
        current_time = min(current_time + step, target_time)
        state = manager.integrate(current_time)

        # Ground truth pitch angle (sum of lower + upper cervical coordinates)
        p1 = math.degrees(coord_p1.getValue(state))
        p2 = math.degrees(coord_p2.getValue(state))
        gt_pitch = p1 + p2

        # Mahony estimated pitch
        q0 = model.getStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q0")
        q1 = model.getStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q1")
        q2 = model.getStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q2")
        q3 = model.getStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q3")
        norm_q = math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
        if norm_q > 1e-4:
            q0 /= norm_q; q1 /= norm_q; q2 /= norm_q; q3 /= norm_q
        mahony_pitch = math.degrees(math.atan2(2.0*(q0*q3 + q1*q2), 1.0 - 2.0*(q2*q2 + q3*q3)))

        t_arr.append(current_time)
        gt_pitch_arr.append(gt_pitch)
        mahony_pitch_arr.append(mahony_pitch)

    return np.array(t_arr), np.array(gt_pitch_arr), np.array(mahony_pitch_arr)

def analyze_performance(t, gt, est, discard_time):
    idx = t >= discard_time
    t_w = t[idx]
    y_gt = gt[idx]
    y_est = est[idx]

    rmse = np.sqrt(np.mean((y_est - y_gt)**2))
    corr = np.corrcoef(y_gt, y_est)[0, 1]
    
    amp_gt = (np.max(y_gt) - np.min(y_gt)) / 2.0
    amp_est = (np.max(y_est) - np.min(y_est)) / 2.0
    amp_ratio = amp_est / amp_gt

    max_err = np.max(np.abs(y_est - y_gt))

    # Phase difference via cross-correlation / FFT
    dt = t_w[1] - t_w[0]
    freq = 1.099
    omega = 2.0 * math.pi * freq
    
    # Detrend
    c_gt = np.polyfit(t_w, y_gt, 1)
    c_est = np.polyfit(t_w, y_est, 1)
    detrend_gt = y_gt - (c_gt[0]*t_w + c_gt[1])
    detrend_est = y_est - (c_est[0]*t_w + c_est[1])

    Is_gt = np.mean(detrend_gt * np.sin(omega * t_w))
    Ic_gt = np.mean(detrend_gt * np.cos(omega * t_w))
    phase_gt = math.degrees(math.atan2(Ic_gt, Is_gt))

    Is_est = np.mean(detrend_est * np.sin(omega * t_w))
    Ic_est = np.mean(detrend_est * np.cos(omega * t_w))
    phase_est = math.degrees(math.atan2(Ic_est, Is_est))

    phase_diff = (phase_est - phase_gt + 180.0) % 360.0 - 180.0

    return {
        "rmse": rmse,
        "corr": corr,
        "amp_ratio": amp_ratio,
        "phase_diff_deg": phase_diff,
        "max_err_deg": max_err
    }

if __name__ == "__main__":
    print("="*75)
    print("COMPARING ANATOMICALLY-SPLIT MAHONY OBSERVERS AGAINST GROUND TRUTH")
    print("Afferent Conduction Delay: tau_aff = 7 ms (Spindle -> Brainstem Observer)")
    print("="*75)

    freq = 1.099
    period = 1.0 / freq
    discard = 1.5 + period # Discard delay + ramp
    tau_aff = 0.007 # 7 ms afferent delay

    print("\n[RUN 1/2] Simulating UNCOMPENSATED Mahony (tau_aff = 7ms, use_smith = False)...")
    t1, gt1, est1 = run_simulation_record(use_smith=False, tau_aff=tau_aff, freq=freq)
    res_orig = analyze_performance(t1, gt1, est1, discard)

    print("\n[RUN 2/2] Simulating SMITH-AUGMENTED Mahony (tau_aff = 7ms, use_smith = True, tau_pred = 7ms)...")
    t2, gt2, est2 = run_simulation_record(use_smith=True, tau_aff=tau_aff, freq=freq)
    res_smith = analyze_performance(t2, gt2, est2, discard)

    print("\n" + "="*75)
    print("## EXPERIMENTAL RESULTS: ANATOMICALLY SPLIT DELAY (7 ms Afferent Delay)")
    print("="*75)
    print(f"| Metric | Uncompensated Mahony | Smith-Augmented Mahony | Relative Outcome |")
    print(f"| :--- | :---: | :---: | :---: |")
    print(f"| **Orientation RMSE** | `{res_orig['rmse']:.3f}°` | **`{res_smith['rmse']:.3f}°`** | **`{((res_orig['rmse'] - res_smith['rmse'])/res_orig['rmse']*100):+.1f}%`** |")
    print(f"| **Correlation (r)** | `{res_orig['corr']:.5f}` | **`{res_smith['corr']:.5f}`** | **`+{(res_smith['corr'] - res_orig['corr']):.5f}`** |")
    print(f"| **Amplitude Ratio** | `{res_orig['amp_ratio']:.4f}` | **`{res_smith['amp_ratio']:.4f}`** | |")
    print(f"| **Phase Delay / Lead** | `{res_orig['phase_diff_deg']:+.2f}°` | **`{res_smith['phase_diff_deg']:+.2f}°`** | **`{abs(res_orig['phase_diff_deg']) - abs(res_smith['phase_diff_deg']):+.2f}° phase correction`** |")
    print(f"| **Max Absolute Error** | `{res_orig['max_err_deg']:.3f}°` | **`{res_smith['max_err_deg']:.3f}°`** | **`{((res_orig['max_err_deg'] - res_smith['max_err_deg'])/res_orig['max_err_deg']*100):+.1f}%`** |")
    print("="*75)

