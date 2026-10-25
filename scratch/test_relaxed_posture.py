import opensim as osim, os, math, numpy as np

osim.LoadOpenSimLibrary('proprioception_plugin/build/Release/osimMillard12EqWithAff.dll')
osim.LoadOpenSimLibrary('HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll')
osim.LoadOpenSimLibrary('FixationController_plugin/build/Release/osimFixation.dll')

def test_condition(Wp_val, label):
    model = osim.Model('osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim')
    for i in range(model.getControllerSet().getSize()):
        ctrl = model.getControllerSet().get(i)
        if ctrl.getConcreteClassName() == 'FixationController': ctrl.set_enabled(False)
        elif ctrl.getConcreteClassName() == 'HeadNeckNeuralController': controller = ctrl

    osim.PropertyHelper.setValueDouble(1.8, controller.updPropertyByName('K_ton'))
    osim.PropertyHelper.setValueDouble(2.0, controller.updPropertyByName('p_ton'))
    osim.PropertyHelper.setValueDouble(0.55, controller.updPropertyByName('K_sc'))
    osim.PropertyHelper.setValueDouble(0.5, controller.updPropertyByName('z_sc'))
    osim.PropertyHelper.setValueDouble(15.0, controller.updPropertyByName('p_sc'))
    osim.PropertyHelper.setValueDouble(0.05, controller.updPropertyByName('K_phas'))
    osim.PropertyHelper.setValueDouble(60.0, controller.updPropertyByName('p_phas'))
    osim.PropertyHelper.setValueDouble(0.45, controller.updPropertyByName('k_p'))
    osim.PropertyHelper.setValueDouble(0.45, controller.updPropertyByName('Kp_proprioception'))
    osim.PropertyHelper.setValueDouble(0.5, controller.updPropertyByName('z_ccr'))
    osim.PropertyHelper.setValueDouble(90.0, controller.updPropertyByName('p_ccr'))
    osim.PropertyHelper.setValueDouble(0.25, controller.updPropertyByName('K_gamma_dyn'))
    osim.PropertyHelper.setValueDouble(0.40, controller.updPropertyByName('K_gamma_stat'))
    osim.PropertyHelper.setValueDouble(0.010, controller.updPropertyByName('delay_vcr'))
    osim.PropertyHelper.setValueDouble(0.015, controller.updPropertyByName('delay_ccr'))

    # Loaded Ki_task
    ki_val = 40.2786
    osim.PropertyHelper.setValueDouble(ki_val, controller.updPropertyByName('Ki_task'))

    # Voluntary pitch costs
    Wv_val = 2.0 * math.sqrt(Wp_val) if Wp_val > 0 else 0.0 # critically damped voluntary
    osim.PropertyHelper.setValueDouble(Wp_val, controller.updPropertyByName('Wp_base_pitch'))
    osim.PropertyHelper.setValueDouble(Wv_val, controller.updPropertyByName('Wv_base_pitch'))

    for ax in ['roll', 'yaw']:
        osim.PropertyHelper.setValueDouble(400.0, controller.updPropertyByName(f'Wp_base_{ax}'))
        osim.PropertyHelper.setValueDouble(40.0, controller.updPropertyByName(f'Wv_base_{ax}'))

    zero_func = osim.Constant(0.0)
    for coord_name in ['gndpitch', 'gndroll', 'gndyaw', 'spine_ty', 'spine_tz']:
        c = model.updCoordinateSet().get(coord_name)
        c.set_locked(False)
        c.setDefaultIsPrescribed(True)
        c.setPrescribedFunction(zero_func)

    c_yaw1 = model.updCoordinateSet().get('yaw1')
    c_yaw1.set_locked(False)
    c_yaw1.setDefaultIsPrescribed(False)

    freq = 0.783
    amplitude = 0.0215 * (0.835 / freq)**2
    delay = 1.5
    period = 1.0 / freq
    ramp_duration = period
    target_time = delay + ramp_duration + 2.0 * period

    sine_function = osim.SimmSpline()
    for t in np.arange(0.0, delay, 0.02):
        sine_function.addPoint(t, 0.0)
    spline_dt = min(0.01, period / 40.0)
    for t in np.arange(delay, target_time + 0.05, spline_dt):
        dt = t - delay
        w = 10.0 * ((dt/ramp_duration)**3) - 15.0 * ((dt/ramp_duration)**4) + 6.0 * ((dt/ramp_duration)**5) if dt < ramp_duration else 1.0
        sine_function.addPoint(t, w * amplitude * math.sin(2.0 * math.pi * freq * dt))

    coord_tx = model.updCoordinateSet().get('spine_tx')
    coord_tx.set_locked(False)
    coord_tx.setDefaultIsPrescribed(True)
    coord_tx.setPrescribedFunction(sine_function)

    state = model.initSystem()
    c_yaw1.setIsPrescribed(state, False)
    coord_tx.setSpeedValue(state, 0.0)

    # Initial states for loaded Ki
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/pitch_error_integral", 0.10528)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/roll_error_integral", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/yaw_error_integral", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_pitch", ki_val * 0.10528)
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

    coord_p1 = model.getCoordinateSet().get('pitch1')
    coord_p2 = model.getCoordinateSet().get('pitch2')

    t_hist, p_hist = [], []
    step = 0.05
    curr = 0.0
    while curr < target_time:
        curr = min(curr + step, target_time)
        state = manager.integrate(curr)
        hp = math.degrees(coord_p1.getValue(state) + coord_p2.getValue(state))
        t_hist.append(curr)
        p_hist.append(hp)

    # Analyze
    discard = delay + ramp_duration
    val_idx = [i for i, t in enumerate(t_hist) if t >= discard]
    t_st = np.array([t_hist[i] for i in val_idx])
    p_st = np.array([p_hist[i] for i in val_idx])
    omega = 2.0 * math.pi * freq
    t_sh = t_st - delay
    coeffs = np.polyfit(t_st, p_st, 1)
    p_det = p_st - (coeffs[0] * t_st + coeffs[1])
    Is = 2.0 * np.mean(p_det * np.sin(omega * t_sh))
    Ic = 2.0 * np.mean(p_det * np.cos(omega * t_sh))
    amp = np.sqrt(Is**2 + Ic**2)
    phase = math.degrees(math.atan2(Ic, Is))
    while phase > 90.0: phase -= 360.0
    while phase < -270.0: phase += 360.0
    gain = amp / amplitude
    mean_pitch_0_15 = np.mean([p for t, p in zip(t_hist, p_hist) if t <= delay])
    print(f"[{label}] Wp={Wp_val:<5.1f} | 0-1.5s Mean Pitch: {mean_pitch_0_15:5.2f} deg | Perturb Mean Pitch: {np.mean(p_st):5.2f} deg | Amp: {amp:5.2f} deg | Gain: {gain:6.1f} deg/m (Tgt: 236.9) | Phase: {phase:6.1f} deg (Tgt: -97.0)")
    return gain, phase

if __name__ == '__main__':
    from concurrent.futures import ProcessPoolExecutor
    print('Testing Relaxed Posture with Loaded Ki=40.28 across 4 conditions in parallel...')
    conditions = [(0.0, "Zero_Wp"), (4.0, "Relaxed_Wp4"), (25.0, "Low_Wp25"), (400.0, "Original_Wp400")]
    with ProcessPoolExecutor(max_workers=4) as executor:
        futures = [executor.submit(test_condition, wp, lbl) for wp, lbl in conditions]
        for f in futures:
            f.result()
