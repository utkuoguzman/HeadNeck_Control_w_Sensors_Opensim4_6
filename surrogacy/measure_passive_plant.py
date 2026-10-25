"""
surrogacy/measure_passive_plant.py
Step 1 of the Differentiable Neuromotor Surrogate Pipeline.

Measures the exact empirical multi-body passive plant frequency response G_passive(j*omega)
from the full non-linear HYOID OpenSim model across the 6 canonical Happee (2017) frequencies.
All sensory reflexes are zeroed, while the calibrated gravity trim (Ki = 40.28, I0 = 0.10528)
is loaded to maintain upright posture (mean pitch ~ 2.8 deg).

Outputs:
    surrogacy/opensim_passive_plant.json: Exact G_passive complex table.
"""

import opensim as osim
import os
import math
import numpy as np
import json
import sys
from concurrent.futures import ProcessPoolExecutor

osim.LoadOpenSimLibrary('proprioception_plugin/build/Release/osimMillard12EqWithAff.dll')
osim.LoadOpenSimLibrary('HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll')
osim.LoadOpenSimLibrary('FixationController_plugin/build/Release/osimFixation.dll')

FREQS = [0.324, 0.783, 1.099, 2.446, 5.583, 7.927]

def measure_frequency(freq):
    model = osim.Model('osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim')
    for i in range(model.getControllerSet().getSize()):
        ctrl = model.getControllerSet().get(i)
        if ctrl.getConcreteClassName() == 'FixationController': ctrl.set_enabled(False)
        elif ctrl.getConcreteClassName() == 'HeadNeckNeuralController': controller = ctrl

    # Set sensory reflexes to virtually zero (pure passive plant)
    osim.PropertyHelper.setValueDouble(0.001, controller.updPropertyByName('K_ton'))
    osim.PropertyHelper.setValueDouble(2.0, controller.updPropertyByName('p_ton'))
    osim.PropertyHelper.setValueDouble(0.001, controller.updPropertyByName('K_sc'))
    osim.PropertyHelper.setValueDouble(0.5, controller.updPropertyByName('z_sc'))
    osim.PropertyHelper.setValueDouble(15.0, controller.updPropertyByName('p_sc'))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName('K_phas'))
    osim.PropertyHelper.setValueDouble(60.0, controller.updPropertyByName('p_phas'))
    osim.PropertyHelper.setValueDouble(0.001, controller.updPropertyByName('k_p'))
    osim.PropertyHelper.setValueDouble(0.001, controller.updPropertyByName('Kp_proprioception'))
    osim.PropertyHelper.setValueDouble(0.5, controller.updPropertyByName('z_ccr'))
    osim.PropertyHelper.setValueDouble(90.0, controller.updPropertyByName('p_ccr'))
    osim.PropertyHelper.setValueDouble(0.25, controller.updPropertyByName('K_gamma_dyn'))
    osim.PropertyHelper.setValueDouble(0.40, controller.updPropertyByName('K_gamma_stat'))
    osim.PropertyHelper.setValueDouble(0.010, controller.updPropertyByName('delay_vcr'))
    osim.PropertyHelper.setValueDouble(0.015, controller.updPropertyByName('delay_ccr'))

    # Loaded Ki for gravity trim
    ki_val = 40.2786
    osim.PropertyHelper.setValueDouble(ki_val, controller.updPropertyByName('Ki_task'))

    # Relaxed voluntary pitch
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName('Wp_base_pitch'))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName('Wv_base_pitch'))

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
        w = 10.0 * ((dt/ramp_duration)**3) - 15.0 * ((dt/ramp_duration)**4) + 6.0 * ((dt/ramp_duration)**5) if dt < ramp_duration else 1.0
        sine_function.addPoint(t, w * amplitude * math.sin(2.0 * math.pi * freq * dt))

    coord_tx = model.updCoordinateSet().get('spine_tx')
    coord_tx.set_locked(False)
    coord_tx.setDefaultIsPrescribed(True)
    coord_tx.setPrescribedFunction(sine_function)

    state = model.initSystem()
    c_yaw1.setIsPrescribed(state, False)
    coord_tx.setSpeedValue(state, 0.0)

    # Initial gravity trim state
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
    step = min(0.01, period / 40.0)
    curr = 0.0
    while curr < target_time:
        curr = min(curr + step, target_time)
        state = manager.integrate(curr)
        hp = math.degrees(coord_p1.getValue(state) + coord_p2.getValue(state))
        t_hist.append(curr)
        p_hist.append(hp)

    # Detrended Fourier Decomposition
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
    amp_deg = np.sqrt(Is**2 + Ic**2)
    phase_rad = math.atan2(Ic, Is)
    phase_deg = math.degrees(phase_rad)
    while phase_deg > 90.0: phase_deg -= 360.0
    while phase_deg < -270.0: phase_deg += 360.0

    gain_deg_m = amp_deg / amplitude
    print(f"[{freq:5.3f} Hz] Passive Plant: Gain = {gain_deg_m:7.2f} deg/m | Phase = {phase_deg:6.1f} deg | Mean Pitch = {np.mean(p_st):5.2f} deg")
    return {
        "freq": freq,
        "omega": omega,
        "amplitude": amplitude,
        "gain_deg_m": gain_deg_m,
        "phase_deg": phase_deg,
        "Is": Is,
        "Ic": Ic,
        "mean_pitch": float(np.mean(p_st))
    }

if __name__ == '__main__':
    print("Measuring Exact Passive OpenSim Plant across 6 Frequencies in parallel...")
    results = {}
    with ProcessPoolExecutor(max_workers=6) as executor:
        futures = {executor.submit(measure_frequency, f): f for f in FREQS}
        for fut in futures:
            res = fut.result()
            results[str(res["freq"])] = res

    os.makedirs("surrogacy", exist_ok=True)
    out_file = "surrogacy/opensim_passive_plant.json"
    with open(out_file, "w") as fp:
        json.dump(results, fp, indent=2)
    print(f"Saved exact OpenSim passive plant to {out_file}")

