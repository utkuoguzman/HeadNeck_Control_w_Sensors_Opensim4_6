import opensim as osim
import numpy as np
import math
import os
import sys

def test_anchor(ki_val=40.2786, wp_val=400.0, freq=0.324):
    os.environ["SIMB_NUM_THREADS"] = "1"
    os.environ["OMP_NUM_THREADS"] = "1"
    osim.Logger.setLevelString("Warn")
    osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'proprioception_plugin/build/Release/osimMillard12EqWithAff.dll'))
    osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll'))
    osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'FixationController_plugin/build/Release/osimFixation.dll'))

    model = osim.Model("osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim")
    controller = None
    for i in range(model.getControllerSet().getSize()):
        ctrl = model.getControllerSet().get(i)
        if ctrl.getConcreteClassName() == "FixationController":
            ctrl.set_enabled(False)
        elif ctrl.getConcreteClassName() == "HeadNeckNeuralController":
            controller = ctrl

    # Best Gen 1 params from W16
    # params: K_ton=4.58, p_ton=6.81, K_sc=3.49, z_sc=11.65, p_sc=62.19, K_phas=1.55, p_phas=33.0, kp_prop=0.706, z_ccr=16.38, p_ccr=51.62, k_dyn=0.26, k_stat=0.58
    osim.PropertyHelper.setValueDouble(4.58, controller.updPropertyByName("K_ton"))
    osim.PropertyHelper.setValueDouble(6.81, controller.updPropertyByName("p_ton"))
    osim.PropertyHelper.setValueDouble(3.49, controller.updPropertyByName("K_sc"))
    osim.PropertyHelper.setValueDouble(11.65, controller.updPropertyByName("z_sc"))
    osim.PropertyHelper.setValueDouble(62.19, controller.updPropertyByName("p_sc"))
    osim.PropertyHelper.setValueDouble(1.55, controller.updPropertyByName("K_phas"))
    osim.PropertyHelper.setValueDouble(33.0, controller.updPropertyByName("p_phas"))
    osim.PropertyHelper.setValueDouble(0.706, controller.updPropertyByName("k_p"))
    osim.PropertyHelper.setValueDouble(0.706, controller.updPropertyByName("Kp_proprioception"))
    osim.PropertyHelper.setValueDouble(16.38, controller.updPropertyByName("z_ccr"))
    osim.PropertyHelper.setValueDouble(51.62, controller.updPropertyByName("p_ccr"))
    osim.PropertyHelper.setValueDouble(0.26, controller.updPropertyByName("K_gamma_dyn"))
    osim.PropertyHelper.setValueDouble(0.58, controller.updPropertyByName("K_gamma_stat"))
    osim.PropertyHelper.setValueDouble(0.010, controller.updPropertyByName("delay_vcr"))
    osim.PropertyHelper.setValueDouble(0.015, controller.updPropertyByName("delay_ccr"))

    for ax in ['pitch', 'roll', 'yaw']:
        osim.PropertyHelper.setValueDouble(wp_val, controller.updPropertyByName(f"Wp_base_{ax}"))
        osim.PropertyHelper.setValueDouble(40.0, controller.updPropertyByName(f"Wv_base_{ax}"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("k_vigor"))
    osim.PropertyHelper.setValueDouble(0.1, controller.updPropertyByName("trajectory_tau"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("K_eso"))
    osim.PropertyHelper.setValueDouble(ki_val, controller.updPropertyByName("Ki_task"))

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

    step = min(0.01, (1.0/freq) / 40.0)
    current_time = 0.0
    coord_p1 = model.getCoordinateSet().get("pitch1")
    coord_p2 = model.getCoordinateSet().get("pitch2")

    t_history = []
    pitch_history = []

    while current_time < target_time:
        current_time = min(current_time + step, target_time)
        state = manager.integrate(current_time)
        head_pitch_deg = math.degrees(coord_p1.getValue(state) + coord_p2.getValue(state))
        t_history.append(current_time)
        pitch_history.append(head_pitch_deg)

    discard_time = delay + ramp_duration
    valid_indices = [i for i, t in enumerate(t_history) if t >= discard_time]
    t_steady = np.array([t_history[i] for i in valid_indices])
    pitch_steady = np.array([pitch_history[i] for i in valid_indices])

    omega = 2.0 * math.pi * freq
    t_shifted = t_steady - delay
    coeffs = np.polyfit(t_steady, pitch_steady, 1)
    pitch_detrended = pitch_steady - (coeffs[0] * t_steady + coeffs[1])

    Is = 2.0 * np.mean(pitch_detrended * np.sin(omega * t_shifted))
    Ic = 2.0 * np.mean(pitch_detrended * np.cos(omega * t_shifted))
    amp_deg = np.sqrt(Is**2 + Ic**2)
    phase_rad = math.atan2(Ic, Is)
    gain_deg_m = amp_deg / amplitude
    phase_deg = math.degrees(phase_rad)
    while phase_deg > 90.0: phase_deg -= 360.0
    while phase_deg < -270.0: phase_deg += 360.0

    target_gain = 86.55
    target_phase = -7.03
    gain_db_diff = 20.0 * math.log10(max(1e-6, gain_deg_m) / target_gain)
    gain_loss = (gain_db_diff ** 2) * 100.0
    phase_diff = (phase_deg - target_phase + 180.0) % 360.0 - 180.0
    phase_loss = ((phase_diff / 10.0) ** 2) * 100.0
    total_score = gain_loss + phase_loss

    print(f"Ki={ki_val:.2f}, Wp={wp_val:.1f} | amp_deg={amp_deg:.2f} deg, gain={gain_deg_m:.2f} deg/m (Tgt: {target_gain}) | GainLoss={gain_loss:.1f}, PhaseLoss={phase_loss:.1f} | Score={total_score:.1f}")

if __name__ == "__main__":
    print("Testing baseline:")
    test_anchor(ki_val=40.2786, wp_val=400.0)
    print("Testing with Ki=0.0, Wp=400.0:")
    test_anchor(ki_val=0.0, wp_val=400.0)
    print("Testing with Ki=0.0, Wp=0.0 (Pure Reflexive like Happee):")
    test_anchor(ki_val=0.0, wp_val=0.0)

