import os
import sys
import math
import numpy as np
import scipy.signal as signal
import opensim as osim

def create_multisine_spline(targets, duration=45.0, delay=1.5, ramp=1.0):
    """
    Creates a crest-factor-optimized Schroeder multisine displacement spline
    containing all 6 canonical frequencies simultaneously.
    """
    # Canonical targets: (freq, target_gain, target_phase)
    # Gain is deg/m, phase in deg
    K = len(targets)
    freqs = np.array([t[0] for t in targets])
    # Schroeder phase angles to minimize peak-to-peak amplitude
    phases = np.array([math.pi * k * (k - 1) / K for k in range(K)])
    
    # Scale individual amplitudes so total displacement remains safe within deadband
    # Nominal single-frequency amplitude: 0.0215 * (0.835 / f)^2
    base_amps = np.array([0.0215 * (0.835 / f)**2 for f in freqs])
    scale_factor = 0.35 / math.sqrt(K) # Keeps total peak-to-peak under ~2.5 cm
    amps = base_amps * scale_factor

    dt = 0.005 # 200 Hz
    t_arr = np.arange(0.0, duration + 0.1, dt)
    
    spline = osim.SimmSpline()
    
    # Pre-delay zeros
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
    """
    Exact analytical closed-form reference pitch response according to Happee (2017) Bode target.
    """
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
        A = amps[i] # in meters
        phi = phases[i]
        target_gain = targets[i][1] # deg / m
        target_phase_rad = math.radians(targets[i][2])
        
        # Steady-state harmonic response
        y_ref_deg += A * target_gain * math.sin(2.0 * math.pi * f * t_rel + phi + target_phase_rad)
        
    return w * y_ref_deg

def run_adaptive_prototype():
    print("="*75)
    print("PROTOTYPE: ONLINE FILTERED-X RECURSIVE LEAST SQUARES (FxRLS) ADAPTIVE CONTROL")
    print("Matching Happee (2017) Bode Response Online in a Single Multisine Simulation")
    print("="*75)
    
    TARGETS = [
        (0.324,  86.55,   -7.03),
        (0.783, 236.93,  -96.96),
        (1.099, 319.74, -109.23),
        (2.446, 423.34, -161.34),
        (5.583, 503.30, -201.38),
        (7.927, 572.69, -247.62)
    ]
    
    sim_duration = 30.0 # 30 seconds of multisine excitation
    delay = 1.5
    ramp = 1.0
    dt_step = 0.005 # 200 Hz control loop
    
    spline, freqs, amps, phases, scale_factor = create_multisine_spline(TARGETS, duration=sim_duration, delay=delay, ramp=ramp)
    print(f"[EXCITATION] Generated 6-frequency Schroeder multisine spline (Duration: {sim_duration}s, scale: {scale_factor:.3f})")

    # Load OpenSim libraries
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

    # Initial parameter centroid
    # 4 Adaptive Gains: [K_sc, K_ton, K_phas, kp_prop]
    theta_0 = np.array([0.450, 0.261, 4.500, 1.400])
    theta = theta_0.copy()
    theta_min = np.array([0.001, 0.001, 0.200, 0.150])
    theta_max = np.array([2.500, 2.500, 8.000, 4.000])

    # Fixed poles and zero ratios
    p_ton = 78.0
    r_lead_sc = 0.24
    p_sc = 90.0
    z_sc = r_lead_sc * p_sc
    p_phas = 15.0
    r_lead_ccr = 0.05
    p_ccr = 170.0
    z_ccr = r_lead_ccr * p_ccr
    k_dyn = 0.70
    k_stat = 1.15
    ki = 40.2786

    osim.PropertyHelper.setValueDouble(theta[1], controller.updPropertyByName("K_ton"))
    osim.PropertyHelper.setValueDouble(p_ton, controller.updPropertyByName("p_ton"))
    osim.PropertyHelper.setValueDouble(theta[0], controller.updPropertyByName("K_sc"))
    osim.PropertyHelper.setValueDouble(z_sc, controller.updPropertyByName("z_sc"))
    osim.PropertyHelper.setValueDouble(p_sc, controller.updPropertyByName("p_sc"))
    osim.PropertyHelper.setValueDouble(theta[2], controller.updPropertyByName("K_phas"))
    osim.PropertyHelper.setValueDouble(p_phas, controller.updPropertyByName("p_phas"))
    osim.PropertyHelper.setValueDouble(theta[3], controller.updPropertyByName("k_p"))
    osim.PropertyHelper.setValueDouble(theta[3], controller.updPropertyByName("Kp_proprioception"))
    osim.PropertyHelper.setValueDouble(z_ccr, controller.updPropertyByName("z_ccr"))
    osim.PropertyHelper.setValueDouble(p_ccr, controller.updPropertyByName("p_ccr"))
    osim.PropertyHelper.setValueDouble(k_dyn, controller.updPropertyByName("K_gamma_dyn"))
    osim.PropertyHelper.setValueDouble(k_stat, controller.updPropertyByName("K_gamma_stat"))
    osim.PropertyHelper.setValueDouble(0.010, controller.updPropertyByName("delay_vcr"))
    osim.PropertyHelper.setValueDouble(0.015, controller.updPropertyByName("delay_ccr"))

    # Anatomical split delay with Smith predictor enabled
    osim.PropertyHelper.setValueBool(True, controller.updPropertyByName("use_smith_predictor"))
    osim.PropertyHelper.setValueDouble(0.007, controller.updPropertyByName("delay_smith_proprio"))
    osim.PropertyHelper.setValueDouble(0.007, controller.updPropertyByName("delay_afferent_proprio"))

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

    coord_tx = model.updCoordinateSet().get("spine_tx")
    coord_tx.set_locked(False)
    coord_tx.setDefaultIsPrescribed(True)
    coord_tx.setPrescribedFunction(spline)

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

    coord_p1 = model.getCoordinateSet().get("pitch1")
    coord_p2 = model.getCoordinateSet().get("pitch2")

    # -------------------------------------------------------------------------
    # Secondary Path Forward Model G_neck(s)
    # 2nd-order head inertia + damping + stiffness:
    # G(s) = 1 / (I*s^2 + B*s + K)
    # -------------------------------------------------------------------------
    I_head = 0.05
    B_neck = 1.5
    K_neck = 20.0
    sys_cont = signal.TransferFunction([1.0], [I_head, B_neck, K_neck])
    sys_disc = signal.cont2discrete((sys_cont.num, sys_cont.den), dt_step, method='bilinear')
    b_filt = sys_disc[0][0]
    a_filt = sys_disc[1]

    # Regressor history buffers for filtering (4 parameters)
    m_params = len(theta)
    psi_hist = np.zeros((m_params, len(b_filt)))
    psif_hist = np.zeros((m_params, len(a_filt) - 1))

    # Continuous RLS state
    # Covariance P matrix: initialized with moderate variance
    P = np.eye(m_params) * 0.5
    P[0, 0] = 0.2  # K_sc
    P[1, 1] = 0.2  # K_ton
    P[2, 2] = 1.0  # K_phas
    P[3, 3] = 0.5  # kp_prop
    
    lambda_forget = 0.9998 # Very slow forgetting for stability
    sigma_leak = 0.01      # Leaky leakage to anchor null-space

    current_time = 0.0
    step = dt_step
    
    print("\n[ADAPTATION LOOP STARTED] Simulating single continuous 30-second multisine run...")
    print(f"Initial Parameters: K_sc={theta[0]:.3f}, K_ton={theta[1]:.3f}, K_phas={theta[2]:.3f}, kp={theta[3]:.3f}")
    print(f"{'Time':>6s} | {'HeadPitch':>9s} | {'RefPitch':>9s} | {'Error':>8s} | {'K_sc':>7s} | {'K_ton':>7s} | {'K_phas':>7s} | {'kp':>7s}")
    print("-" * 75)

    log_t = []
    log_theta = []
    log_err = []

    last_print = 0.0

    while current_time < sim_duration:
        current_time = min(current_time + step, sim_duration)
        state = manager.integrate(current_time)

        p1_deg = math.degrees(coord_p1.getValue(state))
        p2_deg = math.degrees(coord_p2.getValue(state))
        head_pitch_deg = p1_deg + p2_deg
        head_pitch_rad = math.radians(head_pitch_deg)

        ref_pitch_deg = compute_reference_pitch(current_time, freqs, amps, phases, TARGETS, delay=delay, ramp=ramp)
        ref_pitch_rad = math.radians(ref_pitch_deg)

        err_rad = head_pitch_rad - ref_pitch_rad
        err_deg = head_pitch_deg - ref_pitch_deg

        # Extract current sensory signals as regressors
        # 1. psi_sc: angular velocity around pitch (rad/s)
        w_pitch = model.getStateVariableValue(state, "/controllerset/VCR_CCR_Controller/luen_omega_pitch")
        # 2. psi_ton: estimated attitude pitch error (rad)
        q0 = model.getStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q0")
        q1 = model.getStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q1")
        q2 = model.getStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q2")
        q3 = model.getStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q3")
        norm_q = math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
        if norm_q > 1e-4: q0/=norm_q; q1/=norm_q; q2/=norm_q; q3/=norm_q
        est_pitch = math.atan2(2.0*(q0*q3 + q1*q2), 1.0 - 2.0*(q2*q2 + q3*q3))
        # 3. psi_phas: linear acceleration along x (m/s^2)
        # 4. psi_sp: spindle firing rate proxy (approx ~1.0)
        psi_raw = np.array([
            w_pitch,
            est_pitch,
            math.sin(current_time * 2.0 * math.pi * 1.0), # Phase-locked excitation proxy
            0.5 * abs(w_pitch)
        ])

        # Filter regressor through G_neck(s) to produce sensitivity regressor psi_f
        psi_f = np.zeros(m_params)
        for i in range(m_params):
            psi_hist[i] = np.roll(psi_hist[i], 1)
            psi_hist[i, 0] = psi_raw[i]
            
            val = np.dot(b_filt, psi_hist[i]) - np.dot(a_filt[1:], psif_hist[i])
            psif_hist[i] = np.roll(psif_hist[i], 1)
            psif_hist[i, 0] = val
            psi_f[i] = val

        # Only adapt after initial transient ramp (t > 2.5s) to avoid adapting on settling transients
        if current_time > (delay + ramp):
            # RLS Gain vector: k = (P * psi_f) / (lambda + psi_f^T * P * psi_f)
            denom = lambda_forget + float(psi_f.T @ P @ psi_f)
            if denom > 1e-6:
                k_gain = (P @ psi_f) / denom
                
                # Parameter adaptation with sigma-leakage toward theta_0
                delta_theta = - k_gain * err_rad - (sigma_leak * (P @ (theta - theta_0))) * step
                theta = theta + delta_theta
                theta = np.clip(theta, theta_min, theta_max)
                
                # Covariance update
                P = (P - np.outer(k_gain, psi_f.T @ P)) / lambda_forget

            # Dynamically update OpenSim controller properties
            osim.PropertyHelper.setValueDouble(float(theta[0]), controller.updPropertyByName("K_sc"))
            osim.PropertyHelper.setValueDouble(float(theta[1]), controller.updPropertyByName("K_ton"))
            osim.PropertyHelper.setValueDouble(float(theta[2]), controller.updPropertyByName("K_phas"))
            osim.PropertyHelper.setValueDouble(float(theta[3]), controller.updPropertyByName("k_p"))
            osim.PropertyHelper.setValueDouble(float(theta[3]), controller.updPropertyByName("Kp_proprioception"))

        log_t.append(current_time)
        log_theta.append(theta.copy())
        log_err.append(err_deg)

        if current_time - last_print >= 2.5 or current_time >= sim_duration:
            last_print = current_time
            print(f"{current_time:5.1f}s | {head_pitch_deg:8.2f}° | {ref_pitch_deg:8.2f}° | {err_deg:7.2f}° | {theta[0]:7.3f} | {theta[1]:7.3f} | {theta[2]:7.3f} | {theta[3]:7.3f}")

    print("\n" + "="*75)
    print("## ONLINE ADAPTATION CONVERGENCE SUMMARY")
    print("="*75)
    print(f"Simulation Duration: {sim_duration}s continuous multisine")
    print(f"Final Converged Parameters:")
    print(f"  * K_sc:   {theta[0]:.4f} Nm/(rad/s) (Initial: {theta_0[0]:.4f})")
    print(f"  * K_ton:  {theta[1]:.4f} Nm/rad     (Initial: {theta_0[1]:.4f})")
    print(f"  * K_phas: {theta[2]:.4f} Nm/(m/s^2) (Initial: {theta_0[2]:.4f})")
    print(f"  * kp:     {theta[3]:.4f}            (Initial: {theta_0[3]:.4f})")
    
    steady_err = np.array(log_err)[np.array(log_t) > (delay + ramp)]
    print(f"Steady-State Tracking Error RMSE: {np.sqrt(np.mean(steady_err**2)):.2f}° (Max: {np.max(np.abs(steady_err)):.2f}°)")
    print("="*75)

if __name__ == "__main__":
    run_adaptive_prototype()
