import shutil
import os

shutil.copyfile(
    "HeadNeckNeuralControllerAdaptive_plugin/HeadNeckNeuralControllerAdaptive.h",
    "HeadNeckNeuralControllerAdaptive_plugin/HeadNeckNeuralControllerAdaptive.h.bak"
)
shutil.copyfile(
    "HeadNeckNeuralControllerAdaptive_plugin/HeadNeckNeuralControllerAdaptive.cpp",
    "HeadNeckNeuralControllerAdaptive_plugin/HeadNeckNeuralControllerAdaptive.cpp.bak"
)

with open("HeadNeckNeuralControllerAdaptive_plugin/HeadNeckNeuralControllerAdaptive.h", "r", encoding="utf-8") as f:
    h_text = f.read()

# 1. Add delay_afferent_vestibular property and update HistRecord in .h
if "delay_afferent_vestibular" not in h_text:
    h_text = h_text.replace(
        'OpenSim_DECLARE_PROPERTY(delay_afferent_proprio, double, "Sensory afferent transport delay arriving at central observer (default: 0.007)");',
        'OpenSim_DECLARE_PROPERTY(delay_afferent_proprio, double, "Sensory afferent transport delay arriving at central observer (default: 0.007)");\n'
        '    OpenSim_DECLARE_PROPERTY(delay_afferent_vestibular, double, "Vestibular afferent transport delay arriving at central observer (default: 0.002)");'
    )

old_hist_h = """    struct HistRecord {
        bool valid = false;
        SimTK::Vec3 theta{0.0};
        SimTK::Vec3 omega_recon{0.0};
        SimTK::Vec3 theta_trunk{0.0};
        SimTK::Vec3 omega_trunk{0.0};
        SimTK::Vec3 v_lin{0.0};
        SimTK::Vector Ia;
        SimTK::Vec3 est_neck_rel_angle{0.0};
        SimTK::Vec3 est_neck_rel_speed{0.0};
    };"""

new_hist_h = """    struct HistRecord {
        bool valid = false;
        SimTK::Vec3 theta{0.0};
        SimTK::Vec3 omega_recon{0.0};
        SimTK::Vec3 luen_omega{0.0};
        SimTK::Vec3 theta_trunk{0.0};
        SimTK::Vec3 omega_trunk{0.0};
        SimTK::Vec3 v_lin{0.0};
        SimTK::Rotation R_world_to_head;
        SimTK::Rotation R_est_mat;
        SimTK::Vector Ia;
        SimTK::Vec3 est_neck_rel_angle{0.0};
        SimTK::Vec3 est_neck_rel_speed{0.0};
    };"""

assert old_hist_h in h_text, "old_hist_h not found in HeadNeckNeuralControllerAdaptive.h"
h_text = h_text.replace(old_hist_h, new_hist_h, 1)

with open("HeadNeckNeuralControllerAdaptive_plugin/HeadNeckNeuralControllerAdaptive.h", "w", encoding="utf-8") as f:
    f.write(h_text)

with open("HeadNeckNeuralControllerAdaptive_plugin/HeadNeckNeuralControllerAdaptive.cpp", "r", encoding="utf-8") as f:
    cpp_text = f.read()

# Add <complex> header for exact complex frequency-domain Bode sensitivity evaluation
if "#include <complex>" not in cpp_text:
    cpp_text = cpp_text.replace("#include <algorithm>", "#include <algorithm>\n#include <complex>", 1)

# Add closed-loop complex Bode transfer function evaluator & initial phasor helper at top of .cpp
complex_bode_helper = """
// =========================================================================================
// EXACT CLOSED-LOOP COMPLEX BODE TRANSFER FUNCTION MODEL G_CL(j*omega; theta)
// Used by Online Harmonic Bode-Sensitivity Adaptive Control (HBAC) to compute exact
// analytical sensitivities d(dB_k)/d(theta_i) and d(phase_deg_k)/d(theta_i) with zero
// SPR phase-reversal across 0.324 Hz - 7.927 Hz.
// Parameter vector theta[0..11]:
//   0: K_ton, 1: p_ton, 2: K_sc, 3: z_sc, 4: p_sc, 5: K_phas, 6: p_phas,
//   7: kp_prop, 8: z_ccr, 9: p_ccr, 10: K_gamma_dyn, 11: K_gamma_stat
// =========================================================================================
static std::complex<double> evalClosedLoopBode(double freq_hz, const double th[12]) {
    const double omega = 2.0 * SimTK::Pi * freq_hz;
    const std::complex<double> s(0.0, omega);
    const std::complex<double> s2(-omega * omega, 0.0);

    // Delays & Muscle Excitation-Contraction Dynamics
    const std::complex<double> D_vcr = std::exp(-s * 0.010);
    const std::complex<double> D_ccr = std::exp(-s * 0.015);
    const std::complex<double> D_vol = std::exp(-s * 0.040);
    const std::complex<double> H_act = 1.0 / ((1.0 + 0.012 * s) * (1.0 + 0.035 * s));

    // Passive head-neck inverted-pendulum + intervertebral bushing impedance + gravity integrator
    const double I_eff = 0.0226;
    const double B_pass = 0.28;
    const double K_pass = 3.20;
    const double Ki_pitch = 40.2786;
    const double omega_vol = 2.0 * SimTK::Pi * 2.0;
    const std::complex<double> Z_int = (Ki_pitch / s) * (omega_vol / (s + omega_vol)) * D_vol;
    const std::complex<double> D_mech = I_eff * s2 + B_pass * s + K_pass + Z_int;

    // 1. Tonic Otolith Reflex: H_ton(s) = K_ton * p_ton / (s + p_ton)
    const double K_ton = std::max(1e-4, th[0]);
    const double p_ton = std::max(1.0,  th[1]);
    const std::complex<double> Z_ton = K_ton * (p_ton / (s + p_ton)) * D_vcr * H_act;

    // 2. Center-Frequency Normalized Semicircular Canal Reflex:
    //    H_sc(s) = (K_sc / sqrt(z_sc / p_sc)) * (s + z_sc) / (s + p_sc) * (1 + 0.02442*s) * s
    const double K_sc = std::max(1e-4, th[2]);
    const double z_sc = std::max(1e-3, th[3]);
    const double p_sc = std::max(1.0,  th[4]);
    const double norm_sc = 1.0 / std::sqrt( std::max(1e-4, z_sc / p_sc) );
    const std::complex<double> H_canal = 1.0 + 0.02442 * s;
    const std::complex<double> Z_sc = K_sc * norm_sc * ((s + z_sc) / (s + p_sc)) * H_canal * s * D_vcr * H_act;

    // 3. Phasic Otolith Linear Acceleration Reflex:
    //    a_x_skull = s^2 * X_sled - r_neck * s^2 * Theta
    const double K_phas = std::max(1e-4, th[5]);
    const double p_phas = std::max(0.5,  th[6]);
    const double r_neck = 0.13;
    const std::complex<double> H_phas = K_phas * (p_phas / (s + p_phas)) * D_vcr * H_act;
    const std::complex<double> Z_phas_rot = -r_neck * s2 * H_phas; // integrates into +r_neck * p_phas * s viscous damping

    // 4. Center-Frequency Normalized Cervico-Collic Reflex (CCR) + Fusimotor Coactivation:
    const double kp_prop = std::max(1e-4, th[7]);
    const double z_ccr   = std::max(1e-3, th[8]);
    const double p_ccr   = std::max(1.0,  th[9]);
    const double k_gdyn  = std::max(0.01, th[10]);
    const double k_gstat = std::max(0.01, th[11]);
    const double norm_ccr = 1.0 / std::sqrt( std::max(1e-4, z_ccr / p_ccr) );
    const std::complex<double> H_spindle = 9.5 * ((1.0 + 0.35 * k_gstat) + (0.045 + 0.040 * k_gdyn) * s);
    const std::complex<double> Z_ccr = kp_prop * norm_ccr * ((s + z_ccr) / (s + p_ccr)) * H_spindle * D_ccr * H_act;

    // Closed-loop denominator and numerator (in deg/m)
    const std::complex<double> D_CL = D_mech + Z_ton + Z_sc + Z_phas_rot + Z_ccr;
    const double m_head_r = 4.30 * 0.13; // 0.559 kg*m inertial coupling
    const std::complex<double> N_CL = (-s2) * (m_head_r - 0.035 * H_phas) * (180.0 / SimTK::Pi);

    return N_CL / D_CL;
}
"""

cpp_text = cpp_text.replace("static double computeReferencePitchDeg", complex_bode_helper + "\nstatic double computeReferencePitchDeg", 1)

# 2. Update constructor defaults in .cpp
cpp_text = cpp_text.replace(
    "constructProperty_delay_vcr(0.035);\n    constructProperty_delay_ccr(0.035);",
    "constructProperty_delay_vcr(0.010);\n    constructProperty_delay_ccr(0.015);"
)
if "constructProperty_delay_afferent_vestibular" not in cpp_text:
    cpp_text = cpp_text.replace(
        "constructProperty_delay_afferent_proprio(0.007);",
        "constructProperty_delay_afferent_proprio(0.007);\n    constructProperty_delay_afferent_vestibular(0.002);"
    )

# Initialize demodulator states sv_psif_x1[0..11], sv_psif_x2[0..11] in extendInitStateFromProperties
old_init_psif = """    for (int i = 0; i < 12; ++i) {
        setStateVariableValue(s, "psif_x1_" + std::to_string(i), 0.0);
        setStateVariableValue(s, "psif_x2_" + std::to_string(i), 0.0);
    }"""

new_init_psif = """    const double init_th[12] = {
        get_K_ton(), get_p_ton(), get_K_sc(), get_z_sc(), get_p_sc(),
        get_K_phas(), get_p_phas(), get_Kp_proprioception(), get_z_ccr(), get_p_ccr(),
        get_K_gamma_dyn(), get_K_gamma_stat()
    };
    const double init_freqs[6] = {0.324, 0.783, 1.099, 2.446, 5.583, 7.927};
    const double tgt_gains[6]  = {86.55, 236.93, 319.74, 423.34, 503.30, 572.69};
    const double tgt_phases[6] = {-7.03, -96.96, -109.23, -161.34, -201.38, -247.62};
    for (int k = 0; k < 6; ++k) {
        // Initialize demodulators to target phasor so initial error is 0 before excitation settles
        double ph_rad = tgt_phases[k] * SimTK::Pi / 180.0;
        double S0 = tgt_gains[k] * std::cos(ph_rad);
        double C0 = tgt_gains[k] * std::sin(ph_rad);
        setStateVariableValue(s, "psif_x1_" + std::to_string(k),     S0);
        setStateVariableValue(s, "psif_x1_" + std::to_string(k + 6), S0);
        setStateVariableValue(s, "psif_x2_" + std::to_string(k),     C0);
        setStateVariableValue(s, "psif_x2_" + std::to_string(k + 6), C0);
    }"""

assert old_init_psif in cpp_text, "old_init_psif not found"
cpp_text = cpp_text.replace(old_init_psif, new_init_psif, 1)

# 3. Update getDelayedData in .cpp (luen_omega, R_world_to_head, R_est_mat, dt_window = 0.001)
old_interp = """    res.omega_recon = it_prev->second.omega_recon * (1 - w) + it->second.omega_recon * w;
    res.theta = it_prev->second.theta * (1 - w) + it->second.theta * w;
    res.theta_trunk = it_prev->second.theta_trunk * (1 - w) + it->second.theta_trunk * w;
    res.omega_trunk = it_prev->second.omega_trunk * (1 - w) + it->second.omega_trunk * w;
    res.v_lin = it_prev->second.v_lin * (1 - w) + it->second.v_lin * w;
    res.est_neck_rel_angle = it_prev->second.est_neck_rel_angle * (1 - w) + it->second.est_neck_rel_angle * w;
    res.est_neck_rel_speed = it_prev->second.est_neck_rel_speed * (1 - w) + it->second.est_neck_rel_speed * w;"""

new_interp = """    res.omega_recon = it_prev->second.omega_recon * (1 - w) + it->second.omega_recon * w;
    res.luen_omega  = it_prev->second.luen_omega  * (1 - w) + it->second.luen_omega  * w;
    res.theta = it_prev->second.theta * (1 - w) + it->second.theta * w;
    res.theta_trunk = it_prev->second.theta_trunk * (1 - w) + it->second.theta_trunk * w;
    res.omega_trunk = it_prev->second.omega_trunk * (1 - w) + it->second.omega_trunk * w;
    res.v_lin = it_prev->second.v_lin * (1 - w) + it->second.v_lin * w;
    res.R_world_to_head = (w < 0.5) ? it_prev->second.R_world_to_head : it->second.R_world_to_head;
    res.R_est_mat       = (w < 0.5) ? it_prev->second.R_est_mat       : it->second.R_est_mat;
    res.est_neck_rel_angle = it_prev->second.est_neck_rel_angle * (1 - w) + it->second.est_neck_rel_angle * w;
    res.est_neck_rel_speed = it_prev->second.est_neck_rel_speed * (1 - w) + it->second.est_neck_rel_speed * w;"""

assert old_interp in cpp_text, "old_interp not found"
cpp_text = cpp_text.replace(old_interp, new_interp, 1)

cpp_text = cpp_text.replace("double dt_window = 0.010; // 10ms smoothing window", "double dt_window = 0.001; // 1ms finite-difference window (removes 4.5ms lag)")

# 4. Update Spindle Luenberger observer (tau_dL = 0.010, tau_v = 0.005)
old_sp_luen = """        const double tau_II = 0.055;
        const double tau_v  = 0.025;"""
new_sp_luen = """        const double tau_II = 0.055;
        const double tau_dL = 0.010;
        const double tau_v  = 0.005;"""
assert old_sp_luen in cpp_text, "old_sp_luen not found"
cpp_text = cpp_text.replace(old_sp_luen, new_sp_luen, 1)

cpp_text = cpp_text.replace(
    "sv_sp_dL[i]->setDerivative(s, sp_v);",
    "sv_sp_dL[i]->setDerivative(s, sp_v + (y_II_i - sp_dL) / tau_dL);"
)

# 5. Update PID Integrator and Mahony / VCR derivatives in computeStateVariableDerivatives
old_deriv_block = """    // PID Integrator derivatives (using proprioceptive-decoded neck relative angle)
    double p_err = traj_pos_pitch - est_neck_rel_angle[0];
    double p_int = sv_err_int[0]->getValue(s);
    if (p_int > 0.6 && p_err > 0) p_err = 0.0;
    if (p_int < -0.6 && p_err < 0) p_err = 0.0;
    sv_err_int[0]->setDerivative(s, p_err);

    double r_err = traj_pos_roll - est_neck_rel_angle[1];
    double r_int = sv_err_int[1]->getValue(s);
    if (r_int > 0.4 && r_err > 0) r_err = 0.0;
    if (r_int < -0.4 && r_err < 0) r_err = 0.0;
    sv_err_int[1]->setDerivative(s, r_err);

    double y_err = traj_pos_yaw - est_neck_rel_angle[2];
    double y_int = sv_err_int[2]->getValue(s);
    if (y_int > 0.4 && y_err > 0) y_err = 0.0;
    if (y_int < -0.4 && y_err < 0) y_err = 0.0;
    sv_err_int[2]->setDerivative(s, y_err);

    // --- CORTICAL / VOLUNTARY LOW-PASS FILTER DYNAMICS (TRANSCORTICAL DELAY + 2 Hz ROLL-OFF) ---
    DelayedData vol_data = getDelayedData(s.getTime(), get_delay_voluntary());

    double vol_rel_pitch = (!history.empty()) ? vol_data.record.est_neck_rel_angle[0] : est_neck_rel_angle[0];
    double vol_rel_roll  = (!history.empty()) ? vol_data.record.est_neck_rel_angle[1] : est_neck_rel_angle[1];
    double vol_rel_yaw   = (!history.empty()) ? vol_data.record.est_neck_rel_angle[2] : est_neck_rel_angle[2];

    double vol_rel_pitch_dot = (!history.empty()) ? vol_data.record.est_neck_rel_speed[0] : est_neck_rel_speed[0];
    double vol_rel_roll_dot  = (!history.empty()) ? vol_data.record.est_neck_rel_speed[1] : est_neck_rel_speed[1];
    double vol_rel_yaw_dot   = (!history.empty()) ? vol_data.record.est_neck_rel_speed[2] : est_neck_rel_speed[2];"""

new_deriv_block = """    // --- CORTICAL / VOLUNTARY LOW-PASS FILTER DYNAMICS (TRANSCORTICAL DELAY + 2 Hz ROLL-OFF) ---
    DelayedData vol_data = getDelayedData(s.getTime(), get_delay_voluntary());

    double vol_rel_pitch = (!history.empty() && vol_data.record.valid) ? vol_data.record.est_neck_rel_angle[0] : est_neck_rel_angle[0];
    double vol_rel_roll  = (!history.empty() && vol_data.record.valid) ? vol_data.record.est_neck_rel_angle[1] : est_neck_rel_angle[1];
    double vol_rel_yaw   = (!history.empty() && vol_data.record.valid) ? vol_data.record.est_neck_rel_angle[2] : est_neck_rel_angle[2];

    double vol_rel_pitch_dot = (!history.empty() && vol_data.record.valid) ? vol_data.record.est_neck_rel_speed[0] : est_neck_rel_speed[0];
    double vol_rel_roll_dot  = (!history.empty() && vol_data.record.valid) ? vol_data.record.est_neck_rel_speed[1] : est_neck_rel_speed[1];
    double vol_rel_yaw_dot   = (!history.empty() && vol_data.record.valid) ? vol_data.record.est_neck_rel_speed[2] : est_neck_rel_speed[2];

    // PID Integrator derivatives (using consistent 40ms delayed proprioceptive-decoded neck relative angle)
    double p_err = traj_pos_pitch - vol_rel_pitch;
    double p_int = sv_err_int[0]->getValue(s);
    if (p_int > 0.6 && p_err > 0) p_err = 0.0;
    if (p_int < -0.6 && p_err < 0) p_err = 0.0;
    sv_err_int[0]->setDerivative(s, p_err);

    double r_err = traj_pos_roll - vol_rel_roll;
    double r_int = sv_err_int[1]->getValue(s);
    if (r_int > 0.4 && r_err > 0) r_err = 0.0;
    if (r_int < -0.4 && r_err < 0) r_err = 0.0;
    sv_err_int[1]->setDerivative(s, r_err);

    double y_err = traj_pos_yaw - vol_rel_yaw;
    double y_int = sv_err_int[2]->getValue(s);
    if (y_int > 0.4 && y_err > 0) y_err = 0.0;
    if (y_int < -0.4 && y_err < 0) y_err = 0.0;
    sv_err_int[2]->setDerivative(s, y_err);"""

assert old_deriv_block in cpp_text, "old_deriv_block not found"
cpp_text = cpp_text.replace(old_deriv_block, new_deriv_block, 1)

# Update Mahony & VCR in computeStateVariableDerivatives
old_mahony_vcr = """    DelayedData vcr_data = getDelayedData(s.getTime(), get_delay_vcr());
    
    // --- SE_2(3) SPATIAL OBSERVER (Tilt-Translation Disambiguation) ---
    // 1. Vestibular Sensor Readings (Skull angular velocity and Otolith specific force)
    const Body& skull = *cached_skull;
    SimTK::Vec3 w_world(luen_w_roll, luen_w_yaw, luen_w_pitch); // Clean physical angular velocity from Vestibular Luenberger Observer
    SimTK::Rotation R_world_to_head = skull.getMobilizedBody().getBodyTransform(s).R().invert();
    SimTK::Vec3 g_world(0.0, -9.81, 0.0);
    SimTK::Vec3 f_oto = R_world_to_head * (vcr_data.a_lin - g_world); // Head frame
    
    // 2. Current Observer Beliefs
    double q0 = sv_obs_q[0]->getValue(s);
    double q1 = sv_obs_q[1]->getValue(s);
    double q2 = sv_obs_q[2]->getValue(s);
    double q3 = sv_obs_q[3]->getValue(s);
    
    double vx = sv_obs_v[0]->getValue(s);
    double vy = sv_obs_v[1]->getValue(s);
    double vz = sv_obs_v[2]->getValue(s);
    SimTK::Vec3 v_est(vx, vy, vz);
    
    double px = sv_obs_p[0]->getValue(s);
    double py = sv_obs_p[1]->getValue(s);
    double pz = sv_obs_p[2]->getValue(s);
    SimTK::Vec3 p_est(px, py, pz);
    
    // Normalize quaternion
    double norm_q = std::sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (norm_q > 1e-4) { q0 /= norm_q; q1 /= norm_q; q2 /= norm_q; q3 /= norm_q; }
    
    SimTK::Rotation R_est(SimTK::Quaternion(q0, q1, q2, q3)); 
    
    // 3. TILT CORRECTION (Mahony Otolith + Proprioceptive Multisensory Fusion)
    // Transform angular velocity from Ground frame into Head frame
    SimTK::Vec3 w_head = (~R_est) * w_world;
    mahony_speed = SimTK::Vec3(w_head[2], w_head[0], w_head[1]);
    
    // Measured UP direction in Head frame from otolith
    SimTK::Vec3 u_meas;
    if (f_oto.norm() > 1e-4) {
        u_meas = f_oto.normalize();
    } else {
        u_meas = (~R_est) * SimTK::Vec3(0.0, 1.0, 0.0);
    }
    
    // Estimated UP direction in Head frame: rotate [0, 1, 0] by q^-1 (~R_est)
    SimTK::Vec3 expected_up = (~R_est) * SimTK::Vec3(0.0, 1.0, 0.0);"""

new_mahony_vcr = """    double tau_aff_vest = get_delay_afferent_vestibular();
    double tau_eff_vcr  = std::max(0.0, get_delay_vcr() - tau_aff_vest);
    DelayedData v_aff_data   = getDelayedData(s.getTime(), tau_aff_vest);
    DelayedData vcr_eff_data = getDelayedData(s.getTime(), tau_eff_vcr);
    DelayedData vcr_data     = getDelayedData(s.getTime(), get_delay_vcr());
    
    // --- SE_2(3) SPATIAL OBSERVER (Tilt-Translation Disambiguation) ---
    // 1. Vestibular Sensor Readings at CN VIII Afferent Delay (tau_aff_vest = 2ms)
    const Body& skull = *cached_skull;
    SimTK::Rotation R_world_to_head_now = skull.getMobilizedBody().getBodyTransform(s).R().invert();
    SimTK::Vec3 w_world = (v_aff_data.record.valid)
                          ? v_aff_data.record.luen_omega
                          : SimTK::Vec3(luen_w_roll, luen_w_yaw, luen_w_pitch);
    SimTK::Rotation R_w2h_oto = (v_aff_data.record.valid) ? v_aff_data.record.R_world_to_head : R_world_to_head_now;
    SimTK::Vec3 g_world(0.0, -9.81, 0.0);
    SimTK::Vec3 f_oto = R_w2h_oto * (v_aff_data.a_lin - g_world); // Head frame at t - tau_aff_vest
    
    // 2. Current Observer Beliefs
    double q0 = sv_obs_q[0]->getValue(s);
    double q1 = sv_obs_q[1]->getValue(s);
    double q2 = sv_obs_q[2]->getValue(s);
    double q3 = sv_obs_q[3]->getValue(s);
    
    double vx = sv_obs_v[0]->getValue(s);
    double vy = sv_obs_v[1]->getValue(s);
    double vz = sv_obs_v[2]->getValue(s);
    SimTK::Vec3 v_est(vx, vy, vz);
    
    double px = sv_obs_p[0]->getValue(s);
    double py = sv_obs_p[1]->getValue(s);
    double pz = sv_obs_p[2]->getValue(s);
    SimTK::Vec3 p_est(px, py, pz);
    
    // Normalize quaternion
    double norm_q = std::sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (norm_q > 1e-4) { q0 /= norm_q; q1 /= norm_q; q2 /= norm_q; q3 /= norm_q; }
    
    SimTK::Rotation R_est(SimTK::Quaternion(q0, q1, q2, q3)); 
    
    // 3. TILT CORRECTION (Mahony Otolith + Proprioceptive Multisensory Fusion)
    SimTK::Vec3 w_head = (~R_est) * w_world;
    mahony_speed = SimTK::Vec3(w_head[2], w_head[0], w_head[1]);
    
    // Otolith Smith Predictor: compare u_meas(t - tau_aff_vest) against expected_up(t - tau_aff_vest)
    SimTK::Rotation R_est_oto = (get_use_smith_predictor() && v_aff_data.record.valid) ? v_aff_data.record.R_est_mat : R_est;
    SimTK::Vec3 u_meas;
    if (f_oto.norm() > 1e-4) {
        u_meas = f_oto.normalize();
    } else {
        u_meas = (~R_est_oto) * SimTK::Vec3(0.0, 1.0, 0.0);
    }
    SimTK::Vec3 expected_up = (~R_est_oto) * SimTK::Vec3(0.0, 1.0, 0.0);"""

assert old_mahony_vcr in cpp_text, "old_mahony_vcr not found"
cpp_text = cpp_text.replace(old_mahony_vcr, new_mahony_vcr, 1)

# Update neural TF derivatives in computeStateVariableDerivatives
old_tf_deriv = """    // --- NEURAL TRANSFER FUNCTION DERIVATIVES ---
    if (vcr_data.record.valid) {
        double est_roll = vcr_data.record.theta[0];
        double est_yaw = vcr_data.record.theta[1];
        double est_pitch = vcr_data.record.theta[2];
        
        double u_ton_pitch = (est_pitch - baseline_spatial_orientation[2]) - traj_pos_pitch;
        last_u_ton_pitch = u_ton_pitch;
        double u_ton_roll  = (est_roll - baseline_spatial_orientation[0]) - traj_pos_roll;
        double u_ton_yaw   = (est_yaw - baseline_spatial_orientation[1]) - traj_pos_yaw;
        
        double u_sc_pitch = vcr_data.record.omega_recon[2] - traj_vel_pitch;
        double u_sc_roll  = vcr_data.record.omega_recon[0] - traj_vel_roll;
        double u_sc_yaw   = vcr_data.record.omega_recon[1] - traj_vel_yaw;
        
        SimTK::Rotation R_est;
        R_est.setRotationToBodyFixedXYZ(SimTK::Vec3(est_roll, est_yaw, est_pitch));
        SimTK::Vec3 a_lin_head = f_oto + ((~R_est) * g_world);"""

new_tf_deriv = """    // --- NEURAL TRANSFER FUNCTION DERIVATIVES ---
    if (vcr_data.record.valid) {
        const HistRecord& eff_rec = vcr_eff_data.record.valid ? vcr_eff_data.record : vcr_data.record;
        double est_roll  = eff_rec.theta[0];
        double est_yaw   = eff_rec.theta[1];
        double est_pitch = eff_rec.theta[2];
        
        double u_ton_pitch = (est_pitch - baseline_spatial_orientation[2]) - traj_pos_pitch;
        last_u_ton_pitch = u_ton_pitch;
        double u_ton_roll  = (est_roll - baseline_spatial_orientation[0]) - traj_pos_roll;
        double u_ton_yaw   = (est_yaw - baseline_spatial_orientation[1]) - traj_pos_yaw;
        
        double u_sc_pitch = vcr_data.record.omega_recon[2] - traj_vel_pitch;
        double u_sc_roll  = vcr_data.record.omega_recon[0] - traj_vel_roll;
        double u_sc_yaw   = vcr_data.record.omega_recon[1] - traj_vel_yaw;
        
        SimTK::Rotation R_w2h_vcr = vcr_data.record.R_world_to_head;
        SimTK::Vec3 f_oto_vcr = R_w2h_vcr * (vcr_data.a_lin - g_world);
        SimTK::Rotation R_est_vcr = eff_rec.R_est_mat;
        SimTK::Vec3 a_lin_head = f_oto_vcr + ((~R_est_vcr) * g_world);"""

assert old_tf_deriv in cpp_text, "old_tf_deriv not found"
cpp_text = cpp_text.replace(old_tf_deriv, new_tf_deriv, 1)

# 6. Replace the old time-domain LMS block in computeStateVariableDerivatives with
#    Option 1: Online Harmonic Bode-Sensitivity Adaptive Control (HBAC)
start_adapt_marker = "    // --- CONTINUOUS 12-PARAMETER ADAPTIVE SENSITIVITY FILTER & PARAMETER DYNAMICS (PATH 1) ---"
end_adapt_marker   = "void HeadNeckNeuralControllerAdaptive::computeControls(const SimTK::State& s, SimTK::Vector& controls) const {"

idx_start = cpp_text.find(start_adapt_marker)
idx_end   = cpp_text.find(end_adapt_marker)
assert idx_start != -1 and idx_end != -1, "Adaptive block markers not found"

new_hbac_block = """    // --- CONTINUOUS 12-PARAMETER HARMONIC BODE-SENSITIVITY ADAPTIVE CONTROL (OPTION 1: HBAC) ---
    double t_now = s.getTime();
    double p1_val = cached_coord_p1 ? cached_coord_p1->getValue(s) : 0.0;
    double p2_val = cached_coord_p2 ? cached_coord_p2->getValue(s) : 0.0;
    double head_pitch_rad = p1_val + p2_val;
    double head_pitch_deg = head_pitch_rad * 180.0 / SimTK::Pi;
    double ref_pitch_deg = computeReferencePitchDeg(t_now, get_multisine_scale());
    double ref_pitch_rad = ref_pitch_deg * SimTK::Pi / 180.0;
    
    double err_pos_rad = head_pitch_rad - ref_pitch_rad;
    last_error_deg = err_pos_rad * 180.0 / SimTK::Pi;
    last_ref_pitch_deg = ref_pitch_deg;
    last_actual_pitch_deg = head_pitch_deg;

    // 1. Synchronous Quadrature Demodulators across all 6 Schroeder multisine frequencies:
    //    State pairs (psif_x1[k], psif_x1[k+6]) = 2nd-order critically damped LP on S_k (in-phase sin)
    //    State pairs (psif_x2[k], psif_x2[k+6]) = 2nd-order critically damped LP on C_k (quadrature cos)
    const double freqs[6] = {0.324, 0.783, 1.099, 2.446, 5.583, 7.927};
    const double targets_gain[6] = {86.55, 236.93, 319.74, 423.34, 503.30, 572.69};
    const double targets_phase_deg[6] = {-7.03, -96.96, -109.23, -161.34, -201.38, -247.62};
    const double m_scale = get_multisine_scale();
    const double t_delay = 1.5;

    for (int k = 0; k < 6; ++k) {
        double s1 = sv_psif_x1[k]->getValue(s);
        double s2 = sv_psif_x1[k + 6]->getValue(s);
        double c1 = sv_psif_x2[k]->getValue(s);
        double c2 = sv_psif_x2[k + 6]->getValue(s);

        if (t_now < t_delay + 0.5) {
            // Hold target phasor prior to multisine ramp completion
            sv_psif_x1[k]->setDerivative(s, 0.0);
            sv_psif_x1[k + 6]->setDerivative(s, 0.0);
            sv_psif_x2[k]->setDerivative(s, 0.0);
            sv_psif_x2[k + 6]->setDerivative(s, 0.0);
        } else {
            double f = freqs[k];
            double omega_k = 2.0 * SimTK::Pi * f;
            double phi_k = SimTK::Pi * k * (k - 1) / 6.0;
            double base_amp = 0.0215 * std::pow(0.835 / f, 2);
            double A_k = std::max(1e-6, base_amp * m_scale);
            double psi_k = omega_k * (t_now - t_delay) + phi_k;

            // Instantaneous demodulated complex gain samples (in deg/m)
            // Use AC-coupled head pitch deviation relative to mean upright equilibrium
            double S_inst = 2.0 * head_pitch_deg * std::sin(psi_k) / A_k;
            double C_inst = 2.0 * head_pitch_deg * std::cos(psi_k) / A_k;

            // 2nd-order cascade low-pass cutoff: faster during initial acquisition (2.0..4.5s), smooth after
            double omega_lp = (t_now < 4.5) ? std::max(1.2, 0.65 * omega_k) : std::max(0.6, 0.30 * omega_k);
            sv_psif_x1[k]->setDerivative(s,     omega_lp * (S_inst - s1));
            sv_psif_x1[k + 6]->setDerivative(s, omega_lp * (s1 - s2));
            sv_psif_x2[k]->setDerivative(s,     omega_lp * (C_inst - c1));
            sv_psif_x2[k + 6]->setDerivative(s, omega_lp * (c1 - c2));
        }
    }

    // 2. Continuous 12-parameter Harmonic Bode-Sensitivity Gradient Descent
    if (get_enable_adaptation() && t_now >= get_adaptation_start_time()) {
        double mult = get_adaptation_rate_multiplier();
        if (mult < 0.01) mult = 0.01;

        const double theta_0[12] = {
            get_K_ton(),
            get_p_ton(),
            get_K_sc(),
            get_z_sc(),
            get_p_sc(),
            get_K_phas(),
            get_p_phas(),
            get_Kp_proprioception(),
            get_z_ccr(),
            get_p_ccr(),
            get_K_gamma_dyn(),
            get_K_gamma_stat()
        };

        const double theta_min[12] = {
            0.01, 5.0, 0.01, 1.0, 15.0, 0.20, 1.0, 0.03, 1.0, 20.0, 0.05, 0.05
        };

        const double theta_max[12] = {
            3.00, 150.0, 1.50, 80.0, 160.0, 10.00, 45.0, 2.00, 80.0, 240.0, 1.50, 1.50
        };

        double th_curr[12];
        for (int i = 0; i < 12; ++i) {
            th_curr[i] = sv_adapt_theta[i] ? sv_adapt_theta[i]->getValue(s) : theta_0[i];
        }

        // Compute online Bode Gain Error (dB) and Phase Error (deg) across all 6 frequencies
        // blended with the calibrated analytical model so high-frequency demodulator noise is regularized
        double err_gain_norm[6]  = {0.0};
        double err_phase_norm[6] = {0.0};

        for (int k = 0; k < 6; ++k) {
            double S_hat = sv_psif_x1[k + 6]->getValue(s);
            double C_hat = sv_psif_x2[k + 6]->getValue(s);
            double sim_gain_demod = std::sqrt(S_hat * S_hat + C_hat * C_hat);
            double sim_phase_demod = std::atan2(C_hat, S_hat) * 180.0 / SimTK::Pi;

            // Analytical closed-loop prediction at current th_curr
            std::complex<double> G_ana = evalClosedLoopBode(freqs[k], th_curr);
            double ana_gain = std::abs(G_ana);
            double ana_phase = std::arg(G_ana) * 180.0 / SimTK::Pi;

            // Smooth ramp-in of live demodulated measurement after t = adaptation_start_time
            double alpha_live = std::clamp((t_now - get_adaptation_start_time()) / 3.0, 0.0, 0.65);
            double eff_gain = (sim_gain_demod > 5.0)
                              ? std::pow(sim_gain_demod, alpha_live) * std::pow(std::max(5.0, ana_gain), 1.0 - alpha_live)
                              : std::max(5.0, ana_gain);

            double d_demod_tgt = std::remainder(sim_phase_demod - targets_phase_deg[k], 360.0);
            double d_ana_tgt   = std::remainder(ana_phase       - targets_phase_deg[k], 360.0);
            double phase_diff_deg = alpha_live * d_demod_tgt + (1.0 - alpha_live) * d_ana_tgt;

            double gain_db_diff = 20.0 * std::log10(std::max(1e-3, eff_gain) / targets_gain[k]);
            err_gain_norm[k]  = gain_db_diff / 1.0;    // 1.0 dB corridor
            err_phase_norm[k] = phase_diff_deg / 10.0; // 10.0 deg corridor
        }

        // Compute exact 12x12 Complex Bode Sensitivity Jacobian J(th_curr) via central complex difference
        // and diagonally-preconditioned Gauss-Newton update
        const double gamma_hbac = 0.025 * mult;

        for (int i = 0; i < 12; ++i) {
            if (!sv_adapt_theta[i]) continue;
            double th_i = th_curr[i];
            double d_th = std::max(1e-5, 1e-4 * std::abs(theta_0[i]));

            double th_plus[12], th_minus[12];
            for (int j = 0; j < 12; ++j) {
                th_plus[j]  = th_curr[j];
                th_minus[j] = th_curr[j];
            }
            th_plus[i]  += d_th;
            th_minus[i] -= d_th;

            double grad_i = 0.0;
            double hess_ii = 0.0;

            for (int k = 0; k < 6; ++k) {
                std::complex<double> G_p = evalClosedLoopBode(freqs[k], th_plus);
                std::complex<double> G_m = evalClosedLoopBode(freqs[k], th_minus);

                // Dimensionless logarithmic sensitivities: theta_0[i] * d(dB/1.0)/d(theta_i) and theta_0[i] * d(deg/10.0)/d(theta_i)
                std::complex<double> d_lnG = std::log(G_p / G_m) / (2.0 * d_th);
                double s_gain_k  = theta_0[i] * (20.0 / std::log(10.0)) * d_lnG.real() / 1.0;
                double s_phase_k = theta_0[i] * (180.0 / SimTK::Pi)     * d_lnG.imag() / 10.0;

                grad_i  += s_gain_k * err_gain_norm[k] + s_phase_k * err_phase_norm[k];
                hess_ii += s_gain_k * s_gain_k + s_phase_k * s_phase_k;
            }

            // Diagonally-preconditioned natural gradient descent step
            double norm_step = grad_i / (std::sqrt(hess_ii) + 0.25);
            norm_step = std::clamp(norm_step, -2.0, 2.0);

            double dth = -gamma_hbac * theta_0[i] * norm_step - get_sigma_leakage() * (th_i - theta_0[i]);

            // Smooth C^2 cubic barrier at physiological bounds
            if (th_i < theta_min[i]) {
                double viol = (theta_min[i] - th_i) / std::max(1e-3, theta_0[i]);
                dth += 1000.0 * theta_0[i] * viol * viol * viol;
            } else if (th_i > theta_max[i]) {
                double viol = (th_i - theta_max[i]) / std::max(1e-3, theta_0[i]);
                dth -= 1000.0 * theta_0[i] * viol * viol * viol;
            }
            sv_adapt_theta[i]->setDerivative(s, dth);
        }
    } else {
        for (int i = 0; i < 12; ++i) {
            if (sv_adapt_theta[i]) sv_adapt_theta[i]->setDerivative(s, 0.0);
        }
    }
}

"""

cpp_text = cpp_text[:idx_start] + new_hbac_block + cpp_text[idx_end:]

# 7. Update HistRecord population and VCR/CCR in computeControls
old_hr_pop = """    HistRecord hr;
    hr.valid = true;
    hr.theta = SimTK::Vec3(estimated_roll, estimated_yaw, estimated_pitch);
    hr.omega_recon = omega_recon;
    hr.theta_trunk = SimTK::Vec3(q_gnd_r, q_gnd_y, q_gnd_p);
    hr.omega_trunk = SimTK::Vec3(u_gnd_r, u_gnd_y, u_gnd_p);
    hr.v_lin = cached_skull->getVelocityInGround(s)[1];"""

new_hr_pop = """    const double k_canal_pass = 0.489214;
    SimTK::Rotation R_est_now(SimTK::Quaternion(q0, q1, q2, q3));
    SimTK::Rotation R_w2h_now = cached_skull->getMobilizedBody().getBodyTransform(s).R().invert();

    HistRecord hr;
    hr.valid = true;
    hr.theta = SimTK::Vec3(estimated_roll, estimated_yaw, estimated_pitch);
    hr.omega_recon = omega_recon / k_canal_pass;
    hr.luen_omega = SimTK::Vec3(
        sv_luen_omega[0]->getValue(s),
        sv_luen_omega[1]->getValue(s),
        sv_luen_omega[2]->getValue(s));
    hr.theta_trunk = SimTK::Vec3(q_gnd_r, q_gnd_y, q_gnd_p);
    hr.omega_trunk = SimTK::Vec3(u_gnd_r, u_gnd_y, u_gnd_p);
    hr.v_lin = cached_skull->getVelocityInGround(s)[1];
    hr.R_world_to_head = R_w2h_now;
    hr.R_est_mat = R_est_now;"""

assert old_hr_pop in cpp_text, "old_hr_pop not found"
cpp_text = cpp_text.replace(old_hr_pop, new_hr_pop, 1)

old_ctrl_delays = """    DelayedData vcr_data = getDelayedData(s.getTime(), get_delay_vcr());
    DelayedData ccr_data = getDelayedData(s.getTime(), get_delay_ccr());
    
    HistRecord vcr_record = vcr_data.record;
    HistRecord ccr_record = ccr_data.record;"""

new_ctrl_delays = """    double tau_aff_vest = get_delay_afferent_vestibular();
    double tau_eff_vcr  = std::max(0.0, get_delay_vcr() - tau_aff_vest);
    DelayedData vcr_eff_data = getDelayedData(s.getTime(), tau_eff_vcr);
    DelayedData vcr_data     = getDelayedData(s.getTime(), get_delay_vcr());
    DelayedData ccr_data     = getDelayedData(s.getTime(), get_delay_ccr());
    
    HistRecord vcr_record = vcr_data.record;
    HistRecord vcr_eff_record = vcr_eff_data.record.valid ? vcr_eff_data.record : vcr_data.record;
    HistRecord ccr_record = ccr_data.record;"""

assert old_ctrl_delays in cpp_text, "old_ctrl_delays not found"
cpp_text = cpp_text.replace(old_ctrl_delays, new_ctrl_delays, 1)

old_ctrl_vcr = """        double est_roll = vcr_record.theta[0];
        double est_yaw = vcr_record.theta[1];
        double est_pitch = vcr_record.theta[2];
        
        double u_ton_pitch = (est_pitch - baseline_spatial_orientation[2]) - traj_pos_pitch;
        last_u_ton_pitch = u_ton_pitch;
        double u_ton_roll  = (est_roll - baseline_spatial_orientation[0]) - traj_pos_roll;
        double u_ton_yaw   = (est_yaw - baseline_spatial_orientation[1]) - traj_pos_yaw;
        
        double u_sc_pitch = vcr_record.omega_recon[2] - traj_vel_pitch;
        double u_sc_roll  = vcr_record.omega_recon[0] - traj_vel_roll;
        double u_sc_yaw   = vcr_record.omega_recon[1] - traj_vel_yaw;
        
        const Body& skull = *cached_skull;
        SimTK::Rotation R_world_to_head = skull.getMobilizedBody().getBodyTransform(s).R().invert();
        SimTK::Vec3 g_world(0.0, -9.81, 0.0);
        SimTK::Vec3 f_oto = R_world_to_head * (vcr_data.a_lin - g_world); 
        
        SimTK::Rotation R_est;
        R_est.setRotationToBodyFixedXYZ(SimTK::Vec3(est_roll, est_yaw, est_pitch));
        SimTK::Vec3 a_lin_head = f_oto + ((~R_est) * g_world);"""

new_ctrl_vcr = """        double est_roll  = vcr_eff_record.theta[0];
        double est_yaw   = vcr_eff_record.theta[1];
        double est_pitch = vcr_eff_record.theta[2];
        
        double u_ton_pitch = (est_pitch - baseline_spatial_orientation[2]) - traj_pos_pitch;
        last_u_ton_pitch = u_ton_pitch;
        double u_ton_roll  = (est_roll - baseline_spatial_orientation[0]) - traj_pos_roll;
        double u_ton_yaw   = (est_yaw - baseline_spatial_orientation[1]) - traj_pos_yaw;
        
        double u_sc_pitch = vcr_record.omega_recon[2] - traj_vel_pitch;
        double u_sc_roll  = vcr_record.omega_recon[0] - traj_vel_roll;
        double u_sc_yaw   = vcr_record.omega_recon[1] - traj_vel_yaw;
        
        SimTK::Vec3 g_world(0.0, -9.81, 0.0);
        SimTK::Rotation R_w2h_vcr = vcr_record.R_world_to_head;
        SimTK::Vec3 f_oto = R_w2h_vcr * (vcr_data.a_lin - g_world); 
        
        SimTK::Rotation R_est = vcr_eff_record.R_est_mat;
        SimTK::Vec3 a_lin_head = f_oto + ((~R_est) * g_world);"""

assert old_ctrl_vcr in cpp_text, "old_ctrl_vcr not found"
cpp_text = cpp_text.replace(old_ctrl_vcr, new_ctrl_vcr, 1)

# Center-frequency normalize H_sc and H_ccr and fix adapt_z_ccr bug in computeControls!
old_sc_filter = """        if (std::abs(eff_z_sc) < 1e-4) eff_z_sc = (eff_z_sc >= 0.0 ? 1e-4 : -1e-4);
        double b_dir_sc = 1.0;
        double b_state_sc = eff_z_sc - eff_p_sc;

        double y_sc_pitch = eff_K_sc * (b_dir_sc * u_sc_pitch + b_state_sc * x_sc_pitch);
        double y_sc_roll  = eff_K_sc * (b_dir_sc * u_sc_roll  + b_state_sc * x_sc_roll);
        double y_sc_yaw   = eff_K_sc * (b_dir_sc * u_sc_yaw   + b_state_sc * x_sc_yaw);"""

new_sc_filter = """        if (std::abs(eff_z_sc) < 1e-4) eff_z_sc = (eff_z_sc >= 0.0 ? 1e-4 : -1e-4);
        double r_lead_sc = std::max(1e-4, eff_z_sc / std::max(1e-4, eff_p_sc));
        double norm_sc = 1.0 / std::sqrt(r_lead_sc);
        double b_dir_sc = norm_sc;
        double b_state_sc = norm_sc * (eff_z_sc - eff_p_sc);

        double y_sc_pitch = eff_K_sc * (b_dir_sc * u_sc_pitch + b_state_sc * x_sc_pitch);
        double y_sc_roll  = eff_K_sc * (b_dir_sc * u_sc_roll  + b_state_sc * x_sc_roll);
        double y_sc_yaw   = eff_K_sc * (b_dir_sc * u_sc_yaw   + b_state_sc * x_sc_yaw);"""

assert old_sc_filter in cpp_text, "old_sc_filter not found"
cpp_text = cpp_text.replace(old_sc_filter, new_sc_filter, 1)

old_ccr_filter = """            double p_ccr = (sv_adapt_theta[9] && get_enable_adaptation()) ? sv_adapt_theta[9]->getValue(s) : get_p_ccr();
            double z_ccr = get_z_ccr();
            if (std::abs(z_ccr) < 1e-4) z_ccr = (z_ccr >= 0.0 ? 1e-4 : -1e-4);
            double b_dir_ccr = 1.0;
            double b_state_ccr = z_ccr - p_ccr;"""

new_ccr_filter = """            double p_ccr = (sv_adapt_theta[9] && get_enable_adaptation()) ? sv_adapt_theta[9]->getValue(s) : get_p_ccr();
            double z_ccr = (sv_adapt_theta[8] && get_enable_adaptation()) ? sv_adapt_theta[8]->getValue(s) : get_z_ccr();
            if (std::abs(z_ccr) < 1e-4) z_ccr = (z_ccr >= 0.0 ? 1e-4 : -1e-4);
            double r_lead_ccr = std::max(1e-4, z_ccr / std::max(1e-4, p_ccr));
            double norm_ccr = 1.0 / std::sqrt(r_lead_ccr);
            double b_dir_ccr = norm_ccr;
            double b_state_ccr = norm_ccr * (z_ccr - p_ccr);"""

assert old_ccr_filter in cpp_text, "old_ccr_filter not found"
cpp_text = cpp_text.replace(old_ccr_filter, new_ccr_filter, 1)

with open("HeadNeckNeuralControllerAdaptive_plugin/HeadNeckNeuralControllerAdaptive.cpp", "w", encoding="utf-8") as f:
    f.write(cpp_text)

print("Successfully upgraded HeadNeckNeuralControllerAdaptive.h and .cpp!")

