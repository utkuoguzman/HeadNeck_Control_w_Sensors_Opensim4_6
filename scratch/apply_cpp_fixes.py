import os
import shutil

H_PATH = r"HeadNeckNeuralController_plugin/HeadNeckNeuralController.h"
CPP_PATH = r"HeadNeckNeuralController_plugin/HeadNeckNeuralController.cpp"

shutil.copy2(H_PATH, H_PATH + ".bak")
shutil.copy2(CPP_PATH, CPP_PATH + ".bak")

# 1. Patch HeadNeckNeuralController.h
with open(H_PATH, "r", encoding="utf-8") as f:
    h_src = f.read()

old_hist_struct = """    struct HistRecord {
        bool valid = false;
        SimTK::Vec3 theta{0.0};        // (roll, yaw, pitch) - Head global orientation
        SimTK::Vec3 omega_recon{0.0};  // Head global angular velocity
        SimTK::Vec3 theta_trunk{0.0};  // (roll, yaw, pitch) - Trunk global orientation
        SimTK::Vec3 omega_trunk{0.0};  // (roll, yaw, pitch) - Trunk global angular velocity
        SimTK::Vec3 v_lin{0.0};        // Linear velocity of the head
        SimTK::Vector Ia;              // size N (Spindle Firing Rate)
        SimTK::Vec3 est_neck_rel_angle{0.0}; // size 3 (pitch, roll, yaw)
        SimTK::Vec3 est_neck_rel_speed{0.0}; // size 3 (pitch, roll, yaw)
    };"""

new_hist_struct = """    struct HistRecord {
        bool valid = false;
        SimTK::Vec3 theta{0.0};        // (roll, yaw, pitch) - Head global orientation
        SimTK::Vec3 omega_recon{0.0};  // Head global angular velocity (gain-normalized canal firing)
        SimTK::Vec3 luen_omega{0.0};   // Clean Luenberger-inverted physical angular velocity (roll, yaw, pitch)
        SimTK::Mat33 R_world_to_head{1.0}; // True skull R_world_to_head at record timestamp
        SimTK::Mat33 R_est_mat{1.0};   // Observer estimated R_head_to_world at record timestamp
        SimTK::Vec3 theta_trunk{0.0};  // (roll, yaw, pitch) - Trunk global orientation
        SimTK::Vec3 omega_trunk{0.0};  // (roll, yaw, pitch) - Trunk global angular velocity
        SimTK::Vec3 v_lin{0.0};        // Linear velocity of the head
        SimTK::Vector Ia;              // size N (Spindle Firing Rate)
        SimTK::Vec3 est_neck_rel_angle{0.0}; // size 3 (pitch, roll, yaw)
        SimTK::Vec3 est_neck_rel_speed{0.0}; // size 3 (pitch, roll, yaw)
    };"""

assert old_hist_struct in h_src, "old_hist_struct not found in .h"
h_src = h_src.replace(old_hist_struct, new_hist_struct, 1)

# Also add second-order voluntary filter state pointer sv_vol_filt2[3] in .h
old_sv_vol = "    mutable const StateVariable* sv_vol_filt[3] = {nullptr};"
new_sv_vol = """    mutable const StateVariable* sv_vol_filt[3] = {nullptr};
    mutable const StateVariable* sv_vol_filt2[3] = {nullptr};"""
assert old_sv_vol in h_src, "old_sv_vol not found in .h"
h_src = h_src.replace(old_sv_vol, new_sv_vol, 1)

with open(H_PATH, "w", encoding="utf-8", newline="\r\n") as f:
    f.write(h_src)

# 2. Patch HeadNeckNeuralController.cpp
with open(CPP_PATH, "r", encoding="utf-8") as f:
    cpp_src = f.read()

# 2a. Add vol_filt2 state variables for 2nd-order cortical low-pass filter
old_add_sv = """    addStateVariable("vol_filt_pitch", SimTK::Stage::Velocity);
    addStateVariable("vol_filt_roll", SimTK::Stage::Velocity);
    addStateVariable("vol_filt_yaw", SimTK::Stage::Velocity);"""
new_add_sv = """    addStateVariable("vol_filt_pitch", SimTK::Stage::Velocity);
    addStateVariable("vol_filt_roll", SimTK::Stage::Velocity);
    addStateVariable("vol_filt_yaw", SimTK::Stage::Velocity);
    addStateVariable("vol_filt2_pitch", SimTK::Stage::Velocity);
    addStateVariable("vol_filt2_roll", SimTK::Stage::Velocity);
    addStateVariable("vol_filt2_yaw", SimTK::Stage::Velocity);"""
assert old_add_sv in cpp_src, "old_add_sv not found"
cpp_src = cpp_src.replace(old_add_sv, new_add_sv, 1)

old_init_sv = """    setStateVariableValue(s, "vol_filt_pitch", 0.0);
    setStateVariableValue(s, "vol_filt_roll", 0.0);
    setStateVariableValue(s, "vol_filt_yaw", 0.0);"""
new_init_sv = """    setStateVariableValue(s, "vol_filt_pitch", 0.0);
    setStateVariableValue(s, "vol_filt_roll", 0.0);
    setStateVariableValue(s, "vol_filt_yaw", 0.0);
    setStateVariableValue(s, "vol_filt2_pitch", 0.0);
    setStateVariableValue(s, "vol_filt2_roll", 0.0);
    setStateVariableValue(s, "vol_filt2_yaw", 0.0);"""
assert old_init_sv in cpp_src, "old_init_sv not found"
cpp_src = cpp_src.replace(old_init_sv, new_init_sv, 1)

old_trav_sv = """    sv_vol_filt[0] = traverseToStateVariable("vol_filt_pitch");
    sv_vol_filt[1] = traverseToStateVariable("vol_filt_roll");
    sv_vol_filt[2] = traverseToStateVariable("vol_filt_yaw");"""
new_trav_sv = """    sv_vol_filt[0] = traverseToStateVariable("vol_filt_pitch");
    sv_vol_filt[1] = traverseToStateVariable("vol_filt_roll");
    sv_vol_filt[2] = traverseToStateVariable("vol_filt_yaw");
    sv_vol_filt2[0] = traverseToStateVariable("vol_filt2_pitch");
    sv_vol_filt2[1] = traverseToStateVariable("vol_filt2_roll");
    sv_vol_filt2[2] = traverseToStateVariable("vol_filt2_yaw");"""
assert old_trav_sv in cpp_src, "old_trav_sv not found"
cpp_src = cpp_src.replace(old_trav_sv, new_trav_sv, 1)

# 2b. Patch getDelayedData interpolation & 1ms dt_window
old_delayed_interp = """    HistRecord res;
    res.valid = it_prev->second.valid || it->second.valid;
    res.omega_recon = it_prev->second.omega_recon * (1 - w) + it->second.omega_recon * w;
    res.theta = it_prev->second.theta * (1 - w) + it->second.theta * w;
    res.theta_trunk = it_prev->second.theta_trunk * (1 - w) + it->second.theta_trunk * w;
    res.omega_trunk = it_prev->second.omega_trunk * (1 - w) + it->second.omega_trunk * w;
    res.v_lin = it_prev->second.v_lin * (1 - w) + it->second.v_lin * w;
    res.est_neck_rel_angle = it_prev->second.est_neck_rel_angle * (1 - w) + it->second.est_neck_rel_angle * w;
    res.est_neck_rel_speed = it_prev->second.est_neck_rel_speed * (1 - w) + it->second.est_neck_rel_speed * w;"""

new_delayed_interp = """    HistRecord res;
    res.valid = it_prev->second.valid || it->second.valid;
    res.omega_recon = it_prev->second.omega_recon * (1 - w) + it->second.omega_recon * w;
    res.luen_omega = it_prev->second.luen_omega * (1 - w) + it->second.luen_omega * w;
    res.R_world_to_head = it_prev->second.R_world_to_head * (1 - w) + it->second.R_world_to_head * w;
    res.R_est_mat = it_prev->second.R_est_mat * (1 - w) + it->second.R_est_mat * w;
    res.theta = it_prev->second.theta * (1 - w) + it->second.theta * w;
    res.theta_trunk = it_prev->second.theta_trunk * (1 - w) + it->second.theta_trunk * w;
    res.omega_trunk = it_prev->second.omega_trunk * (1 - w) + it->second.omega_trunk * w;
    res.v_lin = it_prev->second.v_lin * (1 - w) + it->second.v_lin * w;
    res.est_neck_rel_angle = it_prev->second.est_neck_rel_angle * (1 - w) + it->second.est_neck_rel_angle * w;
    res.est_neck_rel_speed = it_prev->second.est_neck_rel_speed * (1 - w) + it->second.est_neck_rel_speed * w;"""

assert old_delayed_interp in cpp_src, "old_delayed_interp not found"
cpp_src = cpp_src.replace(old_delayed_interp, new_delayed_interp, 1)

old_dt_win = "    double dt_window = 0.010; // 10ms smoothing window"
new_dt_win = "    double dt_window = 0.001; // 1ms finite-difference window (removes extra 4.5ms lag)"
assert old_dt_win in cpp_src, "old_dt_win not found"
cpp_src = cpp_src.replace(old_dt_win, new_dt_win, 1)

# 2c. Fast Spindle Luenberger Observer (direct position innovation + tau_v = 5ms)
old_spindle_luen = """        const double k_gdyn = 100.0 * get_K_gamma_dyn();
        const double k_gstat = 100.0 * get_K_gamma_stat();
        const double tau_II = 0.055;
        const double tau_v  = 0.025;"""
new_spindle_luen = """        const double k_gdyn = 100.0 * get_K_gamma_dyn();
        const double k_gstat = 100.0 * get_K_gamma_stat();
        const double tau_dL = 0.010;
        const double tau_II = 0.055;
        const double tau_v  = 0.005;"""
assert old_spindle_luen in cpp_src, "old_spindle_luen not found"
cpp_src = cpp_src.replace(old_spindle_luen, new_spindle_luen, 1)

old_spindle_ode = """            sv_sp_dL[i]->setDerivative(s, sp_v);
            sv_sp_v[i]->setDerivative(s, (v_from_II + v_lead_Ia - sp_v) / tau_v);"""
new_spindle_ode = """            sv_sp_dL[i]->setDerivative(s, sp_v + (y_II_i - sp_dL) / tau_dL);
            sv_sp_v[i]->setDerivative(s, (v_from_II + v_lead_Ia - sp_v) / tau_v);"""
assert old_spindle_ode in cpp_src, "old_spindle_ode not found"
cpp_src = cpp_src.replace(old_spindle_ode, new_spindle_ode, 1)

# 2d. Consistent 40ms delay on Voluntary PID + Static Gravity Trim + 2nd-order cortical filter
old_vol_block = """    // PID Integrator derivatives (using proprioceptive-decoded neck relative angle)
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

new_vol_block = """    // --- CORTICAL / VOLUNTARY LOW-PASS FILTER DYNAMICS (TRANSCORTICAL DELAY + 2nd-ORDER 2 Hz ROLL-OFF) ---
    DelayedData vol_data = getDelayedData(s.getTime(), get_delay_voluntary());

    double vol_rel_pitch = (!history.empty() && vol_data.record.valid) ? vol_data.record.est_neck_rel_angle[0] : est_neck_rel_angle[0];
    double vol_rel_roll  = (!history.empty() && vol_data.record.valid) ? vol_data.record.est_neck_rel_angle[1] : est_neck_rel_angle[1];
    double vol_rel_yaw   = (!history.empty() && vol_data.record.valid) ? vol_data.record.est_neck_rel_angle[2] : est_neck_rel_angle[2];

    double vol_rel_pitch_dot = (!history.empty() && vol_data.record.valid) ? vol_data.record.est_neck_rel_speed[0] : est_neck_rel_speed[0];
    double vol_rel_roll_dot  = (!history.empty() && vol_data.record.valid) ? vol_data.record.est_neck_rel_speed[1] : est_neck_rel_speed[1];
    double vol_rel_yaw_dot   = (!history.empty() && vol_data.record.valid) ? vol_data.record.est_neck_rel_speed[2] : est_neck_rel_speed[2];

    // PID Integrator derivatives (using 40ms-delayed vol_rel_* consistently; frozen at static gravity trim during stabilization)
    double p_err = (std::abs(get_desired_pitch()) > 1e-6) ? (traj_pos_pitch - vol_rel_pitch) : 0.0;
    double p_int = sv_err_int[0]->getValue(s);
    if (p_int > 0.6 && p_err > 0) p_err = 0.0;
    if (p_int < -0.6 && p_err < 0) p_err = 0.0;
    sv_err_int[0]->setDerivative(s, p_err);

    double r_err = (std::abs(get_desired_roll()) > 1e-6) ? (traj_pos_roll - vol_rel_roll) : 0.0;
    double r_int = sv_err_int[1]->getValue(s);
    if (r_int > 0.4 && r_err > 0) r_err = 0.0;
    if (r_int < -0.4 && r_err < 0) r_err = 0.0;
    sv_err_int[1]->setDerivative(s, r_err);

    double y_err = (std::abs(get_desired_yaw()) > 1e-6) ? (traj_pos_yaw - vol_rel_yaw) : 0.0;
    double y_int = sv_err_int[2]->getValue(s);
    if (y_int > 0.4 && y_err > 0) y_err = 0.0;
    if (y_int < -0.4 && y_err < 0) y_err = 0.0;
    sv_err_int[2]->setDerivative(s, y_err);"""

assert old_vol_block in cpp_src, "old_vol_block not found"
cpp_src = cpp_src.replace(old_vol_block, new_vol_block, 1)

old_vol_filt_ode = """    double vol_filt_pitch = sv_vol_filt[0]->getValue(s);
    double vol_filt_roll  = sv_vol_filt[1]->getValue(s);
    double vol_filt_yaw   = sv_vol_filt[2]->getValue(s);

    sv_vol_filt[0]->setDerivative(s, omega_vol * (tau_vol_raw_pitch - vol_filt_pitch));
    sv_vol_filt[1]->setDerivative(s, omega_vol * (tau_vol_raw_roll  - vol_filt_roll));
    sv_vol_filt[2]->setDerivative(s, omega_vol * (tau_vol_raw_yaw   - vol_filt_yaw));"""

new_vol_filt_ode = """    double vol_filt_pitch = sv_vol_filt[0]->getValue(s);
    double vol_filt_roll  = sv_vol_filt[1]->getValue(s);
    double vol_filt_yaw   = sv_vol_filt[2]->getValue(s);

    double vol_filt2_pitch = sv_vol_filt2[0]->getValue(s);
    double vol_filt2_roll  = sv_vol_filt2[1]->getValue(s);
    double vol_filt2_yaw   = sv_vol_filt2[2]->getValue(s);

    sv_vol_filt[0]->setDerivative(s, omega_vol * (tau_vol_raw_pitch - vol_filt_pitch));
    sv_vol_filt[1]->setDerivative(s, omega_vol * (tau_vol_raw_roll  - vol_filt_roll));
    sv_vol_filt[2]->setDerivative(s, omega_vol * (tau_vol_raw_yaw   - vol_filt_yaw));

    sv_vol_filt2[0]->setDerivative(s, omega_vol * (vol_filt_pitch - vol_filt2_pitch));
    sv_vol_filt2[1]->setDerivative(s, omega_vol * (vol_filt_roll  - vol_filt2_roll));
    sv_vol_filt2[2]->setDerivative(s, omega_vol * (vol_filt_yaw   - vol_filt2_yaw));"""

assert old_vol_filt_ode in cpp_src, "old_vol_filt_ode not found"
cpp_src = cpp_src.replace(old_vol_filt_ode, new_vol_filt_ode, 1)

# 2e. Anatomical Vestibular Afferent Delay (2ms) + Otolith Frame-Delay Alignment & Smith Predictor in Mahony
old_mahony_vest = """    DelayedData vcr_data = getDelayedData(s.getTime(), get_delay_vcr());
    
    // --- SE_2(3) SPATIAL OBSERVER (Tilt-Translation Disambiguation) ---
    // 1. Vestibular Sensor Readings (Skull angular velocity and Otolith specific force)
    const Body& skull = *cached_skull;
    SimTK::Vec3 w_world(luen_w_roll, luen_w_yaw, luen_w_pitch); // Clean physical angular velocity from Vestibular Luenberger Observer
    SimTK::Rotation R_world_to_head = skull.getMobilizedBody().getBodyTransform(s).R().invert();
    SimTK::Vec3 g_world(0.0, -9.81, 0.0);
    SimTK::Vec3 f_oto = R_world_to_head * (vcr_data.a_lin - g_world); // Head frame"""

new_mahony_vest = """    const double tau_aff_vest = 0.002; // 2 ms CN VIII vestibular afferent delay to brainstem/cerebellum
    DelayedData vest_aff_data = getDelayedData(s.getTime(), tau_aff_vest);
    DelayedData vcr_data = getDelayedData(s.getTime(), get_delay_vcr());
    
    // --- SE_2(3) SPATIAL OBSERVER (Tilt-Translation Disambiguation) ---
    // 1. Vestibular Sensor Readings (2ms-delayed Canal Gyro and Time-Aligned Otolith Specific Force)
    const Body& skull = *cached_skull;
    SimTK::Rotation R_world_to_head = skull.getMobilizedBody().getBodyTransform(s).R().invert();
    SimTK::Vec3 w_world = (!history.empty() && vest_aff_data.record.valid)
                          ? vest_aff_data.record.luen_omega
                          : SimTK::Vec3(luen_w_roll, luen_w_yaw, luen_w_pitch);
    SimTK::Mat33 R_w2h_vest = (!history.empty() && vest_aff_data.record.valid)
                              ? vest_aff_data.record.R_world_to_head
                              : R_world_to_head.asMat33();
    SimTK::Vec3 g_world(0.0, -9.81, 0.0);
    SimTK::Vec3 f_oto = R_w2h_vest * (vest_aff_data.a_lin - g_world); // Time-aligned at t - 2ms in Head frame"""

assert old_mahony_vest in cpp_src, "old_mahony_vest not found"
cpp_src = cpp_src.replace(old_mahony_vest, new_mahony_vest, 1)

old_oto_smith = """    // Estimated UP direction in Head frame: rotate [0, 1, 0] by q^-1 (~R_est)
    SimTK::Vec3 expected_up = (~R_est) * SimTK::Vec3(0.0, 1.0, 0.0);"""

new_oto_smith = """    // Estimated UP direction in Head frame at the matching 2ms delayed horizon (Otolith Smith Predictor)
    SimTK::Mat33 R_est_vest = (get_use_smith_predictor() && !history.empty() && vest_aff_data.record.valid)
                              ? vest_aff_data.record.R_est_mat
                              : R_est.asMat33();
    SimTK::Vec3 expected_up = (~R_est_vest) * SimTK::Vec3(0.0, 1.0, 0.0);"""

assert old_oto_smith in cpp_src, "old_oto_smith not found"
cpp_src = cpp_src.replace(old_oto_smith, new_oto_smith, 1)

# 2f. Time-aligned Otolith & Efferent Delay in computeStateVariableDerivatives (lines 951-975)
old_deriv_vcr = """    // --- NEURAL TRANSFER FUNCTION DERIVATIVES ---
    if (vcr_data.record.valid) {
        double est_roll = vcr_data.record.theta[0];
        double est_yaw = vcr_data.record.theta[1];
        double est_pitch = vcr_data.record.theta[2];
        
        double u_ton_pitch = (est_pitch - baseline_spatial_orientation[2]) - traj_pos_pitch;
        double u_ton_roll  = (est_roll - baseline_spatial_orientation[0]) - traj_pos_roll;
        double u_ton_yaw   = (est_yaw - baseline_spatial_orientation[1]) - traj_pos_yaw;
        
        double u_sc_pitch = vcr_data.record.omega_recon[2] - traj_vel_pitch;
        double u_sc_roll  = vcr_data.record.omega_recon[0] - traj_vel_roll;
        double u_sc_yaw   = vcr_data.record.omega_recon[1] - traj_vel_yaw;
        
        SimTK::Rotation R_est;
        R_est.setRotationToBodyFixedXYZ(SimTK::Vec3(est_roll, est_yaw, est_pitch));
        SimTK::Vec3 a_lin_head = f_oto + ((~R_est) * g_world);"""

new_deriv_vcr = """    // --- NEURAL TRANSFER FUNCTION DERIVATIVES ---
    double tau_eff_vcr = std::max(0.0, get_delay_vcr() - tau_aff_vest); // 8 ms efferent MVST delay (2ms aff + 8ms eff = 10ms total)
    DelayedData vcr_eff_data = getDelayedData(s.getTime(), tau_eff_vcr);
    if (vcr_data.record.valid) {
        const HistRecord& ton_rec = vcr_eff_data.record.valid ? vcr_eff_data.record : vcr_data.record;
        double est_roll  = ton_rec.theta[0];
        double est_yaw   = ton_rec.theta[1];
        double est_pitch = ton_rec.theta[2];
        
        double u_ton_pitch = (est_pitch - baseline_spatial_orientation[2]) - traj_pos_pitch;
        double u_ton_roll  = (est_roll - baseline_spatial_orientation[0]) - traj_pos_roll;
        double u_ton_yaw   = (est_yaw - baseline_spatial_orientation[1]) - traj_pos_yaw;
        
        double u_sc_pitch = vcr_data.record.omega_recon[2] - traj_vel_pitch;
        double u_sc_roll  = vcr_data.record.omega_recon[0] - traj_vel_roll;
        double u_sc_yaw   = vcr_data.record.omega_recon[1] - traj_vel_yaw;
        
        SimTK::Vec3 f_oto_vcr = vcr_data.record.R_world_to_head * (vcr_data.a_lin - g_world);
        SimTK::Vec3 a_lin_head = f_oto_vcr + ((~ton_rec.R_est_mat) * g_world);"""

assert old_deriv_vcr in cpp_src, "old_deriv_vcr not found"
cpp_src = cpp_src.replace(old_deriv_vcr, new_deriv_vcr, 1)

# 2g. Gain-normalize omega_recon by k_canal_pass = 0.489214 and store luen_omega, R_world_to_head, R_est_mat in HistRecord
old_hist_store = """    SimTK::Vec3 v_R_filt(y[0], y[1], y[2]);
    SimTK::Vec3 v_L_filt(y[3], y[4], y[5]);
    SimTK::Vec3 omega_recon = 0.5 * (B_R_inv_mat * v_R_filt + B_L_inv_mat * v_L_filt);
    
    // Observer estimated attitude for vestibular history buffer
    double q0 = sv_obs_q[0]->getValue(s);
    double q1 = sv_obs_q[1]->getValue(s);
    double q2 = sv_obs_q[2]->getValue(s);
    double q3 = sv_obs_q[3]->getValue(s);
    double norm_q = std::sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (norm_q > 1e-4) { q0 /= norm_q; q1 /= norm_q; q2 /= norm_q; q3 /= norm_q; }
    double estimated_roll = std::atan2(2.0*(q0*q1 + q2*q3), 1.0 - 2.0*(q1*q1 + q2*q2));
    double estimated_yaw = std::asin(std::clamp(2.0*(q0*q2 - q3*q1), -1.0, 1.0));
    double estimated_pitch = std::atan2(2.0*(q0*q3 + q1*q2), 1.0 - 2.0*(q2*q2 + q3*q3));
    mahony_angle = SimTK::Vec3(estimated_pitch, estimated_roll, estimated_yaw);
    
    HistRecord hr;
    hr.valid = true;
    hr.theta = SimTK::Vec3(estimated_roll, estimated_yaw, estimated_pitch);
    hr.omega_recon = omega_recon;
    hr.theta_trunk = SimTK::Vec3(q_gnd_r, q_gnd_y, q_gnd_p);
    hr.omega_trunk = SimTK::Vec3(u_gnd_r, u_gnd_y, u_gnd_p);
    hr.v_lin = cached_skull->getVelocityInGround(s)[1];"""

new_hist_store = """    SimTK::Vec3 v_R_filt(y[0], y[1], y[2]);
    SimTK::Vec3 v_L_filt(y[3], y[4], y[5]);
    const double k_canal_pass = 0.489214;
    SimTK::Vec3 omega_recon = (0.5 / k_canal_pass) * (B_R_inv_mat * v_R_filt + B_L_inv_mat * v_L_filt);
    
    // Observer estimated attitude for vestibular history buffer
    double q0 = sv_obs_q[0]->getValue(s);
    double q1 = sv_obs_q[1]->getValue(s);
    double q2 = sv_obs_q[2]->getValue(s);
    double q3 = sv_obs_q[3]->getValue(s);
    double norm_q = std::sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (norm_q > 1e-4) { q0 /= norm_q; q1 /= norm_q; q2 /= norm_q; q3 /= norm_q; }
    double estimated_roll = std::atan2(2.0*(q0*q1 + q2*q3), 1.0 - 2.0*(q1*q1 + q2*q2));
    double estimated_yaw = std::asin(std::clamp(2.0*(q0*q2 - q3*q1), -1.0, 1.0));
    double estimated_pitch = std::atan2(2.0*(q0*q3 + q1*q2), 1.0 - 2.0*(q2*q2 + q3*q3));
    mahony_angle = SimTK::Vec3(estimated_pitch, estimated_roll, estimated_yaw);
    SimTK::Rotation R_head_curr_est(SimTK::Quaternion(q0, q1, q2, q3));
    
    HistRecord hr;
    hr.valid = true;
    hr.theta = SimTK::Vec3(estimated_roll, estimated_yaw, estimated_pitch);
    hr.omega_recon = omega_recon;
    hr.luen_omega = SimTK::Vec3(
        sv_luen_omega[0]->getValue(s),
        sv_luen_omega[1]->getValue(s),
        sv_luen_omega[2]->getValue(s));
    hr.R_world_to_head = cached_skull->getMobilizedBody().getBodyTransform(s).R().invert().asMat33();
    hr.R_est_mat = R_head_curr_est.asMat33();
    hr.theta_trunk = SimTK::Vec3(q_gnd_r, q_gnd_y, q_gnd_p);
    hr.omega_trunk = SimTK::Vec3(u_gnd_r, u_gnd_y, u_gnd_p);
    hr.v_lin = cached_skull->getVelocityInGround(s)[1];"""

assert old_hist_store in cpp_src, "old_hist_store not found"
cpp_src = cpp_src.replace(old_hist_store, new_hist_store, 1)

# 2h. Update computeControls VCR block (time-aligned R_world_to_head, center-freq normalized H_sc, High-Pass y_phas, 2nd-order vol_filt2)
old_ctrl_vcr = """    DelayedData vcr_data = getDelayedData(s.getTime(), get_delay_vcr());
    DelayedData ccr_data = getDelayedData(s.getTime(), get_delay_ccr());
    
    HistRecord vcr_record = vcr_data.record;
    HistRecord ccr_record = ccr_data.record;

    // --- DESCENDING TRACKS (Voluntary) vs ASCENDING TRACKS (Reflex) ---
    Vector tau_vcr(3, 0.0); // Restorative Vestibular Reflexes
    Vector tau_vol(3, 0.0); // Descending Voluntary Task Tracking
    
    // Extract Estimated Angles from SE_2(3) Observer
    SimTK::Rotation R_head_curr(SimTK::Quaternion(q0, q1, q2, q3));
    SimTK::Vec3 w_world(
        sv_luen_omega[0]->getValue(s),
        sv_luen_omega[1]->getValue(s),
        sv_luen_omega[2]->getValue(s)); // Clean physical angular velocity from Vestibular Luenberger Observer
    SimTK::Vec3 w_head = (~R_head_curr) * w_world;
    mahony_speed = SimTK::Vec3(w_head[2], w_head[0], w_head[1]); // pitch, roll, yaw
    
    // 1. Vestibular Reflex with Efference Copy Cancellation (expected vestibular firing from voluntary action)
    if (vcr_record.valid) {
        // Neural Transfer Functions Output Application
        // y = K*(z - p)*x + K*u
        
        double est_roll = vcr_record.theta[0];
        double est_yaw = vcr_record.theta[1];
        double est_pitch = vcr_record.theta[2];
        
        double u_ton_pitch = (est_pitch - baseline_spatial_orientation[2]) - traj_pos_pitch;
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
        SimTK::Vec3 a_lin_head = f_oto + ((~R_est) * g_world);
        
        double r_head = 0.15; 
        double expected_ah_x = traj_acc_pitch * r_head; 
        double expected_ah_z = traj_acc_roll * r_head;
        
        double u_phas_pitch = a_lin_head[0] - expected_ah_x;
        double u_phas_roll  = a_lin_head[2] - expected_ah_z;
        double u_phas_yaw   = 0.0;
        
        double x_ton_pitch = sv_ton[0]->getValue(s);
        double x_ton_roll  = sv_ton[1]->getValue(s);
        double x_ton_yaw   = sv_ton[2]->getValue(s);
        
        double x_sc_pitch = sv_sc[0]->getValue(s);
        double x_sc_roll  = sv_sc[1]->getValue(s);
        double x_sc_yaw   = sv_sc[2]->getValue(s);
        
        double x_phas_pitch = sv_phas[0]->getValue(s);
        double x_phas_roll  = sv_phas[1]->getValue(s);
        double x_phas_yaw   = sv_phas[2]->getValue(s);
        
        double y_ton_pitch = get_K_ton() * get_p_ton() * x_ton_pitch;
        double y_ton_roll  = get_K_ton() * get_p_ton() * x_ton_roll;
        double y_ton_yaw   = get_K_ton() * get_p_ton() * x_ton_yaw;
        
        double p_sc = get_p_sc();
        double z_sc = get_z_sc();
        if (std::abs(z_sc) < 1e-4) z_sc = (z_sc >= 0.0 ? 1e-4 : -1e-4);
        double b_dir_sc = 1.0;
        double b_state_sc = z_sc - p_sc;

        double y_sc_pitch = get_K_sc() * (b_dir_sc * u_sc_pitch + b_state_sc * x_sc_pitch);
        double y_sc_roll  = get_K_sc() * (b_dir_sc * u_sc_roll  + b_state_sc * x_sc_roll);
        double y_sc_yaw   = get_K_sc() * (b_dir_sc * u_sc_yaw   + b_state_sc * x_sc_yaw);
        
        double y_phas_pitch = get_K_phas() * get_p_phas() * x_phas_pitch;
        double y_phas_roll  = get_K_phas() * get_p_phas() * x_phas_roll;
        double y_phas_yaw   = get_K_phas() * get_p_phas() * x_phas_yaw;
        
        tau_vcr[0] -= y_ton_pitch + y_sc_pitch + y_phas_pitch;
        tau_vcr[1] -= y_ton_roll  + y_sc_roll  + y_phas_roll;
        tau_vcr[2] -= y_ton_yaw   + y_sc_yaw   + y_phas_yaw;
    }
    
    // 2. Filtered Voluntary Drive from Cortical Low-Pass Filter (Transcortical Delay + ~2 Hz Cutoff)
    tau_vol[0] = sv_vol_filt[0]->getValue(s);
    tau_vol[1] = sv_vol_filt[1]->getValue(s);
    tau_vol[2] = sv_vol_filt[2]->getValue(s);"""

new_ctrl_vcr = """    const double tau_aff_vest = 0.002;
    double tau_eff_vcr = std::max(0.0, get_delay_vcr() - tau_aff_vest);
    DelayedData vcr_eff_data = getDelayedData(s.getTime(), tau_eff_vcr);
    DelayedData vcr_data = getDelayedData(s.getTime(), get_delay_vcr());
    DelayedData ccr_data = getDelayedData(s.getTime(), get_delay_ccr());
    
    HistRecord vcr_record = vcr_data.record;
    HistRecord ccr_record = ccr_data.record;

    // --- DESCENDING TRACKS (Voluntary) vs ASCENDING TRACKS (Reflex) ---
    Vector tau_vcr(3, 0.0); // Restorative Vestibular Reflexes
    Vector tau_vol(3, 0.0); // Descending Voluntary Task Tracking
    
    // Extract Estimated Angles from SE_2(3) Observer
    SimTK::Rotation R_head_curr(SimTK::Quaternion(q0, q1, q2, q3));
    SimTK::Vec3 w_world(
        sv_luen_omega[0]->getValue(s),
        sv_luen_omega[1]->getValue(s),
        sv_luen_omega[2]->getValue(s)); // Clean physical angular velocity from Vestibular Luenberger Observer
    SimTK::Vec3 w_head = (~R_head_curr) * w_world;
    mahony_speed = SimTK::Vec3(w_head[2], w_head[0], w_head[1]); // pitch, roll, yaw
    
    // 1. Vestibular Reflex with Efference Copy Cancellation (expected vestibular firing from voluntary action)
    if (vcr_record.valid) {
        const HistRecord& ton_rec = vcr_eff_data.record.valid ? vcr_eff_data.record : vcr_record;
        double est_roll  = ton_rec.theta[0];
        double est_yaw   = ton_rec.theta[1];
        double est_pitch = ton_rec.theta[2];
        
        double u_ton_pitch = (est_pitch - baseline_spatial_orientation[2]) - traj_pos_pitch;
        double u_ton_roll  = (est_roll - baseline_spatial_orientation[0]) - traj_pos_roll;
        double u_ton_yaw   = (est_yaw - baseline_spatial_orientation[1]) - traj_pos_yaw;
        
        double u_sc_pitch = vcr_record.omega_recon[2] - traj_vel_pitch;
        double u_sc_roll  = vcr_record.omega_recon[0] - traj_vel_roll;
        double u_sc_yaw   = vcr_record.omega_recon[1] - traj_vel_yaw;
        
        SimTK::Vec3 g_world(0.0, -9.81, 0.0);
        SimTK::Vec3 f_oto = vcr_record.R_world_to_head * (vcr_data.a_lin - g_world);
        SimTK::Vec3 a_lin_head = f_oto + ((~ton_rec.R_est_mat) * g_world);
        
        double r_head = 0.15; 
        double expected_ah_x = traj_acc_pitch * r_head; 
        double expected_ah_z = traj_acc_roll * r_head;
        
        double u_phas_pitch = a_lin_head[0] - expected_ah_x;
        double u_phas_roll  = a_lin_head[2] - expected_ah_z;
        double u_phas_yaw   = 0.0;
        
        double x_ton_pitch = sv_ton[0]->getValue(s);
        double x_ton_roll  = sv_ton[1]->getValue(s);
        double x_ton_yaw   = sv_ton[2]->getValue(s);
        
        double x_sc_pitch = sv_sc[0]->getValue(s);
        double x_sc_roll  = sv_sc[1]->getValue(s);
        double x_sc_yaw   = sv_sc[2]->getValue(s);
        
        double x_phas_pitch = sv_phas[0]->getValue(s);
        double x_phas_roll  = sv_phas[1]->getValue(s);
        double x_phas_yaw   = sv_phas[2]->getValue(s);
        
        double y_ton_pitch = get_K_ton() * get_p_ton() * x_ton_pitch;
        double y_ton_roll  = get_K_ton() * get_p_ton() * x_ton_roll;
        double y_ton_yaw   = get_K_ton() * get_p_ton() * x_ton_yaw;
        
        // Center-frequency normalized lead filter: |H_sc(j*sqrt(z*p))| = K_sc (decouples gain from r_lead_sc)
        double p_sc = get_p_sc();
        double z_sc = get_z_sc();
        if (std::abs(z_sc) < 1e-4) z_sc = (z_sc >= 0.0 ? 1e-4 : -1e-4);
        double r_sc_norm = std::sqrt(std::clamp(std::abs(z_sc) / std::max(1e-4, std::abs(p_sc)), 1e-4, 100.0));
        double b_dir_sc = 1.0 / r_sc_norm;
        double b_state_sc = (z_sc - p_sc) / r_sc_norm;

        double y_sc_pitch = get_K_sc() * (b_dir_sc * u_sc_pitch + b_state_sc * x_sc_pitch);
        double y_sc_roll  = get_K_sc() * (b_dir_sc * u_sc_roll  + b_state_sc * x_sc_roll);
        double y_sc_yaw   = get_K_sc() * (b_dir_sc * u_sc_yaw   + b_state_sc * x_sc_yaw);
        
        // True Phasic (High-Pass) Otolith Filter: H_phas(s) = K_phas * s / (s + p_phas)
        double y_phas_pitch = get_K_phas() * (u_phas_pitch - get_p_phas() * x_phas_pitch);
        double y_phas_roll  = get_K_phas() * (u_phas_roll  - get_p_phas() * x_phas_roll);
        double y_phas_yaw   = get_K_phas() * (u_phas_yaw   - get_p_phas() * x_phas_yaw);
        
        tau_vcr[0] -= y_ton_pitch + y_sc_pitch + y_phas_pitch;
        tau_vcr[1] -= y_ton_roll  + y_sc_roll  + y_phas_roll;
        tau_vcr[2] -= y_ton_yaw   + y_sc_yaw   + y_phas_yaw;
    }
    
    // 2. Filtered Voluntary Drive from 2nd-Order Cortical Low-Pass Filter (Transcortical Delay + 2 Hz Roll-Off)
    tau_vol[0] = sv_vol_filt2[0]->getValue(s);
    tau_vol[1] = sv_vol_filt2[1]->getValue(s);
    tau_vol[2] = sv_vol_filt2[2]->getValue(s);"""

assert old_ctrl_vcr in cpp_src, "old_ctrl_vcr not found"
cpp_src = cpp_src.replace(old_ctrl_vcr, new_ctrl_vcr, 1)

# 2i. Update CCR center-frequency gain normalization & Sherrington Reciprocal Inhibition
old_ccr_block = """            double p_ccr = get_p_ccr();
            double z_ccr = get_z_ccr();
            if (std::abs(z_ccr) < 1e-4) z_ccr = (z_ccr >= 0.0 ? 1e-4 : -1e-4);
            double b_dir_ccr = 1.0;
            double b_state_ccr = z_ccr - p_ccr;

            double Ia_filtered = b_dir_ccr * Ia_delayed + b_state_ccr * x_ccr;
            
            // In case filter makes it negative
            double Ia_firing = std::max(0.0, Ia_filtered);
            
            // Synergy-Based Reciprocal Inhibition (Descending Presynaptic Inhibition)
            // We use a C1-continuous smoothstep to prevent integrator chattering!
            // If u_alpha is near the 0.01 lower bound, we apply K_recip_inhib.
            // By u_alpha = 0.05, the stretch reflex is fully active (1.0).
            double vol_act = std::clamp(
                (std::abs(traj_pos_pitch) + std::abs(traj_pos_roll) + std::abs(traj_pos_yaw)
                 + 0.2 * (std::abs(traj_vel_pitch) + std::abs(traj_vel_roll) + std::abs(traj_vel_yaw))) / 0.02,
                0.0, 1.0);
            double K_recip = 1.0 - vol_act * (1.0 - get_K_recip_inhib());"""

new_ccr_block = """            double p_ccr = get_p_ccr();
            double z_ccr = get_z_ccr();
            if (std::abs(z_ccr) < 1e-4) z_ccr = (z_ccr >= 0.0 ? 1e-4 : -1e-4);
            double r_ccr_norm = std::sqrt(std::clamp(std::abs(z_ccr) / std::max(1e-4, std::abs(p_ccr)), 1e-4, 100.0));
            double b_dir_ccr = 1.0 / r_ccr_norm;
            double b_state_ccr = (z_ccr - p_ccr) / r_ccr_norm;

            double Ia_filtered = b_dir_ccr * Ia_delayed + b_state_ccr * x_ccr;
            
            // In case filter makes it negative
            double Ia_firing = std::max(0.0, Ia_filtered);
            
            // Synergy-Based Sherrington Reciprocal Inhibition (Descending Presynaptic Inhibition)
            // Uses a C1-continuous smoothstep based on descending motor command u_alpha[i]:
            // When u_alpha[i] <= u_min_i (antagonist floor), stretch reflex is gated by K_recip_inhib.
            // By u_alpha[i] >= u_min_i + 0.05 (active agonist), stretch reflex is 100% active (1.0).
            double K_recip = get_K_recip_inhib();"""

assert old_ccr_block in cpp_src, "old_ccr_block not found"
cpp_src = cpp_src.replace(old_ccr_block, new_ccr_block, 1)

with open(CPP_PATH, "w", encoding="utf-8", newline="\r\n") as f:
    f.write(cpp_src)

print("Successfully patched HeadNeckNeuralController.h and HeadNeckNeuralController.cpp!")

