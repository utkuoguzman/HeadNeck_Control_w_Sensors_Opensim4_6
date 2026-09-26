#include "HeadNeckNeuralController.h"
#include <OpenSim/Simulation/Model/Muscle.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <iostream>
#include <algorithm>

using namespace OpenSim;
using namespace SimTK;

HeadNeckNeuralController::HeadNeckNeuralController() {
    constructProperty_delay_time(0.013);
    constructProperty_Kp_proprioception(0.5); 
    constructProperty_Kg_proprioception(0.0);
    constructProperty_K_vestibular(2.0);
    constructProperty_K_otolith(4.0);
    constructProperty_desired_yaw(0.0);
    constructProperty_desired_pitch(0.0);
    constructProperty_desired_roll(0.0);
    constructProperty_desired_yaw_v(0.0);
    constructProperty_desired_pitch_v(0.0);
    constructProperty_desired_roll_v(0.0);
    constructProperty_desired_yaw_a(0.0);
    constructProperty_desired_pitch_a(0.0);
    constructProperty_desired_roll_a(0.0);
    constructProperty_G_sc(1.0);
    constructProperty_G_ton(0.01);
    constructProperty_G_phas(0.01);
    constructProperty_k_p(0.45);
    constructProperty_k_v(0.13);
    constructProperty_K_gamma_dyn(0.0);
    constructProperty_K_gamma_stat(1.0);
    constructProperty_K_gamma(1.0);
    constructProperty_Kp_task(100.0);
    constructProperty_Ki_task(0.0);
    constructProperty_Kd_task(10.0);
    constructProperty_use_dynamic_jacobian(true);
    
    double k_reg = 2.83, T1_reg = 0.0175, T2_reg = 0.0027, Tc_reg = 5.7;
    fparams[0].a0 = 1.0 / (Tc_reg * T2_reg);
    fparams[0].a1 = 1.0 / Tc_reg + 1.0 / T2_reg;
    fparams[0].b1 = 1.0 / T1_reg;
    fparams[0].d = k_reg;
    fparams[0].c0 = -k_reg * fparams[0].a0;
    fparams[0].c1 = k_reg * (fparams[0].b1 - fparams[0].a1);

    double k_irr = 27.09, T1_irr = 0.03, T2_irr = 0.0006, Tc_irr = 5.7;
    fparams[1].a0 = 1.0 / (Tc_irr * T2_irr);
    fparams[1].a1 = 1.0 / Tc_irr + 1.0 / T2_irr;
    fparams[1].b1 = 1.0 / T1_irr;
    fparams[1].d = k_irr;
    fparams[1].c0 = -k_irr * fparams[1].a0;
    fparams[1].c1 = k_irr * (fparams[1].b1 - fparams[1].a1);
}

void HeadNeckNeuralController::extendAddToSystem(SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);
    for(int i=0; i<6; ++i) {
        for(int j=0; j<2; ++j) {
            for(int k=0; k<2; ++k) {
                addStateVariable("canal_" + std::to_string(i) + "_filter_" + std::to_string(j) + "_state_" + std::to_string(k), Stage::Dynamics);
            }
        }
    }
    
    // PID Integral state variables
    addStateVariable("pitch_error_integral", Stage::Dynamics);
    addStateVariable("roll_error_integral", Stage::Dynamics);
    addStateVariable("yaw_error_integral", Stage::Dynamics);
    
    // Mahony Filter State Variables
    addStateVariable("mahony_q0", Stage::Dynamics);
    addStateVariable("mahony_q1", Stage::Dynamics);
    addStateVariable("mahony_q2", Stage::Dynamics);
    addStateVariable("mahony_q3", Stage::Dynamics);
    
    // Efference Copy States (Internal Forward Model)
    addStateVariable("eff_tau_pitch", Stage::Dynamics);
    addStateVariable("eff_tau_roll", Stage::Dynamics);
    addStateVariable("eff_tau_yaw", Stage::Dynamics);
    
    // Cerebellar Disturbance Rejection (ESO) States
    addStateVariable("eso_tau_dist_pitch", Stage::Dynamics);
    addStateVariable("eso_tau_dist_roll", Stage::Dynamics);
    addStateVariable("eso_tau_dist_yaw", Stage::Dynamics);
}

void HeadNeckNeuralController::extendConnectToModel(Model& model) {
    Super::extendConnectToModel(model);
    is_initialized = false;
    last_jacobian_update_time = -1.0;
}

void HeadNeckNeuralController::extendInitStateFromProperties(SimTK::State& s) const {
    Super::extendInitStateFromProperties(s);
    is_initialized = false;
    last_jacobian_update_time = -1.0;
    history.clear();
    
    setStateVariableValue(s, "mahony_q0", 1.0);
    setStateVariableValue(s, "mahony_q1", 0.0);
    setStateVariableValue(s, "mahony_q2", 0.0);
    setStateVariableValue(s, "mahony_q3", 0.0);
    
    setStateVariableValue(s, "eff_tau_pitch", 0.0);
    setStateVariableValue(s, "eff_tau_roll", 0.0);
    setStateVariableValue(s, "eff_tau_yaw", 0.0);
    
    setStateVariableValue(s, "eso_tau_dist_pitch", 0.0);
    setStateVariableValue(s, "eso_tau_dist_roll", 0.0);
    setStateVariableValue(s, "eso_tau_dist_yaw", 0.0);
}

void HeadNeckNeuralController::initializeGeometryAndBaseline(const SimTK::State& s) const {
    const Model& model = getModel();
    const auto& muscles = model.getMuscles();
    int total_muscles = muscles.getSize();
    
    active_muscle_indices.clear();
    muscle_names.clear();
    
    std::vector<std::string> target_coords = {"pitch1", "pitch2", "roll1", "roll2", "yaw1", "yaw2"};
    
    for(int i=0; i<total_muscles; ++i) {
        const Muscle& m = muscles.get(i);
        double total_j = 0;
        for(const auto& cname : target_coords) {
            if(model.getCoordinateSet().contains(cname)) {
                Coordinate& coord = const_cast<Coordinate&>(model.getCoordinateSet().get(cname));
                total_j += std::abs(m.computeMomentArm(s, coord));
            }
        }
        if (total_j > 1e-6) {
            active_muscle_indices.push_back(i);
            muscle_names.push_back(m.getName());
        }
    }
    
    int num_muscles = active_muscle_indices.size();
    J_moment_arms.resize(3, num_muscles);
    base_lengths.resize(num_muscles, 0.0);
    base_activations.resize(num_muscles, 0.01);
    
    for(size_t i=0; i<active_muscle_indices.size(); ++i) {
        const Muscle& m = muscles.get(active_muscle_indices[i]);
        base_lengths[i] = m.getLength(s);
        base_activations[i] = m.getActivation(s);
        
        double r_p1 = model.getCoordinateSet().contains("pitch1") ? m.computeMomentArm(s, const_cast<Coordinate&>(model.getCoordinateSet().get("pitch1"))) : 0;
        double r_p2 = model.getCoordinateSet().contains("pitch2") ? m.computeMomentArm(s, const_cast<Coordinate&>(model.getCoordinateSet().get("pitch2"))) : 0;
        double r_r1 = model.getCoordinateSet().contains("roll1") ? m.computeMomentArm(s, const_cast<Coordinate&>(model.getCoordinateSet().get("roll1"))) : 0;
        double r_r2 = model.getCoordinateSet().contains("roll2") ? m.computeMomentArm(s, const_cast<Coordinate&>(model.getCoordinateSet().get("roll2"))) : 0;
        double r_y1 = model.getCoordinateSet().contains("yaw1") ? m.computeMomentArm(s, const_cast<Coordinate&>(model.getCoordinateSet().get("yaw1"))) : 0;
        double r_y2 = model.getCoordinateSet().contains("yaw2") ? m.computeMomentArm(s, const_cast<Coordinate&>(model.getCoordinateSet().get("yaw2"))) : 0;
        
        J_moment_arms(0, i) = (r_p1 + r_p2) / 2.0; // Pitch
        J_moment_arms(1, i) = (r_r1 + r_r2) / 2.0; // Roll
        J_moment_arms(2, i) = (r_y1 + r_y2) / 2.0; // Yaw
    }
    
    history.clear();
    is_initialized = true;
    last_jacobian_update_time = s.getTime();
}

HeadNeckNeuralController::DelayedData HeadNeckNeuralController::getDelayedData(double t, double delay) const {
    DelayedData result;
    result.a_lin = SimTK::Vec3(0.0);
    
    double target_t = t - delay;
    if (history.empty()) return result;
    auto it = history.lower_bound(target_t);
    if (it == history.end()) {
        result.record = history.rbegin()->second;
        return result;
    }
    if (it == history.begin()) {
        result.record = it->second;
        return result;
    }
    auto it_prev = it;
    --it_prev;
    double t1 = it_prev->first;
    double t2 = it->first;
    double w = (t2 > t1) ? (target_t - t1) / (t2 - t1) : 0.0;
    
    HistRecord res;
    res.omega_recon = it_prev->second.omega_recon * (1 - w) + it->second.omega_recon * w;
    if (it_prev->second.theta.size() == 3) {
        res.theta = it_prev->second.theta * (1 - w) + it->second.theta * w;
    }
    res.v_lin = it_prev->second.v_lin * (1 - w) + it->second.v_lin * w;
    
    if (it_prev->second.L.size() > 0 && it->second.L.size() == it_prev->second.L.size()) {
        res.L.resize(it_prev->second.L.size());
        res.L_dot.resize(it_prev->second.L_dot.size());
        for (int i = 0; i < res.L.size(); ++i) {
            res.L[i] = it_prev->second.L[i] * (1 - w) + it->second.L[i] * w;
            res.L_dot[i] = it_prev->second.L_dot[i] * (1 - w) + it->second.L_dot[i] * w;
        }
    }
    result.record = res;
    
    // --- NUMERICAL DIFFERENTIATION FOR ACCELERATION ---
    double dt_window = 0.010; // 10ms smoothing window
    double old_t = target_t - dt_window;
    auto it_old = history.lower_bound(old_t);
    if (it_old != history.end() && it_old != history.begin()) {
        auto it_old_prev = it_old;
        --it_old_prev;
        double t1_o = it_old_prev->first;
        double t2_o = it_old->first;
        double w_o = (t2_o > t1_o) ? (old_t - t1_o) / (t2_o - t1_o) : 0.0;
        
        SimTK::Vec3 v_old = it_old_prev->second.v_lin * (1 - w_o) + it_old->second.v_lin * w_o;
        SimTK::Vector w_old = it_old_prev->second.omega_recon * (1 - w_o) + it_old->second.omega_recon * w_o;
        
        result.a_lin = (res.v_lin - v_old) / dt_window;
        if (res.omega_recon.size() == 3 && w_old.size() == 3) {
            result.alpha_recon = (res.omega_recon - w_old) / dt_window;
        } else {
            result.alpha_recon = SimTK::Vector(3, 0.0);
        }
    } else {
        result.alpha_recon = SimTK::Vector(3, 0.0);
    }
    
    return result;
}

void HeadNeckNeuralController::computeStateVariableDerivatives(const SimTK::State& s) const {
    const Model& model = getModel();
    
    double u_p1 = model.getCoordinateSet().contains("pitch1") ? model.getCoordinateSet().get("pitch1").getSpeedValue(s) : 0;
    double u_p2 = model.getCoordinateSet().contains("pitch2") ? model.getCoordinateSet().get("pitch2").getSpeedValue(s) : 0;
    double u_r1 = model.getCoordinateSet().contains("roll1") ? model.getCoordinateSet().get("roll1").getSpeedValue(s) : 0;
    double u_r2 = model.getCoordinateSet().contains("roll2") ? model.getCoordinateSet().get("roll2").getSpeedValue(s) : 0;
    double u_y1 = model.getCoordinateSet().contains("yaw1") ? model.getCoordinateSet().get("yaw1").getSpeedValue(s) : 0;
    double u_y2 = model.getCoordinateSet().contains("yaw2") ? model.getCoordinateSet().get("yaw2").getSpeedValue(s) : 0;
    
    SimTK::Vector omega(3);
    omega[0] = u_r1 + u_r2; // roll
    omega[1] = u_y1 + u_y2; // yaw
    omega[2] = u_p1 + u_p2; // pitch
    
    // Della Santina et al. 2005 3D CT Human Semicircular Canal Normal Vectors in OpenSim [Roll=X, Yaw=Y, Pitch=Z]
    // Right Ear: RA, RP, RH (Sagittal symmetry: only Z changes sign relative to Left Ear)
    SimTK::Matrix B_R(3, 3);
    B_R(0,0) = -0.589; B_R(0,1) = -0.177; B_R(0,2) =  0.788; // RA
    B_R(1,0) = -0.694; B_R(1,1) = -0.270; B_R(1,2) = -0.667; // RP
    B_R(2,0) = -0.323; B_R(2,1) =  0.946; B_R(2,2) = -0.038; // RH

    // Left Ear: LA, LP, LH
    SimTK::Matrix B_L(3, 3);
    B_L(0,0) = -0.589; B_L(0,1) = -0.177; B_L(0,2) = -0.788; // LA
    B_L(1,0) = -0.694; B_L(1,1) = -0.270; B_L(1,2) =  0.667; // LP
    B_L(2,0) = -0.323; B_L(2,1) =  0.946; B_L(2,2) =  0.038; // LH
    
    SimTK::Vector v_R = B_R * omega;
    SimTK::Vector v_L = B_L * omega;
    
    double u[6] = {v_R[0], v_R[1], v_R[2], v_L[0], v_L[1], v_L[2]};
    
    for(int i=0; i<6; ++i) {
        for(int j=0; j<2; ++j) {
            std::string prefix = "canal_" + std::to_string(i) + "_filter_" + std::to_string(j);
            double x1 = getStateVariableValue(s, prefix + "_state_0");
            double x2 = getStateVariableValue(s, prefix + "_state_1");
            
            double dx1 = x2;
            double dx2 = -fparams[j].a0 * x1 - fparams[j].a1 * x2 + u[i];
            
            setStateVariableDerivativeValue(s, prefix + "_state_0", dx1);
            setStateVariableDerivativeValue(s, prefix + "_state_1", dx2);
        }
    }
    
    // PID Integrator derivatives (error = desired - actual)
    double q_p1 = model.getCoordinateSet().contains("pitch1") ? model.getCoordinateSet().get("pitch1").getValue(s) : 0;
    double q_p2 = model.getCoordinateSet().contains("pitch2") ? model.getCoordinateSet().get("pitch2").getValue(s) : 0;
    double q_r1 = model.getCoordinateSet().contains("roll1") ? model.getCoordinateSet().get("roll1").getValue(s) : 0;
    double q_r2 = model.getCoordinateSet().contains("roll2") ? model.getCoordinateSet().get("roll2").getValue(s) : 0;
    double q_y1 = model.getCoordinateSet().contains("yaw1") ? model.getCoordinateSet().get("yaw1").getValue(s) : 0;
    double q_y2 = model.getCoordinateSet().contains("yaw2") ? model.getCoordinateSet().get("yaw2").getValue(s) : 0;
    
    double head_pitch = q_p1 + q_p2;
    double head_roll = q_r1 + q_r2;
    double head_yaw = q_y1 + q_y2;
    
    setStateVariableDerivativeValue(s, "pitch_error_integral", get_desired_pitch() - head_pitch);
    setStateVariableDerivativeValue(s, "roll_error_integral", get_desired_roll() - head_roll);
    setStateVariableDerivativeValue(s, "yaw_error_integral", get_desired_yaw() - head_yaw);
    
    // --- EFFERENCE COPY (Muscle activation lag on commanded torque) ---
    double T_muscle = 0.050; // 50ms lag
    double eff_tau_pitch = getStateVariableValue(s, "eff_tau_pitch");
    double eff_tau_roll = getStateVariableValue(s, "eff_tau_roll");
    double eff_tau_yaw = getStateVariableValue(s, "eff_tau_yaw");
    
    double d_eff_pitch = (last_tau_des.size() == 3 ? last_tau_des[0] - eff_tau_pitch : 0.0) / T_muscle;
    double d_eff_roll = (last_tau_des.size() == 3 ? last_tau_des[1] - eff_tau_roll : 0.0) / T_muscle;
    double d_eff_yaw = (last_tau_des.size() == 3 ? last_tau_des[2] - eff_tau_yaw : 0.0) / T_muscle;
    
    setStateVariableDerivativeValue(s, "eff_tau_pitch", d_eff_pitch);
    setStateVariableDerivativeValue(s, "eff_tau_roll", d_eff_roll);
    setStateVariableDerivativeValue(s, "eff_tau_yaw", d_eff_yaw);
    
    // --- CEREBELLAR DISTURBANCE REJECTION (ESO) ---
    DelayedData vcr_data = getDelayedData(s.getTime(), 0.013);
    
    double I_pitch = 0.035; // Estimated Head Inertia (kg*m^2)
    double I_roll = 0.035;
    double I_yaw = 0.035;
    double T_eso = 0.100;   // 100ms Cerebellar processing delay / low-pass filter
    
    if (vcr_data.alpha_recon.size() == 3) {
        // 1. Actual Acceleration (from delayed Vestibular data)
        // Note: alpha_recon order is [Roll=0, Yaw=1, Pitch=2] to match omega_recon
        double alpha_actual_pitch = vcr_data.alpha_recon[2]; 
        double alpha_actual_roll = vcr_data.alpha_recon[0];
        double alpha_actual_yaw = vcr_data.alpha_recon[1];
        
        // 2. Expected Acceleration (from internal forward model)
        double alpha_expected_pitch = eff_tau_pitch / I_pitch;
        double alpha_expected_roll = eff_tau_roll / I_roll;
        double alpha_expected_yaw = eff_tau_yaw / I_yaw;
        
        // 3. Instantaneous Disturbance Torque
        double tau_inst_pitch = I_pitch * (alpha_actual_pitch - alpha_expected_pitch);
        double tau_inst_roll = I_roll * (alpha_actual_roll - alpha_expected_roll);
        double tau_inst_yaw = I_yaw * (alpha_actual_yaw - alpha_expected_yaw);
        
        // 4. Update the ESO Low-Pass Filter State
        double tau_dist_pitch = getStateVariableValue(s, "eso_tau_dist_pitch");
        double tau_dist_roll = getStateVariableValue(s, "eso_tau_dist_roll");
        double tau_dist_yaw = getStateVariableValue(s, "eso_tau_dist_yaw");
        
        setStateVariableDerivativeValue(s, "eso_tau_dist_pitch", (tau_inst_pitch - tau_dist_pitch) / T_eso);
        setStateVariableDerivativeValue(s, "eso_tau_dist_roll", (tau_inst_roll - tau_dist_roll) / T_eso);
        setStateVariableDerivativeValue(s, "eso_tau_dist_yaw", (tau_inst_yaw - tau_dist_yaw) / T_eso);
    } else {
        setStateVariableDerivativeValue(s, "eso_tau_dist_pitch", 0.0);
        setStateVariableDerivativeValue(s, "eso_tau_dist_roll", 0.0);
        setStateVariableDerivativeValue(s, "eso_tau_dist_yaw", 0.0);
    }
    
    // --- MAHONY FILTER (Cerebellar Sensory Fusion) ---
    SimTK::Vector omega_sense = vcr_data.record.omega_recon;
    
    if (omega_sense.size() == 3) {
        // 1. Simulate Otolith Attachment (Rotate World Accel -> Head Frame + Gravity)
        const Body& skull = model.getBodySet().get("skull");
        SimTK::Rotation R_world_to_head = skull.getMobilizedBody().getBodyTransform(s).R().invert();
        SimTK::Vec3 g_world(0.0, -9.81, 0.0);
        SimTK::Vec3 a_oto_head = R_world_to_head * (vcr_data.a_lin - g_world);
        
        // 2. Internal Forward Model (Efference Copy Subtraction)
        double K_eff_pitch = 1.33; // Pitch torque to X-axis acceleration
        // Disable Roll Efference Copy to isolate Pitch
        SimTK::Vec3 a_expected(K_eff_pitch * eff_tau_pitch, 0.0, 0.0);
        
        SimTK::Vec3 v_corrected = a_oto_head - a_expected;
        if (v_corrected.norm() > 1e-4) {
            v_corrected = v_corrected.normalize();
        }
        
        // 3. Current Mahony Belief
        double q0 = getStateVariableValue(s, "mahony_q0");
        double q1 = getStateVariableValue(s, "mahony_q1");
        double q2 = getStateVariableValue(s, "mahony_q2");
        double q3 = getStateVariableValue(s, "mahony_q3");
        
        // Normalize quaternion to prevent numerical drift
        double norm_q = std::sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        if (norm_q > 1e-4) { q0 /= norm_q; q1 /= norm_q; q2 /= norm_q; q3 /= norm_q; }
        
        // Estimated UP direction (rotate [0, 1, 0] by q^-1)
        // Since a_oto_head measures (a - g), when static it measures -g (which points UP).
        // Therefore, v_est must also estimate the UP vector to match it!
        double v_est_x = 2.0 * (q1*q2 + q0*q3);
        double v_est_y = (q0*q0 - q1*q1 + q2*q2 - q3*q3);
        double v_est_z = 2.0 * (q2*q3 - q0*q1);
        SimTK::Vec3 v_est(v_est_x, v_est_y, v_est_z);
        
        // Error = Corrected Otolith x Estimated UP Vector
        SimTK::Vec3 e = v_corrected % v_est;
        
        // 4. PI Gyroscope Correction
        double Kp_mahony = 2.0;
        double gx = omega_sense[0] + Kp_mahony * e[0]; // Roll
        double gy = omega_sense[1] + Kp_mahony * e[1]; // Yaw
        double gz = omega_sense[2] + Kp_mahony * e[2]; // Pitch
        
        // 5. Quaternion Derivative
        double dq0 = 0.5 * (-q1*gx - q2*gy - q3*gz);
        double dq1 = 0.5 * ( q0*gx + q2*gz - q3*gy);
        double dq2 = 0.5 * ( q0*gy - q1*gz + q3*gx);
        double dq3 = 0.5 * ( q0*gz + q1*gy - q2*gx);
        
        setStateVariableDerivativeValue(s, "mahony_q0", dq0);
        setStateVariableDerivativeValue(s, "mahony_q1", dq1);
        setStateVariableDerivativeValue(s, "mahony_q2", dq2);
        setStateVariableDerivativeValue(s, "mahony_q3", dq3);
    } else {
        setStateVariableDerivativeValue(s, "mahony_q0", 0.0);
        setStateVariableDerivativeValue(s, "mahony_q1", 0.0);
        setStateVariableDerivativeValue(s, "mahony_q2", 0.0);
        setStateVariableDerivativeValue(s, "mahony_q3", 0.0);
    }
}

void HeadNeckNeuralController::computeControls(const SimTK::State& s, SimTK::Vector& controls) const {
    if (!is_initialized) {
        initializeGeometryAndBaseline(s);
    }
    
    const Model& model = getModel();
    const auto& muscles = model.getMuscles();
    int N = active_muscle_indices.size();
    
    // 1. Update Jacobian slowly
    if (s.getTime() - last_jacobian_update_time > 0.05) {
        for(int i=0; i<N; ++i) {
            const Muscle& m = muscles.get(active_muscle_indices[i]);
            double r_p1 = model.getCoordinateSet().contains("pitch1") ? m.computeMomentArm(s, const_cast<Coordinate&>(model.getCoordinateSet().get("pitch1"))) : 0;
            double r_p2 = model.getCoordinateSet().contains("pitch2") ? m.computeMomentArm(s, const_cast<Coordinate&>(model.getCoordinateSet().get("pitch2"))) : 0;
            double r_r1 = model.getCoordinateSet().contains("roll1") ? m.computeMomentArm(s, const_cast<Coordinate&>(model.getCoordinateSet().get("roll1"))) : 0;
            double r_r2 = model.getCoordinateSet().contains("roll2") ? m.computeMomentArm(s, const_cast<Coordinate&>(model.getCoordinateSet().get("roll2"))) : 0;
            double r_y1 = model.getCoordinateSet().contains("yaw1") ? m.computeMomentArm(s, const_cast<Coordinate&>(model.getCoordinateSet().get("yaw1"))) : 0;
            double r_y2 = model.getCoordinateSet().contains("yaw2") ? m.computeMomentArm(s, const_cast<Coordinate&>(model.getCoordinateSet().get("yaw2"))) : 0;
            
            J_moment_arms(0, i) = (r_p1 + r_p2) / 2.0;
            J_moment_arms(1, i) = (r_r1 + r_r2) / 2.0;
            J_moment_arms(2, i) = (r_y1 + r_y2) / 2.0;
        }
        last_jacobian_update_time = s.getTime();
    }
    
    double q_p1 = model.getCoordinateSet().contains("pitch1") ? model.getCoordinateSet().get("pitch1").getValue(s) : 0;
    double q_p2 = model.getCoordinateSet().contains("pitch2") ? model.getCoordinateSet().get("pitch2").getValue(s) : 0;
    double u_p1 = model.getCoordinateSet().contains("pitch1") ? model.getCoordinateSet().get("pitch1").getSpeedValue(s) : 0;
    double u_p2 = model.getCoordinateSet().contains("pitch2") ? model.getCoordinateSet().get("pitch2").getSpeedValue(s) : 0;
    
    double q_r1 = model.getCoordinateSet().contains("roll1") ? model.getCoordinateSet().get("roll1").getValue(s) : 0;
    double q_r2 = model.getCoordinateSet().contains("roll2") ? model.getCoordinateSet().get("roll2").getValue(s) : 0;
    double u_r1 = model.getCoordinateSet().contains("roll1") ? model.getCoordinateSet().get("roll1").getSpeedValue(s) : 0;
    double u_r2 = model.getCoordinateSet().contains("roll2") ? model.getCoordinateSet().get("roll2").getSpeedValue(s) : 0;
    
    double q_y1 = model.getCoordinateSet().contains("yaw1") ? model.getCoordinateSet().get("yaw1").getValue(s) : 0;
    double q_y2 = model.getCoordinateSet().contains("yaw2") ? model.getCoordinateSet().get("yaw2").getValue(s) : 0;
    double u_y1 = model.getCoordinateSet().contains("yaw1") ? model.getCoordinateSet().get("yaw1").getSpeedValue(s) : 0;
    double u_y2 = model.getCoordinateSet().contains("yaw2") ? model.getCoordinateSet().get("yaw2").getSpeedValue(s) : 0;
    
    double head_pitch = q_p1 + q_p2;
    double head_roll = q_r1 + q_r2;
    double head_yaw = q_y1 + q_y2;
    
    double head_pitch_dot = u_p1 + u_p2;
    double head_roll_dot = u_r1 + u_r2;
    double head_yaw_dot = u_y1 + u_y2;
    
    // --- VESTIBULAR RECONSTRUCTION AND HISTORY ---
    SimTK::Vector omega(3);
    omega[0] = head_roll_dot;
    omega[1] = head_yaw_dot;
    omega[2] = head_pitch_dot;
    
    // Della Santina et al. 2005 3D CT Human Semicircular Canal Normal Vectors in OpenSim [Roll=X, Yaw=Y, Pitch=Z]
    // Right Ear: RA, RP, RH (Sagittal symmetry: only Z changes sign relative to Left Ear)
    SimTK::Matrix B_R(3, 3);
    B_R(0,0) = -0.589; B_R(0,1) = -0.177; B_R(0,2) =  0.788; // RA
    B_R(1,0) = -0.694; B_R(1,1) = -0.270; B_R(1,2) = -0.667; // RP
    B_R(2,0) = -0.323; B_R(2,1) =  0.946; B_R(2,2) = -0.038; // RH

    // Left Ear: LA, LP, LH
    SimTK::Matrix B_L(3, 3);
    B_L(0,0) = -0.589; B_L(0,1) = -0.177; B_L(0,2) = -0.788; // LA
    B_L(1,0) = -0.694; B_L(1,1) = -0.270; B_L(1,2) =  0.667; // LP
    B_L(2,0) = -0.323; B_L(2,1) =  0.946; B_L(2,2) =  0.038; // LH
    
    SimTK::Vector v_R = B_R * omega;
    SimTK::Vector v_L = B_L * omega;
    double u[6] = {v_R[0], v_R[1], v_R[2], v_L[0], v_L[1], v_L[2]};
    
    double y[6] = {0};
    for(int i=0; i<6; ++i) {
        double y_total = 0;
        for(int j=0; j<2; ++j) {
            std::string prefix = "canal_" + std::to_string(i) + "_filter_" + std::to_string(j);
            double x1 = getStateVariableValue(s, prefix + "_state_0");
            double x2 = getStateVariableValue(s, prefix + "_state_1");
            double y_j = fparams[j].c0 * x1 + fparams[j].c1 * x2 + fparams[j].d * u[i];
            y_total += y_j;
        }
        y[i] = y_total / 2.0;
    }
    
    SimTK::Vector v_R_filt(3); v_R_filt[0] = y[0]; v_R_filt[1] = y[1]; v_R_filt[2] = y[2];
    SimTK::Vector v_L_filt(3); v_L_filt[0] = y[3]; v_L_filt[1] = y[4]; v_L_filt[2] = y[5];
    
    SimTK::Vector omega_R_recon = B_R.invert() * v_R_filt;
    SimTK::Vector omega_L_recon = B_L.invert() * v_L_filt;
    SimTK::Vector omega_recon = (omega_R_recon + omega_L_recon) / 2.0;
    
    HistRecord hr;
    hr.theta = SimTK::Vector(3);
    hr.theta[0] = head_roll;
    hr.theta[1] = head_yaw;
    hr.theta[2] = head_pitch;
    hr.omega_recon = omega_recon;
    hr.v_lin = model.getBodySet().get("skull").getVelocityInGround(s)[1];
    
    hr.L.resize(N);
    hr.L_dot.resize(N);
    hr.Ia.resize(N);
    for(int i=0; i<N; ++i) {
        const Muscle& m = muscles.get(active_muscle_indices[i]);
        hr.L[i] = m.getLength(s);
        hr.L_dot[i] = m.getLengtheningSpeed(s);
        hr.Ia[i] = 0.0;
        
        try {
            double L_norm = m.getNormalizedFiberLength(s);
            double u = (L_norm - 1.0) * 100.0;
            
            // Try to find the spindle component inside the muscle dynamically without linking!
            const Component* spindle = nullptr;
            for (const auto& comp : m.getComponentList()) {
                if (comp.getName() == "spindle") {
                    spindle = &comp;
                    break;
                }
            }
            if (spindle) {
                // Mileusnic06 Spindle
                try {
                    double T_bag1 = spindle->getStateVariableValue(s, "tension_bag1");
                    double T_bag2 = spindle->getStateVariableValue(s, "tension_bag2");
                    double T_chain = spindle->getStateVariableValue(s, "tension_chain");
                    
                    // Parameters from Mileusnic06 Table 1
                    double L_0SR_bag1 = 0.04, L_NSR_bag1 = 0.0423, K_SR_bag1 = 10.4649, G_bag1 = 20000.0;
                    double L_0SR_bag2 = 0.04, L_NSR_bag2 = 0.0423, K_SR_bag2 = 10.4649, L_sec_bag2 = 0.04, L_0PR_bag2 = 0.76, L_NPR_bag2 = 0.89, X_bag2 = 0.7, G_pri_bag2 = 7200.0;
                    double L_0SR_chain = 0.04, L_NSR_chain = 0.0423, K_SR_chain = 10.4649, L_sec_chain = 0.04, L_0PR_chain = 0.76, L_NPR_chain = 0.89, X_chain = 0.7, G_pri_chain = 7200.0;
                    
                    double L = L_norm; // normalized length
                    
                    double APbag1 = G_bag1 * ( (T_bag1/K_SR_bag1) - (L_NSR_bag1 - L_0SR_bag1) );
                    double APbag2 = X_bag2 * (L_sec_bag2/L_0SR_bag2) * ((T_bag2/K_SR_bag2) - (L_NSR_bag2 - L_0SR_bag2)) + (1.0-X_bag2) * (L_sec_bag2/L_0PR_bag2) * (L - (T_bag2/K_SR_bag2) - L_0SR_bag2 - L_NPR_bag2); 
                    double APchain = X_chain * (L_sec_chain/L_0SR_chain) * ((T_chain/K_SR_chain) - (L_NSR_chain - L_0SR_chain)) + (1.0-X_chain) * (L_sec_chain/L_0PR_chain) * (L - (T_chain/K_SR_chain) - L_0SR_chain - L_NPR_chain);
                    
                    double pri_stat = G_pri_bag2*APbag2 + G_pri_chain*APchain;
                    double diff = APbag1 - pri_stat;
                    double S = 0.156;
                    double s_max = 0.5 * (APbag1 + pri_stat + std::sqrt(diff*diff + 1e-4));
                    double s_min = 0.5 * (APbag1 + pri_stat - std::sqrt(diff*diff + 1e-4));
                    
                    double primary = s_max + S * s_min;
                    
                    if (primary < 0.0) primary = 0.0;
                    if (primary > 100000.0) primary = 100000.0; // Avoid NaNs
                    
                    hr.Ia[i] = primary;
                } catch(...) {
                    // Not a Mileusnic spindle
                }
            }
        } catch(...) {
            // Ignore
        }
    }
    history[s.getTime()] = hr;
    
    double oldest = s.getTime() - 0.050; // keep 50ms
    while(!history.empty() && history.begin()->first < oldest) {
        history.erase(history.begin());
    }
    
    DelayedData vcr_data = getDelayedData(s.getTime(), 0.013);
    DelayedData ccr_data = getDelayedData(s.getTime(), 0.018);
    
    HistRecord vcr_record = vcr_data.record;
    HistRecord ccr_record = ccr_data.record;

    // --- VCR COMMAND ---
    Vector tau_des(3, 0.0);
    
    // Extract Estimated Angles from Mahony Quaternion
    double q0 = getStateVariableValue(s, "mahony_q0");
    double q1 = getStateVariableValue(s, "mahony_q1");
    double q2 = getStateVariableValue(s, "mahony_q2");
    double q3 = getStateVariableValue(s, "mahony_q3");
    
    double norm_q = std::sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (norm_q > 1e-4) { q0 /= norm_q; q1 /= norm_q; q2 /= norm_q; q3 /= norm_q; }
    
    // Euler angles (Roll=X, Yaw=Y, Pitch=Z)
    // Standard aerospace sequence for quaternion extraction
    double estimated_roll = std::atan2(2.0*(q0*q1 + q2*q3), 1.0 - 2.0*(q1*q1 + q2*q2));
    double estimated_yaw = std::asin(std::clamp(2.0*(q0*q2 - q3*q1), -1.0, 1.0));
    double estimated_pitch = std::atan2(2.0*(q0*q3 + q1*q2), 1.0 - 2.0*(q2*q2 + q3*q3));
    
    // Vestibular Reflex with Efference Copy Cancellation (expected vestibular firing from voluntary action)
    if (vcr_record.omega_recon.size() == 3) {
        // G_ton (Otolith Tonic) acts as a proportional gravity compensator
        // We subtract the Efference Copy of expected static tilt
        tau_des[0] -= get_G_ton() * (estimated_pitch - get_desired_pitch()); // pitch
        tau_des[1] -= get_G_ton() * (estimated_roll - get_desired_roll());  // roll
        tau_des[2] -= get_G_ton() * (estimated_yaw - get_desired_yaw());   // yaw
        
        // G_sc (Semicircular Canal) dampens angular velocity
        // We subtract the Efference Copy of expected angular velocity
        tau_des[0] -= get_G_sc() * (vcr_record.omega_recon[2] - get_desired_pitch_v()); // pitch
        tau_des[1] -= get_G_sc() * (vcr_record.omega_recon[0] - get_desired_roll_v()); // roll
        tau_des[2] -= get_G_sc() * (vcr_record.omega_recon[1] - get_desired_yaw_v()); // yaw
        
        // G_phas (Otolith Phasic) dampens linear acceleration
        // We use vcr_data.a_lin to apply the delayed acceleration.
        // We rotate the delayed World acceleration into the Head frame using the Mahony quaternion!
        SimTK::Vec3 a_world = vcr_data.a_lin;
        
        // Rotate a_world into a_head using q^-1
        double ah_x = (q0*q0 + q1*q1 - q2*q2 - q3*q3)*a_world[0] + 2.0*(q1*q2 + q0*q3)*a_world[1] + 2.0*(q1*q3 - q0*q2)*a_world[2];
        double ah_y = 2.0*(q1*q2 - q0*q3)*a_world[0] + (q0*q0 - q1*q1 + q2*q2 - q3*q3)*a_world[1] + 2.0*(q2*q3 + q0*q1)*a_world[2];
        double ah_z = 2.0*(q1*q3 + q0*q2)*a_world[0] + 2.0*(q2*q3 - q0*q1)*a_world[1] + (q0*q0 - q1*q1 - q2*q2 + q3*q3)*a_world[2];
        
        // Estimate expected linear acceleration caused by desired angular acceleration
        // Approximating head radius (e.g. 0.15m from neck pivot to otoliths)
        double r_head = 0.15; 
        double expected_ah_x = get_desired_pitch_a() * r_head; 
        double expected_ah_z = get_desired_roll_a() * r_head;
        
        // Pitch responds to X (Forward), Roll responds to Z (Lateral)
        tau_des[0] -= get_G_phas() * (ah_x - expected_ah_x); 
        tau_des[1] -= get_G_phas() * (ah_z - expected_ah_z); 
    }
    
    // Voluntary Postural PID Drive (compensates for gravity droop / Na_post equivalent)
    double pitch_int = getStateVariableValue(s, "pitch_error_integral");
    double roll_int = getStateVariableValue(s, "roll_error_integral");
    double yaw_int = getStateVariableValue(s, "yaw_error_integral");
    
    // Pitch uses the dynamically tuned Kp_task
    tau_des[0] += get_Kp_task() * (get_desired_pitch() - head_pitch) - get_Kd_task() * omega[2] + get_Ki_task() * pitch_int;
    tau_des[1] += get_Kp_task() * (get_desired_roll() - head_roll) - get_Kd_task() * omega[0] + get_Ki_task() * roll_int;
    tau_des[2] += get_Kp_task() * (get_desired_yaw() - head_yaw) - get_Kd_task() * omega[1] + get_Ki_task() * yaw_int;
    
    // Inject Cerebellar Disturbance Rejection (ESO)
    double eso_pitch = getStateVariableValue(s, "eso_tau_dist_pitch");
    double eso_roll = getStateVariableValue(s, "eso_tau_dist_roll");
    double eso_yaw = getStateVariableValue(s, "eso_tau_dist_yaw");
    
    double K_eso = 0.8; // Fractional confidence to prevent inertia overestimation oscillations
    tau_des[0] -= K_eso * eso_pitch;
    tau_des[1] -= K_eso * eso_roll;
    tau_des[2] -= K_eso * eso_yaw;
    
    tau_des[0] = std::clamp(tau_des[0], -300.0, 300.0);
    tau_des[1] = std::clamp(tau_des[1], -100.0, 100.0);
    tau_des[2] = std::clamp(tau_des[2], -100.0, 100.0);
    
    if (last_tau_des.size() != 3) last_tau_des.resize(3);
    last_tau_des[0] = tau_des[0];
    last_tau_des[1] = tau_des[1];
    last_tau_des[2] = tau_des[2];
    
    // --- QP SOLVER ---
    std::vector<double> F_max(N);
    for(int i=0; i<N; ++i) F_max[i] = muscles.get(active_muscle_indices[i]).getMaxIsometricForce();

    std::vector<double> u_alpha(N, 0.01);
    std::vector<bool> is_free(N, true);

    for(int iter=0; iter<15; ++iter) {
        Vector tau_fixed(3, 0.0);
        for(int i=0; i<N; ++i) {
            if(!is_free[i]) {
                for(int k=0; k<3; ++k) tau_fixed[k] += (J_moment_arms(k, i) * F_max[i]) * u_alpha[i];
            }
        }
        
        Vector tau_rem = tau_des - tau_fixed;
        
        Matrix A_A_T(3, 3);
        A_A_T.setToZero();
        for(int i=0; i<N; ++i) {
            if(is_free[i]) {
                for(int r=0; r<3; ++r) {
                    for(int c=0; c<3; ++c) {
                        A_A_T(r, c) += (J_moment_arms(r, i) * F_max[i]) * (J_moment_arms(c, i) * F_max[i]);
                    }
                }
            }
        }
        
        A_A_T(0,0) += 1.0; A_A_T(1,1) += 1.0; A_A_T(2,2) += 1.0; // Strong regularization
        
        FactorLU lu(A_A_T);
        Vector rem_vec(3);
        for(int k=0; k<3; ++k) rem_vec[k] = tau_rem[k];
        
        Vector lambda_vec(3);
        lu.solve(rem_vec, lambda_vec);
        
        bool bounds_violated = false;
        for(int i=0; i<N; ++i) {
            if(is_free[i]) {
                double a_unconstrained = 0.0;
                for(int k=0; k<3; ++k) a_unconstrained += (J_moment_arms(k, i) * F_max[i]) * lambda_vec[k];
                
                if(a_unconstrained < 0.01) {
                    u_alpha[i] = 0.01;
                    is_free[i] = false;
                    bounds_violated = true;
                } else if(a_unconstrained > 1.0) {
                    u_alpha[i] = 1.0;
                    is_free[i] = false;
                    bounds_violated = true;
                } else {
                    u_alpha[i] = a_unconstrained;
                }
            }
        }
        if(!bounds_violated) break;
    }
    
    // --- CCR AND FINAL ASSEMBLY ---
    for(int i=0; i<N; ++i) {
        const Muscle& m = muscles.get(active_muscle_indices[i]);
        
        double excitation = u_alpha[i];
        
        if (ccr_record.Ia.size() == N) {
            // Cervico-Collic Reflex (CCR) - using biological firing rate!
            double Ia_firing = ccr_record.Ia[i];
            
            // We scale the firing rate (0-500) into excitation (0-1) using Kp_proprioception
            // Optionally, we could subtract a baseline firing rate so resting state = 0 excitation
            excitation += get_Kp_proprioception() * (Ia_firing / 100.0); // divide by 100 to normalize to roughly 0-1 range
        }
        
        excitation = std::clamp(excitation, 0.01, 1.0);
        
        int n_controls = m.numControls();
        Vector my_ctrl(n_controls, 0.0);
        my_ctrl[0] = excitation; // Alpha motor command
        
        // If this muscle supports Gamma motor neurons (e.g., Millard12EqMuscleWithAfferents)
        if (n_controls >= 3) {
            my_ctrl[1] = get_K_gamma_dyn() * excitation;  // Gamma dynamic
            my_ctrl[2] = get_K_gamma_stat() * excitation; // Gamma static
        }
        
        m.addInControls(my_ctrl, controls);
    }
}
