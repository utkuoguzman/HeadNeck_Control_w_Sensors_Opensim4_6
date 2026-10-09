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
    constructProperty_z_ccr(10.0);
    constructProperty_p_ccr(10.0);
    constructProperty_K_recip_inhib(0.0); // 0.0 means complete reciprocal inhibition (no antagonist reflex)
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
    constructProperty_K_ton(52.35);
    constructProperty_z_ton(0.01);
    constructProperty_p_ton(8.03);
    constructProperty_K_sc(18.09);
    constructProperty_z_sc(0.01);
    constructProperty_p_sc(3.93);
    constructProperty_K_phas(8.68);
    constructProperty_z_phas(17.17);
    constructProperty_p_phas(28.72);
    constructProperty_k_p(0.45);
    constructProperty_k_v(0.13);
    constructProperty_K_gamma_dyn(0.0);
    constructProperty_K_gamma_stat(1.0);
    constructProperty_K_gamma(1.0);
    constructProperty_omega_n_pitch(4.47);
    constructProperty_zeta_pitch(1.0);
    constructProperty_omega_n_roll(4.47);
    constructProperty_zeta_roll(1.0);
    constructProperty_omega_n_yaw(4.47);
    constructProperty_zeta_yaw(1.0);
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
                addStateVariable("canal_" + std::to_string(i) + "_filter_" + std::to_string(j) + "_state_" + std::to_string(k), SimTK::Stage::Velocity);
            }
        }
    }
    
    // PID Integral state variables
    addStateVariable("pitch_error_integral", SimTK::Stage::Velocity);
    addStateVariable("roll_error_integral", SimTK::Stage::Velocity);
    addStateVariable("yaw_error_integral", SimTK::Stage::Velocity);
    
    // SE_2(3) Spatial Observer States
    addStateVariable("obs_q0", SimTK::Stage::Velocity);
    addStateVariable("obs_q1", SimTK::Stage::Velocity);
    addStateVariable("obs_q2", SimTK::Stage::Velocity);
    addStateVariable("obs_q3", SimTK::Stage::Velocity);
    addStateVariable("obs_vx", SimTK::Stage::Velocity);
    addStateVariable("obs_vy", SimTK::Stage::Velocity);
    addStateVariable("obs_vz", SimTK::Stage::Velocity);
    addStateVariable("obs_px", SimTK::Stage::Velocity);
    addStateVariable("obs_py", SimTK::Stage::Velocity);
    addStateVariable("obs_pz", SimTK::Stage::Velocity);
    
    // Efference Copy States (Internal Forward Model)
    addStateVariable("eff_tau_pitch", SimTK::Stage::Velocity);
    addStateVariable("eff_tau_roll", SimTK::Stage::Velocity);
    addStateVariable("eff_tau_yaw", SimTK::Stage::Velocity);
    
    // Cerebellar Disturbance Rejection (ESO) States
    addStateVariable("eso_tau_dist_pitch", SimTK::Stage::Velocity);
    addStateVariable("eso_tau_dist_roll", SimTK::Stage::Velocity);
    addStateVariable("eso_tau_dist_yaw", SimTK::Stage::Velocity);
    
    // 2nd stage of ESO (making it a 2nd-order low-pass filter)
    addStateVariable("eso_tau_dist_pitch_2", SimTK::Stage::Velocity);
    addStateVariable("eso_tau_dist_roll_2", SimTK::Stage::Velocity);
    addStateVariable("eso_tau_dist_yaw_2", SimTK::Stage::Velocity);
    
    // Continuous dirty derivative states for acceleration
    addStateVariable("omega_filt_pitch", SimTK::Stage::Velocity);
    addStateVariable("omega_filt_roll", SimTK::Stage::Velocity);
    addStateVariable("omega_filt_yaw", SimTK::Stage::Velocity);

    addStateVariable("ton_state_pitch", SimTK::Stage::Velocity);
    addStateVariable("ton_state_roll", SimTK::Stage::Velocity);
    addStateVariable("ton_state_yaw", SimTK::Stage::Velocity);
    addStateVariable("sc_state_pitch", SimTK::Stage::Velocity);
    addStateVariable("sc_state_roll", SimTK::Stage::Velocity);
    addStateVariable("sc_state_yaw", SimTK::Stage::Velocity);
    addStateVariable("phas_state_pitch", SimTK::Stage::Velocity);
    addStateVariable("phas_state_roll", SimTK::Stage::Velocity);
    addStateVariable("phas_state_yaw", SimTK::Stage::Velocity);

    // Cervico-Collic Reflex Continuous State Variables
    // The number of muscles is fixed during model building.
    const auto& muscles = getModel().getMuscles();
    for (int i = 0; i < muscles.getSize(); ++i) {
        addStateVariable("ccr_state_" + muscles.get(i).getName(), SimTK::Stage::Velocity);
    }
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
    
    setStateVariableValue(s, "obs_q0", 1.0);
    setStateVariableValue(s, "obs_q1", 0.0);
    setStateVariableValue(s, "obs_q2", 0.0);
    setStateVariableValue(s, "obs_q3", 0.0);
    setStateVariableValue(s, "obs_vx", 0.0);
    setStateVariableValue(s, "obs_vy", 0.0);
    setStateVariableValue(s, "obs_vz", 0.0);
    setStateVariableValue(s, "obs_px", 0.0);
    setStateVariableValue(s, "obs_py", 0.0);
    setStateVariableValue(s, "obs_pz", 0.0);
    
    setStateVariableValue(s, "eff_tau_pitch", 0.0);
    setStateVariableValue(s, "eff_tau_roll", 0.0);
    setStateVariableValue(s, "eff_tau_yaw", 0.0);
    
    setStateVariableValue(s, "eso_tau_dist_pitch", 0.0);
    setStateVariableValue(s, "eso_tau_dist_roll", 0.0);
    setStateVariableValue(s, "eso_tau_dist_yaw", 0.0);
    
    setStateVariableValue(s, "eso_tau_dist_pitch_2", 0.0);
    setStateVariableValue(s, "eso_tau_dist_roll_2", 0.0);
    setStateVariableValue(s, "eso_tau_dist_yaw_2", 0.0);
    
    setStateVariableValue(s, "omega_filt_pitch", 0.0);
    setStateVariableValue(s, "omega_filt_roll", 0.0);
    setStateVariableValue(s, "omega_filt_yaw", 0.0);
    
    const auto& muscles = getModel().getMuscles();
    for (int i = 0; i < muscles.getSize(); ++i) {
        setStateVariableValue(s, "ccr_state_" + muscles.get(i).getName(), 0.0);
    }
}

void HeadNeckNeuralController::initializeGeometryAndBaseline(const SimTK::State& s) const {
    const Model& model = getModel();
    
    // Capture anatomical resting orientation as Vestibular Baseline!
    const Body& skull = model.getBodySet().get("skull");
    SimTK::Rotation R_head_to_world = skull.getMobilizedBody().getBodyTransform(s).R();
    SimTK::Quaternion base_q = R_head_to_world.convertRotationToQuaternion();
    double bq0 = base_q[0], bq1 = base_q[1], bq2 = base_q[2], bq3 = base_q[3];
    baseline_spatial_orientation[0] = std::atan2(2.0*(bq0*bq1 + bq2*bq3), 1.0 - 2.0*(bq1*bq1 + bq2*bq2)); // roll
    baseline_spatial_orientation[1] = std::asin(std::clamp(2.0*(bq0*bq2 - bq3*bq1), -1.0, 1.0)); // yaw
    baseline_spatial_orientation[2] = std::atan2(2.0*(bq0*bq3 + bq1*bq2), 1.0 - 2.0*(bq2*bq2 + bq3*bq3)); // pitch

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
        
        std::string mname = m.getName();
        if (mname.find("hyoid") != std::string::npos || 
            mname.find("digastric") != std::string::npos || 
            mname.find("Mylohyoid") != std::string::npos || 
            mname.find("Geniohyoid") != std::string::npos || 
            mname.find("Stylohyoid") != std::string::npos || 
            mname.find("Omo_hyoid") != std::string::npos || 
            mname.find("Sterno_hyoid") != std::string::npos) {
            // J_moment_arms(2, i) = 0.0;
        }
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
        result.alpha_recon = SimTK::Vector(3, 0.0); // Now calculated continuously via state-space dirty derivative
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
    
    double p_err = get_desired_pitch() - head_pitch;
    double p_int = getStateVariableValue(s, "pitch_error_integral");
    if (p_int > 0.6 && p_err > 0) p_err = 0.0;
    if (p_int < -0.6 && p_err < 0) p_err = 0.0;
    setStateVariableDerivativeValue(s, "pitch_error_integral", p_err);

    double r_err = get_desired_roll() - head_roll;
    double r_int = getStateVariableValue(s, "roll_error_integral");
    if (r_int > 0.4 && r_err > 0) r_err = 0.0;
    if (r_int < -0.4 && r_err < 0) r_err = 0.0;
    setStateVariableDerivativeValue(s, "roll_error_integral", r_err);

    double y_err = get_desired_yaw() - head_yaw;
    double y_int = getStateVariableValue(s, "yaw_error_integral");
    if (y_int > 0.4 && y_err > 0) y_err = 0.0;
    if (y_int < -0.4 && y_err < 0) y_err = 0.0;
    setStateVariableDerivativeValue(s, "yaw_error_integral", y_err);
    
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
    
    if (vcr_data.record.omega_recon.size() == 3) {
        // --- CONTINUOUS DIRTY DERIVATIVE FOR ACCELERATION ---
        double T_diff = 0.020; // 20ms smoothing time constant for derivative
        double omega_raw_roll = vcr_data.record.omega_recon[0];
        double omega_raw_yaw = vcr_data.record.omega_recon[1];
        double omega_raw_pitch = vcr_data.record.omega_recon[2];
        
        double omega_filt_roll = getStateVariableValue(s, "omega_filt_roll");
        double omega_filt_yaw = getStateVariableValue(s, "omega_filt_yaw");
        double omega_filt_pitch = getStateVariableValue(s, "omega_filt_pitch");
        
        double alpha_actual_roll = (omega_raw_roll - omega_filt_roll) / T_diff;
        double alpha_actual_yaw = (omega_raw_yaw - omega_filt_yaw) / T_diff;
        double alpha_actual_pitch = (omega_raw_pitch - omega_filt_pitch) / T_diff;
        
        setStateVariableDerivativeValue(s, "omega_filt_roll", alpha_actual_roll);
        setStateVariableDerivativeValue(s, "omega_filt_yaw", alpha_actual_yaw);
        setStateVariableDerivativeValue(s, "omega_filt_pitch", alpha_actual_pitch);
        
        // 2. Expected Acceleration (from internal forward model)
        double alpha_expected_pitch = eff_tau_pitch / I_pitch;
        double alpha_expected_roll = eff_tau_roll / I_roll;
        double alpha_expected_yaw = eff_tau_yaw / I_yaw;
        
        // 3. Instantaneous Disturbance Torque
        double tau_inst_pitch = I_pitch * (alpha_actual_pitch - alpha_expected_pitch);
        double tau_inst_roll = I_roll * (alpha_actual_roll - alpha_expected_roll);
        double tau_inst_yaw = I_yaw * (alpha_actual_yaw - alpha_expected_yaw);
        
        // 4. Update the ESO Low-Pass Filter State (1st stage)
        double tau_dist_pitch_1 = getStateVariableValue(s, "eso_tau_dist_pitch");
        double tau_dist_roll_1 = getStateVariableValue(s, "eso_tau_dist_roll");
        double tau_dist_yaw_1 = getStateVariableValue(s, "eso_tau_dist_yaw");
        
        setStateVariableDerivativeValue(s, "eso_tau_dist_pitch", (tau_inst_pitch - tau_dist_pitch_1) / T_eso);
        setStateVariableDerivativeValue(s, "eso_tau_dist_roll", (tau_inst_roll - tau_dist_roll_1) / T_eso);
        setStateVariableDerivativeValue(s, "eso_tau_dist_yaw", (tau_inst_yaw - tau_dist_yaw_1) / T_eso);
        
        // 5. Update the ESO Low-Pass Filter State (2nd stage)
        double tau_dist_pitch_2 = getStateVariableValue(s, "eso_tau_dist_pitch_2");
        double tau_dist_roll_2 = getStateVariableValue(s, "eso_tau_dist_roll_2");
        double tau_dist_yaw_2 = getStateVariableValue(s, "eso_tau_dist_yaw_2");
        
        setStateVariableDerivativeValue(s, "eso_tau_dist_pitch_2", (tau_dist_pitch_1 - tau_dist_pitch_2) / T_eso);
        setStateVariableDerivativeValue(s, "eso_tau_dist_roll_2", (tau_dist_roll_1 - tau_dist_roll_2) / T_eso);
        setStateVariableDerivativeValue(s, "eso_tau_dist_yaw_2", (tau_dist_yaw_1 - tau_dist_yaw_2) / T_eso);
    } else {
        setStateVariableDerivativeValue(s, "omega_filt_roll", 0.0);
        setStateVariableDerivativeValue(s, "omega_filt_yaw", 0.0);
        setStateVariableDerivativeValue(s, "omega_filt_pitch", 0.0);
        
        setStateVariableDerivativeValue(s, "eso_tau_dist_pitch", 0.0);
        setStateVariableDerivativeValue(s, "eso_tau_dist_roll", 0.0);
        setStateVariableDerivativeValue(s, "eso_tau_dist_yaw", 0.0);
        
        setStateVariableDerivativeValue(s, "eso_tau_dist_pitch_2", 0.0);
        setStateVariableDerivativeValue(s, "eso_tau_dist_roll_2", 0.0);
        setStateVariableDerivativeValue(s, "eso_tau_dist_yaw_2", 0.0);
    }
    
    // --- SE_2(3) SPATIAL OBSERVER (Tilt-Translation Disambiguation) ---
    SimTK::Vector omega_sense = vcr_data.record.omega_recon;
    
    if (omega_sense.size() == 3) {
        // 1. Simulate Otolith Attachment (Raw specific force: f = a_lin - g)
        const Body& skull = model.getBodySet().get("skull");
        SimTK::Rotation R_world_to_head = skull.getMobilizedBody().getBodyTransform(s).R().invert();
        SimTK::Vec3 g_world(0.0, -9.81, 0.0);
        SimTK::Vec3 f_oto = R_world_to_head * (vcr_data.a_lin - g_world); // Head frame
        
        // 2. Current Observer Beliefs
        double q0 = getStateVariableValue(s, "obs_q0");
        double q1 = getStateVariableValue(s, "obs_q1");
        double q2 = getStateVariableValue(s, "obs_q2");
        double q3 = getStateVariableValue(s, "obs_q3");
        
        double vx = getStateVariableValue(s, "obs_vx");
        double vy = getStateVariableValue(s, "obs_vy");
        double vz = getStateVariableValue(s, "obs_vz");
        SimTK::Vec3 v_est(vx, vy, vz);
        
        double px = getStateVariableValue(s, "obs_px");
        double py = getStateVariableValue(s, "obs_py");
        double pz = getStateVariableValue(s, "obs_pz");
        SimTK::Vec3 p_est(px, py, pz);
        
        // Normalize quaternion
        double norm_q = std::sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        if (norm_q > 1e-4) { q0 /= norm_q; q1 /= norm_q; q2 /= norm_q; q3 /= norm_q; }
        
        // 3. TILT CORRECTION (Mahony)
        // Internal Forward Model (Efference Copy) - Subtract expected acceleration from f_oto
        double K_eff_pitch = 1.33; // Pitch torque to X-axis acceleration
        SimTK::Vec3 a_expected(K_eff_pitch * getStateVariableValue(s, "eff_tau_pitch"), 0.0, 0.0);
        SimTK::Vec3 f_corrected = f_oto - a_expected;
        if (f_corrected.norm() > 1e-4) {
            f_corrected = f_corrected.normalize();
        }
        
        // Estimated UP direction (rotate [0, 1, 0] by q^-1)
        double expected_up_x = 2.0 * (q1*q2 + q0*q3);
        double expected_up_y = (q0*q0 - q1*q1 + q2*q2 - q3*q3);
        double expected_up_z = 2.0 * (q2*q3 - q0*q1);
        SimTK::Vec3 expected_up(expected_up_x, expected_up_y, expected_up_z);
        
        SimTK::Vec3 e = f_corrected % expected_up; // Cross product error
        
        // 4. ORIENTATION DERIVATIVE (PI Gyroscope Correction)
        double Kp_obs = 2.0;
        double gx = omega_sense[0] + Kp_obs * e[0];
        double gy = omega_sense[1] + Kp_obs * e[1];
        double gz = omega_sense[2] + Kp_obs * e[2];
        
        double dq0 = 0.5 * (-q1*gx - q2*gy - q3*gz);
        double dq1 = 0.5 * ( q0*gx + q2*gz - q3*gy);
        double dq2 = 0.5 * ( q0*gy - q1*gz + q3*gx);
        double dq3 = 0.5 * ( q0*gz + q1*gy - q2*gx);
        
        // 5. TRANSLATION DERIVATIVES
        // Reconstruct pure linear acceleration in WORLD frame: a_lin = R * f_oto + g_world
        // R matrix from q (World to Head):
        // Wait, q here represents Head to World or World to Head?
        // Standard Mahony q represents World to Sensor (Head).
        // Let's use SimTK::Rotation for safety.
        SimTK::Rotation R_est(SimTK::Quaternion(q0, q1, q2, q3)); 
        // Assuming R_est transforms from World to Head.
        // Then f_oto (Head frame) to world frame is ~R_est * f_oto.
        SimTK::Vec3 a_lin_world = (~R_est * f_oto) + g_world;
        
        // Biological Leaky Integrator (Velocity Storage & Egocentric tether)
        double tau_v = 15.0; // Decay in dark
        double tau_p = 5.0;
        
        SimTK::Vec3 dv = a_lin_world - (1.0 / tau_v) * v_est;
        SimTK::Vec3 dp = v_est - (1.0 / tau_p) * p_est;
        
        // Write to state derivatives
        setStateVariableDerivativeValue(s, "obs_q0", dq0);
        setStateVariableDerivativeValue(s, "obs_q1", dq1);
        setStateVariableDerivativeValue(s, "obs_q2", dq2);
        setStateVariableDerivativeValue(s, "obs_q3", dq3);
        
        setStateVariableDerivativeValue(s, "obs_vx", dv[0]);
        setStateVariableDerivativeValue(s, "obs_vy", dv[1]);
        setStateVariableDerivativeValue(s, "obs_vz", dv[2]);
        
        setStateVariableDerivativeValue(s, "obs_px", dp[0]);
        setStateVariableDerivativeValue(s, "obs_py", dp[1]);
        setStateVariableDerivativeValue(s, "obs_pz", dp[2]);
        
    } else {
        setStateVariableDerivativeValue(s, "obs_q0", 0.0);
        setStateVariableDerivativeValue(s, "obs_q1", 0.0);
        setStateVariableDerivativeValue(s, "obs_q2", 0.0);
        setStateVariableDerivativeValue(s, "obs_q3", 0.0);
        
        setStateVariableDerivativeValue(s, "obs_vx", 0.0);
        setStateVariableDerivativeValue(s, "obs_vy", 0.0);
        setStateVariableDerivativeValue(s, "obs_vz", 0.0);
        
        setStateVariableDerivativeValue(s, "obs_px", 0.0);
        setStateVariableDerivativeValue(s, "obs_py", 0.0);
        setStateVariableDerivativeValue(s, "obs_pz", 0.0);
    }
    // --- NEURAL TRANSFER FUNCTION DERIVATIVES ---
    if (vcr_data.record.omega_recon.size() == 3) {
        double est_roll = vcr_data.record.theta[0];
        double est_yaw = vcr_data.record.theta[1];
        double est_pitch = vcr_data.record.theta[2];
        
        double u_ton_pitch = (est_pitch - baseline_spatial_orientation[2]) - get_desired_pitch();
        double u_ton_roll  = (est_roll - baseline_spatial_orientation[0]) - get_desired_roll();
        double u_ton_yaw   = (est_yaw - baseline_spatial_orientation[1]) - get_desired_yaw();
        
        double u_sc_pitch = vcr_data.record.omega_recon[2] - get_desired_pitch_v();
        double u_sc_roll  = vcr_data.record.omega_recon[0] - get_desired_roll_v();
        double u_sc_yaw   = vcr_data.record.omega_recon[1] - get_desired_yaw_v();
        
        const Body& skull = model.getBodySet().get("skull");
        SimTK::Rotation R_world_to_head = skull.getMobilizedBody().getBodyTransform(s).R().invert();
        SimTK::Vec3 g_world(0.0, -9.81, 0.0);
        SimTK::Vec3 f_oto = R_world_to_head * (vcr_data.a_lin - g_world);
        
        SimTK::Rotation R_est;
        R_est.setRotationToBodyFixedXYZ(SimTK::Vec3(est_roll, est_yaw, est_pitch));
        SimTK::Vec3 a_lin_head = f_oto + (R_est * g_world);
        
        double r_head = 0.15; 
        double expected_ah_x = get_desired_pitch_a() * r_head; 
        double expected_ah_z = get_desired_roll_a() * r_head;
        
        double u_phas_pitch = a_lin_head[0] - expected_ah_x;
        double u_phas_roll  = a_lin_head[2] - expected_ah_z;
        double u_phas_yaw   = 0.0;

        double x_ton_pitch = getStateVariableValue(s, "ton_state_pitch");
        double x_ton_roll  = getStateVariableValue(s, "ton_state_roll");
        double x_ton_yaw   = getStateVariableValue(s, "ton_state_yaw");
        
        double x_sc_pitch = getStateVariableValue(s, "sc_state_pitch");
        double x_sc_roll  = getStateVariableValue(s, "sc_state_roll");
        double x_sc_yaw   = getStateVariableValue(s, "sc_state_yaw");
        
        double x_phas_pitch = getStateVariableValue(s, "phas_state_pitch");
        double x_phas_roll  = getStateVariableValue(s, "phas_state_roll");
        double x_phas_yaw   = getStateVariableValue(s, "phas_state_yaw");

        setStateVariableDerivativeValue(s, "ton_state_pitch", -get_p_ton() * x_ton_pitch + u_ton_pitch);
        setStateVariableDerivativeValue(s, "ton_state_roll", -get_p_ton() * x_ton_roll + u_ton_roll);
        setStateVariableDerivativeValue(s, "ton_state_yaw", -get_p_ton() * x_ton_yaw + u_ton_yaw);

        setStateVariableDerivativeValue(s, "sc_state_pitch", -get_p_sc() * x_sc_pitch + u_sc_pitch);
        setStateVariableDerivativeValue(s, "sc_state_roll", -get_p_sc() * x_sc_roll + u_sc_roll);
        setStateVariableDerivativeValue(s, "sc_state_yaw", -get_p_sc() * x_sc_yaw + u_sc_yaw);

        setStateVariableDerivativeValue(s, "phas_state_pitch", -get_p_phas() * x_phas_pitch + u_phas_pitch);
        setStateVariableDerivativeValue(s, "phas_state_roll", -get_p_phas() * x_phas_roll + u_phas_roll);
        setStateVariableDerivativeValue(s, "phas_state_yaw", -get_p_phas() * x_phas_yaw + u_phas_yaw);
    } else {
        setStateVariableDerivativeValue(s, "ton_state_pitch", 0.0);
        setStateVariableDerivativeValue(s, "ton_state_roll", 0.0);
        setStateVariableDerivativeValue(s, "ton_state_yaw", 0.0);
        setStateVariableDerivativeValue(s, "sc_state_pitch", 0.0);
        setStateVariableDerivativeValue(s, "sc_state_roll", 0.0);
        setStateVariableDerivativeValue(s, "sc_state_yaw", 0.0);
        setStateVariableDerivativeValue(s, "phas_state_pitch", 0.0);
        setStateVariableDerivativeValue(s, "phas_state_roll", 0.0);
        setStateVariableDerivativeValue(s, "phas_state_yaw", 0.0);
    }

    // --- NEURAL TRANSFER FUNCTION DERIVATIVES ---
    if (vcr_data.record.omega_recon.size() == 3) {
        double est_roll = vcr_data.record.theta[0];
        double est_yaw = vcr_data.record.theta[1];
        double est_pitch = vcr_data.record.theta[2];
        
        double u_ton_pitch = (est_pitch - baseline_spatial_orientation[2]) - get_desired_pitch();
        double u_ton_roll  = (est_roll - baseline_spatial_orientation[0]) - get_desired_roll();
        double u_ton_yaw   = (est_yaw - baseline_spatial_orientation[1]) - get_desired_yaw();
        
        double u_sc_pitch = vcr_data.record.omega_recon[2] - get_desired_pitch_v();
        double u_sc_roll  = vcr_data.record.omega_recon[0] - get_desired_roll_v();
        double u_sc_yaw   = vcr_data.record.omega_recon[1] - get_desired_yaw_v();
        
        const Body& skull = model.getBodySet().get("skull");
        SimTK::Rotation R_world_to_head = skull.getMobilizedBody().getBodyTransform(s).R().invert();
        SimTK::Vec3 g_world(0.0, -9.81, 0.0);
        SimTK::Vec3 f_oto = R_world_to_head * (vcr_data.a_lin - g_world);
        
        SimTK::Rotation R_est;
        R_est.setRotationToBodyFixedXYZ(SimTK::Vec3(est_roll, est_yaw, est_pitch));
        SimTK::Vec3 a_lin_head = f_oto + (R_est * g_world);
        
        double r_head = 0.15; 
        double expected_ah_x = get_desired_pitch_a() * r_head; 
        double expected_ah_z = get_desired_roll_a() * r_head;
        
        double u_phas_pitch = a_lin_head[0] - expected_ah_x;
        double u_phas_roll  = a_lin_head[2] - expected_ah_z;
        double u_phas_yaw   = 0.0;

        double x_ton_pitch = getStateVariableValue(s, "ton_state_pitch");
        double x_ton_roll  = getStateVariableValue(s, "ton_state_roll");
        double x_ton_yaw   = getStateVariableValue(s, "ton_state_yaw");
        
        double x_sc_pitch = getStateVariableValue(s, "sc_state_pitch");
        double x_sc_roll  = getStateVariableValue(s, "sc_state_roll");
        double x_sc_yaw   = getStateVariableValue(s, "sc_state_yaw");
        
        double x_phas_pitch = getStateVariableValue(s, "phas_state_pitch");
        double x_phas_roll  = getStateVariableValue(s, "phas_state_roll");
        double x_phas_yaw   = getStateVariableValue(s, "phas_state_yaw");

        setStateVariableDerivativeValue(s, "ton_state_pitch", -get_p_ton() * x_ton_pitch + u_ton_pitch);
        setStateVariableDerivativeValue(s, "ton_state_roll", -get_p_ton() * x_ton_roll + u_ton_roll);
        setStateVariableDerivativeValue(s, "ton_state_yaw", -get_p_ton() * x_ton_yaw + u_ton_yaw);

        setStateVariableDerivativeValue(s, "sc_state_pitch", -get_p_sc() * x_sc_pitch + u_sc_pitch);
        setStateVariableDerivativeValue(s, "sc_state_roll", -get_p_sc() * x_sc_roll + u_sc_roll);
        setStateVariableDerivativeValue(s, "sc_state_yaw", -get_p_sc() * x_sc_yaw + u_sc_yaw);

        setStateVariableDerivativeValue(s, "phas_state_pitch", -get_p_phas() * x_phas_pitch + u_phas_pitch);
        setStateVariableDerivativeValue(s, "phas_state_roll", -get_p_phas() * x_phas_roll + u_phas_roll);
        setStateVariableDerivativeValue(s, "phas_state_yaw", -get_p_phas() * x_phas_yaw + u_phas_yaw);
    } else {
        setStateVariableDerivativeValue(s, "ton_state_pitch", 0.0);
        setStateVariableDerivativeValue(s, "ton_state_roll", 0.0);
        setStateVariableDerivativeValue(s, "ton_state_yaw", 0.0);
        setStateVariableDerivativeValue(s, "sc_state_pitch", 0.0);
        setStateVariableDerivativeValue(s, "sc_state_roll", 0.0);
        setStateVariableDerivativeValue(s, "sc_state_yaw", 0.0);
        setStateVariableDerivativeValue(s, "phas_state_pitch", 0.0);
        setStateVariableDerivativeValue(s, "phas_state_roll", 0.0);
        setStateVariableDerivativeValue(s, "phas_state_yaw", 0.0);
    }

    double p_ccr = get_p_ccr();
    DelayedData ccr_data = getDelayedData(s.getTime(), 0.018);
    HistRecord ccr_record = ccr_data.record;
    const auto& muscles = getModel().getMuscles();
    
    // Default to 0 for all
    for (int i = 0; i < muscles.getSize(); ++i) {
        setStateVariableDerivativeValue(s, "ccr_state_" + muscles.get(i).getName(), 0.0);
    }
    
    if (ccr_record.Ia.size() == active_muscle_indices.size() && active_muscle_indices.size() > 0) {
        for (size_t i = 0; i < active_muscle_indices.size(); ++i) {
            const Muscle& m = muscles.get(active_muscle_indices[i]);
            std::string state_name = "ccr_state_" + m.getName();
            double x_ccr = getStateVariableValue(s, state_name);
            double Ia_delayed = ccr_record.Ia[i];
            setStateVariableDerivativeValue(s, state_name, -p_ccr * x_ccr + Ia_delayed);
        }
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
                    if (primary > 500.0) primary = 500.0; // Biological max firing rate (~500 Hz) to avoid torque spikes
                    
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

    // --- DESCENDING TRACKS (Voluntary) vs ASCENDING TRACKS (Reflex) ---
    Vector tau_vcr(3, 0.0); // Restorative Vestibular Reflexes
    Vector tau_vol(3, 0.0); // Descending Voluntary Task Tracking
    Vector tau_eso(3, 0.0); // Cerebellar Disturbance Rejection
    
    // Extract Estimated Angles from SE_2(3) Observer
    double q0 = getStateVariableValue(s, "obs_q0");
    double q1 = getStateVariableValue(s, "obs_q1");
    double q2 = getStateVariableValue(s, "obs_q2");
    double q3 = getStateVariableValue(s, "obs_q3");
    
    double v_est_x = getStateVariableValue(s, "obs_vx");
    double v_est_y = getStateVariableValue(s, "obs_vy");
    double v_est_z = getStateVariableValue(s, "obs_vz");
    
    double norm_q = std::sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (norm_q > 1e-4) { q0 /= norm_q; q1 /= norm_q; q2 /= norm_q; q3 /= norm_q; }
    
    // Euler angles (Roll=X, Yaw=Y, Pitch=Z)
    // Standard aerospace sequence for quaternion extraction
    double estimated_roll = std::atan2(2.0*(q0*q1 + q2*q3), 1.0 - 2.0*(q1*q1 + q2*q2));
    double estimated_yaw = std::asin(std::clamp(2.0*(q0*q2 - q3*q1), -1.0, 1.0));
    double estimated_pitch = std::atan2(2.0*(q0*q3 + q1*q2), 1.0 - 2.0*(q2*q2 + q3*q3));
    
    // 1. Vestibular Reflex with Efference Copy Cancellation (expected vestibular firing from voluntary action)
    if (vcr_record.omega_recon.size() == 3) {
        // Neural Transfer Functions Output Application
        // y = K*(z - p)*x + K*u
        
        double est_roll = vcr_record.theta[0];
        double est_yaw = vcr_record.theta[1];
        double est_pitch = vcr_record.theta[2];
        
        double u_ton_pitch = (est_pitch - baseline_spatial_orientation[2]) - get_desired_pitch();
        double u_ton_roll  = (est_roll - baseline_spatial_orientation[0]) - get_desired_roll();
        double u_ton_yaw   = (est_yaw - baseline_spatial_orientation[1]) - get_desired_yaw();
        
        double u_sc_pitch = vcr_record.omega_recon[2] - get_desired_pitch_v();
        double u_sc_roll  = vcr_record.omega_recon[0] - get_desired_roll_v();
        double u_sc_yaw   = vcr_record.omega_recon[1] - get_desired_yaw_v();
        
        const Body& skull = model.getBodySet().get("skull");
        SimTK::Rotation R_world_to_head = skull.getMobilizedBody().getBodyTransform(s).R().invert();
        SimTK::Vec3 g_world(0.0, -9.81, 0.0);
        SimTK::Vec3 f_oto = R_world_to_head * (vcr_data.a_lin - g_world); 
        
        SimTK::Rotation R_est;
        R_est.setRotationToBodyFixedXYZ(SimTK::Vec3(est_roll, est_yaw, est_pitch));
        SimTK::Vec3 a_lin_head = f_oto + (R_est * g_world);
        
        double r_head = 0.15; 
        double expected_ah_x = get_desired_pitch_a() * r_head; 
        double expected_ah_z = get_desired_roll_a() * r_head;
        
        double u_phas_pitch = a_lin_head[0] - expected_ah_x;
        double u_phas_roll  = a_lin_head[2] - expected_ah_z;
        double u_phas_yaw   = 0.0;
        
        double x_ton_pitch = getStateVariableValue(s, "ton_state_pitch");
        double x_ton_roll  = getStateVariableValue(s, "ton_state_roll");
        double x_ton_yaw   = getStateVariableValue(s, "ton_state_yaw");
        
        double x_sc_pitch = getStateVariableValue(s, "sc_state_pitch");
        double x_sc_roll  = getStateVariableValue(s, "sc_state_roll");
        double x_sc_yaw   = getStateVariableValue(s, "sc_state_yaw");
        
        double x_phas_pitch = getStateVariableValue(s, "phas_state_pitch");
        double x_phas_roll  = getStateVariableValue(s, "phas_state_roll");
        double x_phas_yaw   = getStateVariableValue(s, "phas_state_yaw");
        
        double y_ton_pitch = get_K_ton() * get_p_ton() * x_ton_pitch;
        double y_ton_roll  = get_K_ton() * get_p_ton() * x_ton_roll;
        double y_ton_yaw   = get_K_ton() * get_p_ton() * x_ton_yaw;
        
        double y_sc_pitch = get_K_sc() * get_p_sc() * x_sc_pitch;
        double y_sc_roll  = get_K_sc() * get_p_sc() * x_sc_roll;
        double y_sc_yaw   = get_K_sc() * get_p_sc() * x_sc_yaw;
        
        double y_phas_pitch = get_K_phas() * get_p_phas() * x_phas_pitch;
        double y_phas_roll  = get_K_phas() * get_p_phas() * x_phas_roll;
        double y_phas_yaw   = get_K_phas() * get_p_phas() * x_phas_yaw;
        
        tau_vcr[0] -= y_ton_pitch + y_sc_pitch + y_phas_pitch;
        tau_vcr[1] -= y_ton_roll  + y_sc_roll  + y_phas_roll;
        tau_vcr[2] -= y_ton_yaw   + y_sc_yaw   + y_phas_yaw;
    }
    
    // 2. Voluntary Postural PID Drive (compensates for gravity droop / Na_post equivalent)
    double pitch_int = getStateVariableValue(s, "pitch_error_integral");
    double roll_int = getStateVariableValue(s, "roll_error_integral");
    double yaw_int = getStateVariableValue(s, "yaw_error_integral");
    
    // Feedforward inertia approximation (T = I*alpha)
    double I_head_pitch = 0.05; 
    double I_head_roll = 0.05;
    double I_head_yaw = 0.05;
    
    // Pitch uses the dynamically tuned omega_n and zeta.
    double kp_pitch = I_head_pitch * get_omega_n_pitch() * get_omega_n_pitch();
    double kd_pitch = I_head_pitch * 2.0 * get_zeta_pitch() * get_omega_n_pitch();
    tau_vol[0] = kp_pitch * (get_desired_pitch() - head_pitch) 
               + kd_pitch * (get_desired_pitch_v() - omega[2]) 
               + I_head_pitch * get_desired_pitch_a();
               
    double kp_roll = I_head_roll * get_omega_n_roll() * get_omega_n_roll();
    double kd_roll = I_head_roll * 2.0 * get_zeta_roll() * get_omega_n_roll();
    tau_vol[1] = kp_roll * (get_desired_roll() - head_roll) 
               + kd_roll * (get_desired_roll_v() - omega[0]) 
               + I_head_roll * get_desired_roll_a();
               
    double kp_yaw = I_head_yaw * get_omega_n_yaw() * get_omega_n_yaw();
    double kd_yaw = I_head_yaw * 2.0 * get_zeta_yaw() * get_omega_n_yaw();
    tau_vol[2] = kp_yaw * (get_desired_yaw() - head_yaw) 
               + kd_yaw * (get_desired_yaw_v() - omega[1]) 
               + I_head_yaw * get_desired_yaw_a();
    
    // 3. Inject Cerebellar Disturbance Rejection (ESO)
    // We use the 2nd stage state for a 2nd-order critically damped low-pass response (-40 dB/dec)
    double eso_pitch = getStateVariableValue(s, "eso_tau_dist_pitch_2");
    double eso_roll = getStateVariableValue(s, "eso_tau_dist_roll_2");
    double eso_yaw = getStateVariableValue(s, "eso_tau_dist_yaw_2");
    
    double K_eso = 0.8; // Re-enabled! 2nd-order filter prevents high-frequency noise amplification
    tau_eso[0] = -K_eso * eso_pitch;
    tau_eso[1] = -K_eso * eso_roll;
    tau_eso[2] = -K_eso * eso_yaw;
    
    // --- TOTAL MOTOR CORTEX DRIVE TO SPINAL CORD ---
    Vector tau_des(3, 0.0);
    tau_des[0] = tau_vol[0] + tau_vcr[0] + tau_eso[0];
    tau_des[1] = tau_vol[1] + tau_vcr[1] + tau_eso[1];
    tau_des[2] = tau_vol[2] + tau_vcr[2] + tau_eso[2];
    
    tau_des[0] = std::clamp(tau_des[0], -150.0, 150.0);
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
            std::string state_name = "ccr_state_" + m.getName();
            double x_ccr = getStateVariableValue(s, state_name);
            double Ia_delayed = ccr_record.Ia[i];
            double Ia_filtered = get_p_ccr() * x_ccr;
            
            // In case filter makes it negative
            double Ia_firing = std::max(0.0, Ia_filtered);
            
            // Synergy-Based Reciprocal Inhibition (Descending Presynaptic Inhibition)
            // We use a C1-continuous smoothstep to prevent integrator chattering!
            // If u_alpha is near the 0.01 lower bound, we apply K_recip_inhib.
            // By u_alpha = 0.05, the stretch reflex is fully active (1.0).
            double K_recip = get_K_recip_inhib();
            double inhibition_factor = 1.0;
            double u_min = 0.01;
            double u_max = 0.05;
            
            if (u_alpha[i] <= u_min) {
                inhibition_factor = K_recip;
            } else if (u_alpha[i] >= u_max) {
                inhibition_factor = 1.0;
            } else {
                double t = (u_alpha[i] - u_min) / (u_max - u_min);
                double smooth_t = t * t * (3.0 - 2.0 * t);
                inhibition_factor = K_recip + smooth_t * (1.0 - K_recip);
            }
            
            // We scale the firing rate (0-500) into excitation (0-1) using Kp_proprioception
            excitation += get_Kp_proprioception() * (Ia_firing / 100.0) * inhibition_factor; 
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

