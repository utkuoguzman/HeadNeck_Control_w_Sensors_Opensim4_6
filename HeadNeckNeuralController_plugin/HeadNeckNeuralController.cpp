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
    constructProperty_Kp_proprioception(2.5030); 
    constructProperty_z_ccr(500.0);
    constructProperty_p_ccr(24.4852);
    constructProperty_K_recip_inhib(0.0); // 0.0 means complete reciprocal inhibition (no antagonist reflex)
    constructProperty_Kg_proprioception(0.0);
    constructProperty_K_vestibular(2.0);
    constructProperty_K_otolith(4.0);
    constructProperty_desired_yaw(0.0);
    constructProperty_desired_pitch(0.0);
    constructProperty_desired_roll(0.0);
    constructProperty_K_ton(74.3907);
    constructProperty_p_ton(17.7531);
    constructProperty_K_sc(49.3723);
    constructProperty_z_sc(500.0);
    constructProperty_p_sc(37.8562);
    constructProperty_K_phas(4.9396);
    constructProperty_p_phas(63.8079);
    constructProperty_k_p(2.5030);
    constructProperty_k_v(0.13);
    constructProperty_K_gamma_dyn(3.7051);
    constructProperty_K_gamma_stat(2.6738);
    constructProperty_K_gamma(1.0);
    constructProperty_Wp_base_pitch(400.0);
    constructProperty_Wv_base_pitch(40.0);
    constructProperty_k_vigor(0.0);
    constructProperty_Wp_base_roll(400.0);
    constructProperty_Wv_base_roll(40.0);
    
    constructProperty_Wp_base_yaw(400.0);
    constructProperty_Wv_base_yaw(40.0);
    constructProperty_Ki_task(6.54);
    constructProperty_trajectory_tau(0.1);
    constructProperty_use_dynamic_jacobian(true);
    constructProperty_K_eso(0.0);
    constructProperty_delay_voluntary(0.040);
    constructProperty_fc_voluntary(2.0);
    
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
    
    addStateVariable("traj_pos_pitch", SimTK::Stage::Velocity);
    addStateVariable("traj_vel_pitch", SimTK::Stage::Velocity);
    addStateVariable("traj_pos_roll", SimTK::Stage::Velocity);
    addStateVariable("traj_vel_roll", SimTK::Stage::Velocity);
    addStateVariable("traj_pos_yaw", SimTK::Stage::Velocity);
    addStateVariable("traj_vel_yaw", SimTK::Stage::Velocity);
    addStateVariable("pitch_error_integral", SimTK::Stage::Velocity);
    addStateVariable("roll_error_integral", SimTK::Stage::Velocity);
    addStateVariable("yaw_error_integral", SimTK::Stage::Velocity);
    addStateVariable("vol_filt_pitch", SimTK::Stage::Velocity);
    addStateVariable("vol_filt_roll", SimTK::Stage::Velocity);
    addStateVariable("vol_filt_yaw", SimTK::Stage::Velocity);
    
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

double HeadNeckNeuralController::getMahonyPitch(const SimTK::State& s) const {
    double q0 = getStateVariableValue(s, "obs_q0");
    double q1 = getStateVariableValue(s, "obs_q1");
    double q2 = getStateVariableValue(s, "obs_q2");
    double q3 = getStateVariableValue(s, "obs_q3");
    double norm_q = std::sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (norm_q > 1e-4) { q0 /= norm_q; q1 /= norm_q; q2 /= norm_q; q3 /= norm_q; }
    return std::atan2(2.0*(q0*q3 + q1*q2), 1.0 - 2.0*(q2*q2 + q3*q3));
}

double HeadNeckNeuralController::getMahonyRoll(const SimTK::State& s) const {
    double q0 = getStateVariableValue(s, "obs_q0");
    double q1 = getStateVariableValue(s, "obs_q1");
    double q2 = getStateVariableValue(s, "obs_q2");
    double q3 = getStateVariableValue(s, "obs_q3");
    double norm_q = std::sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (norm_q > 1e-4) { q0 /= norm_q; q1 /= norm_q; q2 /= norm_q; q3 /= norm_q; }
    return std::atan2(2.0*(q0*q1 + q2*q3), 1.0 - 2.0*(q1*q1 + q2*q2));
}

double HeadNeckNeuralController::getMahonyYaw(const SimTK::State& s) const {
    double q0 = getStateVariableValue(s, "obs_q0");
    double q1 = getStateVariableValue(s, "obs_q1");
    double q2 = getStateVariableValue(s, "obs_q2");
    double q3 = getStateVariableValue(s, "obs_q3");
    double norm_q = std::sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (norm_q > 1e-4) { q0 /= norm_q; q1 /= norm_q; q2 /= norm_q; q3 /= norm_q; }
    return std::asin(std::clamp(2.0*(q0*q2 - q3*q1), -1.0, 1.0));
}

double HeadNeckNeuralController::getMahonyPitchSpeed(const SimTK::State& s) const {
    return mahony_speed[0];
}

double HeadNeckNeuralController::getMahonyRollSpeed(const SimTK::State& s) const {
    return mahony_speed[1];
}

double HeadNeckNeuralController::getMahonyYawSpeed(const SimTK::State& s) const {
    return mahony_speed[2];
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
    est_neck_rel_angle = SimTK::Vec3(0.0);
    est_neck_rel_speed = SimTK::Vec3(0.0);
    mahony_angle = SimTK::Vec3(0.0);
    mahony_speed = SimTK::Vec3(0.0);
    
    setStateVariableValue(s, "traj_pos_pitch", 0.0);
    setStateVariableValue(s, "traj_vel_pitch", 0.0);
    setStateVariableValue(s, "traj_pos_roll", 0.0);
    setStateVariableValue(s, "traj_vel_roll", 0.0);
    setStateVariableValue(s, "traj_pos_yaw", 0.0);
    setStateVariableValue(s, "traj_vel_yaw", 0.0);
    setStateVariableValue(s, "vol_filt_pitch", 0.0);
    setStateVariableValue(s, "vol_filt_roll", 0.0);
    setStateVariableValue(s, "vol_filt_yaw", 0.0);
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
    
    updateDecoderMatrix();
    history.clear();
    is_initialized = true;
    last_jacobian_update_time = s.getTime();
}

void HeadNeckNeuralController::updateDecoderMatrix() const {
    int num_muscles = active_muscle_indices.size();
    if (num_muscles == 0) return;
    
    // A = J * J^T (3 x 3)
    SimTK::Matrix A(3, 3);
    A.setToZero();
    for (int i = 0; i < num_muscles; ++i) {
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                A(r, c) += J_moment_arms(r, i) * J_moment_arms(c, i);
            }
        }
    }
    
    // Tikhonov regularization (1e-12) to ensure strict positive definiteness
    A(0,0) += 1e-12;
    A(1,1) += 1e-12;
    A(2,2) += 1e-12;
    
    SimTK::Matrix A_inv = A.invert();
    
    // K_decode = - (J J^T)^{-1} J  (3 x num_muscles)
    K_proprio_decode.resize(3, num_muscles);
    for (int r = 0; r < 3; ++r) {
        for (int i = 0; i < num_muscles; ++i) {
            double sum = 0.0;
            for (int k = 0; k < 3; ++k) {
                sum += A_inv(r, k) * J_moment_arms(k, i);
            }
            K_proprio_decode(r, i) = -sum;
        }
    }
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
    if (it_prev->second.theta_trunk.size() == 3) {
        res.theta_trunk = it_prev->second.theta_trunk * (1 - w) + it->second.theta_trunk * w;
    }
    if (it_prev->second.omega_trunk.size() == 3) {
        res.omega_trunk = it_prev->second.omega_trunk * (1 - w) + it->second.omega_trunk * w;
    }
    res.v_lin = it_prev->second.v_lin * (1 - w) + it->second.v_lin * w;
    res.est_neck_rel_angle = it_prev->second.est_neck_rel_angle * (1 - w) + it->second.est_neck_rel_angle * w;
    res.est_neck_rel_speed = it_prev->second.est_neck_rel_speed * (1 - w) + it->second.est_neck_rel_speed * w;
    res.neck_rel_q = it_prev->second.neck_rel_q * (1 - w) + it->second.neck_rel_q * w;
    res.neck_rel_u = it_prev->second.neck_rel_u * (1 - w) + it->second.neck_rel_u * w;
    
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
    double tau_traj = get_trajectory_tau();
    if (tau_traj < 1e-4) tau_traj = 1e-4;
    double traj_pos_pitch = getStateVariableValue(s, "traj_pos_pitch");
    double traj_vel_pitch = getStateVariableValue(s, "traj_vel_pitch");
    double traj_acc_pitch = (1.0 / (tau_traj * tau_traj)) * (get_desired_pitch() - traj_pos_pitch) - (2.0 / tau_traj) * traj_vel_pitch;
    
    double traj_pos_roll = getStateVariableValue(s, "traj_pos_roll");
    double traj_vel_roll = getStateVariableValue(s, "traj_vel_roll");
    double traj_acc_roll = (1.0 / (tau_traj * tau_traj)) * (get_desired_roll() - traj_pos_roll) - (2.0 / tau_traj) * traj_vel_roll;

    double traj_pos_yaw = getStateVariableValue(s, "traj_pos_yaw");
    double traj_vel_yaw = getStateVariableValue(s, "traj_vel_yaw");
    double traj_acc_yaw = (1.0 / (tau_traj * tau_traj)) * (get_desired_yaw() - traj_pos_yaw) - (2.0 / tau_traj) * traj_vel_yaw;

    setStateVariableDerivativeValue(s, "traj_pos_pitch", traj_vel_pitch);
    setStateVariableDerivativeValue(s, "traj_vel_pitch", traj_acc_pitch);
    setStateVariableDerivativeValue(s, "traj_pos_roll", traj_vel_roll);
    setStateVariableDerivativeValue(s, "traj_vel_roll", traj_acc_roll);
    setStateVariableDerivativeValue(s, "traj_pos_yaw", traj_vel_yaw);
    setStateVariableDerivativeValue(s, "traj_vel_yaw", traj_acc_yaw);

    const Model& model = getModel();
    
    double u_p1 = model.getCoordinateSet().contains("pitch1") ? model.getCoordinateSet().get("pitch1").getSpeedValue(s) : 0;
    double u_p2 = model.getCoordinateSet().contains("pitch2") ? model.getCoordinateSet().get("pitch2").getSpeedValue(s) : 0;
    double u_r1 = model.getCoordinateSet().contains("roll1") ? model.getCoordinateSet().get("roll1").getSpeedValue(s) : 0;
    double u_r2 = model.getCoordinateSet().contains("roll2") ? model.getCoordinateSet().get("roll2").getSpeedValue(s) : 0;
    double u_y1 = model.getCoordinateSet().contains("yaw1") ? model.getCoordinateSet().get("yaw1").getSpeedValue(s) : 0;
    double u_y2 = model.getCoordinateSet().contains("yaw2") ? model.getCoordinateSet().get("yaw2").getSpeedValue(s) : 0;

    double u_gnd_p = model.getCoordinateSet().contains("gndpitch") ? model.getCoordinateSet().get("gndpitch").getSpeedValue(s) : 0;
    double u_gnd_r = model.getCoordinateSet().contains("gndroll") ? model.getCoordinateSet().get("gndroll").getSpeedValue(s) : 0;
    double u_gnd_y = model.getCoordinateSet().contains("gndyaw") ? model.getCoordinateSet().get("gndyaw").getSpeedValue(s) : 0;
    if (std::abs(u_gnd_p) < 1e-12) u_gnd_p = 0.0;
    if (std::abs(u_gnd_r) < 1e-12) u_gnd_r = 0.0;
    if (std::abs(u_gnd_y) < 1e-12) u_gnd_y = 0.0;

    double q_gnd_p = model.getCoordinateSet().contains("gndpitch") ? model.getCoordinateSet().get("gndpitch").getValue(s) : 0;
    double q_gnd_r = model.getCoordinateSet().contains("gndroll") ? model.getCoordinateSet().get("gndroll").getValue(s) : 0;
    double q_gnd_y = model.getCoordinateSet().contains("gndyaw") ? model.getCoordinateSet().get("gndyaw").getValue(s) : 0;
    if (std::abs(q_gnd_p) < 1e-12) q_gnd_p = 0.0;
    if (std::abs(q_gnd_r) < 1e-12) q_gnd_r = 0.0;
    if (std::abs(q_gnd_y) < 1e-12) q_gnd_y = 0.0;

    double head_roll_dot_global  = (u_r1 + u_r2) + u_gnd_r;
    double head_yaw_dot_global   = (u_y1 + u_y2) + u_gnd_y;
    double head_pitch_dot_global = (u_p1 + u_p2) + u_gnd_p;
    
    SimTK::Vector omega(3);
    omega[0] = head_roll_dot_global; // roll
    omega[1] = head_yaw_dot_global;  // yaw
    omega[2] = head_pitch_dot_global;// pitch
    
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
    
    // PID Integrator derivatives (error = desired - actual neck relative angle)
    double q_p1 = model.getCoordinateSet().contains("pitch1") ? model.getCoordinateSet().get("pitch1").getValue(s) : 0;
    double q_p2 = model.getCoordinateSet().contains("pitch2") ? model.getCoordinateSet().get("pitch2").getValue(s) : 0;
    double q_r1 = model.getCoordinateSet().contains("roll1") ? model.getCoordinateSet().get("roll1").getValue(s) : 0;
    double q_r2 = model.getCoordinateSet().contains("roll2") ? model.getCoordinateSet().get("roll2").getValue(s) : 0;
    double q_y1 = model.getCoordinateSet().contains("yaw1") ? model.getCoordinateSet().get("yaw1").getValue(s) : 0;
    double q_y2 = model.getCoordinateSet().contains("yaw2") ? model.getCoordinateSet().get("yaw2").getValue(s) : 0;
    
    double head_pitch_global = (q_p1 + q_p2) + q_gnd_p;
    double head_roll_global  = (q_r1 + q_r2) + q_gnd_r;
    double head_yaw_global   = (q_y1 + q_y2) + q_gnd_y;

    double head_rel_pitch = head_pitch_global - q_gnd_p;
    double head_rel_roll  = head_roll_global  - q_gnd_r;
    double head_rel_yaw   = head_yaw_global   - q_gnd_y;
    
    double p_err = traj_pos_pitch - head_rel_pitch;
    double p_int = getStateVariableValue(s, "pitch_error_integral");
    if (p_int > 0.6 && p_err > 0) p_err = 0.0;
    if (p_int < -0.6 && p_err < 0) p_err = 0.0;
    setStateVariableDerivativeValue(s, "pitch_error_integral", p_err);

    double r_err = traj_pos_roll - head_rel_roll;
    double r_int = getStateVariableValue(s, "roll_error_integral");
    if (r_int > 0.4 && r_err > 0) r_err = 0.0;
    if (r_int < -0.4 && r_err < 0) r_err = 0.0;
    setStateVariableDerivativeValue(s, "roll_error_integral", r_err);

    double y_err = traj_pos_yaw - head_rel_yaw;
    double y_int = getStateVariableValue(s, "yaw_error_integral");
    if (y_int > 0.4 && y_err > 0) y_err = 0.0;
    if (y_int < -0.4 && y_err < 0) y_err = 0.0;
    setStateVariableDerivativeValue(s, "yaw_error_integral", y_err);

    // --- CORTICAL / VOLUNTARY LOW-PASS FILTER DYNAMICS (TRANSCORTICAL DELAY + 2 Hz ROLL-OFF) ---
    DelayedData vol_data = getDelayedData(s.getTime(), get_delay_voluntary());
    double head_rel_pitch_dot = u_p1 + u_p2;
    double head_rel_roll_dot  = u_r1 + u_r2;
    double head_rel_yaw_dot   = u_y1 + u_y2;

    double vol_rel_pitch = (!history.empty()) ? vol_data.record.neck_rel_q[0] : head_rel_pitch;
    double vol_rel_roll  = (!history.empty()) ? vol_data.record.neck_rel_q[1] : head_rel_roll;
    double vol_rel_yaw   = (!history.empty()) ? vol_data.record.neck_rel_q[2] : head_rel_yaw;

    double vol_rel_pitch_dot = (!history.empty()) ? vol_data.record.neck_rel_u[0] : head_rel_pitch_dot;
    double vol_rel_roll_dot  = (!history.empty()) ? vol_data.record.neck_rel_u[1] : head_rel_roll_dot;
    double vol_rel_yaw_dot   = (!history.empty()) ? vol_data.record.neck_rel_u[2] : head_rel_yaw_dot;

    double I_head_pitch = 0.05; 
    double I_head_roll = 0.05;
    double I_head_yaw = 0.05;

    double vigor_pitch = 1.0 + get_k_vigor() * std::abs(traj_vel_pitch);
    double kp_pitch = std::sqrt(get_Wp_base_pitch() * vigor_pitch);
    double kd_pitch = std::sqrt(get_Wv_base_pitch() * vigor_pitch + 2.0 * kp_pitch);
    double tau_vol_raw_pitch = kp_pitch * (traj_pos_pitch - vol_rel_pitch) 
                             + kd_pitch * (traj_vel_pitch - vol_rel_pitch_dot) 
                             + get_Ki_task() * p_int 
                             + I_head_pitch * traj_acc_pitch;

    double vigor_roll = 1.0 + get_k_vigor() * std::abs(traj_vel_roll);
    double kp_roll = std::sqrt(get_Wp_base_roll() * vigor_roll);
    double kd_roll = std::sqrt(get_Wv_base_roll() * vigor_roll + 2.0 * kp_roll);
    double tau_vol_raw_roll = kp_roll * (traj_pos_roll - vol_rel_roll) 
                            + kd_roll * (traj_vel_roll - vol_rel_roll_dot) 
                            + get_Ki_task() * r_int 
                            + I_head_roll * traj_acc_roll;

    double vigor_yaw = 1.0 + get_k_vigor() * std::abs(traj_vel_yaw);
    double kp_yaw = std::sqrt(get_Wp_base_yaw() * vigor_yaw);
    double kd_yaw = std::sqrt(get_Wv_base_yaw() * vigor_yaw + 2.0 * kp_yaw);
    double tau_vol_raw_yaw = kp_yaw * (traj_pos_yaw - vol_rel_yaw) 
                           + kd_yaw * (traj_vel_yaw - vol_rel_yaw_dot) 
                           + get_Ki_task() * y_int 
                           + I_head_yaw * traj_acc_yaw;

    double fc_vol = get_fc_voluntary();
    if (fc_vol < 0.1) fc_vol = 0.1;
    double omega_vol = 2.0 * SimTK::Pi * fc_vol;

    double vol_filt_pitch = getStateVariableValue(s, "vol_filt_pitch");
    double vol_filt_roll  = getStateVariableValue(s, "vol_filt_roll");
    double vol_filt_yaw   = getStateVariableValue(s, "vol_filt_yaw");

    setStateVariableDerivativeValue(s, "vol_filt_pitch", omega_vol * (tau_vol_raw_pitch - vol_filt_pitch));
    setStateVariableDerivativeValue(s, "vol_filt_roll",  omega_vol * (tau_vol_raw_roll  - vol_filt_roll));
    setStateVariableDerivativeValue(s, "vol_filt_yaw",   omega_vol * (tau_vol_raw_yaw   - vol_filt_yaw));
    
    DelayedData vcr_data = getDelayedData(s.getTime(), 0.013);
    
    // --- SE_2(3) SPATIAL OBSERVER (Tilt-Translation Disambiguation) ---
    // 1. Vestibular Sensor Readings (Skull angular velocity and Otolith specific force)
    const Body& skull = model.getBodySet().get("skull");
    SimTK::Vec3 w_world = skull.getVelocityInGround(s)[0];
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
    
    SimTK::Rotation R_est(SimTK::Quaternion(q0, q1, q2, q3)); 
    
    // 3. TILT CORRECTION (Mahony Otolith + Proprioceptive Multisensory Fusion)
    // Transform angular velocity from Ground frame into Head frame
    SimTK::Vec3 w_head = (~R_est) * w_world;
    
    // Measured UP direction in Head frame from otolith
    SimTK::Vec3 u_meas;
    if (f_oto.norm() > 1e-4) {
        u_meas = f_oto.normalize();
    } else {
        u_meas = R_world_to_head * SimTK::Vec3(0.0, 1.0, 0.0);
    }
    
    // Estimated UP direction in Head frame: rotate [0, 1, 0] by q^-1 (~R_est)
    SimTK::Vec3 expected_up = (~R_est) * SimTK::Vec3(0.0, 1.0, 0.0);
    
    // Otolith gravity error cross product in Head frame (Pitch & Roll)
    SimTK::Vec3 e_oto = u_meas % expected_up;
    
    // 4. PROPRIOCEPTIVE INNOVATION (Head-on-Trunk Spindle Feedback)
    double est_pitch = std::atan2(2.0*(q0*q3 + q1*q2), 1.0 - 2.0*(q2*q2 + q3*q3));
    double est_roll  = std::atan2(2.0*(q0*q1 + q2*q3), 1.0 - 2.0*(q1*q1 + q2*q2));
    double est_yaw   = std::asin(std::clamp(2.0*(q0*q2 - q3*q1), -1.0, 1.0));
    
    // Proprioceptive global belief (trunk angle + relative neck angle from 72 muscles)
    double prop_gnd_pitch = q_gnd_p + est_neck_rel_angle[0];
    double prop_gnd_roll  = q_gnd_r + est_neck_rel_angle[1];
    double prop_gnd_yaw   = q_gnd_y + est_neck_rel_angle[2];
    
    // Error vector in World frame [Roll=0, Yaw=1, Pitch=2]
    SimTK::Vec3 e_prop_world(prop_gnd_roll - est_roll, prop_gnd_yaw - est_yaw, prop_gnd_pitch - est_pitch);
    // Rotate into Head frame
    SimTK::Vec3 e_prop_head = (~R_est) * e_prop_world;
    
    // Multisensory gains:
    // Otolith gravity tether (Kp_oto ~ 0.05 rad/s)
    double Kp_oto = 0.05;
    // Proprioceptive gains: strong in Yaw to prevent unobservable drift, moderate in Pitch/Roll
    double K_prop_roll = 0.2;
    double K_prop_yaw = 2.0;
    double K_prop_pitch = 0.2;
    SimTK::Vec3 e_prop_gain(K_prop_roll * e_prop_head[0], K_prop_yaw * e_prop_head[1], K_prop_pitch * e_prop_head[2]);
    
    // Fused angular velocity for quaternion kinematics
    SimTK::Vec3 w_fused = w_head + Kp_oto * e_oto + e_prop_gain;
    
    double gx = w_fused[0];
    double gy = w_fused[1];
    double gz = w_fused[2];
    
    double dq0 = 0.5 * (-q1*gx - q2*gy - q3*gz);
    double dq1 = 0.5 * ( q0*gx + q2*gz - q3*gy);
    double dq2 = 0.5 * ( q0*gy - q1*gz + q3*gx);
    double dq3 = 0.5 * ( q0*gz + q1*gy - q2*gx);
        
        // 5. TRANSLATION DERIVATIVES
        // Reconstruct pure linear acceleration in WORLD frame: a_lin = R_est * f_oto + g_world
        SimTK::Vec3 a_lin_world = (R_est * f_oto) + g_world;
        
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
    
    // --- NEURAL TRANSFER FUNCTION DERIVATIVES ---
    if (vcr_data.record.omega_recon.size() == 3) {
        double est_roll = vcr_data.record.theta[0];
        double est_yaw = vcr_data.record.theta[1];
        double est_pitch = vcr_data.record.theta[2];
        
        double u_ton_pitch = (est_pitch - baseline_spatial_orientation[2]) - traj_pos_pitch;
        double u_ton_roll  = (est_roll - baseline_spatial_orientation[0]) - traj_pos_roll;
        double u_ton_yaw   = (est_yaw - baseline_spatial_orientation[1]) - traj_pos_yaw;
        
        double u_sc_pitch = vcr_data.record.omega_recon[2] - traj_vel_pitch;
        double u_sc_roll  = vcr_data.record.omega_recon[0] - traj_vel_roll;
        double u_sc_yaw   = vcr_data.record.omega_recon[1] - traj_vel_yaw;
        
        const Body& skull = model.getBodySet().get("skull");
        SimTK::Rotation R_world_to_head = skull.getMobilizedBody().getBodyTransform(s).R().invert();
        SimTK::Vec3 g_world(0.0, -9.81, 0.0);
        SimTK::Vec3 f_oto = R_world_to_head * (vcr_data.a_lin - g_world);
        
        SimTK::Rotation R_est;
        R_est.setRotationToBodyFixedXYZ(SimTK::Vec3(est_roll, est_yaw, est_pitch));
        SimTK::Vec3 a_lin_head = f_oto + (R_est * g_world);
        
        double r_head = 0.15; 
        double expected_ah_x = traj_acc_pitch * r_head; 
        double expected_ah_z = traj_acc_roll * r_head;
        
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
    double tau_traj = get_trajectory_tau();
    if (tau_traj < 1e-4) tau_traj = 1e-4;
    double traj_pos_pitch = getStateVariableValue(s, "traj_pos_pitch");
    double traj_vel_pitch = getStateVariableValue(s, "traj_vel_pitch");
    double traj_acc_pitch = (1.0 / (tau_traj * tau_traj)) * (get_desired_pitch() - traj_pos_pitch) - (2.0 / tau_traj) * traj_vel_pitch;
    
    double traj_pos_roll = getStateVariableValue(s, "traj_pos_roll");
    double traj_vel_roll = getStateVariableValue(s, "traj_vel_roll");
    double traj_acc_roll = (1.0 / (tau_traj * tau_traj)) * (get_desired_roll() - traj_pos_roll) - (2.0 / tau_traj) * traj_vel_roll;

    double traj_pos_yaw = getStateVariableValue(s, "traj_pos_yaw");
    double traj_vel_yaw = getStateVariableValue(s, "traj_vel_yaw");
    double traj_acc_yaw = (1.0 / (tau_traj * tau_traj)) * (get_desired_yaw() - traj_pos_yaw) - (2.0 / tau_traj) * traj_vel_yaw;

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
        updateDecoderMatrix();
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

    double q_gnd_p = model.getCoordinateSet().contains("gndpitch") ? model.getCoordinateSet().get("gndpitch").getValue(s) : 0;
    double q_gnd_r = model.getCoordinateSet().contains("gndroll") ? model.getCoordinateSet().get("gndroll").getValue(s) : 0;
    double q_gnd_y = model.getCoordinateSet().contains("gndyaw") ? model.getCoordinateSet().get("gndyaw").getValue(s) : 0;
    if (std::abs(q_gnd_p) < 1e-12) q_gnd_p = 0.0;
    if (std::abs(q_gnd_r) < 1e-12) q_gnd_r = 0.0;
    if (std::abs(q_gnd_y) < 1e-12) q_gnd_y = 0.0;

    double u_gnd_p = model.getCoordinateSet().contains("gndpitch") ? model.getCoordinateSet().get("gndpitch").getSpeedValue(s) : 0;
    double u_gnd_r = model.getCoordinateSet().contains("gndroll") ? model.getCoordinateSet().get("gndroll").getSpeedValue(s) : 0;
    double u_gnd_y = model.getCoordinateSet().contains("gndyaw") ? model.getCoordinateSet().get("gndyaw").getSpeedValue(s) : 0;
    if (std::abs(u_gnd_p) < 1e-12) u_gnd_p = 0.0;
    if (std::abs(u_gnd_r) < 1e-12) u_gnd_r = 0.0;
    if (std::abs(u_gnd_y) < 1e-12) u_gnd_y = 0.0;

    double head_pitch_global = (q_p1 + q_p2) + q_gnd_p;
    double head_roll_global  = (q_r1 + q_r2) + q_gnd_r;
    double head_yaw_global   = (q_y1 + q_y2) + q_gnd_y;

    double head_pitch_dot_global = (u_p1 + u_p2) + u_gnd_p;
    double head_roll_dot_global  = (u_r1 + u_r2) + u_gnd_r;
    double head_yaw_dot_global   = (u_y1 + u_y2) + u_gnd_y;

    double head_rel_pitch = head_pitch_global - q_gnd_p;
    double head_rel_roll  = head_roll_global  - q_gnd_r;
    double head_rel_yaw   = head_yaw_global   - q_gnd_y;

    double head_rel_pitch_dot = head_pitch_dot_global - u_gnd_p;
    double head_rel_roll_dot  = head_roll_dot_global  - u_gnd_r;
    double head_rel_yaw_dot   = head_yaw_dot_global   - u_gnd_y;

    // --- VESTIBULAR RECONSTRUCTION AND HISTORY ---
    SimTK::Vector omega(3);
    omega[0] = head_roll_dot_global;
    omega[1] = head_yaw_dot_global;
    omega[2] = head_pitch_dot_global;
    
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
    
    // Observer estimated attitude for vestibular history buffer
    double q0 = getStateVariableValue(s, "obs_q0");
    double q1 = getStateVariableValue(s, "obs_q1");
    double q2 = getStateVariableValue(s, "obs_q2");
    double q3 = getStateVariableValue(s, "obs_q3");
    double norm_q = std::sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (norm_q > 1e-4) { q0 /= norm_q; q1 /= norm_q; q2 /= norm_q; q3 /= norm_q; }
    double estimated_roll = std::atan2(2.0*(q0*q1 + q2*q3), 1.0 - 2.0*(q1*q1 + q2*q2));
    double estimated_yaw = std::asin(std::clamp(2.0*(q0*q2 - q3*q1), -1.0, 1.0));
    double estimated_pitch = std::atan2(2.0*(q0*q3 + q1*q2), 1.0 - 2.0*(q2*q2 + q3*q3));
    mahony_angle = SimTK::Vec3(estimated_pitch, estimated_roll, estimated_yaw);
    
    HistRecord hr;
    hr.theta = SimTK::Vector(3);
    hr.theta[0] = estimated_roll;
    hr.theta[1] = estimated_yaw;
    hr.theta[2] = estimated_pitch;
    hr.omega_recon = omega_recon;
    hr.theta_trunk = SimTK::Vector(3);
    hr.theta_trunk[0] = q_gnd_r;
    hr.theta_trunk[1] = q_gnd_y;
    hr.theta_trunk[2] = q_gnd_p;
    hr.omega_trunk = SimTK::Vector(3);
    hr.omega_trunk[0] = u_gnd_r;
    hr.omega_trunk[1] = u_gnd_y;
    hr.omega_trunk[2] = u_gnd_p;
    hr.v_lin = model.getBodySet().get("skull").getVelocityInGround(s)[1];
    
    hr.L.resize(N);
    hr.L_dot.resize(N);
    hr.Ia.resize(N);
    SimTK::Vector dL_unexp(N, 0.0);
    SimTK::Vector dL_dot_unexp(N, 0.0);
    for(int i=0; i<N; ++i) {
        const Muscle& m = muscles.get(active_muscle_indices[i]);
        hr.L[i] = m.getLength(s);
        hr.L_dot[i] = m.getLengtheningSpeed(s);
        hr.Ia[i] = 0.0;
        
        try {
            double L_norm = m.getNormalizedFiberLength(s);
            double u = (L_norm - 1.0) * 100.0;
            
            // Equilibrium Point (lambda-model): Multi-tier voluntary task feedforward bias
            // 1. Position bias (expected voluntary length deformation):
            double opt_len = m.getOptimalFiberLength();
            double dL_task = -(J_moment_arms(0, i) * traj_pos_pitch + 
                               J_moment_arms(1, i) * traj_pos_roll + 
                               J_moment_arms(2, i) * traj_pos_yaw);
            double dL_norm_task = (std::abs(dL_task) > 1e-6 && opt_len > 1e-4) ? (dL_task / opt_len) : 0.0;
            
            // 2. Velocity bias (expected voluntary lengthening speed):
            double dL_dot_task = -(J_moment_arms(0, i) * traj_vel_pitch + 
                                   J_moment_arms(1, i) * traj_vel_roll + 
                                   J_moment_arms(2, i) * traj_vel_yaw);
            double dL_dot_norm_task = (std::abs(dL_dot_task) > 1e-6 && opt_len > 1e-4) ? (dL_dot_task / opt_len) : 0.0;

            // 3. Acceleration bias (expected voluntary lengthening acceleration):
            double dL_ddot_task = -(J_moment_arms(0, i) * traj_acc_pitch + 
                                    J_moment_arms(1, i) * traj_acc_roll + 
                                    J_moment_arms(2, i) * traj_acc_yaw);
            double dL_ddot_norm_task = (std::abs(dL_ddot_task) > 1e-6 && opt_len > 1e-4) ? (dL_ddot_task / opt_len) : 0.0;

            // Unexpected muscle length and lengthening velocity
            dL_unexp[i] = (hr.L[i] - base_lengths[i]) - dL_task;
            dL_dot_unexp[i] = hr.L_dot[i] - dL_dot_task;

            // Effective fiber length for spindle stretch (subtracting intentional voluntary deformation)
            double L_eff = L_norm - dL_norm_task;

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
                    
                    double L = L_eff; // Operating length offset by intentional voluntary task deformation
                    
                    double APbag1 = G_bag1 * ( (T_bag1/K_SR_bag1) - (L_NSR_bag1 - L_0SR_bag1) );
                    double APbag2 = X_bag2 * (L_sec_bag2/L_0SR_bag2) * ((T_bag2/K_SR_bag2) - (L_NSR_bag2 - L_0SR_bag2)) + (1.0-X_bag2) * (L_sec_bag2/L_0PR_bag2) * (L - (T_bag2/K_SR_bag2) - L_0SR_bag2 - L_NPR_bag2); 
                    double APchain = X_chain * (L_sec_chain/L_0SR_chain) * ((T_chain/K_SR_chain) - (L_NSR_chain - L_0SR_chain)) + (1.0-X_chain) * (L_sec_chain/L_0PR_chain) * (L - (T_chain/K_SR_chain) - L_0SR_chain - L_NPR_chain);
                    
                    // Cancel expected dynamic Bag 1 firing produced by voluntary velocity and acceleration:
                    double v_task = (dL_dot_norm_task > 1e-4) ? dL_dot_norm_task : 0.0;
                    double a_task = (dL_ddot_norm_task > 1e-4) ? dL_ddot_norm_task : 0.0;
                    double APbag1_expected = (v_task > 0.0 || a_task > 0.0) ? (124.0 * std::pow(v_task, 0.6) + 10.0 * a_task) : 0.0;
                    double APbag1_eff = APbag1 - APbag1_expected;

                    double pri_stat = G_pri_bag2*APbag2 + G_pri_chain*APchain;
                    double diff = APbag1_eff - pri_stat;
                    double S = 0.156;
                    double s_max = 0.5 * (APbag1_eff + pri_stat + std::sqrt(diff*diff + 1e-4));
                    double s_min = 0.5 * (APbag1_eff + pri_stat - std::sqrt(diff*diff + 1e-4));
                    
                    double primary = s_max + S * s_min;
                    
                    if (primary < 0.0) primary = 0.0;
                    if (primary > 500.0) primary = 500.0; // Biological max firing rate (~500 Hz) to avoid torque spikes
                    
                    hr.Ia[i] = primary;
                } catch(...) {
                    // Not a Mileusnic spindle
                }
            } else {
                // Fallback for standard muscle without subcomponent spindle:
                double dL = (m.getLength(s) - base_lengths[i]) - dL_task;
                double dL_dot = m.getLengtheningSpeed(s) - dL_dot_task;
                double dL_norm = (opt_len > 1e-4) ? (dL / opt_len) : 0.0;
                double dL_dot_norm = (opt_len > 1e-4) ? (dL_dot / opt_len) : 0.0;
                hr.Ia[i] = std::max(0.0, dL_norm * 100.0 + dL_dot_norm * 20.0);
            }
        } catch(...) {
            // Ignore
        }
    }
    
    // --- INVERSE PROPRIOCEPTIVE DECODER ---
    // Decodes relative 3D head-on-neck attitude and velocity from unexpected muscle deformations:
    // -(J*J^T)^-1 * J * dL_unexp. Kept purely passive (not connected to control loops).
    if (K_proprio_decode.nrow() == 3 && K_proprio_decode.ncol() == N) {
        for (int r = 0; r < 3; ++r) {
            double sum_angle = 0.0;
            double sum_speed = 0.0;
            for (int i = 0; i < N; ++i) {
                sum_angle += K_proprio_decode(r, i) * dL_unexp[i];
                sum_speed += K_proprio_decode(r, i) * dL_dot_unexp[i];
            }
            est_neck_rel_angle[r] = sum_angle;
            est_neck_rel_speed[r] = sum_speed;
        }
    }
    hr.est_neck_rel_angle = est_neck_rel_angle;
    hr.est_neck_rel_speed = est_neck_rel_speed;
    hr.neck_rel_q = SimTK::Vec3(head_rel_pitch, head_rel_roll, head_rel_yaw);
    hr.neck_rel_u = SimTK::Vec3(head_rel_pitch_dot, head_rel_roll_dot, head_rel_yaw_dot);
    
    history[s.getTime()] = hr;
    
    double oldest = s.getTime() - 0.100; // keep 100ms
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
    
    // Extract Estimated Angles from SE_2(3) Observer
    SimTK::Rotation R_head_curr(SimTK::Quaternion(q0, q1, q2, q3));
    const Body& skull = model.getBodySet().get("skull");
    SimTK::Vec3 w_world = skull.getVelocityInGround(s)[0];
    SimTK::Vec3 w_head = (~R_head_curr) * w_world;
    mahony_speed = SimTK::Vec3(w_head[2], w_head[0], w_head[1]); // pitch, roll, yaw
    
    // 1. Vestibular Reflex with Efference Copy Cancellation (expected vestibular firing from voluntary action)
    if (vcr_record.omega_recon.size() == 3) {
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
        
        const Body& skull = model.getBodySet().get("skull");
        SimTK::Rotation R_world_to_head = skull.getMobilizedBody().getBodyTransform(s).R().invert();
        SimTK::Vec3 g_world(0.0, -9.81, 0.0);
        SimTK::Vec3 f_oto = R_world_to_head * (vcr_data.a_lin - g_world); 
        
        SimTK::Rotation R_est;
        R_est.setRotationToBodyFixedXYZ(SimTK::Vec3(est_roll, est_yaw, est_pitch));
        SimTK::Vec3 a_lin_head = f_oto + (R_est * g_world);
        
        double r_head = 0.15; 
        double expected_ah_x = traj_acc_pitch * r_head; 
        double expected_ah_z = traj_acc_roll * r_head;
        
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
        
        double p_sc = get_p_sc();
        double z_sc = get_z_sc();
        if (std::abs(z_sc) < 1e-4) z_sc = (z_sc >= 0.0 ? 1e-4 : -1e-4);
        double b_dir_sc = p_sc / z_sc;
        double b_state_sc = (p_sc * (z_sc - p_sc)) / z_sc;

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
    tau_vol[0] = getStateVariableValue(s, "vol_filt_pitch");
    tau_vol[1] = getStateVariableValue(s, "vol_filt_roll");
    tau_vol[2] = getStateVariableValue(s, "vol_filt_yaw");
    
    // --- TOTAL MOTOR CORTEX DRIVE TO SPINAL CORD ---
    Vector tau_des(3, 0.0);
    tau_des[0] = tau_vol[0] + tau_vcr[0];
    tau_des[1] = tau_vol[1] + tau_vcr[1];
    tau_des[2] = tau_vol[2] + tau_vcr[2];
    
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
            
            double p_ccr = get_p_ccr();
            double z_ccr = get_z_ccr();
            if (std::abs(z_ccr) < 1e-4) z_ccr = (z_ccr >= 0.0 ? 1e-4 : -1e-4);
            double b_dir_ccr = p_ccr / z_ccr;
            double b_state_ccr = (p_ccr * (z_ccr - p_ccr)) / z_ccr;

            double Ia_filtered = b_dir_ccr * Ia_delayed + b_state_ccr * x_ccr;
            
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

