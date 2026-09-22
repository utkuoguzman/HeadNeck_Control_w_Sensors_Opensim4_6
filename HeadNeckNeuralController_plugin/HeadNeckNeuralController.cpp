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
    constructProperty_G_sc(1.0);
    constructProperty_G_ton(0.01);
    constructProperty_k_p(0.45);
    constructProperty_k_v(0.13);
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

HeadNeckNeuralController::HistRecord HeadNeckNeuralController::getDelayedRecord(double t, double delay) const {
    double target_t = t - delay;
    if (history.empty()) return HistRecord();
    auto it = history.lower_bound(target_t);
    if (it == history.end()) {
        return history.rbegin()->second;
    }
    if (it == history.begin()) {
        return it->second;
    }
    auto it_prev = it;
    --it_prev;
    double t1 = it_prev->first;
    double t2 = it->first;
    double w = (t2 > t1) ? (target_t - t1) / (t2 - t1) : 0.0;
    
    HistRecord res;
    res.omega_recon = it_prev->second.omega_recon * (1 - w) + it->second.omega_recon * w;
    if (it_prev->second.L.size() > 0 && it->second.L.size() == it_prev->second.L.size()) {
        res.L.resize(it_prev->second.L.size());
        res.L_dot.resize(it_prev->second.L_dot.size());
        for (int i = 0; i < res.L.size(); ++i) {
            res.L[i] = it_prev->second.L[i] * (1 - w) + it->second.L[i] * w;
            res.L_dot[i] = it_prev->second.L_dot[i] * (1 - w) + it->second.L_dot[i] * w;
        }
    }
    return res;
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
    
    SimTK::Matrix B_R(3, 3);
    B_R(0,0) = 0.707;  B_R(0,1) = 0.0; B_R(0,2) = -0.707;
    B_R(1,0) = -0.707; B_R(1,1) = 0.0; B_R(1,2) = -0.707;
    B_R(2,0) = 0.0;    B_R(2,1) = 1.0; B_R(2,2) = 0.0;

    SimTK::Matrix B_L(3, 3);
    B_L(0,0) = 0.707;  B_L(0,1) = 0.0; B_L(0,2) = 0.707;
    B_L(1,0) = -0.707; B_L(1,1) = 0.0; B_L(1,2) = 0.707;
    B_L(2,0) = 0.0;    B_L(2,1) = 1.0; B_L(2,2) = 0.0;
    
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
    
    SimTK::Matrix B_R(3, 3);
    B_R(0,0) = 0.707;  B_R(0,1) = 0.0; B_R(0,2) = -0.707;
    B_R(1,0) = -0.707; B_R(1,1) = 0.0; B_R(1,2) = -0.707;
    B_R(2,0) = 0.0;    B_R(2,1) = 1.0; B_R(2,2) = 0.0;

    SimTK::Matrix B_L(3, 3);
    B_L(0,0) = 0.707;  B_L(0,1) = 0.0; B_L(0,2) = 0.707;
    B_L(1,0) = -0.707; B_L(1,1) = 0.0; B_L(1,2) = 0.707;
    B_L(2,0) = 0.0;    B_L(2,1) = 1.0; B_L(2,2) = 0.0;
    
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
    hr.L.resize(N);
    hr.L_dot.resize(N);
    for(int i=0; i<N; ++i) {
        const Muscle& m = muscles.get(active_muscle_indices[i]);
        hr.L[i] = m.getLength(s);
        hr.L_dot[i] = m.getLengtheningSpeed(s);
    }
    history[s.getTime()] = hr;
    
    double oldest = s.getTime() - 0.050; // keep 50ms
    while(!history.empty() && history.begin()->first < oldest) {
        history.erase(history.begin());
    }
    
    HistRecord vcr_record = getDelayedRecord(s.getTime(), 0.013);
    HistRecord ccr_record = getDelayedRecord(s.getTime(), 0.018);

    // --- VCR COMMAND ---
    Vector tau_des(3, 0.0);
    
    // Pure Happee Vestibular Reflex (Stabilization opposes motion and restores posture)
    if (vcr_record.omega_recon.size() == 3 && vcr_record.theta.size() == 3) {
        // G_ton (Otolith Tonic) acts as a static proportional gravity compensator pushing back to 0
        tau_des[0] -= get_G_ton() * vcr_record.theta[2]; // pitch
        tau_des[1] -= get_G_ton() * vcr_record.theta[0]; // roll
        tau_des[2] -= get_G_ton() * vcr_record.theta[1]; // yaw
        
        // G_sc (Semicircular Canal) dampens angular velocity
        tau_des[0] -= get_G_sc() * vcr_record.omega_recon[2]; // pitch
        tau_des[1] -= get_G_sc() * vcr_record.omega_recon[0]; // roll
        tau_des[2] -= get_G_sc() * vcr_record.omega_recon[1]; // yaw
    }
    
    // Voluntary Postural PID Drive (compensates for gravity droop / Na_post equivalent)
    double pitch_int = getStateVariableValue(s, "pitch_error_integral");
    double roll_int = getStateVariableValue(s, "roll_error_integral");
    double yaw_int = getStateVariableValue(s, "yaw_error_integral");
    
    tau_des[0] += get_Kp_task() * (get_desired_pitch() - head_pitch) - get_Kd_task() * omega[2] + get_Ki_task() * pitch_int;
    tau_des[1] += get_Kp_task() * (get_desired_roll() - head_roll) - get_Kd_task() * omega[0] + get_Ki_task() * roll_int;
    tau_des[2] += get_Kp_task() * (get_desired_yaw() - head_yaw) - get_Kd_task() * omega[1] + get_Ki_task() * yaw_int;
    
    tau_des[0] = std::clamp(tau_des[0], -300.0, 300.0);
    tau_des[1] = std::clamp(tau_des[1], -100.0, 100.0);
    tau_des[2] = std::clamp(tau_des[2], -100.0, 100.0);
    
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
        
        if (ccr_record.L.size() == N) {
            double delta_L = ccr_record.L[i] - base_lengths[i];
            double L_dot = ccr_record.L_dot[i];
            
            // Cervico-Collic Reflex (CCR) - Excites muscle when stretched
            excitation += get_k_p() * delta_L + get_k_v() * L_dot;
        }
        
        excitation = std::clamp(excitation, 0.01, 1.0);
        
        Vector my_ctrl(1, excitation);
        m.addInControls(my_ctrl, controls);
    }
}
