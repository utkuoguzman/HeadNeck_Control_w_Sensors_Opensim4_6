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
    constructProperty_K_gamma(1.0);
    constructProperty_Kp_task(100.0);
    constructProperty_Kd_task(10.0);
    constructProperty_use_dynamic_jacobian(true);
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
    delay_buffer.clear();
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
    J_moment_arms.resize(3, num_muscles); // Back to 3-DOF mapping!
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
        
        // Average the moment arms to find the Virtual Work leverage across the head's total angular change
        J_moment_arms(0, i) = (r_p1 + r_p2) / 2.0; // Pitch
        J_moment_arms(1, i) = (r_r1 + r_r2) / 2.0; // Roll
        J_moment_arms(2, i) = (r_y1 + r_y2) / 2.0; // Yaw
    }
    
    delay_buffer.clear();
    is_initialized = true;
    last_jacobian_update_time = s.getTime();
    std::cout << "[CMC Controller] Initialized 3-DOF Virtual Work tracking for " << num_muscles << " neck muscles.\n";
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
    
    // 2. Head-level PD Law (3-DOF)
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
    
    Vector tau_des(3, 0.0);
    tau_des[0] = get_Kp_task() * (get_desired_pitch() - head_pitch) - get_Kd_task() * head_pitch_dot;
    tau_des[1] = get_Kp_task() * (get_desired_roll() - head_roll) - get_Kd_task() * head_roll_dot;
    tau_des[2] = get_Kp_task() * (get_desired_yaw() - head_yaw) - get_Kd_task() * head_yaw_dot;
    
    tau_des[0] = std::clamp(tau_des[0], -300.0, 300.0);
    tau_des[1] = std::clamp(tau_des[1], -100.0, 100.0);
    tau_des[2] = std::clamp(tau_des[2], -100.0, 100.0);
    
    // 3. STATIC OPTIMIZATION (Active-Set QP, 3xN strictly well-posed)
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
    
    // 4. Final Assembly (Sensors disabled as per user request)
    for(int i=0; i<N; ++i) {
        const Muscle& m = muscles.get(active_muscle_indices[i]);
        Vector my_ctrl(1, u_alpha[i]);
        m.addInControls(my_ctrl, controls);
    }
}
