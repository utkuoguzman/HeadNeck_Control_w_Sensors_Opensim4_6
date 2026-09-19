#include "FixationController.h"
#include <OpenSim/Simulation/Model/Muscle.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <iostream>
#include <algorithm>

// Import Vestibular class to read firing rate
#include "../vestibular_plugin/Schneider15VestibularAfferent.h"

using namespace OpenSim;
using namespace SimTK;

FixationController::FixationController() {
    constructProperties();
}

void FixationController::constructProperties() {
    constructProperty_target_location(Vec3(1.0, 0.0, 0.0)); // Default to 1m straight ahead
    constructProperty_kp_horizontal(0.5);
    constructProperty_kd_horizontal(0.05);
    constructProperty_kp_vertical(0.5);
    constructProperty_kd_vertical(0.05);
    constructProperty_kp_torsion(0.5);
    constructProperty_kd_torsion(0.05);
    constructProperty_K_vor(0.0);
    constructProperty_K_cor(0.0);
}

void FixationController::extendConnectToModel(Model& model) {
    Super::extendConnectToModel(model);
    is_initialized = false;
    last_jacobian_update_time = -1.0;
}

void FixationController::extendInitStateFromProperties(SimTK::State& s) const {
    Super::extendInitStateFromProperties(s);
    is_initialized = false;
    last_jacobian_update_time = -1.0;
}

void FixationController::initializeGeometryAndBaseline(const SimTK::State& s) const {
    const Model& model = getModel();
    const auto& muscles = model.getMuscles();
    
    r_eye_muscle_indices.clear();
    l_eye_muscle_indices.clear();
    
    std::string r_names[] = {"r_Lateral_Rectus", "r_Medial_Rectus", "r_Superior_Rectus", "r_Inferior_Rectus", "r_Superior_Oblique", "r_Inferior_Oblique"};
    std::string l_names[] = {"l_Lateral_Rectus", "l_Medial_Rectus", "l_Superior_Rectus", "l_Inferior_Rectus", "l_Superior_Oblique", "l_Inferior_Oblique"};
    
    for (int i=0; i<muscles.getSize(); ++i) {
        std::string name = muscles.get(i).getName();
        for (int m=0; m<6; ++m) {
            if (name == r_names[m]) r_eye_muscle_indices.push_back(i);
            if (name == l_names[m]) l_eye_muscle_indices.push_back(i);
        }
    }
    
    J_r.resize(3, r_eye_muscle_indices.size());
    J_l.resize(3, l_eye_muscle_indices.size());
    J_r.setToZero();
    J_l.setToZero();
    
    is_initialized = true;
    last_jacobian_update_time = -1.0;
    std::cout << "[Fixation Controller] Initialized 3-DOF Virtual Work tracking for Eyes.\n";
}

void FixationController::computeControls(const SimTK::State& s, SimTK::Vector& controls) const {
    if (!is_initialized) {
        initializeGeometryAndBaseline(s);
    }
    
    const Model& model = getModel();
    if (r_eye_muscle_indices.empty() || l_eye_muscle_indices.empty()) return;

    // We converge to 1m straight ahead in the SKULL frame (X is forward)
    Vec3 target_skull(1.0, 0.0, 0.0);
    const auto& ground = model.getGround();
    const auto& skull = model.getBodySet().get("skull");

    // VOR: Get head rotation velocity from Vestibular Sensor
    Vec3 canal_rate(0.0);
    for (const auto& v : model.getComponentList<Schneider15VestibularAfferent>()) {
        canal_rate = v.getCanalFiringRate(s);
    }

    // Update Jacobian (moment arms) sparsely (e.g. 50Hz) for speed
    if (s.getTime() - last_jacobian_update_time > 0.02) {
        const auto& muscles = model.getMuscles();
        std::string prefixes[2] = {"r_", "l_"};
        auto* indices = &r_eye_muscle_indices;
        auto* J_mat = &J_r;

        for (int eye=0; eye<2; ++eye) {
            if (eye == 1) {
                indices = &l_eye_muscle_indices;
                J_mat = &J_l;
            }
            std::string pre = prefixes[eye];
            auto& coord_yaw = model.getCoordinateSet().get(pre + "eye_add_abd");
            auto& coord_pitch = model.getCoordinateSet().get(pre + "eye_sup_inf");
            auto& coord_torsion = model.getCoordinateSet().get(pre + "eye_inc_exc");
            
            for (size_t i=0; i<indices->size(); ++i) {
                const Muscle& m = muscles.get((*indices)[i]);
                (*J_mat)(0, i) = m.computeMomentArm(s, const_cast<Coordinate&>(coord_yaw));
                (*J_mat)(1, i) = m.computeMomentArm(s, const_cast<Coordinate&>(coord_pitch));
                (*J_mat)(2, i) = m.computeMomentArm(s, const_cast<Coordinate&>(coord_torsion));
            }
        }
        last_jacobian_update_time = s.getTime();
    }

    std::string prefixes[2] = {"r_", "l_"};
    auto* indices = &r_eye_muscle_indices;
    auto* J_mat = &J_r;

    for (int eye=0; eye<2; ++eye) {
        if (eye == 1) {
            indices = &l_eye_muscle_indices;
            J_mat = &J_l;
        }
        std::string pre = prefixes[eye];
        
        const auto& eye_body = model.getBodySet().get(pre + "eye");
        Vec3 eye_origin_G = eye_body.getPositionInGround(s);
        Vec3 eye_origin_skull = ground.findStationLocationInAnotherFrame(s, eye_origin_G, skull);
        Vec3 to_target_skull = target_skull - eye_origin_skull;
        
        // Desired kinematics
        // Right hand rule: thumb UP (+Y), fingers curl +Z to +X (so positive yaw is turning left)
        double des_yaw = std::atan2(-to_target_skull[2], to_target_skull[0]);
        double des_pitch = std::atan2(to_target_skull[1], to_target_skull[0]);
        double des_torsion = 0.0;
        
        const auto& coord_yaw = model.getCoordinateSet().get(pre + "eye_add_abd");
        const auto& coord_pitch = model.getCoordinateSet().get(pre + "eye_sup_inf");
        const auto& coord_torsion = model.getCoordinateSet().get(pre + "eye_inc_exc");

        double current_yaw = coord_yaw.getValue(s);
        double current_yaw_v = coord_yaw.getSpeedValue(s);
        double current_pitch = coord_pitch.getValue(s);
        double current_pitch_v = coord_pitch.getSpeedValue(s);
        double current_torsion = coord_torsion.getValue(s);
        double current_torsion_v = coord_torsion.getSpeedValue(s);

        // PD Feedback
        Vector tau_des(3, 0.0);
        tau_des[0] = get_kp_horizontal() * (des_yaw - current_yaw) - get_kd_horizontal() * current_yaw_v;
        tau_des[1] = get_kp_vertical() * (des_pitch - current_pitch) - get_kd_vertical() * current_pitch_v;
        tau_des[2] = get_kp_torsion() * (des_torsion - current_torsion) - get_kd_torsion() * current_torsion_v;

        // VOR Feedforward
        tau_des[0] -= get_K_vor() * canal_rate[1]; // Yaw opposes head yaw
        tau_des[1] -= get_K_vor() * canal_rate[2]; // Pitch opposes head pitch
        tau_des[2] -= get_K_vor() * canal_rate[0]; // Torsion opposes head roll

        // Clamp tau_des
        tau_des[0] = std::max(-10.0, std::min(10.0, tau_des[0]));
        tau_des[1] = std::max(-10.0, std::min(10.0, tau_des[1]));
        tau_des[2] = std::max(-10.0, std::min(10.0, tau_des[2]));

        // ACTIVE-SET QP STATIC OPTIMIZATION
        int N = indices->size();
        std::vector<double> F_max(N);
        const auto& muscles = model.getMuscles();
        for(int i=0; i<N; ++i) F_max[i] = muscles.get((*indices)[i]).getMaxIsometricForce();

        std::vector<double> u_alpha(N, 0.01);
        std::vector<bool> is_free(N, true);

        for(int iter=0; iter<10; ++iter) {
            Vector tau_fixed(3, 0.0);
            for(int i=0; i<N; ++i) {
                if(!is_free[i]) {
                    for(int k=0; k<3; ++k) tau_fixed[k] += ((*J_mat)(k, i) * F_max[i]) * u_alpha[i];
                }
            }
            
            Vector tau_rem = tau_des - tau_fixed;
            
            Matrix A_A_T(3, 3);
            A_A_T.setToZero();
            for(int i=0; i<N; ++i) {
                if(is_free[i]) {
                    for(int r=0; r<3; ++r) {
                        for(int c=0; c<3; ++c) {
                            A_A_T(r, c) += ((*J_mat)(r, i) * F_max[i]) * ((*J_mat)(c, i) * F_max[i]);
                        }
                    }
                }
            }
            
            A_A_T(0,0) += 1e-6; A_A_T(1,1) += 1e-6; A_A_T(2,2) += 1e-6; // Regularization
            
            FactorLU lu(A_A_T);
            Vector rem_vec(3);
            for(int k=0; k<3; ++k) rem_vec[k] = tau_rem[k];
            
            Vector lambda_vec(3);
            lu.solve(rem_vec, lambda_vec);
            
            bool bounds_violated = false;
            for(int i=0; i<N; ++i) {
                if(is_free[i]) {
                    double a_unconstrained = 0.0;
                    for(int k=0; k<3; ++k) a_unconstrained += ((*J_mat)(k, i) * F_max[i]) * lambda_vec[k];
                    
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

        // Apply controls
        for(int i=0; i<N; ++i) {
            muscles.get((*indices)[i]).addInControls(Vector(1, u_alpha[i]), controls);
        }
    }
}
