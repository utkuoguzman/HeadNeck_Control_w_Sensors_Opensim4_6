#include "HeadNeckNeuralController.h"
#include <OpenSim/Simulation/Model/Muscle.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <iostream>
#include <algorithm>

using namespace OpenSim;
using namespace SimTK;

HeadNeckNeuralController::HeadNeckNeuralController() {
    constructProperty_delay_time(0.035);
    constructProperty_delay_vcr(0.035);
    constructProperty_delay_ccr(0.035);
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
    constructProperty_enable_eye_afferents(false); // Disabled by default during neck study; set true for eye-movement study
    constructProperty_delay_voluntary(0.040);
    constructProperty_fc_voluntary(2.0);
    
    B_R_mat = SimTK::Mat33(
        -0.589, -0.177,  0.788,
        -0.694, -0.270, -0.667,
        -0.323,  0.946, -0.038);
    B_L_mat = SimTK::Mat33(
        -0.589, -0.177, -0.788,
        -0.694, -0.270,  0.667,
        -0.323,  0.946,  0.038);
    B_R_inv_mat = B_R_mat.invert();
    B_L_inv_mat = B_L_mat.invert();
    
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
    
    // Luenberger Inverse-Sensor Observer States (Vestibular & Proprioceptive)
    addStateVariable("luen_omega_roll", SimTK::Stage::Velocity);
    addStateVariable("luen_omega_yaw", SimTK::Stage::Velocity);
    addStateVariable("luen_omega_pitch", SimTK::Stage::Velocity);


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
        std::string mname = muscles.get(i).getName();
        if ((mname.rfind("r_", 0) == 0 || mname.rfind("l_", 0) == 0) && !get_enable_eye_afferents()) continue;
        addStateVariable("ccr_state_" + mname, SimTK::Stage::Velocity);
        addStateVariable("sp_dL_" + mname, SimTK::Stage::Velocity);
        addStateVariable("sp_v_" + mname, SimTK::Stage::Velocity);
        addStateVariable("sp_fdyn_" + mname, SimTK::Stage::Velocity);
        addStateVariable("sp_fstat_" + mname, SimTK::Stage::Velocity);
    }
}

double HeadNeckNeuralController::getMahonyPitch(const SimTK::State& s) const {
    double q0 = sv_obs_q[0] ? sv_obs_q[0]->getValue(s) : getStateVariableValue(s, "obs_q0");
    double q1 = sv_obs_q[1] ? sv_obs_q[1]->getValue(s) : getStateVariableValue(s, "obs_q1");
    double q2 = sv_obs_q[2] ? sv_obs_q[2]->getValue(s) : getStateVariableValue(s, "obs_q2");
    double q3 = sv_obs_q[3] ? sv_obs_q[3]->getValue(s) : getStateVariableValue(s, "obs_q3");
    double norm_q = std::sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (norm_q > 1e-4) { q0 /= norm_q; q1 /= norm_q; q2 /= norm_q; q3 /= norm_q; }
    return std::atan2(2.0*(q0*q3 + q1*q2), 1.0 - 2.0*(q2*q2 + q3*q3));
}

double HeadNeckNeuralController::getMahonyRoll(const SimTK::State& s) const {
    double q0 = sv_obs_q[0] ? sv_obs_q[0]->getValue(s) : getStateVariableValue(s, "obs_q0");
    double q1 = sv_obs_q[1] ? sv_obs_q[1]->getValue(s) : getStateVariableValue(s, "obs_q1");
    double q2 = sv_obs_q[2] ? sv_obs_q[2]->getValue(s) : getStateVariableValue(s, "obs_q2");
    double q3 = sv_obs_q[3] ? sv_obs_q[3]->getValue(s) : getStateVariableValue(s, "obs_q3");
    double norm_q = std::sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (norm_q > 1e-4) { q0 /= norm_q; q1 /= norm_q; q2 /= norm_q; q3 /= norm_q; }
    return std::atan2(2.0*(q0*q1 + q2*q3), 1.0 - 2.0*(q1*q1 + q2*q2));
}

double HeadNeckNeuralController::getMahonyYaw(const SimTK::State& s) const {
    double q0 = sv_obs_q[0] ? sv_obs_q[0]->getValue(s) : getStateVariableValue(s, "obs_q0");
    double q1 = sv_obs_q[1] ? sv_obs_q[1]->getValue(s) : getStateVariableValue(s, "obs_q1");
    double q2 = sv_obs_q[2] ? sv_obs_q[2]->getValue(s) : getStateVariableValue(s, "obs_q2");
    double q3 = sv_obs_q[3] ? sv_obs_q[3]->getValue(s) : getStateVariableValue(s, "obs_q3");
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
    setStateVariableValue(s, "luen_omega_roll", 0.0);
    setStateVariableValue(s, "luen_omega_yaw", 0.0);
    setStateVariableValue(s, "luen_omega_pitch", 0.0);

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
        std::string mname = muscles.get(i).getName();
        if ((mname.rfind("r_", 0) == 0 || mname.rfind("l_", 0) == 0) && !get_enable_eye_afferents()) continue;
        setStateVariableValue(s, "ccr_state_" + mname, 0.0);
        setStateVariableValue(s, "sp_dL_" + mname, 0.0);
        setStateVariableValue(s, "sp_v_" + mname, 0.0);
        setStateVariableValue(s, "sp_fdyn_" + mname, 0.0);
        setStateVariableValue(s, "sp_fstat_" + mname, 0.0);
    }
}

void HeadNeckNeuralController::initializeGeometryAndBaseline(const SimTK::State& s) const {
    const Model& model = getModel();
    
    for(int i=0; i<6; ++i) {
        for(int j=0; j<2; ++j) {
            for(int k=0; k<2; ++k) {
                sv_canal[i][j][k] = traverseToStateVariable("canal_" + std::to_string(i) + "_filter_" + std::to_string(j) + "_state_" + std::to_string(k));
            }
        }
    }
    sv_traj_pos[0] = traverseToStateVariable("traj_pos_pitch");
    sv_traj_pos[1] = traverseToStateVariable("traj_pos_roll");
    sv_traj_pos[2] = traverseToStateVariable("traj_pos_yaw");
    sv_traj_vel[0] = traverseToStateVariable("traj_vel_pitch");
    sv_traj_vel[1] = traverseToStateVariable("traj_vel_roll");
    sv_traj_vel[2] = traverseToStateVariable("traj_vel_yaw");
    sv_err_int[0]  = traverseToStateVariable("pitch_error_integral");
    sv_err_int[1]  = traverseToStateVariable("roll_error_integral");
    sv_err_int[2]  = traverseToStateVariable("yaw_error_integral");
    sv_vol_filt[0] = traverseToStateVariable("vol_filt_pitch");
    sv_vol_filt[1] = traverseToStateVariable("vol_filt_roll");
    sv_vol_filt[2] = traverseToStateVariable("vol_filt_yaw");
    sv_luen_omega[0] = traverseToStateVariable("luen_omega_roll");
    sv_luen_omega[1] = traverseToStateVariable("luen_omega_yaw");
    sv_luen_omega[2] = traverseToStateVariable("luen_omega_pitch");
    sv_obs_q[0] = traverseToStateVariable("obs_q0");
    sv_obs_q[1] = traverseToStateVariable("obs_q1");
    sv_obs_q[2] = traverseToStateVariable("obs_q2");
    sv_obs_q[3] = traverseToStateVariable("obs_q3");
    sv_obs_v[0] = traverseToStateVariable("obs_vx");
    sv_obs_v[1] = traverseToStateVariable("obs_vy");
    sv_obs_v[2] = traverseToStateVariable("obs_vz");
    sv_obs_p[0] = traverseToStateVariable("obs_px");
    sv_obs_p[1] = traverseToStateVariable("obs_py");
    sv_obs_p[2] = traverseToStateVariable("obs_pz");
    sv_ton[0] = traverseToStateVariable("ton_state_pitch");
    sv_ton[1] = traverseToStateVariable("ton_state_roll");
    sv_ton[2] = traverseToStateVariable("ton_state_yaw");
    sv_sc[0]  = traverseToStateVariable("sc_state_pitch");
    sv_sc[1]  = traverseToStateVariable("sc_state_roll");
    sv_sc[2]  = traverseToStateVariable("sc_state_yaw");
    sv_phas[0] = traverseToStateVariable("phas_state_pitch");
    sv_phas[1] = traverseToStateVariable("phas_state_roll");
    sv_phas[2] = traverseToStateVariable("phas_state_yaw");

    const auto& cset = model.getCoordinateSet();
    cached_coord_p1 = cset.contains("pitch1") ? &cset.get("pitch1") : nullptr;
    cached_coord_p2 = cset.contains("pitch2") ? &cset.get("pitch2") : nullptr;
    cached_coord_r1 = cset.contains("roll1")  ? &cset.get("roll1")  : nullptr;
    cached_coord_r2 = cset.contains("roll2")  ? &cset.get("roll2")  : nullptr;
    cached_coord_y1 = cset.contains("yaw1")   ? &cset.get("yaw1")   : nullptr;
    cached_coord_y2 = cset.contains("yaw2")   ? &cset.get("yaw2")   : nullptr;
    cached_coord_gnd_p = cset.contains("gndpitch") ? &cset.get("gndpitch") : nullptr;
    cached_coord_gnd_r = cset.contains("gndroll")  ? &cset.get("gndroll")  : nullptr;
    cached_coord_gnd_y = cset.contains("gndyaw")   ? &cset.get("gndyaw")   : nullptr;
    cached_skull = &model.getBodySet().get("skull");

    // Capture anatomical resting orientation as Vestibular Baseline!
    const Body& skull = *cached_skull;
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
    J_6dof_moment_arms.resize(6, num_muscles);
    base_activations.resize(num_muscles, 0.03);
    base_Ia.resize(num_muscles, 0.0);
    base_II.resize(num_muscles, 0.0);
    last_u_alpha.assign(num_muscles, 0.03);
    name_ccr_state.resize(num_muscles);
    name_sp_dL.resize(num_muscles);
    name_sp_v.resize(num_muscles);
    name_sp_fdyn.resize(num_muscles);
    name_sp_fstat.resize(num_muscles);
    sv_ccr_state.resize(num_muscles, nullptr);
    sv_sp_dL.resize(num_muscles, nullptr);
    sv_sp_v.resize(num_muscles, nullptr);
    sv_sp_fdyn.resize(num_muscles, nullptr);
    sv_sp_fstat.resize(num_muscles, nullptr);
    cached_Ia_out.assign(num_muscles, nullptr);
    cached_II_out.assign(num_muscles, nullptr);
    cached_scale_L.resize(num_muscles, 0.0);
    cached_scale_Ia.resize(num_muscles, 0.0);
    cached_F_max.resize(num_muscles, 0.0);
    
    for(size_t i=0; i<active_muscle_indices.size(); ++i) {
        const Muscle& m = muscles.get(active_muscle_indices[i]);
        std::string mname_i = m.getName();
        name_ccr_state[i] = "ccr_state_" + mname_i;
        name_sp_dL[i] = "sp_dL_" + mname_i;
        name_sp_v[i] = "sp_v_" + mname_i;
        name_sp_fdyn[i] = "sp_fdyn_" + mname_i;
        name_sp_fstat[i] = "sp_fstat_" + mname_i;
        sv_ccr_state[i] = traverseToStateVariable(name_ccr_state[i]);
        sv_sp_dL[i] = traverseToStateVariable(name_sp_dL[i]);
        sv_sp_v[i] = traverseToStateVariable(name_sp_v[i]);
        sv_sp_fdyn[i] = traverseToStateVariable(name_sp_fdyn[i]);
        sv_sp_fstat[i] = traverseToStateVariable(name_sp_fstat[i]);
        double L0_i = m.getOptimalFiberLength();
        double cos_p_i = std::max(0.5, std::cos(m.getPennationAngleAtOptimalFiberLength()));
        cached_scale_L[i] = L0_i / (369.1427 * cos_p_i);
        cached_scale_Ia[i] = L0_i / (35000.0 * cos_p_i);
        cached_F_max[i] = m.getMaxIsometricForce();
        base_activations[i] = m.getActivation(s);
        const Component* spindle = nullptr;
        for (const auto& comp : m.getComponentList()) {
            if (comp.getName() == "spindle") {
                spindle = &comp;
                break;
            }
        }
        if (spindle) {
            try {
                cached_Ia_out[i] = static_cast<const Output<double>*>(&spindle->getOutput("primary_Ia"));
                cached_II_out[i] = static_cast<const Output<double>*>(&spindle->getOutput("secondary_II"));
                base_Ia[i] = cached_Ia_out[i]->getValue(s);
                base_II[i] = cached_II_out[i]->getValue(s);
            } catch(...) {
                base_Ia[i] = 0.0;
                base_II[i] = 0.0;
            }
        }
        
        double r_p1 = model.getCoordinateSet().contains("pitch1") ? m.computeMomentArm(s, const_cast<Coordinate&>(model.getCoordinateSet().get("pitch1"))) : 0;
        double r_p2 = model.getCoordinateSet().contains("pitch2") ? m.computeMomentArm(s, const_cast<Coordinate&>(model.getCoordinateSet().get("pitch2"))) : 0;
        double r_r1 = model.getCoordinateSet().contains("roll1") ? m.computeMomentArm(s, const_cast<Coordinate&>(model.getCoordinateSet().get("roll1"))) : 0;
        double r_r2 = model.getCoordinateSet().contains("roll2") ? m.computeMomentArm(s, const_cast<Coordinate&>(model.getCoordinateSet().get("roll2"))) : 0;
        double r_y1 = model.getCoordinateSet().contains("yaw1") ? m.computeMomentArm(s, const_cast<Coordinate&>(model.getCoordinateSet().get("yaw1"))) : 0;
        double r_y2 = model.getCoordinateSet().contains("yaw2") ? m.computeMomentArm(s, const_cast<Coordinate&>(model.getCoordinateSet().get("yaw2"))) : 0;
        
        J_moment_arms(0, i) = (r_p1 + r_p2) / 2.0; // Pitch
        J_moment_arms(1, i) = (r_r1 + r_r2) / 2.0; // Roll
        J_moment_arms(2, i) = (r_y1 + r_y2) / 2.0; // Yaw
        J_6dof_moment_arms(0, i) = r_p1;
        J_6dof_moment_arms(1, i) = r_p2;
        J_6dof_moment_arms(2, i) = r_r1;
        J_6dof_moment_arms(3, i) = r_r2;
        J_6dof_moment_arms(4, i) = r_y1;
        J_6dof_moment_arms(5, i) = r_y2;
        
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
    
    // Full 6-DOF cervical spine Jacobian inversion: A6 = J6 * J6^T (6 x 6)
    // Eliminates cross-talk from inter-vertebral buckling (pitch1 vs pitch2, roll1 vs roll2, yaw1 vs yaw2)
    SimTK::Matrix A6(6, 6);
    A6.setToZero();
    for (int i = 0; i < num_muscles; ++i) {
        for (int r = 0; r < 6; ++r) {
            for (int col = 0; col < 6; ++col) {
                A6(r, col) += J_6dof_moment_arms(r, i) * J_6dof_moment_arms(col, i);
            }
        }
    }
    for (int r = 0; r < 6; ++r) A6(r, r) += 1e-8;
    
    SimTK::Matrix A6_inv = A6.invert();
    
    // K6 = -(J6 * J6^T)^{-1} * J6 (6 x num_muscles), then sum (p1+p2, r1+r2, y1+y2) into 3 x num_muscles
    K_proprio_decode.resize(3, num_muscles);
    for (int i = 0; i < num_muscles; ++i) {
        double k6[6] = {0.0};
        for (int r = 0; r < 6; ++r) {
            double sum = 0.0;
            for (int k = 0; k < 6; ++k) {
                sum += A6_inv(r, k) * J_6dof_moment_arms(k, i);
            }
            k6[r] = -sum;
        }
        K_proprio_decode(0, i) = k6[0] + k6[1]; // pitch1 + pitch2
        K_proprio_decode(1, i) = k6[2] + k6[3]; // roll1 + roll2
        K_proprio_decode(2, i) = k6[4] + k6[5]; // yaw1 + yaw2
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
    res.valid = it_prev->second.valid || it->second.valid;
    res.omega_recon = it_prev->second.omega_recon * (1 - w) + it->second.omega_recon * w;
    res.theta = it_prev->second.theta * (1 - w) + it->second.theta * w;
    res.theta_trunk = it_prev->second.theta_trunk * (1 - w) + it->second.theta_trunk * w;
    res.omega_trunk = it_prev->second.omega_trunk * (1 - w) + it->second.omega_trunk * w;
    res.v_lin = it_prev->second.v_lin * (1 - w) + it->second.v_lin * w;
    res.est_neck_rel_angle = it_prev->second.est_neck_rel_angle * (1 - w) + it->second.est_neck_rel_angle * w;
    res.est_neck_rel_speed = it_prev->second.est_neck_rel_speed * (1 - w) + it->second.est_neck_rel_speed * w;
    

    if (it_prev->second.Ia.size() > 0 && it->second.Ia.size() == it_prev->second.Ia.size()) {
        res.Ia.resize(it_prev->second.Ia.size());
        for (int i = 0; i < res.Ia.size(); ++i) {
            res.Ia[i] = it_prev->second.Ia[i] * (1 - w) + it->second.Ia[i] * w;
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
        
        result.a_lin = (res.v_lin - v_old) / dt_window;
        result.alpha_recon = SimTK::Vec3(0.0);
    } else {
        result.alpha_recon = SimTK::Vec3(0.0);
    }
    
    return result;
}

void HeadNeckNeuralController::computeStateVariableDerivatives(const SimTK::State& s) const {
    if (!is_initialized) {
        initializeGeometryAndBaseline(s);
    }
    double tau_traj = get_trajectory_tau();
    if (tau_traj < 1e-4) tau_traj = 1e-4;
    double traj_pos_pitch = sv_traj_pos[0]->getValue(s);
    double traj_vel_pitch = sv_traj_vel[0]->getValue(s);
    double traj_acc_pitch = (1.0 / (tau_traj * tau_traj)) * (get_desired_pitch() - traj_pos_pitch) - (2.0 / tau_traj) * traj_vel_pitch;
    
    double traj_pos_roll = sv_traj_pos[1]->getValue(s);
    double traj_vel_roll = sv_traj_vel[1]->getValue(s);
    double traj_acc_roll = (1.0 / (tau_traj * tau_traj)) * (get_desired_roll() - traj_pos_roll) - (2.0 / tau_traj) * traj_vel_roll;

    double traj_pos_yaw = sv_traj_pos[2]->getValue(s);
    double traj_vel_yaw = sv_traj_vel[2]->getValue(s);
    double traj_acc_yaw = (1.0 / (tau_traj * tau_traj)) * (get_desired_yaw() - traj_pos_yaw) - (2.0 / tau_traj) * traj_vel_yaw;

    sv_traj_pos[0]->setDerivative(s, traj_vel_pitch);
    sv_traj_vel[0]->setDerivative(s, traj_acc_pitch);
    sv_traj_pos[1]->setDerivative(s, traj_vel_roll);
    sv_traj_vel[1]->setDerivative(s, traj_acc_roll);
    sv_traj_pos[2]->setDerivative(s, traj_vel_yaw);
    sv_traj_vel[2]->setDerivative(s, traj_acc_yaw);

    double u_p1 = cached_coord_p1 ? cached_coord_p1->getSpeedValue(s) : 0.0;
    double u_p2 = cached_coord_p2 ? cached_coord_p2->getSpeedValue(s) : 0.0;
    double u_r1 = cached_coord_r1 ? cached_coord_r1->getSpeedValue(s) : 0.0;
    double u_r2 = cached_coord_r2 ? cached_coord_r2->getSpeedValue(s) : 0.0;
    double u_y1 = cached_coord_y1 ? cached_coord_y1->getSpeedValue(s) : 0.0;
    double u_y2 = cached_coord_y2 ? cached_coord_y2->getSpeedValue(s) : 0.0;

    double u_gnd_p = cached_coord_gnd_p ? cached_coord_gnd_p->getSpeedValue(s) : 0.0;
    double u_gnd_r = cached_coord_gnd_r ? cached_coord_gnd_r->getSpeedValue(s) : 0.0;
    double u_gnd_y = cached_coord_gnd_y ? cached_coord_gnd_y->getSpeedValue(s) : 0.0;
    if (std::abs(u_gnd_p) < 1e-12) u_gnd_p = 0.0;
    if (std::abs(u_gnd_r) < 1e-12) u_gnd_r = 0.0;
    if (std::abs(u_gnd_y) < 1e-12) u_gnd_y = 0.0;

    double q_gnd_p = cached_coord_gnd_p ? cached_coord_gnd_p->getValue(s) : 0.0;
    double q_gnd_r = cached_coord_gnd_r ? cached_coord_gnd_r->getValue(s) : 0.0;
    double q_gnd_y = cached_coord_gnd_y ? cached_coord_gnd_y->getValue(s) : 0.0;
    if (std::abs(q_gnd_p) < 1e-12) q_gnd_p = 0.0;
    if (std::abs(q_gnd_r) < 1e-12) q_gnd_r = 0.0;
    if (std::abs(q_gnd_y) < 1e-12) q_gnd_y = 0.0;

    double head_roll_dot_global  = (u_r1 + u_r2) + u_gnd_r;
    double head_yaw_dot_global   = (u_y1 + u_y2) + u_gnd_y;
    double head_pitch_dot_global = (u_p1 + u_p2) + u_gnd_p;
    
    SimTK::Vec3 omega(head_roll_dot_global, head_yaw_dot_global, head_pitch_dot_global);
    SimTK::Vec3 v_R = B_R_mat * omega;
    SimTK::Vec3 v_L = B_L_mat * omega;
    
    double u[6] = {v_R[0], v_R[1], v_R[2], v_L[0], v_L[1], v_L[2]};
    
    double y_canal[6] = {0.0};
    for(int i=0; i<6; ++i) {
        double y_total = 0.0;
        for(int j=0; j<2; ++j) {
            double x1 = sv_canal[i][j][0]->getValue(s);
            double x2 = sv_canal[i][j][1]->getValue(s);
            
            double dx1 = x2;
            double dx2 = -fparams[j].a0 * x1 - fparams[j].a1 * x2 + u[i];
            
            sv_canal[i][j][0]->setDerivative(s, dx1);
            sv_canal[i][j][1]->setDerivative(s, dx2);
            
            double y_j = fparams[j].c0 * x1 + fparams[j].c1 * x2 + fparams[j].d * u[i];
            y_total += y_j;
        }
        y_canal[i] = y_total / 2.0;
    }
    SimTK::Vec3 v_R_filt_d(y_canal[0], y_canal[1], y_canal[2]);
    SimTK::Vec3 v_L_filt_d(y_canal[3], y_canal[4], y_canal[5]);
    SimTK::Vec3 omega_recon_d = 0.5 * (B_R_inv_mat * v_R_filt_d + B_L_inv_mat * v_L_filt_d);
    
    // Vestibular Luenberger Observer: invert Schneider (2015) canal lead (T_lead = 0.02442s) and passband gain (k_pass = 0.489214)
    const double k_canal_pass = 0.489214;
    const double T_canal_lead = 0.02442;
    double luen_w_roll  = sv_luen_omega[0]->getValue(s);
    double luen_w_yaw   = sv_luen_omega[1]->getValue(s);
    double luen_w_pitch = sv_luen_omega[2]->getValue(s);
    sv_luen_omega[0]->setDerivative(s, ((omega_recon_d[0] / k_canal_pass) - luen_w_roll)  / T_canal_lead);
    sv_luen_omega[1]->setDerivative(s, ((omega_recon_d[1] / k_canal_pass) - luen_w_yaw)   / T_canal_lead);
    sv_luen_omega[2]->setDerivative(s, ((omega_recon_d[2] / k_canal_pass) - luen_w_pitch) / T_canal_lead);

    if (!is_initialized) {
        initializeGeometryAndBaseline(s);
    }
    {
        int N_d = active_muscle_indices.size();
        SimTK::Vector est_dL_vec(N_d, 0.0);
        SimTK::Vector est_v_vec(N_d, 0.0);
        const SimTK::Vector& Y_vec = s.getY();
        const double k_gdyn = 100.0 * get_K_gamma_dyn();
        const double k_gstat = 100.0 * get_K_gamma_stat();
        const double tau_II = 0.055;
        const double tau_v  = 0.025;
        for (int i = 0; i < N_d; ++i) {
            double sp_dL  = sv_sp_dL[i]->getValue(s);
            double sp_v   = sv_sp_v[i]->getValue(s);
            double f_dyn  = sv_sp_fdyn[i]->getValue(s);
            double f_stat = sv_sp_fstat[i]->getValue(s);

            double u_a = (i < (int)last_u_alpha.size()) ? last_u_alpha[i] : 0.03;
            double g_dyn  = k_gdyn  * u_a;
            double g_stat = k_gstat * u_a;
            double f_dyn_inf   = (g_dyn * g_dyn)   / (g_dyn * g_dyn   + 3600.0);
            double f_stat_inf  = (g_stat * g_stat) / (g_stat * g_stat + 3600.0);
            double f_chain_now = (g_stat * g_stat) / (g_stat * g_stat + 8100.0);

            sv_sp_fdyn[i]->setDerivative(s, (f_dyn_inf  - f_dyn)  / 0.149);
            sv_sp_fstat[i]->setDerivative(s, (f_stat_inf - f_stat) / 0.205);

            double Ia_val = cached_Ia_out[i] ? cached_Ia_out[i]->getValue(s) : base_Ia[i];
            double II_val = cached_II_out[i] ? cached_II_out[i]->getValue(s) : base_II[i];

            double dII_gamma = 29.72138 * f_stat + 44.58207 * f_chain_now;
            double dIa_gamma = 54.45176 * f_dyn  + 0.21517  * dII_gamma;
            double delta_II = (II_val - base_II[i]) - dII_gamma;
            double delta_Ia = (Ia_val - base_Ia[i]) - dIa_gamma;

            double y_II_i = cached_scale_L[i] * delta_II;
            double delta_Ia_dyn = delta_Ia - 1.37931034 * delta_II;
            double v_from_II = (y_II_i - sp_dL) / tau_II;
            double v_lead_Ia = cached_scale_Ia[i] * delta_Ia_dyn;

            sv_sp_dL[i]->setDerivative(s, sp_v);
            sv_sp_v[i]->setDerivative(s, (v_from_II + v_lead_Ia - sp_v) / tau_v);

            est_dL_vec[i] = sp_dL;
            est_v_vec[i]  = sp_v;
        }
        // Stage 2: Combine all 72 per-spindle Luenberger states via 6-DOF Cervical Jacobian Decoder
        if (K_proprio_decode.nrow() == 3 && K_proprio_decode.ncol() == N_d) {
            for (int r = 0; r < 3; ++r) {
                double sum_angle = 0.0;
                double sum_speed = 0.0;
                for (int i = 0; i < N_d; ++i) {
                    sum_angle += K_proprio_decode(r, i) * est_dL_vec[i];
                    sum_speed += K_proprio_decode(r, i) * est_v_vec[i];
                }
                est_neck_rel_angle[r] = sum_angle;
                est_neck_rel_speed[r] = sum_speed;
            }
        }
    }

    // PID Integrator derivatives (using proprioceptive-decoded neck relative angle)
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
    double vol_rel_yaw_dot   = (!history.empty()) ? vol_data.record.est_neck_rel_speed[2] : est_neck_rel_speed[2];

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

    double vol_filt_pitch = sv_vol_filt[0]->getValue(s);
    double vol_filt_roll  = sv_vol_filt[1]->getValue(s);
    double vol_filt_yaw   = sv_vol_filt[2]->getValue(s);

    sv_vol_filt[0]->setDerivative(s, omega_vol * (tau_vol_raw_pitch - vol_filt_pitch));
    sv_vol_filt[1]->setDerivative(s, omega_vol * (tau_vol_raw_roll  - vol_filt_roll));
    sv_vol_filt[2]->setDerivative(s, omega_vol * (tau_vol_raw_yaw   - vol_filt_yaw));
    
    DelayedData vcr_data = getDelayedData(s.getTime(), get_delay_vcr());
    
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
    // Otolith gravity tether (Kp_oto ~ 0.05 rad/s so linear acceleration does not corrupt pitch/roll)
    double Kp_oto = 0.05;
    // Proprioceptive gains: anchor low-frequency 3D attitude to Luenberger proprioceptive state
    double K_prop_roll = 2.0;
    double K_prop_yaw = 2.0;
    double K_prop_pitch = 2.0;
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
        sv_obs_q[0]->setDerivative(s, dq0);
        sv_obs_q[1]->setDerivative(s, dq1);
        sv_obs_q[2]->setDerivative(s, dq2);
        sv_obs_q[3]->setDerivative(s, dq3);
        
        sv_obs_v[0]->setDerivative(s, dv[0]);
        sv_obs_v[1]->setDerivative(s, dv[1]);
        sv_obs_v[2]->setDerivative(s, dv[2]);
        
        sv_obs_p[0]->setDerivative(s, dp[0]);
        sv_obs_p[1]->setDerivative(s, dp[1]);
        sv_obs_p[2]->setDerivative(s, dp[2]);
    
    // --- NEURAL TRANSFER FUNCTION DERIVATIVES ---
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

        sv_ton[0]->setDerivative(s, -get_p_ton() * x_ton_pitch + u_ton_pitch);
        sv_ton[1]->setDerivative(s, -get_p_ton() * x_ton_roll + u_ton_roll);
        sv_ton[2]->setDerivative(s, -get_p_ton() * x_ton_yaw + u_ton_yaw);

        sv_sc[0]->setDerivative(s, -get_p_sc() * x_sc_pitch + u_sc_pitch);
        sv_sc[1]->setDerivative(s, -get_p_sc() * x_sc_roll + u_sc_roll);
        sv_sc[2]->setDerivative(s, -get_p_sc() * x_sc_yaw + u_sc_yaw);

        sv_phas[0]->setDerivative(s, -get_p_phas() * x_phas_pitch + u_phas_pitch);
        sv_phas[1]->setDerivative(s, -get_p_phas() * x_phas_roll + u_phas_roll);
        sv_phas[2]->setDerivative(s, -get_p_phas() * x_phas_yaw + u_phas_yaw);
    } else {
        sv_ton[0]->setDerivative(s, 0.0);
        sv_ton[1]->setDerivative(s, 0.0);
        sv_ton[2]->setDerivative(s, 0.0);
        sv_sc[0]->setDerivative(s, 0.0);
        sv_sc[1]->setDerivative(s, 0.0);
        sv_sc[2]->setDerivative(s, 0.0);
        sv_phas[0]->setDerivative(s, 0.0);
        sv_phas[1]->setDerivative(s, 0.0);
        sv_phas[2]->setDerivative(s, 0.0);
    }

    double p_ccr = get_p_ccr();
    DelayedData ccr_data = getDelayedData(s.getTime(), get_delay_ccr());
    HistRecord ccr_record = ccr_data.record;
    const SimTK::Vector& Y_ccr = s.getY();
    if (ccr_record.Ia.size() == active_muscle_indices.size() && active_muscle_indices.size() > 0) {
        for (size_t i = 0; i < active_muscle_indices.size(); ++i) {
            double x_ccr = sv_ccr_state[i]->getValue(s);
            double Ia_delayed = ccr_record.Ia[i];
            sv_ccr_state[i]->setDerivative(s, -p_ccr * x_ccr + Ia_delayed);
        }
    } else {
        for (size_t i = 0; i < active_muscle_indices.size(); ++i) {
            sv_ccr_state[i]->setDerivative(s, 0.0);
        }
    }

}

void HeadNeckNeuralController::computeControls(const SimTK::State& s, SimTK::Vector& controls) const {
    if (!is_initialized) {
        initializeGeometryAndBaseline(s);
    }
    double tau_traj = get_trajectory_tau();
    if (tau_traj < 1e-4) tau_traj = 1e-4;
    double traj_pos_pitch = sv_traj_pos[0]->getValue(s);
    double traj_vel_pitch = sv_traj_vel[0]->getValue(s);
    double traj_acc_pitch = (1.0 / (tau_traj * tau_traj)) * (get_desired_pitch() - traj_pos_pitch) - (2.0 / tau_traj) * traj_vel_pitch;
    
    double traj_pos_roll = sv_traj_pos[1]->getValue(s);
    double traj_vel_roll = sv_traj_vel[1]->getValue(s);
    double traj_acc_roll = (1.0 / (tau_traj * tau_traj)) * (get_desired_roll() - traj_pos_roll) - (2.0 / tau_traj) * traj_vel_roll;

    double traj_pos_yaw = sv_traj_pos[2]->getValue(s);
    double traj_vel_yaw = sv_traj_vel[2]->getValue(s);
    double traj_acc_yaw = (1.0 / (tau_traj * tau_traj)) * (get_desired_yaw() - traj_pos_yaw) - (2.0 / tau_traj) * traj_vel_yaw;
    
    const Model& model = getModel();
    const auto& muscles = model.getMuscles();
    int N = active_muscle_indices.size();
    
    // 1. Update Jacobian slowly
    if (s.getTime() - last_jacobian_update_time > 0.05) {
        for(int i=0; i<N; ++i) {
            const Muscle& m = muscles.get(active_muscle_indices[i]);
            double r_p1 = cached_coord_p1 ? m.computeMomentArm(s, *const_cast<Coordinate*>(cached_coord_p1)) : 0.0;
            double r_p2 = cached_coord_p2 ? m.computeMomentArm(s, *const_cast<Coordinate*>(cached_coord_p2)) : 0.0;
            double r_r1 = cached_coord_r1 ? m.computeMomentArm(s, *const_cast<Coordinate*>(cached_coord_r1)) : 0.0;
            double r_r2 = cached_coord_r2 ? m.computeMomentArm(s, *const_cast<Coordinate*>(cached_coord_r2)) : 0.0;
            double r_y1 = cached_coord_y1 ? m.computeMomentArm(s, *const_cast<Coordinate*>(cached_coord_y1)) : 0.0;
            double r_y2 = cached_coord_y2 ? m.computeMomentArm(s, *const_cast<Coordinate*>(cached_coord_y2)) : 0.0;
            
            J_moment_arms(0, i) = (r_p1 + r_p2) / 2.0;
            J_moment_arms(1, i) = (r_r1 + r_r2) / 2.0;
            J_moment_arms(2, i) = (r_y1 + r_y2) / 2.0;
            J_6dof_moment_arms(0, i) = r_p1;
            J_6dof_moment_arms(1, i) = r_p2;
            J_6dof_moment_arms(2, i) = r_r1;
            J_6dof_moment_arms(3, i) = r_r2;
            J_6dof_moment_arms(4, i) = r_y1;
            J_6dof_moment_arms(5, i) = r_y2;
        }
        updateDecoderMatrix();
        last_jacobian_update_time = s.getTime();
    }
    
    double u_p1 = cached_coord_p1 ? cached_coord_p1->getSpeedValue(s) : 0.0;
    double u_p2 = cached_coord_p2 ? cached_coord_p2->getSpeedValue(s) : 0.0;
    double u_r1 = cached_coord_r1 ? cached_coord_r1->getSpeedValue(s) : 0.0;
    double u_r2 = cached_coord_r2 ? cached_coord_r2->getSpeedValue(s) : 0.0;
    double u_y1 = cached_coord_y1 ? cached_coord_y1->getSpeedValue(s) : 0.0;
    double u_y2 = cached_coord_y2 ? cached_coord_y2->getSpeedValue(s) : 0.0;

    double q_gnd_p = cached_coord_gnd_p ? cached_coord_gnd_p->getValue(s) : 0.0;
    double q_gnd_r = cached_coord_gnd_r ? cached_coord_gnd_r->getValue(s) : 0.0;
    double q_gnd_y = cached_coord_gnd_y ? cached_coord_gnd_y->getValue(s) : 0.0;
    if (std::abs(q_gnd_p) < 1e-12) q_gnd_p = 0.0;
    if (std::abs(q_gnd_r) < 1e-12) q_gnd_r = 0.0;
    if (std::abs(q_gnd_y) < 1e-12) q_gnd_y = 0.0;

    double u_gnd_p = cached_coord_gnd_p ? cached_coord_gnd_p->getSpeedValue(s) : 0.0;
    double u_gnd_r = cached_coord_gnd_r ? cached_coord_gnd_r->getSpeedValue(s) : 0.0;
    double u_gnd_y = cached_coord_gnd_y ? cached_coord_gnd_y->getSpeedValue(s) : 0.0;
    if (std::abs(u_gnd_p) < 1e-12) u_gnd_p = 0.0;
    if (std::abs(u_gnd_r) < 1e-12) u_gnd_r = 0.0;
    if (std::abs(u_gnd_y) < 1e-12) u_gnd_y = 0.0;

    double head_pitch_dot_global = (u_p1 + u_p2) + u_gnd_p;
    double head_roll_dot_global  = (u_r1 + u_r2) + u_gnd_r;
    double head_yaw_dot_global   = (u_y1 + u_y2) + u_gnd_y;

    // --- VESTIBULAR RECONSTRUCTION AND HISTORY ---
    SimTK::Vec3 omega(head_roll_dot_global, head_yaw_dot_global, head_pitch_dot_global);
    SimTK::Vec3 v_R = B_R_mat * omega;
    SimTK::Vec3 v_L = B_L_mat * omega;
    double u[6] = {v_R[0], v_R[1], v_R[2], v_L[0], v_L[1], v_L[2]};
    
    double y[6] = {0};
    for(int i=0; i<6; ++i) {
        double y_total = 0;
        for(int j=0; j<2; ++j) {
            double x1 = sv_canal[i][j][0]->getValue(s);
            double x2 = sv_canal[i][j][1]->getValue(s);
            double y_j = fparams[j].c0 * x1 + fparams[j].c1 * x2 + fparams[j].d * u[i];
            y_total += y_j;
        }
        y[i] = y_total / 2.0;
    }
    
    SimTK::Vec3 v_R_filt(y[0], y[1], y[2]);
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
    hr.v_lin = cached_skull->getVelocityInGround(s)[1];
    
    if (history.empty()) {
        for (int i = 0; i < N; ++i) {
            base_Ia[i] = cached_Ia_out[i] ? cached_Ia_out[i]->getValue(s) : 0.0;
            base_II[i] = cached_II_out[i] ? cached_II_out[i]->getValue(s) : 0.0;
        }
    }
    hr.Ia.resize(N);
    SimTK::Vector est_dL_vec(N, 0.0);
    SimTK::Vector est_v_vec(N, 0.0);
    const SimTK::Vector& Y_ctrl = s.getY();
    for(int i=0; i<N; ++i) {
        est_dL_vec[i] = sv_sp_dL[i]->getValue(s);
        est_v_vec[i]  = sv_sp_v[i]->getValue(s);
        double primary = cached_Ia_out[i] ? cached_Ia_out[i]->getValue(s) : base_Ia[i];
        double delta_Ia = std::max(0.0, primary - base_Ia[i]);
        hr.Ia[i] = std::clamp(delta_Ia, 0.0, 500.0);
    }
    
    // --- STAGE 2: 6-DOF PROPRIOCEPTIVE DECODER FROM 72 PER-SPINDLE LUENBERGER STATES ---
    if (K_proprio_decode.nrow() == 3 && K_proprio_decode.ncol() == N) {
        for (int r = 0; r < 3; ++r) {
            double sum_angle = 0.0;
            double sum_speed = 0.0;
            for (int i = 0; i < N; ++i) {
                sum_angle += K_proprio_decode(r, i) * est_dL_vec[i];
                sum_speed += K_proprio_decode(r, i) * est_v_vec[i];
            }
            est_neck_rel_angle[r] = sum_angle;
            est_neck_rel_speed[r] = sum_speed;
        }
    }
    hr.est_neck_rel_angle = est_neck_rel_angle;
    hr.est_neck_rel_speed = est_neck_rel_speed;
    
    auto it_future = history.upper_bound(s.getTime());
    if (it_future != history.end()) {
        history.erase(it_future, history.end());
    }
    history[s.getTime()] = hr;
    
    double oldest = s.getTime() - 0.100; // keep 100ms
    while(!history.empty() && history.begin()->first < oldest) {
        history.erase(history.begin());
    }
    
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
    tau_vol[2] = sv_vol_filt[2]->getValue(s);
    
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
    const std::vector<double>& F_max = cached_F_max;

    std::vector<double> u_alpha(N, 0.03);
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
                
                if(a_unconstrained < 0.03) {
                    u_alpha[i] = 0.03;
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
    last_u_alpha = u_alpha;
    
    // --- CCR AND FINAL ASSEMBLY ---
    for(int i=0; i<N; ++i) {
        const Muscle& m = muscles.get(active_muscle_indices[i]);
        
        double excitation = u_alpha[i];
        
        if (ccr_record.Ia.size() == N) {
            // Cervico-Collic Reflex (CCR) - using biological firing rate!
            double x_ccr = sv_ccr_state[i]->getValue(s);
            double Ia_delayed = ccr_record.Ia[i];
            
            double p_ccr = get_p_ccr();
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
            double K_recip = 1.0 - vol_act * (1.0 - get_K_recip_inhib());
            double inhibition_factor = 1.0;
            double u_min = 0.03;
            double u_max = 0.08;
            
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
        
        excitation = std::clamp(excitation, 0.03, 1.0);
        
        int n_controls = m.numControls();
        Vector my_ctrl(n_controls, 0.0);
        my_ctrl[0] = excitation; // Alpha motor command
        
        // Supraspinal alpha-gamma coactivation driven by descending command u_alpha[i] (no Ia->gamma loop)
        if (n_controls >= 3) {
            my_ctrl[1] = 100.0 * get_K_gamma_dyn() * u_alpha[i];  // Gamma dynamic (pulses/s)
            my_ctrl[2] = 100.0 * get_K_gamma_stat() * u_alpha[i]; // Gamma static (pulses/s)
        }
        
        m.addInControls(my_ctrl, controls);
    }
}

