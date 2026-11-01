#include "HeadNeckNeuralController.h"
#include <OpenSim/Simulation/Model/Muscle.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <iostream>
#include <algorithm>

using namespace OpenSim;
using namespace SimTK;

HeadNeckNeuralController::HeadNeckNeuralController() {

    constructProperty_delay_vcr(0.010);
    constructProperty_delay_ccr(0.015);
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
    constructProperty_z_phas(-1.0);
    constructProperty_p_phas(63.8079);
    constructProperty_p_phas2(-1.0);
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
    constructProperty_Kp_task_pitch(-1.0);
    constructProperty_Kd_task_pitch(-1.0);
    constructProperty_Kp_task_roll(-1.0);
    constructProperty_Kd_task_roll(-1.0);
    constructProperty_Kp_task_yaw(-1.0);
    constructProperty_Kd_task_yaw(-1.0);
    constructProperty_Ki_task(6.54);
    constructProperty_Ki_task_pitch(-1.0);
    constructProperty_Ki_task_roll(0.0);
    constructProperty_Ki_task_yaw(0.0);
    constructProperty_trajectory_tau(0.1);
    constructProperty_use_dynamic_jacobian(true);
    constructProperty_K_eso(0.0);
    constructProperty_enable_eye_afferents(false);
    constructProperty_use_smith_predictor(true);
    constructProperty_delay_smith_proprio(0.007);
    constructProperty_delay_afferent_proprio(0.007);
    constructProperty_delay_voluntary(0.040);
    constructProperty_fc_voluntary(2.0);

    right_canal_projection_matrix = SimTK::Mat33(
        -0.589, -0.177,  0.788,
        -0.694, -0.270, -0.667,
        -0.323,  0.946, -0.038);
    left_canal_projection_matrix = SimTK::Mat33(
        -0.589, -0.177, -0.788,
        -0.694, -0.270,  0.667,
        -0.323,  0.946,  0.038);
    right_canal_inverse_matrix = right_canal_projection_matrix.invert();
    left_canal_inverse_matrix  = left_canal_projection_matrix.invert();

    double regular_afferent_gain         = 2.83;
    double regular_time_constant_1       = 0.0175;
    double regular_time_constant_2       = 0.0027;
    double regular_cupula_time_constant  = 5.7;
    canal_filter_parameters[0].denominator_constant_a0 = 1.0 / (regular_cupula_time_constant * regular_time_constant_2);
    canal_filter_parameters[0].denominator_linear_a1   = 1.0 / regular_cupula_time_constant + 1.0 / regular_time_constant_2;
    canal_filter_parameters[0].numerator_zero_b1       = 1.0 / regular_time_constant_1;
    canal_filter_parameters[0].direct_feedthrough_d    = regular_afferent_gain;
    canal_filter_parameters[0].state_space_output_c0   = -regular_afferent_gain * canal_filter_parameters[0].denominator_constant_a0;
    canal_filter_parameters[0].state_space_output_c1   = regular_afferent_gain * (canal_filter_parameters[0].numerator_zero_b1 - canal_filter_parameters[0].denominator_linear_a1);

    double irregular_afferent_gain        = 27.09;
    double irregular_time_constant_1      = 0.03;
    double irregular_time_constant_2      = 0.0006;
    double irregular_cupula_time_constant = 5.7;
    canal_filter_parameters[1].denominator_constant_a0 = 1.0 / (irregular_cupula_time_constant * irregular_time_constant_2);
    canal_filter_parameters[1].denominator_linear_a1   = 1.0 / irregular_cupula_time_constant + 1.0 / irregular_time_constant_2;
    canal_filter_parameters[1].numerator_zero_b1       = 1.0 / irregular_time_constant_1;
    canal_filter_parameters[1].direct_feedthrough_d    = irregular_afferent_gain;
    canal_filter_parameters[1].state_space_output_c0   = -irregular_afferent_gain * canal_filter_parameters[1].denominator_constant_a0;
    canal_filter_parameters[1].state_space_output_c1   = irregular_afferent_gain * (canal_filter_parameters[1].numerator_zero_b1 - canal_filter_parameters[1].denominator_linear_a1);
}

void HeadNeckNeuralController::extendAddToSystem(SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);
    for (int canal_index = 0; canal_index < 6; ++canal_index) {
        for (int filter_index = 0; filter_index < 2; ++filter_index) {
            for (int state_index = 0; state_index < 2; ++state_index) {
                addStateVariable(
                    "canal_" + std::to_string(canal_index) + "_filter_" + std::to_string(filter_index) + "_state_" + std::to_string(state_index),
                    SimTK::Stage::Velocity);
            }
        }
    }

    // Trajectory and PID integral state variables
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
    addStateVariable("vol_filt2_pitch", SimTK::Stage::Velocity);
    addStateVariable("vol_filt2_roll", SimTK::Stage::Velocity);
    addStateVariable("vol_filt2_yaw", SimTK::Stage::Velocity);

    // Luenberger Inverse-Sensor Observer States (Vestibular & Proprioceptive)
    addStateVariable("luen_omega_roll", SimTK::Stage::Velocity);
    addStateVariable("luen_omega_yaw", SimTK::Stage::Velocity);
    addStateVariable("luen_omega_pitch", SimTK::Stage::Velocity);
    addStateVariable("otolith_reg_adapt_x", SimTK::Stage::Velocity);
    addStateVariable("otolith_reg_adapt_y", SimTK::Stage::Velocity);
    addStateVariable("otolith_reg_adapt_z", SimTK::Stage::Velocity);
    addStateVariable("otolith_reg_state_x", SimTK::Stage::Velocity);
    addStateVariable("otolith_reg_state_y", SimTK::Stage::Velocity);
    addStateVariable("otolith_reg_state_z", SimTK::Stage::Velocity);
    addStateVariable("otolith_irr_adapt_x", SimTK::Stage::Velocity);
    addStateVariable("otolith_irr_adapt_y", SimTK::Stage::Velocity);
    addStateVariable("otolith_irr_adapt_z", SimTK::Stage::Velocity);
    addStateVariable("otolith_irr_state_x", SimTK::Stage::Velocity);
    addStateVariable("otolith_irr_state_y", SimTK::Stage::Velocity);
    addStateVariable("otolith_irr_state_z", SimTK::Stage::Velocity);
    addStateVariable("luen_otolith_x", SimTK::Stage::Velocity);
    addStateVariable("luen_otolith_y", SimTK::Stage::Velocity);
    addStateVariable("luen_otolith_z", SimTK::Stage::Velocity);
    addStateVariable("luen_otolith_adapt_x", SimTK::Stage::Velocity);
    addStateVariable("luen_otolith_adapt_y", SimTK::Stage::Velocity);
    addStateVariable("luen_otolith_adapt_z", SimTK::Stage::Velocity);

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
    addStateVariable("phas2_state_pitch", SimTK::Stage::Velocity);
    addStateVariable("phas2_state_roll", SimTK::Stage::Velocity);
    addStateVariable("phas2_state_yaw", SimTK::Stage::Velocity);

    // Cervico-Collic Reflex Continuous State Variables
    // The number of muscles is fixed during model building.
    const auto& muscles = getModel().getMuscles();
    for (int muscle_index = 0; muscle_index < muscles.getSize(); ++muscle_index) {
        std::string muscle_name = muscles.get(muscle_index).getName();
        if ((muscle_name.rfind("r_", 0) == 0 || muscle_name.rfind("l_", 0) == 0) && !get_enable_eye_afferents()) continue;
        addStateVariable("ccr_state_" + muscle_name, SimTK::Stage::Velocity);
        addStateVariable("sp_dL_" + muscle_name, SimTK::Stage::Velocity);
        addStateVariable("sp_v_" + muscle_name, SimTK::Stage::Velocity);
        addStateVariable("sp_fdyn_" + muscle_name, SimTK::Stage::Velocity);
        addStateVariable("sp_fstat_" + muscle_name, SimTK::Stage::Velocity);
    }
}

double HeadNeckNeuralController::getMahonyPitch(const SimTK::State& state) const {
    double quaternion_0 = state_variable_observer_quaternion[0] ? state_variable_observer_quaternion[0]->getValue(state) : getStateVariableValue(state, "obs_q0");
    double quaternion_1 = state_variable_observer_quaternion[1] ? state_variable_observer_quaternion[1]->getValue(state) : getStateVariableValue(state, "obs_q1");
    double quaternion_2 = state_variable_observer_quaternion[2] ? state_variable_observer_quaternion[2]->getValue(state) : getStateVariableValue(state, "obs_q2");
    double quaternion_3 = state_variable_observer_quaternion[3] ? state_variable_observer_quaternion[3]->getValue(state) : getStateVariableValue(state, "obs_q3");
    double quaternion_norm = std::sqrt(
        quaternion_0 * quaternion_0 + quaternion_1 * quaternion_1 +
        quaternion_2 * quaternion_2 + quaternion_3 * quaternion_3);
    if (quaternion_norm > 1e-4) {
        quaternion_0 /= quaternion_norm;
        quaternion_1 /= quaternion_norm;
        quaternion_2 /= quaternion_norm;
        quaternion_3 /= quaternion_norm;
    }
    return std::atan2(
        2.0 * (quaternion_0 * quaternion_3 + quaternion_1 * quaternion_2),
        1.0 - 2.0 * (quaternion_2 * quaternion_2 + quaternion_3 * quaternion_3));
}

double HeadNeckNeuralController::getMahonyRoll(const SimTK::State& state) const {
    double quaternion_0 = state_variable_observer_quaternion[0] ? state_variable_observer_quaternion[0]->getValue(state) : getStateVariableValue(state, "obs_q0");
    double quaternion_1 = state_variable_observer_quaternion[1] ? state_variable_observer_quaternion[1]->getValue(state) : getStateVariableValue(state, "obs_q1");
    double quaternion_2 = state_variable_observer_quaternion[2] ? state_variable_observer_quaternion[2]->getValue(state) : getStateVariableValue(state, "obs_q2");
    double quaternion_3 = state_variable_observer_quaternion[3] ? state_variable_observer_quaternion[3]->getValue(state) : getStateVariableValue(state, "obs_q3");
    double quaternion_norm = std::sqrt(
        quaternion_0 * quaternion_0 + quaternion_1 * quaternion_1 +
        quaternion_2 * quaternion_2 + quaternion_3 * quaternion_3);
    if (quaternion_norm > 1e-4) {
        quaternion_0 /= quaternion_norm;
        quaternion_1 /= quaternion_norm;
        quaternion_2 /= quaternion_norm;
        quaternion_3 /= quaternion_norm;
    }
    return std::atan2(
        2.0 * (quaternion_0 * quaternion_1 + quaternion_2 * quaternion_3),
        1.0 - 2.0 * (quaternion_1 * quaternion_1 + quaternion_2 * quaternion_2));
}

double HeadNeckNeuralController::getMahonyYaw(const SimTK::State& state) const {
    double quaternion_0 = state_variable_observer_quaternion[0] ? state_variable_observer_quaternion[0]->getValue(state) : getStateVariableValue(state, "obs_q0");
    double quaternion_1 = state_variable_observer_quaternion[1] ? state_variable_observer_quaternion[1]->getValue(state) : getStateVariableValue(state, "obs_q1");
    double quaternion_2 = state_variable_observer_quaternion[2] ? state_variable_observer_quaternion[2]->getValue(state) : getStateVariableValue(state, "obs_q2");
    double quaternion_3 = state_variable_observer_quaternion[3] ? state_variable_observer_quaternion[3]->getValue(state) : getStateVariableValue(state, "obs_q3");
    double quaternion_norm = std::sqrt(
        quaternion_0 * quaternion_0 + quaternion_1 * quaternion_1 +
        quaternion_2 * quaternion_2 + quaternion_3 * quaternion_3);
    if (quaternion_norm > 1e-4) {
        quaternion_0 /= quaternion_norm;
        quaternion_1 /= quaternion_norm;
        quaternion_2 /= quaternion_norm;
        quaternion_3 /= quaternion_norm;
    }
    return std::asin(std::clamp(2.0 * (quaternion_0 * quaternion_2 - quaternion_3 * quaternion_1), -1.0, 1.0));
}

double HeadNeckNeuralController::getMahonyPitchSpeed(const SimTK::State& state) const {
    return mahony_estimated_angular_velocity[0];
}

double HeadNeckNeuralController::getMahonyRollSpeed(const SimTK::State& state) const {
    return mahony_estimated_angular_velocity[1];
}

double HeadNeckNeuralController::getMahonyYawSpeed(const SimTK::State& state) const {
    return mahony_estimated_angular_velocity[2];
}

void HeadNeckNeuralController::extendConnectToModel(Model& model) {
    Super::extendConnectToModel(model);
    is_initialized = false;
    last_jacobian_update_time = -1.0;
}

void HeadNeckNeuralController::extendInitStateFromProperties(SimTK::State& state) const {
    Super::extendInitStateFromProperties(state);
    is_initialized = false;
    last_jacobian_update_time = -1.0;
    history.clear();
    estimated_neck_relative_angle = SimTK::Vec3(0.0);
    estimated_neck_relative_speed = SimTK::Vec3(0.0);
    mahony_estimated_angle = SimTK::Vec3(0.0);
    mahony_estimated_angular_velocity = SimTK::Vec3(0.0);

    setStateVariableValue(state, "traj_pos_pitch", 0.0);
    setStateVariableValue(state, "traj_vel_pitch", 0.0);
    setStateVariableValue(state, "traj_pos_roll", 0.0);
    setStateVariableValue(state, "traj_vel_roll", 0.0);
    setStateVariableValue(state, "traj_pos_yaw", 0.0);
    setStateVariableValue(state, "traj_vel_yaw", 0.0);
    setStateVariableValue(state, "vol_filt_pitch", 0.0);
    setStateVariableValue(state, "vol_filt_roll", 0.0);
    setStateVariableValue(state, "vol_filt_yaw", 0.0);
    setStateVariableValue(state, "vol_filt2_pitch", 0.0);
    setStateVariableValue(state, "vol_filt2_roll", 0.0);
    setStateVariableValue(state, "vol_filt2_yaw", 0.0);
    setStateVariableValue(state, "luen_omega_roll", 0.0);
    setStateVariableValue(state, "luen_omega_yaw", 0.0);
    setStateVariableValue(state, "luen_omega_pitch", 0.0);
    const double reg_adapt_ratio = 37.4 / 66.0;  // k0 / k_reg (Schneider et al. 2015 / Happee et al. 2017)
    const double irr_adapt_ratio = 37.4 / 245.0; // k0 / k_irr (Schneider et al. 2015 / Happee et al. 2017)
    setStateVariableValue(state, "otolith_reg_adapt_x", 0.0);
    setStateVariableValue(state, "otolith_reg_adapt_y", 9.81);
    setStateVariableValue(state, "otolith_reg_adapt_z", 0.0);
    setStateVariableValue(state, "otolith_reg_state_x", 0.0);
    setStateVariableValue(state, "otolith_reg_state_y", reg_adapt_ratio * 9.81);
    setStateVariableValue(state, "otolith_reg_state_z", 0.0);
    setStateVariableValue(state, "otolith_irr_adapt_x", 0.0);
    setStateVariableValue(state, "otolith_irr_adapt_y", 9.81);
    setStateVariableValue(state, "otolith_irr_adapt_z", 0.0);
    setStateVariableValue(state, "otolith_irr_state_x", 0.0);
    setStateVariableValue(state, "otolith_irr_state_y", irr_adapt_ratio * 9.81);
    setStateVariableValue(state, "otolith_irr_state_z", 0.0);
    setStateVariableValue(state, "luen_otolith_x", 0.0);
    setStateVariableValue(state, "luen_otolith_y", reg_adapt_ratio * 9.81);
    setStateVariableValue(state, "luen_otolith_z", 0.0);
    setStateVariableValue(state, "luen_otolith_adapt_x", 0.0);
    setStateVariableValue(state, "luen_otolith_adapt_y", reg_adapt_ratio * 9.81);
    setStateVariableValue(state, "luen_otolith_adapt_z", 0.0);

    setStateVariableValue(state, "obs_q0", 1.0);
    setStateVariableValue(state, "obs_q1", 0.0);
    setStateVariableValue(state, "obs_q2", 0.0);
    setStateVariableValue(state, "obs_q3", 0.0);
    setStateVariableValue(state, "obs_vx", 0.0);
    setStateVariableValue(state, "obs_vy", 0.0);
    setStateVariableValue(state, "obs_vz", 0.0);
    setStateVariableValue(state, "obs_px", 0.0);
    setStateVariableValue(state, "obs_py", 0.0);
    setStateVariableValue(state, "obs_pz", 0.0);
    setStateVariableValue(state, "phas2_state_pitch", 0.0);
    setStateVariableValue(state, "phas2_state_roll", 0.0);
    setStateVariableValue(state, "phas2_state_yaw", 0.0);

    const auto& muscles = getModel().getMuscles();
    for (int muscle_index = 0; muscle_index < muscles.getSize(); ++muscle_index) {
        std::string muscle_name = muscles.get(muscle_index).getName();
        if ((muscle_name.rfind("r_", 0) == 0 || muscle_name.rfind("l_", 0) == 0) && !get_enable_eye_afferents()) continue;
        setStateVariableValue(state, "ccr_state_" + muscle_name, 0.0);
        setStateVariableValue(state, "sp_dL_" + muscle_name, 0.0);
        setStateVariableValue(state, "sp_v_" + muscle_name, 0.0);
        setStateVariableValue(state, "sp_fdyn_" + muscle_name, 0.0);
        setStateVariableValue(state, "sp_fstat_" + muscle_name, 0.0);
    }
}

void HeadNeckNeuralController::initializeGeometryAndBaseline(const SimTK::State& state) const {
    const Model& model = getModel();

    for (int canal_index = 0; canal_index < 6; ++canal_index) {
        for (int filter_index = 0; filter_index < 2; ++filter_index) {
            for (int state_index = 0; state_index < 2; ++state_index) {
                state_variable_semicircular_canal[canal_index][filter_index][state_index] =
                    traverseToStateVariable("canal_" + std::to_string(canal_index) + "_filter_" + std::to_string(filter_index) + "_state_" + std::to_string(state_index));
            }
        }
    }
    state_variable_trajectory_position[0] = traverseToStateVariable("traj_pos_pitch");
    state_variable_trajectory_position[1] = traverseToStateVariable("traj_pos_roll");
    state_variable_trajectory_position[2] = traverseToStateVariable("traj_pos_yaw");
    state_variable_trajectory_velocity[0] = traverseToStateVariable("traj_vel_pitch");
    state_variable_trajectory_velocity[1] = traverseToStateVariable("traj_vel_roll");
    state_variable_trajectory_velocity[2] = traverseToStateVariable("traj_vel_yaw");
    state_variable_error_integral[0]      = traverseToStateVariable("pitch_error_integral");
    state_variable_error_integral[1]      = traverseToStateVariable("roll_error_integral");
    state_variable_error_integral[2]      = traverseToStateVariable("yaw_error_integral");
    state_variable_voluntary_filter_stage1[0] = traverseToStateVariable("vol_filt_pitch");
    state_variable_voluntary_filter_stage1[1] = traverseToStateVariable("vol_filt_roll");
    state_variable_voluntary_filter_stage1[2] = traverseToStateVariable("vol_filt_yaw");
    state_variable_voluntary_filter_stage2[0] = traverseToStateVariable("vol_filt2_pitch");
    state_variable_voluntary_filter_stage2[1] = traverseToStateVariable("vol_filt2_roll");
    state_variable_voluntary_filter_stage2[2] = traverseToStateVariable("vol_filt2_yaw");
    state_variable_luenberger_angular_velocity[0] = traverseToStateVariable("luen_omega_roll");
    state_variable_luenberger_angular_velocity[1] = traverseToStateVariable("luen_omega_yaw");
    state_variable_luenberger_angular_velocity[2] = traverseToStateVariable("luen_omega_pitch");
    state_variable_regular_otolith_adapt[0]       = traverseToStateVariable("otolith_reg_adapt_x");
    state_variable_regular_otolith_adapt[1]       = traverseToStateVariable("otolith_reg_adapt_y");
    state_variable_regular_otolith_adapt[2]       = traverseToStateVariable("otolith_reg_adapt_z");
    state_variable_regular_otolith[0]             = traverseToStateVariable("otolith_reg_state_x");
    state_variable_regular_otolith[1]             = traverseToStateVariable("otolith_reg_state_y");
    state_variable_regular_otolith[2]             = traverseToStateVariable("otolith_reg_state_z");
    state_variable_irregular_otolith_adapt[0]     = traverseToStateVariable("otolith_irr_adapt_x");
    state_variable_irregular_otolith_adapt[1]     = traverseToStateVariable("otolith_irr_adapt_y");
    state_variable_irregular_otolith_adapt[2]     = traverseToStateVariable("otolith_irr_adapt_z");
    state_variable_irregular_otolith[0]           = traverseToStateVariable("otolith_irr_state_x");
    state_variable_irregular_otolith[1]           = traverseToStateVariable("otolith_irr_state_y");
    state_variable_irregular_otolith[2]           = traverseToStateVariable("otolith_irr_state_z");
    state_variable_luenberger_otolith[0]          = traverseToStateVariable("luen_otolith_x");
    state_variable_luenberger_otolith[1]          = traverseToStateVariable("luen_otolith_y");
    state_variable_luenberger_otolith[2]          = traverseToStateVariable("luen_otolith_z");
    state_variable_luenberger_otolith_adapt[0]    = traverseToStateVariable("luen_otolith_adapt_x");
    state_variable_luenberger_otolith_adapt[1]    = traverseToStateVariable("luen_otolith_adapt_y");
    state_variable_luenberger_otolith_adapt[2]    = traverseToStateVariable("luen_otolith_adapt_z");
    state_variable_observer_quaternion[0] = traverseToStateVariable("obs_q0");
    state_variable_observer_quaternion[1] = traverseToStateVariable("obs_q1");
    state_variable_observer_quaternion[2] = traverseToStateVariable("obs_q2");
    state_variable_observer_quaternion[3] = traverseToStateVariable("obs_q3");
    state_variable_observer_velocity[0]   = traverseToStateVariable("obs_vx");
    state_variable_observer_velocity[1]   = traverseToStateVariable("obs_vy");
    state_variable_observer_velocity[2]   = traverseToStateVariable("obs_vz");
    state_variable_observer_position[0]   = traverseToStateVariable("obs_px");
    state_variable_observer_position[1]   = traverseToStateVariable("obs_py");
    state_variable_observer_position[2]   = traverseToStateVariable("obs_pz");
    state_variable_tonic_otolith[0]       = traverseToStateVariable("ton_state_pitch");
    state_variable_tonic_otolith[1]       = traverseToStateVariable("ton_state_roll");
    state_variable_tonic_otolith[2]       = traverseToStateVariable("ton_state_yaw");
    state_variable_semicircular_canal_reflex[0] = traverseToStateVariable("sc_state_pitch");
    state_variable_semicircular_canal_reflex[1] = traverseToStateVariable("sc_state_roll");
    state_variable_semicircular_canal_reflex[2] = traverseToStateVariable("sc_state_yaw");
    state_variable_phasic_otolith[0]      = traverseToStateVariable("phas_state_pitch");
    state_variable_phasic_otolith[1]      = traverseToStateVariable("phas_state_roll");
    state_variable_phasic_otolith[2]      = traverseToStateVariable("phas_state_yaw");
    state_variable_phasic_otolith_stage2[0] = traverseToStateVariable("phas2_state_pitch");
    state_variable_phasic_otolith_stage2[1] = traverseToStateVariable("phas2_state_roll");
    state_variable_phasic_otolith_stage2[2] = traverseToStateVariable("phas2_state_yaw");

    const auto& coordinate_set = model.getCoordinateSet();
    cached_coordinate_pitch1       = coordinate_set.contains("pitch1")   ? &coordinate_set.get("pitch1")   : nullptr;
    cached_coordinate_pitch2       = coordinate_set.contains("pitch2")   ? &coordinate_set.get("pitch2")   : nullptr;
    cached_coordinate_roll1        = coordinate_set.contains("roll1")    ? &coordinate_set.get("roll1")    : nullptr;
    cached_coordinate_roll2        = coordinate_set.contains("roll2")    ? &coordinate_set.get("roll2")    : nullptr;
    cached_coordinate_yaw1         = coordinate_set.contains("yaw1")     ? &coordinate_set.get("yaw1")     : nullptr;
    cached_coordinate_yaw2         = coordinate_set.contains("yaw2")     ? &coordinate_set.get("yaw2")     : nullptr;
    cached_coordinate_ground_pitch = coordinate_set.contains("gndpitch") ? &coordinate_set.get("gndpitch") : nullptr;
    cached_coordinate_ground_roll  = coordinate_set.contains("gndroll")  ? &coordinate_set.get("gndroll")  : nullptr;
    cached_coordinate_ground_yaw   = coordinate_set.contains("gndyaw")   ? &coordinate_set.get("gndyaw")   : nullptr;
    cached_skull = &model.getBodySet().get("skull");

    // Capture anatomical resting orientation as Vestibular Baseline
    const Body& skull = *cached_skull;
    SimTK::Rotation rotation_head_to_world = skull.getMobilizedBody().getBodyTransform(state).R();
    SimTK::Quaternion baseline_quaternion = rotation_head_to_world.convertRotationToQuaternion();
    double baseline_quaternion_0 = baseline_quaternion[0];
    double baseline_quaternion_1 = baseline_quaternion[1];
    double baseline_quaternion_2 = baseline_quaternion[2];
    double baseline_quaternion_3 = baseline_quaternion[3];
    baseline_spatial_orientation[0] = std::atan2(
        2.0 * (baseline_quaternion_0 * baseline_quaternion_1 + baseline_quaternion_2 * baseline_quaternion_3),
        1.0 - 2.0 * (baseline_quaternion_1 * baseline_quaternion_1 + baseline_quaternion_2 * baseline_quaternion_2)); // roll
    baseline_spatial_orientation[1] = std::asin(
        std::clamp(2.0 * (baseline_quaternion_0 * baseline_quaternion_2 - baseline_quaternion_3 * baseline_quaternion_1), -1.0, 1.0)); // yaw
    baseline_spatial_orientation[2] = std::atan2(
        2.0 * (baseline_quaternion_0 * baseline_quaternion_3 + baseline_quaternion_1 * baseline_quaternion_2),
        1.0 - 2.0 * (baseline_quaternion_2 * baseline_quaternion_2 + baseline_quaternion_3 * baseline_quaternion_3)); // pitch

    const auto& muscles = model.getMuscles();
    int total_muscles = muscles.getSize();

    active_muscle_indices.clear();
    muscle_names.clear();

    std::vector<std::string> target_coordinate_names = {"pitch1", "pitch2", "roll1", "roll2", "yaw1", "yaw2"};

    for (int muscle_index = 0; muscle_index < total_muscles; ++muscle_index) {
        const Muscle& muscle = muscles.get(muscle_index);
        double total_moment_arm = 0.0;
        for (const auto& coordinate_name : target_coordinate_names) {
            if (model.getCoordinateSet().contains(coordinate_name)) {
                Coordinate& coordinate = const_cast<Coordinate&>(model.getCoordinateSet().get(coordinate_name));
                total_moment_arm += std::abs(muscle.computeMomentArm(state, coordinate));
            }
        }
        if (total_moment_arm > 1e-6) {
            active_muscle_indices.push_back(muscle_index);
            muscle_names.push_back(muscle.getName());
        }
    }

    int number_of_active_muscles = active_muscle_indices.size();
    moment_arm_jacobian_3dof.resize(3, number_of_active_muscles);
    moment_arm_jacobian_6dof.resize(6, number_of_active_muscles);
    baseline_muscle_activations.resize(number_of_active_muscles, 0.03);
    baseline_primary_afferent_rates.resize(number_of_active_muscles, 0.0);
    baseline_secondary_afferent_rates.resize(number_of_active_muscles, 0.0);
    last_alpha_motor_excitations.assign(number_of_active_muscles, 0.03);
    cervico_collic_state_names.resize(number_of_active_muscles);
    spindle_length_change_state_names.resize(number_of_active_muscles);
    spindle_velocity_state_names.resize(number_of_active_muscles);
    spindle_dynamic_fusimotor_state_names.resize(number_of_active_muscles);
    spindle_static_fusimotor_state_names.resize(number_of_active_muscles);
    state_variable_cervico_collic.resize(number_of_active_muscles, nullptr);
    state_variable_spindle_length_change.resize(number_of_active_muscles, nullptr);
    state_variable_spindle_velocity.resize(number_of_active_muscles, nullptr);
    state_variable_spindle_dynamic_fusimotor.resize(number_of_active_muscles, nullptr);
    state_variable_spindle_static_fusimotor.resize(number_of_active_muscles, nullptr);
    cached_primary_afferent_outputs.assign(number_of_active_muscles, nullptr);
    cached_secondary_afferent_outputs.assign(number_of_active_muscles, nullptr);
    cached_fiber_length_scales.resize(number_of_active_muscles, 0.0);
    cached_primary_afferent_velocity_scales.resize(number_of_active_muscles, 0.0);
    cached_maximum_isometric_forces.resize(number_of_active_muscles, 0.0);
    cached_minimum_excitations.resize(number_of_active_muscles, 0.03);

    for (size_t active_index = 0; active_index < active_muscle_indices.size(); ++active_index) {
        const Muscle& muscle = muscles.get(active_muscle_indices[active_index]);
        std::string muscle_name = muscle.getName();

        // Anatomical Co-Contraction Tiers
        double minimum_excitation_value = 0.030; // Default for eye muscles / unclassified
        if (muscle_name.find("stern_mast") != std::string::npos ||
            muscle_name.find("cleid_mast") != std::string::npos ||
            muscle_name.find("cleid_occ")  != std::string::npos ||
            muscle_name.find("digastric")  != std::string::npos ||
            muscle_name.find("Geniohyoid") != std::string::npos ||
            muscle_name.find("Mylohyoid")  != std::string::npos ||
            muscle_name.find("Stylohyoid") != std::string::npos ||
            muscle_name.find("Sterno_hyoid") != std::string::npos ||
            muscle_name.find("SternoThyroid") != std::string::npos ||
            muscle_name.find("Omo_hyoid")  != std::string::npos) {
            minimum_excitation_value = 0.025; // Superficial Flexors (2.5%)
        } else if (muscle_name.find("long_cap") != std::string::npos ||
                   muscle_name.find("long_col") != std::string::npos ||
                   muscle_name.find("scalenus") != std::string::npos) {
            minimum_excitation_value = 0.040; // Deep Cervical Flexors (4.0%)
        } else if (muscle_name.find("trap_") != std::string::npos ||
                   muscle_name.find("splen_cap") != std::string::npos ||
                   muscle_name.find("splen_cerv") != std::string::npos ||
                   muscle_name.find("levator_scap") != std::string::npos) {
            minimum_excitation_value = 0.050; // Superficial Extensors (5.0%)
        } else if (muscle_name.find("semi_cap") != std::string::npos ||
                   muscle_name.find("semi_cerv") != std::string::npos ||
                   muscle_name.find("longissi_") != std::string::npos ||
                   muscle_name.find("iliocost_") != std::string::npos ||
                   muscle_name.find("rectcap_") != std::string::npos ||
                   muscle_name.find("obl_cap")  != std::string::npos) {
            minimum_excitation_value = 0.070; // Deep Cervical Extensors (7.0%)
        }
        cached_minimum_excitations[active_index] = minimum_excitation_value;
        cervico_collic_state_names[active_index] = "ccr_state_" + muscle_name;
        spindle_length_change_state_names[active_index] = "sp_dL_" + muscle_name;
        spindle_velocity_state_names[active_index] = "sp_v_" + muscle_name;
        spindle_dynamic_fusimotor_state_names[active_index] = "sp_fdyn_" + muscle_name;
        spindle_static_fusimotor_state_names[active_index] = "sp_fstat_" + muscle_name;
        state_variable_cervico_collic[active_index] = traverseToStateVariable(cervico_collic_state_names[active_index]);
        state_variable_spindle_length_change[active_index] = traverseToStateVariable(spindle_length_change_state_names[active_index]);
        state_variable_spindle_velocity[active_index] = traverseToStateVariable(spindle_velocity_state_names[active_index]);
        state_variable_spindle_dynamic_fusimotor[active_index] = traverseToStateVariable(spindle_dynamic_fusimotor_state_names[active_index]);
        state_variable_spindle_static_fusimotor[active_index] = traverseToStateVariable(spindle_static_fusimotor_state_names[active_index]);

        double optimal_fiber_length = muscle.getOptimalFiberLength();
        double cosine_pennation_angle = std::max(0.5, std::cos(muscle.getPennationAngleAtOptimalFiberLength()));
        cached_fiber_length_scales[active_index] = optimal_fiber_length / (369.1427 * cosine_pennation_angle);
        cached_primary_afferent_velocity_scales[active_index] = optimal_fiber_length / (35000.0 * cosine_pennation_angle);
        cached_maximum_isometric_forces[active_index] = muscle.getMaxIsometricForce();
        baseline_muscle_activations[active_index] = muscle.getActivation(state);

        const Component* spindle_component = nullptr;
        for (const auto& component : muscle.getComponentList()) {
            if (component.getName() == "spindle") {
                spindle_component = &component;
                break;
            }
        }
        if (spindle_component) {
            try {
                cached_primary_afferent_outputs[active_index] = static_cast<const Output<double>*>(&spindle_component->getOutput("primary_Ia"));
                cached_secondary_afferent_outputs[active_index] = static_cast<const Output<double>*>(&spindle_component->getOutput("secondary_II"));
                baseline_primary_afferent_rates[active_index] = cached_primary_afferent_outputs[active_index]->getValue(state);
                baseline_secondary_afferent_rates[active_index] = cached_secondary_afferent_outputs[active_index]->getValue(state);
            } catch (...) {
                baseline_primary_afferent_rates[active_index] = 0.0;
                baseline_secondary_afferent_rates[active_index] = 0.0;
            }
        }

        double moment_arm_pitch1 = model.getCoordinateSet().contains("pitch1") ? muscle.computeMomentArm(state, const_cast<Coordinate&>(model.getCoordinateSet().get("pitch1"))) : 0.0;
        double moment_arm_pitch2 = model.getCoordinateSet().contains("pitch2") ? muscle.computeMomentArm(state, const_cast<Coordinate&>(model.getCoordinateSet().get("pitch2"))) : 0.0;
        double moment_arm_roll1  = model.getCoordinateSet().contains("roll1")  ? muscle.computeMomentArm(state, const_cast<Coordinate&>(model.getCoordinateSet().get("roll1")))  : 0.0;
        double moment_arm_roll2  = model.getCoordinateSet().contains("roll2")  ? muscle.computeMomentArm(state, const_cast<Coordinate&>(model.getCoordinateSet().get("roll2")))  : 0.0;
        double moment_arm_yaw1   = model.getCoordinateSet().contains("yaw1")   ? muscle.computeMomentArm(state, const_cast<Coordinate&>(model.getCoordinateSet().get("yaw1")))   : 0.0;
        double moment_arm_yaw2   = model.getCoordinateSet().contains("yaw2")   ? muscle.computeMomentArm(state, const_cast<Coordinate&>(model.getCoordinateSet().get("yaw2")))   : 0.0;

        moment_arm_jacobian_3dof(0, active_index) = (moment_arm_pitch1 + moment_arm_pitch2) / 2.0; // Pitch
        moment_arm_jacobian_3dof(1, active_index) = (moment_arm_roll1  + moment_arm_roll2)  / 2.0; // Roll
        moment_arm_jacobian_3dof(2, active_index) = (moment_arm_yaw1   + moment_arm_yaw2)   / 2.0; // Yaw
        moment_arm_jacobian_6dof(0, active_index) = moment_arm_pitch1;
        moment_arm_jacobian_6dof(1, active_index) = moment_arm_pitch2;
        moment_arm_jacobian_6dof(2, active_index) = moment_arm_roll1;
        moment_arm_jacobian_6dof(3, active_index) = moment_arm_roll2;
        moment_arm_jacobian_6dof(4, active_index) = moment_arm_yaw1;
        moment_arm_jacobian_6dof(5, active_index) = moment_arm_yaw2;
    }

    updateDecoderMatrix();
    history.clear();
    is_initialized = true;
    last_jacobian_update_time = state.getTime();
}

void HeadNeckNeuralController::updateDecoderMatrix() const {
    int number_of_active_muscles = active_muscle_indices.size();
    if (number_of_active_muscles == 0) return;

    // Full 6-DOF cervical spine Jacobian inversion: gram_matrix_6dof = J6 * J6^T (6 x 6)
    // Eliminates cross-talk from inter-vertebral buckling (pitch1 vs pitch2, roll1 vs roll2, yaw1 vs yaw2)
    SimTK::Matrix gram_matrix_6dof(6, 6);
    gram_matrix_6dof.setToZero();
    for (int muscle_index = 0; muscle_index < number_of_active_muscles; ++muscle_index) {
        for (int row = 0; row < 6; ++row) {
            for (int column = 0; column < 6; ++column) {
                gram_matrix_6dof(row, column) += moment_arm_jacobian_6dof(row, muscle_index) * moment_arm_jacobian_6dof(column, muscle_index);
            }
        }
    }
    for (int row = 0; row < 6; ++row) gram_matrix_6dof(row, row) += 1e-8;

    SimTK::Matrix gram_matrix_6dof_inverse = gram_matrix_6dof.invert();

    // K6 = -(J6 * J6^T)^{-1} * J6 (6 x number_of_active_muscles), then sum (pitch1+pitch2, roll1+roll2, yaw1+yaw2) into 3 x number_of_active_muscles
    proprioceptive_decoder_matrix.resize(3, number_of_active_muscles);
    for (int muscle_index = 0; muscle_index < number_of_active_muscles; ++muscle_index) {
        double decoder_column_6dof[6] = {0.0};
        for (int row = 0; row < 6; ++row) {
            double row_sum = 0.0;
            for (int inner_index = 0; inner_index < 6; ++inner_index) {
                row_sum += gram_matrix_6dof_inverse(row, inner_index) * moment_arm_jacobian_6dof(inner_index, muscle_index);
            }
            decoder_column_6dof[row] = -row_sum;
        }
        proprioceptive_decoder_matrix(0, muscle_index) = decoder_column_6dof[0] + decoder_column_6dof[1]; // pitch1 + pitch2
        proprioceptive_decoder_matrix(1, muscle_index) = decoder_column_6dof[2] + decoder_column_6dof[3]; // roll1 + roll2
        proprioceptive_decoder_matrix(2, muscle_index) = decoder_column_6dof[4] + decoder_column_6dof[5]; // yaw1 + yaw2
    }
}

HeadNeckNeuralController::DelayedData HeadNeckNeuralController::getDelayedData(double current_time, double delay_duration, bool interpolate_spindle_afferents) const {
    DelayedData delayed_result;
    delayed_result.linear_acceleration = SimTK::Vec3(0.0);

    double target_time = current_time - delay_duration;
    if (history.empty()) return delayed_result;
    auto compare_timestamp = [](const std::pair<double, HistoryRecord>& entry, double timestamp) {
        return entry.first < timestamp;
    };
    auto iterator_upper = std::lower_bound(history.begin(), history.end(), target_time, compare_timestamp);
    if (iterator_upper == history.end()) {
        const HistoryRecord& source_record = history.back().second;
        delayed_result.record.valid = source_record.valid;
        delayed_result.record.estimated_head_orientation = source_record.estimated_head_orientation;
        delayed_result.record.reconstructed_canal_angular_velocity = source_record.reconstructed_canal_angular_velocity;
        delayed_result.record.luenberger_angular_velocity = source_record.luenberger_angular_velocity;
        delayed_result.record.reconstructed_otolith_specific_force = source_record.reconstructed_otolith_specific_force;
        delayed_result.record.luenberger_otolith_specific_force = source_record.luenberger_otolith_specific_force;
        delayed_result.record.rotation_world_to_head = source_record.rotation_world_to_head;
        delayed_result.record.estimated_rotation_head_to_world = source_record.estimated_rotation_head_to_world;
        delayed_result.record.trunk_orientation = source_record.trunk_orientation;
        delayed_result.record.trunk_angular_velocity = source_record.trunk_angular_velocity;
        delayed_result.record.head_linear_velocity = source_record.head_linear_velocity;
        delayed_result.record.estimated_neck_relative_angle = source_record.estimated_neck_relative_angle;
        delayed_result.record.estimated_neck_relative_speed = source_record.estimated_neck_relative_speed;
        delayed_result.record.mahony_estimated_angular_velocity = source_record.mahony_estimated_angular_velocity;
        delayed_result.record.cortical_voluntary_torque = source_record.cortical_voluntary_torque;
        delayed_result.record.trajectory_position = source_record.trajectory_position;
        delayed_result.record.trajectory_velocity = source_record.trajectory_velocity;
        delayed_result.record.trajectory_acceleration = source_record.trajectory_acceleration;
        if (interpolate_spindle_afferents) {
            delayed_result.record.primary_afferent_firing_rate = source_record.primary_afferent_firing_rate;
        }
        return delayed_result;
    }
    if (iterator_upper == history.begin()) {
        const HistoryRecord& source_record = iterator_upper->second;
        delayed_result.record.valid = source_record.valid;
        delayed_result.record.estimated_head_orientation = source_record.estimated_head_orientation;
        delayed_result.record.reconstructed_canal_angular_velocity = source_record.reconstructed_canal_angular_velocity;
        delayed_result.record.luenberger_angular_velocity = source_record.luenberger_angular_velocity;
        delayed_result.record.reconstructed_otolith_specific_force = source_record.reconstructed_otolith_specific_force;
        delayed_result.record.luenberger_otolith_specific_force = source_record.luenberger_otolith_specific_force;
        delayed_result.record.rotation_world_to_head = source_record.rotation_world_to_head;
        delayed_result.record.estimated_rotation_head_to_world = source_record.estimated_rotation_head_to_world;
        delayed_result.record.trunk_orientation = source_record.trunk_orientation;
        delayed_result.record.trunk_angular_velocity = source_record.trunk_angular_velocity;
        delayed_result.record.head_linear_velocity = source_record.head_linear_velocity;
        delayed_result.record.estimated_neck_relative_angle = source_record.estimated_neck_relative_angle;
        delayed_result.record.estimated_neck_relative_speed = source_record.estimated_neck_relative_speed;
        delayed_result.record.mahony_estimated_angular_velocity = source_record.mahony_estimated_angular_velocity;
        delayed_result.record.cortical_voluntary_torque = source_record.cortical_voluntary_torque;
        delayed_result.record.trajectory_position = source_record.trajectory_position;
        delayed_result.record.trajectory_velocity = source_record.trajectory_velocity;
        delayed_result.record.trajectory_acceleration = source_record.trajectory_acceleration;
        if (interpolate_spindle_afferents) {
            delayed_result.record.primary_afferent_firing_rate = source_record.primary_afferent_firing_rate;
        }
        return delayed_result;
    }
    auto iterator_lower = iterator_upper;
    --iterator_lower;
    double time_lower = iterator_lower->first;
    double time_upper = iterator_upper->first;
    double interpolation_weight = (time_upper > time_lower) ? (target_time - time_lower) / (time_upper - time_lower) : 0.0;

    HistoryRecord& interpolated_record = delayed_result.record;
    interpolated_record.valid = iterator_lower->second.valid || iterator_upper->second.valid;
    interpolated_record.reconstructed_canal_angular_velocity =
        iterator_lower->second.reconstructed_canal_angular_velocity * (1.0 - interpolation_weight) +
        iterator_upper->second.reconstructed_canal_angular_velocity * interpolation_weight;
    interpolated_record.luenberger_angular_velocity =
        iterator_lower->second.luenberger_angular_velocity * (1.0 - interpolation_weight) +
        iterator_upper->second.luenberger_angular_velocity * interpolation_weight;
    interpolated_record.reconstructed_otolith_specific_force =
        iterator_lower->second.reconstructed_otolith_specific_force * (1.0 - interpolation_weight) +
        iterator_upper->second.reconstructed_otolith_specific_force * interpolation_weight;
    interpolated_record.luenberger_otolith_specific_force =
        iterator_lower->second.luenberger_otolith_specific_force * (1.0 - interpolation_weight) +
        iterator_upper->second.luenberger_otolith_specific_force * interpolation_weight;
    interpolated_record.rotation_world_to_head =
        iterator_lower->second.rotation_world_to_head * (1.0 - interpolation_weight) +
        iterator_upper->second.rotation_world_to_head * interpolation_weight;
    interpolated_record.estimated_rotation_head_to_world =
        iterator_lower->second.estimated_rotation_head_to_world * (1.0 - interpolation_weight) +
        iterator_upper->second.estimated_rotation_head_to_world * interpolation_weight;
    interpolated_record.estimated_head_orientation =
        iterator_lower->second.estimated_head_orientation * (1.0 - interpolation_weight) +
        iterator_upper->second.estimated_head_orientation * interpolation_weight;
    interpolated_record.trunk_orientation =
        iterator_lower->second.trunk_orientation * (1.0 - interpolation_weight) +
        iterator_upper->second.trunk_orientation * interpolation_weight;
    interpolated_record.trunk_angular_velocity =
        iterator_lower->second.trunk_angular_velocity * (1.0 - interpolation_weight) +
        iterator_upper->second.trunk_angular_velocity * interpolation_weight;
    interpolated_record.head_linear_velocity =
        iterator_lower->second.head_linear_velocity * (1.0 - interpolation_weight) +
        iterator_upper->second.head_linear_velocity * interpolation_weight;
    interpolated_record.estimated_neck_relative_angle =
        iterator_lower->second.estimated_neck_relative_angle * (1.0 - interpolation_weight) +
        iterator_upper->second.estimated_neck_relative_angle * interpolation_weight;
    interpolated_record.estimated_neck_relative_speed =
        iterator_lower->second.estimated_neck_relative_speed * (1.0 - interpolation_weight) +
        iterator_upper->second.estimated_neck_relative_speed * interpolation_weight;
    interpolated_record.mahony_estimated_angular_velocity =
        iterator_lower->second.mahony_estimated_angular_velocity * (1.0 - interpolation_weight) +
        iterator_upper->second.mahony_estimated_angular_velocity * interpolation_weight;
    interpolated_record.cortical_voluntary_torque =
        iterator_lower->second.cortical_voluntary_torque * (1.0 - interpolation_weight) +
        iterator_upper->second.cortical_voluntary_torque * interpolation_weight;
    interpolated_record.trajectory_position =
        iterator_lower->second.trajectory_position * (1.0 - interpolation_weight) +
        iterator_upper->second.trajectory_position * interpolation_weight;
    interpolated_record.trajectory_velocity =
        iterator_lower->second.trajectory_velocity * (1.0 - interpolation_weight) +
        iterator_upper->second.trajectory_velocity * interpolation_weight;
    interpolated_record.trajectory_acceleration =
        iterator_lower->second.trajectory_acceleration * (1.0 - interpolation_weight) +
        iterator_upper->second.trajectory_acceleration * interpolation_weight;

    if (interpolate_spindle_afferents &&
        iterator_lower->second.primary_afferent_firing_rate.size() > 0 &&
        iterator_upper->second.primary_afferent_firing_rate.size() == iterator_lower->second.primary_afferent_firing_rate.size()) {
        interpolated_record.primary_afferent_firing_rate.resize(iterator_lower->second.primary_afferent_firing_rate.size());
        for (int muscle_index = 0; muscle_index < interpolated_record.primary_afferent_firing_rate.size(); ++muscle_index) {
            interpolated_record.primary_afferent_firing_rate[muscle_index] =
                iterator_lower->second.primary_afferent_firing_rate[muscle_index] * (1.0 - interpolation_weight) +
                iterator_upper->second.primary_afferent_firing_rate[muscle_index] * interpolation_weight;
        }
    }

    // --- NUMERICAL DIFFERENTIATION FOR ACCELERATION ---
    double finite_difference_window = 0.001; // 1ms finite-difference window (removes extra 4.5ms lag)
    double previous_target_time = target_time - finite_difference_window;
    auto previous_iterator_upper = std::lower_bound(history.begin(), history.end(), previous_target_time, compare_timestamp);
    if (previous_iterator_upper != history.end() && previous_iterator_upper != history.begin()) {
        auto previous_iterator_lower = previous_iterator_upper;
        --previous_iterator_lower;
        double previous_time_lower = previous_iterator_lower->first;
        double previous_time_upper = previous_iterator_upper->first;
        double previous_interpolation_weight =
            (previous_time_upper > previous_time_lower)
                ? (previous_target_time - previous_time_lower) / (previous_time_upper - previous_time_lower)
                : 0.0;

        SimTK::Vec3 previous_linear_velocity =
            previous_iterator_lower->second.head_linear_velocity * (1.0 - previous_interpolation_weight) +
            previous_iterator_upper->second.head_linear_velocity * previous_interpolation_weight;

        delayed_result.linear_acceleration = (interpolated_record.head_linear_velocity - previous_linear_velocity) / finite_difference_window;
        delayed_result.reconstructed_angular_acceleration = SimTK::Vec3(0.0);
    } else {
        delayed_result.reconstructed_angular_acceleration = SimTK::Vec3(0.0);
    }

    return delayed_result;
}

void HeadNeckNeuralController::computeStateVariableDerivatives(const SimTK::State& state) const {
    if (!is_initialized) {
        initializeGeometryAndBaseline(state);
    }
    double trajectory_time_constant = get_trajectory_tau();
    if (trajectory_time_constant < 1e-4) trajectory_time_constant = 1e-4;

    double trajectory_position_pitch = state_variable_trajectory_position[0]->getValue(state);
    double trajectory_velocity_pitch = state_variable_trajectory_velocity[0]->getValue(state);
    double trajectory_acceleration_pitch =
        (1.0 / (trajectory_time_constant * trajectory_time_constant)) * (get_desired_pitch() - trajectory_position_pitch) -
        (2.0 / trajectory_time_constant) * trajectory_velocity_pitch;

    double trajectory_position_roll = state_variable_trajectory_position[1]->getValue(state);
    double trajectory_velocity_roll = state_variable_trajectory_velocity[1]->getValue(state);
    double trajectory_acceleration_roll =
        (1.0 / (trajectory_time_constant * trajectory_time_constant)) * (get_desired_roll() - trajectory_position_roll) -
        (2.0 / trajectory_time_constant) * trajectory_velocity_roll;

    double trajectory_position_yaw = state_variable_trajectory_position[2]->getValue(state);
    double trajectory_velocity_yaw = state_variable_trajectory_velocity[2]->getValue(state);
    double trajectory_acceleration_yaw =
        (1.0 / (trajectory_time_constant * trajectory_time_constant)) * (get_desired_yaw() - trajectory_position_yaw) -
        (2.0 / trajectory_time_constant) * trajectory_velocity_yaw;

    state_variable_trajectory_position[0]->setDerivative(state, trajectory_velocity_pitch);
    state_variable_trajectory_velocity[0]->setDerivative(state, trajectory_acceleration_pitch);
    state_variable_trajectory_position[1]->setDerivative(state, trajectory_velocity_roll);
    state_variable_trajectory_velocity[1]->setDerivative(state, trajectory_acceleration_roll);
    state_variable_trajectory_position[2]->setDerivative(state, trajectory_velocity_yaw);
    state_variable_trajectory_velocity[2]->setDerivative(state, trajectory_acceleration_yaw);

    double speed_pitch1 = cached_coordinate_pitch1 ? cached_coordinate_pitch1->getSpeedValue(state) : 0.0;
    double speed_pitch2 = cached_coordinate_pitch2 ? cached_coordinate_pitch2->getSpeedValue(state) : 0.0;
    double speed_roll1  = cached_coordinate_roll1  ? cached_coordinate_roll1->getSpeedValue(state)  : 0.0;
    double speed_roll2  = cached_coordinate_roll2  ? cached_coordinate_roll2->getSpeedValue(state)  : 0.0;
    double speed_yaw1   = cached_coordinate_yaw1   ? cached_coordinate_yaw1->getSpeedValue(state)   : 0.0;
    double speed_yaw2   = cached_coordinate_yaw2   ? cached_coordinate_yaw2->getSpeedValue(state)   : 0.0;

    double speed_ground_pitch = cached_coordinate_ground_pitch ? cached_coordinate_ground_pitch->getSpeedValue(state) : 0.0;
    double speed_ground_roll  = cached_coordinate_ground_roll  ? cached_coordinate_ground_roll->getSpeedValue(state)  : 0.0;
    double speed_ground_yaw   = cached_coordinate_ground_yaw   ? cached_coordinate_ground_yaw->getSpeedValue(state)   : 0.0;
    if (std::abs(speed_ground_pitch) < 1e-12) speed_ground_pitch = 0.0;
    if (std::abs(speed_ground_roll)  < 1e-12) speed_ground_roll  = 0.0;
    if (std::abs(speed_ground_yaw)   < 1e-12) speed_ground_yaw   = 0.0;

    double angle_ground_pitch = cached_coordinate_ground_pitch ? cached_coordinate_ground_pitch->getValue(state) : 0.0;
    double angle_ground_roll  = cached_coordinate_ground_roll  ? cached_coordinate_ground_roll->getValue(state)  : 0.0;
    double angle_ground_yaw   = cached_coordinate_ground_yaw   ? cached_coordinate_ground_yaw->getValue(state)   : 0.0;
    if (std::abs(angle_ground_pitch) < 1e-12) angle_ground_pitch = 0.0;
    if (std::abs(angle_ground_roll)  < 1e-12) angle_ground_roll  = 0.0;
    if (std::abs(angle_ground_yaw)   < 1e-12) angle_ground_yaw   = 0.0;

    double head_roll_dot_global  = (speed_roll1  + speed_roll2)  + speed_ground_roll;
    double head_yaw_dot_global   = (speed_yaw1   + speed_yaw2)   + speed_ground_yaw;
    double head_pitch_dot_global = (speed_pitch1 + speed_pitch2) + speed_ground_pitch;

    SimTK::Vec3 head_global_angular_velocity(head_roll_dot_global, head_yaw_dot_global, head_pitch_dot_global);
    SimTK::Vec3 right_canal_projection = right_canal_projection_matrix * head_global_angular_velocity;
    SimTK::Vec3 left_canal_projection  = left_canal_projection_matrix  * head_global_angular_velocity;

    double canal_angular_velocity_inputs[6] = {
        right_canal_projection[0], right_canal_projection[1], right_canal_projection[2],
        left_canal_projection[0],  left_canal_projection[1],  left_canal_projection[2]};

    double regular_canal_afferent_outputs[6] = {0.0};
    for (int canal_index = 0; canal_index < 6; ++canal_index) {
        for (int filter_index = 0; filter_index < 2; ++filter_index) {
            double filter_state_1 = state_variable_semicircular_canal[canal_index][filter_index][0]->getValue(state);
            double filter_state_2 = state_variable_semicircular_canal[canal_index][filter_index][1]->getValue(state);

            double filter_state_1_derivative = filter_state_2;
            double filter_state_2_derivative =
                -canal_filter_parameters[filter_index].denominator_constant_a0 * filter_state_1 -
                 canal_filter_parameters[filter_index].denominator_linear_a1   * filter_state_2 +
                 canal_angular_velocity_inputs[canal_index];

            state_variable_semicircular_canal[canal_index][filter_index][0]->setDerivative(state, filter_state_1_derivative);
            state_variable_semicircular_canal[canal_index][filter_index][1]->setDerivative(state, filter_state_2_derivative);

            if (filter_index == 0) {
                regular_canal_afferent_outputs[canal_index] =
                    canal_filter_parameters[0].state_space_output_c0 * filter_state_1 +
                    canal_filter_parameters[0].state_space_output_c1 * filter_state_2 +
                    canal_filter_parameters[0].direct_feedthrough_d  * canal_angular_velocity_inputs[canal_index];
            }
        }
    }
    SimTK::Vec3 filtered_right_regular_canal_output(regular_canal_afferent_outputs[0], regular_canal_afferent_outputs[1], regular_canal_afferent_outputs[2]);
    SimTK::Vec3 filtered_left_regular_canal_output(regular_canal_afferent_outputs[3], regular_canal_afferent_outputs[4], regular_canal_afferent_outputs[5]);
    SimTK::Vec3 raw_reconstructed_regular_canal_velocity =
        0.5 * (right_canal_inverse_matrix * filtered_right_regular_canal_output + left_canal_inverse_matrix * filtered_left_regular_canal_output);

    // Vestibular Luenberger Observer: exact pole-zero inverse H_Luen(s) = (1 + T2_reg * s) / (1 + T1_reg * s)
    // of Regular canal dynamics (T1_reg = 0.0175s, T2_reg = 0.0027s, G_pass,reg = 2.83 * 0.0027 / 0.0175 = 0.436629)
    const double regular_canal_passband_gain      = 0.436629;
    const double regular_canal_lead_time_constant = 0.0175;
    const double regular_canal_pole_time_constant = 0.0027;
    const double luenberger_direct_feedthrough    = regular_canal_pole_time_constant / regular_canal_lead_time_constant; // 0.154286
    const double luenberger_state_weight          = 1.0 - luenberger_direct_feedthrough;                                 // 0.845714
    SimTK::Vec3 reconstructed_regular_canal_velocity = raw_reconstructed_regular_canal_velocity / regular_canal_passband_gain;
    double luenberger_state_roll  = state_variable_luenberger_angular_velocity[0]->getValue(state);
    double luenberger_state_yaw   = state_variable_luenberger_angular_velocity[1]->getValue(state);
    double luenberger_state_pitch = state_variable_luenberger_angular_velocity[2]->getValue(state);
    state_variable_luenberger_angular_velocity[0]->setDerivative(
        state, (reconstructed_regular_canal_velocity[0] - luenberger_state_roll) / regular_canal_lead_time_constant);
    state_variable_luenberger_angular_velocity[1]->setDerivative(
        state, (reconstructed_regular_canal_velocity[1] - luenberger_state_yaw) / regular_canal_lead_time_constant);
    state_variable_luenberger_angular_velocity[2]->setDerivative(
        state, (reconstructed_regular_canal_velocity[2] - luenberger_state_pitch) / regular_canal_lead_time_constant);
    double luenberger_angular_velocity_roll  = luenberger_state_weight * luenberger_state_roll  + luenberger_direct_feedthrough * reconstructed_regular_canal_velocity[0];
    double luenberger_angular_velocity_yaw   = luenberger_state_weight * luenberger_state_yaw   + luenberger_direct_feedthrough * reconstructed_regular_canal_velocity[1];
    double luenberger_angular_velocity_pitch = luenberger_state_weight * luenberger_state_pitch + luenberger_direct_feedthrough * reconstructed_regular_canal_velocity[2];

    // --- SUBSYSTEM 2 OTOLITH MACULAE (ADJUSTED SCHNEIDER ET AL. 2015 / HAPPEE ET AL. 2017) ---
    // Physical stimulus to Utricle & Saccule maculae: total Gravitoinertial Force f_GIF,head = R_W^H * (a_lin,W - g_W)
    const Body& skull = *cached_skull;
    SimTK::Rotation rotation_world_to_head = skull.getMobilizedBody().getBodyTransform(state).R().invert();
    SimTK::Vec3 gravity_vector_world(0.0, -9.81, 0.0);
    DelayedData current_otolith_kinematics = getDelayedData(state.getTime(), 0.0005);
    SimTK::Vec3 current_gravitoinertial_force_head =
        rotation_world_to_head * (current_otolith_kinematics.linear_acceleration - gravity_vector_world);

    // Adjusted Schneider et al. (2015) + Fernandez et al. (1972) / Happee et al. (2017, Appendix Eq. 3):
    //   1. Biproper 2-Stage Realization: H_oto(s) = G_pass,oto * [(k0/k + c*s)/(1 + c*s)] * [(1 + tau1*s)/(1 + b*s)]
    //      where tau1 = 2*a (Regular: k0=37.4, k=66.0 imp/s/g, c=1.0s, tau1=27.6ms, b=17.5ms;
    //                        Irregular: k0=37.4, k=245.0 imp/s/g, c=0.25s, tau1=27.2ms, b=7.0ms).
    //   2. Mid-Frequency Passband Normalization in Vestibular Nuclei: G_pass,oto,reg = 66.0/9.81 = 6.727829 (imp/s)/(m/s^2),
    //      G_pass,oto,irr = 245.0/9.81 = 24.974516 (imp/s)/(m/s^2).
    //   3. Exact 2-Stage Minimum-Phase Luenberger Observer: H_Luen,oto(s) = [(1 + c_reg*s)/(k0/k_reg + c_reg*s)] * [(1 + b_reg*s)/(1 + tau1_reg*s)]
    const double reg_oto_adapt_ratio        = 37.4 / 66.0;   // 0.566667
    const double irr_oto_adapt_ratio        = 37.4 / 245.0;  // 0.152653
    const double reg_oto_adapt_tau          = 1.0000;        // c_reg = 1.0 s
    const double irr_oto_adapt_tau          = 0.2500;        // c_irr = 0.25 s
    const double reg_oto_lead_tau1          = 0.0276;        // 2 * a_reg = 27.6 ms
    const double reg_oto_pole_b             = 0.0175;        // b_reg = 17.5 ms
    const double irr_oto_lead_tau1          = 0.0272;        // 2 * a_irr = 27.2 ms
    const double irr_oto_pole_b             = 0.0070;        // b_irr = 7.0 ms
    const double regular_otolith_passband_gain   = 66.0 / 9.81;  // 6.727829 (imp/s)/(m/s^2)
    const double irregular_otolith_passband_gain = 245.0 / 9.81; // 24.974516 (imp/s)/(m/s^2)
    const double reg_macular_feedthrough    = reg_oto_lead_tau1 / reg_oto_pole_b;  // 1.577143
    const double reg_macular_state_weight   = 1.0 - reg_macular_feedthrough;       // -0.577143
    const double luen_macular_feedthrough   = reg_oto_pole_b / reg_oto_lead_tau1;  // 0.634058
    const double luen_macular_state_weight  = 1.0 - luen_macular_feedthrough;      // 0.365942
    const double luen_adapt_tau             = reg_oto_adapt_tau / reg_oto_adapt_ratio; // 1.764706 s
    const double luen_adapt_state_weight    = (1.0 / reg_oto_adapt_ratio) - 1.0;       // 0.764706

    SimTK::Vec3 current_luenberger_otolith_force_head(0.0);
    for (int axis_index = 0; axis_index < 3; ++axis_index) {
        double u_gif           = current_gravitoinertial_force_head[axis_index];
        double reg_adapt_state = state_variable_regular_otolith_adapt[axis_index]->getValue(state);
        double reg_mac_state   = state_variable_regular_otolith[axis_index]->getValue(state);
        double irr_adapt_state = state_variable_irregular_otolith_adapt[axis_index]->getValue(state);
        double irr_mac_state   = state_variable_irregular_otolith[axis_index]->getValue(state);
        double luen_mac_state  = state_variable_luenberger_otolith[axis_index]->getValue(state);
        double luen_adp_state  = state_variable_luenberger_otolith_adapt[axis_index]->getValue(state);

        // Regular Otolith Stage 1 (Adaptation) & Stage 2 (Macular Lead-Lag)
        state_variable_regular_otolith_adapt[axis_index]->setDerivative(
            state, (u_gif - reg_adapt_state) / reg_oto_adapt_tau);
        double reg_stage1_out = (reg_oto_adapt_ratio - 1.0) * reg_adapt_state + u_gif;
        state_variable_regular_otolith[axis_index]->setDerivative(
            state, (reg_stage1_out - reg_mac_state) / reg_oto_pole_b);
        double regular_otolith_firing_rate =
            regular_otolith_passband_gain * (reg_macular_state_weight * reg_mac_state + reg_macular_feedthrough * reg_stage1_out);

        // Irregular Otolith Stage 1 (Adaptation) & Stage 2 (Macular Lead-Lag)
        state_variable_irregular_otolith_adapt[axis_index]->setDerivative(
            state, (u_gif - irr_adapt_state) / irr_oto_adapt_tau);
        double irr_stage1_out = (irr_oto_adapt_ratio - 1.0) * irr_adapt_state + u_gif;
        state_variable_irregular_otolith[axis_index]->setDerivative(
            state, (irr_stage1_out - irr_mac_state) / irr_oto_pole_b);

        // Vestibular Nuclei Passband Normalization & 2-Stage Otolith Luenberger Observer
        double reconstructed_regular_otolith_force = regular_otolith_firing_rate / regular_otolith_passband_gain;
        state_variable_luenberger_otolith[axis_index]->setDerivative(
            state, (reconstructed_regular_otolith_force - luen_mac_state) / reg_oto_lead_tau1);
        double luen_stage1_out =
            luen_macular_state_weight * luen_mac_state + luen_macular_feedthrough * reconstructed_regular_otolith_force;
        state_variable_luenberger_otolith_adapt[axis_index]->setDerivative(
            state, (luen_stage1_out - luen_adp_state) / luen_adapt_tau);
        current_luenberger_otolith_force_head[axis_index] =
            luen_adapt_state_weight * luen_adp_state + luen_stage1_out;
    }

    if (!is_initialized) {
        initializeGeometryAndBaseline(state);
    }
    {
        int number_of_active_muscles = active_muscle_indices.size();
        std::vector<double> estimated_fiber_length_changes(number_of_active_muscles, 0.0);
        std::vector<double> estimated_fiber_velocities(number_of_active_muscles, 0.0);
        const double dynamic_gamma_gain_scaled        = 100.0 * get_K_gamma_dyn();
        const double static_gamma_gain_scaled         = 100.0 * get_K_gamma_stat();
        const double length_observer_time_constant    = 0.010;
        const double secondary_afferent_time_constant = 0.055;
        const double velocity_observer_time_constant  = 0.005;
        for (int muscle_index = 0; muscle_index < number_of_active_muscles; ++muscle_index) {
            double spindle_length_change        = state_variable_spindle_length_change[muscle_index]->getValue(state);
            double spindle_velocity             = state_variable_spindle_velocity[muscle_index]->getValue(state);
            double dynamic_fusimotor_activation = state_variable_spindle_dynamic_fusimotor[muscle_index]->getValue(state);
            double static_fusimotor_activation  = state_variable_spindle_static_fusimotor[muscle_index]->getValue(state);

            double alpha_motor_excitation =
                (muscle_index < (int)last_alpha_motor_excitations.size()) ? last_alpha_motor_excitations[muscle_index] : 0.03;
            double dynamic_gamma_firing_rate = dynamic_gamma_gain_scaled * alpha_motor_excitation;
            double static_gamma_firing_rate  = static_gamma_gain_scaled  * alpha_motor_excitation;
            double steady_state_dynamic_fusimotor =
                (dynamic_gamma_firing_rate * dynamic_gamma_firing_rate) / (dynamic_gamma_firing_rate * dynamic_gamma_firing_rate + 3600.0);
            double steady_state_static_fusimotor =
                (static_gamma_firing_rate * static_gamma_firing_rate) / (static_gamma_firing_rate * static_gamma_firing_rate + 3600.0);
            double instantaneous_nuclear_chain_activation =
                (static_gamma_firing_rate * static_gamma_firing_rate) / (static_gamma_firing_rate * static_gamma_firing_rate + 8100.0);

            state_variable_spindle_dynamic_fusimotor[muscle_index]->setDerivative(
                state, (steady_state_dynamic_fusimotor - dynamic_fusimotor_activation) / 0.149);
            state_variable_spindle_static_fusimotor[muscle_index]->setDerivative(
                state, (steady_state_static_fusimotor - static_fusimotor_activation) / 0.205);

            double primary_afferent_value =
                cached_primary_afferent_outputs[muscle_index]
                    ? cached_primary_afferent_outputs[muscle_index]->getValue(state)
                    : baseline_primary_afferent_rates[muscle_index];
            double secondary_afferent_value =
                cached_secondary_afferent_outputs[muscle_index]
                    ? cached_secondary_afferent_outputs[muscle_index]->getValue(state)
                    : baseline_secondary_afferent_rates[muscle_index];

            double fusimotor_secondary_afferent_bias = 29.72138 * static_fusimotor_activation + 44.58207 * instantaneous_nuclear_chain_activation;
            double fusimotor_primary_afferent_bias   = 54.45176 * dynamic_fusimotor_activation + 0.21517 * fusimotor_secondary_afferent_bias;
            double net_secondary_afferent_change     = (secondary_afferent_value - baseline_secondary_afferent_rates[muscle_index]) - fusimotor_secondary_afferent_bias;
            double net_primary_afferent_change       = (primary_afferent_value   - baseline_primary_afferent_rates[muscle_index])   - fusimotor_primary_afferent_bias;

            double reconstructed_length_from_secondary = cached_fiber_length_scales[muscle_index] * net_secondary_afferent_change;
            double dynamic_primary_afferent_component  = net_primary_afferent_change - 1.37931034 * net_secondary_afferent_change;
            double velocity_from_length_error          = (reconstructed_length_from_secondary - spindle_length_change) / secondary_afferent_time_constant;
            double velocity_from_primary_afferent      = cached_primary_afferent_velocity_scales[muscle_index] * dynamic_primary_afferent_component;

            state_variable_spindle_length_change[muscle_index]->setDerivative(
                state, spindle_velocity + (reconstructed_length_from_secondary - spindle_length_change) / length_observer_time_constant);
            state_variable_spindle_velocity[muscle_index]->setDerivative(
                state, (velocity_from_length_error + velocity_from_primary_afferent - spindle_velocity) / velocity_observer_time_constant);

            estimated_fiber_length_changes[muscle_index] = spindle_length_change;
            estimated_fiber_velocities[muscle_index]     = spindle_velocity;
        }
        // Stage 2: Combine all 72 per-spindle Luenberger states via 6-DOF Cervical Jacobian Decoder
        if (proprioceptive_decoder_matrix.nrow() == 3 && proprioceptive_decoder_matrix.ncol() == number_of_active_muscles) {
            for (int row = 0; row < 3; ++row) {
                double sum_angle = 0.0;
                double sum_speed = 0.0;
                for (int muscle_index = 0; muscle_index < number_of_active_muscles; ++muscle_index) {
                    sum_angle += proprioceptive_decoder_matrix(row, muscle_index) * estimated_fiber_length_changes[muscle_index];
                    sum_speed += proprioceptive_decoder_matrix(row, muscle_index) * estimated_fiber_velocities[muscle_index];
                }
                estimated_neck_relative_angle[row] = sum_angle;
                estimated_neck_relative_speed[row] = sum_speed;
            }
        }
    }

    const double vestibular_afferent_delay = 0.002; // 2 ms CN VIII vestibular afferent delay to brainstem/cerebellum
    DelayedData delayed_vestibular_afferent_data = getDelayedData(state.getTime(), vestibular_afferent_delay);
    DelayedData delayed_vestibulo_collic_data    = getDelayedData(state.getTime(), get_delay_vcr());

    // --- SE_2(3) SPATIAL OBSERVER (Tilt-Translation Disambiguation) ---
    // 1. Vestibular Sensor Readings (2ms-delayed Canal Luenberger Gyro and 2ms-delayed Otolith Luenberger GIF from Subsystem 2)
    SimTK::Vec3 world_angular_velocity =
        (!history.empty() && delayed_vestibular_afferent_data.record.valid)
            ? delayed_vestibular_afferent_data.record.luenberger_angular_velocity
            : SimTK::Vec3(luenberger_angular_velocity_roll, luenberger_angular_velocity_yaw, luenberger_angular_velocity_pitch);
    SimTK::Vec3 otolith_specific_force_head =
        (!history.empty() && delayed_vestibular_afferent_data.record.valid)
            ? delayed_vestibular_afferent_data.record.luenberger_otolith_specific_force
            : current_luenberger_otolith_force_head; // Time-aligned at t - 2ms from Subsystem 2 Regular Otolith + Luenberger Inverse

    // 2. Current Observer Beliefs
    double quaternion_0 = state_variable_observer_quaternion[0]->getValue(state);
    double quaternion_1 = state_variable_observer_quaternion[1]->getValue(state);
    double quaternion_2 = state_variable_observer_quaternion[2]->getValue(state);
    double quaternion_3 = state_variable_observer_quaternion[3]->getValue(state);

    double observer_velocity_x = state_variable_observer_velocity[0]->getValue(state);
    double observer_velocity_y = state_variable_observer_velocity[1]->getValue(state);
    double observer_velocity_z = state_variable_observer_velocity[2]->getValue(state);
    SimTK::Vec3 estimated_linear_velocity(observer_velocity_x, observer_velocity_y, observer_velocity_z);

    double observer_position_x = state_variable_observer_position[0]->getValue(state);
    double observer_position_y = state_variable_observer_position[1]->getValue(state);
    double observer_position_z = state_variable_observer_position[2]->getValue(state);
    SimTK::Vec3 estimated_linear_position(observer_position_x, observer_position_y, observer_position_z);

    // Normalize quaternion
    double quaternion_norm = std::sqrt(
        quaternion_0 * quaternion_0 + quaternion_1 * quaternion_1 +
        quaternion_2 * quaternion_2 + quaternion_3 * quaternion_3);
    if (quaternion_norm > 1e-4) {
        quaternion_0 /= quaternion_norm;
        quaternion_1 /= quaternion_norm;
        quaternion_2 /= quaternion_norm;
        quaternion_3 /= quaternion_norm;
    }

    SimTK::Rotation estimated_rotation_head_to_world(SimTK::Quaternion(quaternion_0, quaternion_1, quaternion_2, quaternion_3));

    // 3. TILT CORRECTION (Mahony Otolith + Proprioceptive Multisensory Fusion)
    // Transform angular velocity from Ground frame into Head frame
    SimTK::Vec3 head_frame_angular_velocity = (~estimated_rotation_head_to_world) * world_angular_velocity;
    mahony_estimated_angular_velocity = SimTK::Vec3(
        head_frame_angular_velocity[2],
        head_frame_angular_velocity[0],
        head_frame_angular_velocity[1]);

    double estimated_head_pitch = std::atan2(
        2.0 * (quaternion_0 * quaternion_3 + quaternion_1 * quaternion_2),
        1.0 - 2.0 * (quaternion_2 * quaternion_2 + quaternion_3 * quaternion_3));
    double estimated_head_roll = std::atan2(
        2.0 * (quaternion_0 * quaternion_1 + quaternion_2 * quaternion_3),
        1.0 - 2.0 * (quaternion_1 * quaternion_1 + quaternion_2 * quaternion_2));
    double estimated_head_yaw = std::asin(
        std::clamp(2.0 * (quaternion_0 * quaternion_2 - quaternion_3 * quaternion_1), -1.0, 1.0));

    // --- CORTICAL / VOLUNTARY LOW-PASS FILTER DYNAMICS (ANATOMICALLY SPLIT ASCENDING DELAY + MAHONY FUSED FEEDBACK + 2 Hz ROLL-OFF) ---
    double ascending_voluntary_delay  = 0.5 * get_delay_voluntary(); // 20 ms ascending thalamocortical sensory delay
    double descending_voluntary_delay = 0.5 * get_delay_voluntary(); // 20 ms descending corticospinal motor delay
    DelayedData delayed_ascending_voluntary_data  = getDelayedData(state.getTime(), ascending_voluntary_delay);
    DelayedData delayed_descending_voluntary_data = getDelayedData(state.getTime(), descending_voluntary_delay);

    double delayed_mahony_pitch =
        (!history.empty() && delayed_ascending_voluntary_data.record.valid)
            ? (delayed_ascending_voluntary_data.record.estimated_head_orientation[2] - baseline_spatial_orientation[2])
            : (estimated_head_pitch - baseline_spatial_orientation[2]);
    double delayed_mahony_roll =
        (!history.empty() && delayed_ascending_voluntary_data.record.valid)
            ? (delayed_ascending_voluntary_data.record.estimated_head_orientation[0] - baseline_spatial_orientation[0])
            : (estimated_head_roll - baseline_spatial_orientation[0]);
    double delayed_mahony_yaw =
        (!history.empty() && delayed_ascending_voluntary_data.record.valid)
            ? (delayed_ascending_voluntary_data.record.estimated_head_orientation[1] - baseline_spatial_orientation[1])
            : (estimated_head_yaw - baseline_spatial_orientation[1]);

    double delayed_mahony_pitch_speed =
        (!history.empty() && delayed_ascending_voluntary_data.record.valid)
            ? delayed_ascending_voluntary_data.record.mahony_estimated_angular_velocity[0]
            : mahony_estimated_angular_velocity[0];
    double delayed_mahony_roll_speed =
        (!history.empty() && delayed_ascending_voluntary_data.record.valid)
            ? delayed_ascending_voluntary_data.record.mahony_estimated_angular_velocity[1]
            : mahony_estimated_angular_velocity[1];
    double delayed_mahony_yaw_speed =
        (!history.empty() && delayed_ascending_voluntary_data.record.valid)
            ? delayed_ascending_voluntary_data.record.mahony_estimated_angular_velocity[2]
            : mahony_estimated_angular_velocity[2];

    // PID Integrator derivatives (using 20ms-delayed Mahony-fused spatial orientation consistently)
    double ki_pitch = (get_Ki_task_pitch() >= 0.0) ? get_Ki_task_pitch() : get_Ki_task();
    double pitch_integral_limit = std::max(0.6, 25.0 / std::max(1.0, ki_pitch));
    double pitch_position_error = trajectory_position_pitch - delayed_mahony_pitch;
    double pitch_error_integral = state_variable_error_integral[0]->getValue(state);
    if (pitch_error_integral >  pitch_integral_limit && pitch_position_error > 0.0) pitch_position_error = 0.0;
    if (pitch_error_integral < -pitch_integral_limit && pitch_position_error < 0.0) pitch_position_error = 0.0;
    state_variable_error_integral[0]->setDerivative(state, pitch_position_error);

    double roll_position_error = trajectory_position_roll - delayed_mahony_roll;
    double roll_error_integral = state_variable_error_integral[1]->getValue(state);
    if (roll_error_integral >  0.4 && roll_position_error > 0.0) roll_position_error = 0.0;
    if (roll_error_integral < -0.4 && roll_position_error < 0.0) roll_position_error = 0.0;
    state_variable_error_integral[1]->setDerivative(state, roll_position_error);

    double yaw_position_error = trajectory_position_yaw - delayed_mahony_yaw;
    double yaw_error_integral = state_variable_error_integral[2]->getValue(state);
    if (yaw_error_integral >  0.4 && yaw_position_error > 0.0) yaw_position_error = 0.0;
    if (yaw_error_integral < -0.4 && yaw_position_error < 0.0) yaw_position_error = 0.0;
    state_variable_error_integral[2]->setDerivative(state, yaw_position_error);

    double head_inertia_pitch = 0.05;
    double head_inertia_roll  = 0.05;
    double head_inertia_yaw   = 0.05;

    double vigor_pitch = 1.0 + get_k_vigor() * std::abs(trajectory_velocity_pitch);
    double kp_pitch = (get_Kp_task_pitch() >= 0.0) ? get_Kp_task_pitch() : std::sqrt(get_Wp_base_pitch() * vigor_pitch);
    double kd_pitch = (get_Kd_task_pitch() >= 0.0) ? get_Kd_task_pitch() : std::sqrt(get_Wv_base_pitch() * vigor_pitch + 2.0 * kp_pitch);
    double ki_roll  = (get_Ki_task_roll()  >= 0.0) ? get_Ki_task_roll()  : 0.0;
    double ki_yaw   = (get_Ki_task_yaw()   >= 0.0) ? get_Ki_task_yaw()   : 0.0;

    double unfiltered_voluntary_torque_pitch =
        kp_pitch * (trajectory_position_pitch - delayed_mahony_pitch) +
        kd_pitch * (trajectory_velocity_pitch - delayed_mahony_pitch_speed) +
        ki_pitch * pitch_error_integral +
        head_inertia_pitch * trajectory_acceleration_pitch;

    double vigor_roll = 1.0 + get_k_vigor() * std::abs(trajectory_velocity_roll);
    double kp_roll = (get_Kp_task_roll() >= 0.0) ? get_Kp_task_roll() : std::sqrt(get_Wp_base_roll() * vigor_roll);
    double kd_roll = (get_Kd_task_roll() >= 0.0) ? get_Kd_task_roll() : std::sqrt(get_Wv_base_roll() * vigor_roll + 2.0 * kp_roll);
    double unfiltered_voluntary_torque_roll =
        kp_roll * (trajectory_position_roll - delayed_mahony_roll) +
        kd_roll * (trajectory_velocity_roll - delayed_mahony_roll_speed) +
        ki_roll * roll_error_integral +
        head_inertia_roll * trajectory_acceleration_roll;

    double vigor_yaw = 1.0 + get_k_vigor() * std::abs(trajectory_velocity_yaw);
    double kp_yaw = (get_Kp_task_yaw() >= 0.0) ? get_Kp_task_yaw() : std::sqrt(get_Wp_base_yaw() * vigor_yaw);
    double kd_yaw = (get_Kd_task_yaw() >= 0.0) ? get_Kd_task_yaw() : std::sqrt(get_Wv_base_yaw() * vigor_yaw + 2.0 * kp_yaw);
    double unfiltered_voluntary_torque_yaw =
        kp_yaw * (trajectory_position_yaw - delayed_mahony_yaw) +
        kd_yaw * (trajectory_velocity_yaw - delayed_mahony_yaw_speed) +
        ki_yaw * yaw_error_integral +
        head_inertia_yaw * trajectory_acceleration_yaw;

    double voluntary_cutoff_frequency = get_fc_voluntary();
    if (voluntary_cutoff_frequency < 0.1) voluntary_cutoff_frequency = 0.1;
    double voluntary_cutoff_angular_frequency = 2.0 * SimTK::Pi * voluntary_cutoff_frequency;

    double voluntary_filtered_stage1_pitch = state_variable_voluntary_filter_stage1[0]->getValue(state);
    double voluntary_filtered_stage1_roll  = state_variable_voluntary_filter_stage1[1]->getValue(state);
    double voluntary_filtered_stage1_yaw   = state_variable_voluntary_filter_stage1[2]->getValue(state);

    double voluntary_filtered_stage2_pitch = state_variable_voluntary_filter_stage2[0]->getValue(state);
    double voluntary_filtered_stage2_roll  = state_variable_voluntary_filter_stage2[1]->getValue(state);
    double voluntary_filtered_stage2_yaw   = state_variable_voluntary_filter_stage2[2]->getValue(state);

    state_variable_voluntary_filter_stage1[0]->setDerivative(
        state, voluntary_cutoff_angular_frequency * (unfiltered_voluntary_torque_pitch - voluntary_filtered_stage1_pitch));
    state_variable_voluntary_filter_stage1[1]->setDerivative(
        state, voluntary_cutoff_angular_frequency * (unfiltered_voluntary_torque_roll  - voluntary_filtered_stage1_roll));
    state_variable_voluntary_filter_stage1[2]->setDerivative(
        state, voluntary_cutoff_angular_frequency * (unfiltered_voluntary_torque_yaw   - voluntary_filtered_stage1_yaw));

    state_variable_voluntary_filter_stage2[0]->setDerivative(
        state, voluntary_cutoff_angular_frequency * (voluntary_filtered_stage1_pitch - voluntary_filtered_stage2_pitch));
    state_variable_voluntary_filter_stage2[1]->setDerivative(
        state, voluntary_cutoff_angular_frequency * (voluntary_filtered_stage1_roll  - voluntary_filtered_stage2_roll));
    state_variable_voluntary_filter_stage2[2]->setDerivative(
        state, voluntary_cutoff_angular_frequency * (voluntary_filtered_stage1_yaw   - voluntary_filtered_stage2_yaw));

    // Measured UP direction in Head frame from otolith
    SimTK::Vec3 measured_up_direction_head;
    if (otolith_specific_force_head.norm() > 1e-4) {
        measured_up_direction_head = otolith_specific_force_head.normalize();
    } else {
        measured_up_direction_head = (~estimated_rotation_head_to_world) * SimTK::Vec3(0.0, 1.0, 0.0);
    }

    // Estimated UP direction in Head frame at the matching 2ms delayed horizon (Otolith Smith Predictor)
    SimTK::Mat33 delayed_estimated_rotation_head_to_world =
        (get_use_smith_predictor() && !history.empty() && delayed_vestibular_afferent_data.record.valid)
            ? delayed_vestibular_afferent_data.record.estimated_rotation_head_to_world
            : estimated_rotation_head_to_world.asMat33();
    SimTK::Vec3 expected_up_direction_head = (~delayed_estimated_rotation_head_to_world) * SimTK::Vec3(0.0, 1.0, 0.0);

    // Otolith gravity error cross product in Head frame (Pitch & Roll)
    SimTK::Vec3 otolith_attitude_error_head = measured_up_direction_head % expected_up_direction_head;

    // 4. PROPRIOCEPTIVE INNOVATION (Head-on-Trunk Spindle Feedback)

    // --- ANATOMICALLY SPLIT PROPRIOCEPTIVE AFFERENT CONDUCTION ---
    // In human anatomy, spindle action potentials take ~5-8 ms (delay_afferent_proprio)
    // to travel along dorsal roots and cuneate fasciculus to the brainstem/cerebellum.
    double arriving_relative_neck_pitch = estimated_neck_relative_angle[0];
    double arriving_relative_neck_roll  = estimated_neck_relative_angle[1];
    double arriving_relative_neck_yaw   = estimated_neck_relative_angle[2];
    double arriving_trunk_pitch = angle_ground_pitch;
    double arriving_trunk_roll  = angle_ground_roll;
    double arriving_trunk_yaw   = angle_ground_yaw;

    double proprioceptive_afferent_delay = get_delay_afferent_proprio();
    DelayedData delayed_proprioceptive_afferent_data;
    if (proprioceptive_afferent_delay > 1e-5 && !history.empty()) {
        delayed_proprioceptive_afferent_data = getDelayedData(state.getTime(), proprioceptive_afferent_delay);
        if (delayed_proprioceptive_afferent_data.record.valid) {
            arriving_relative_neck_pitch = delayed_proprioceptive_afferent_data.record.estimated_neck_relative_angle[0];
            arriving_relative_neck_roll  = delayed_proprioceptive_afferent_data.record.estimated_neck_relative_angle[1];
            arriving_relative_neck_yaw   = delayed_proprioceptive_afferent_data.record.estimated_neck_relative_angle[2];
            arriving_trunk_pitch = delayed_proprioceptive_afferent_data.record.trunk_orientation[2];
            arriving_trunk_roll  = delayed_proprioceptive_afferent_data.record.trunk_orientation[0];
            arriving_trunk_yaw   = delayed_proprioceptive_afferent_data.record.trunk_orientation[1];
        }
    }

    double proprioceptive_head_in_world_pitch = arriving_trunk_pitch + arriving_relative_neck_pitch;
    double proprioceptive_head_in_world_roll  = arriving_trunk_roll  + arriving_relative_neck_roll;
    double proprioceptive_head_in_world_yaw   = arriving_trunk_yaw   + arriving_relative_neck_yaw;

    // Error vector in World frame [Roll=0, Yaw=1, Pitch=2]
    SimTK::Vec3 proprioceptive_attitude_error_world;
    if (get_use_smith_predictor() && !history.empty()) {
        // Smith Predictor / Cerebellar Forward Model:
        // Compares the arriving delayed measurement proprioceptive_head_in_world(t - proprioceptive_afferent_delay)
        // against the stored estimate from the EXACT SAME DELAYED HORIZON predicted_head(t - proprioceptive_afferent_delay)
        double smith_predictor_delay = get_delay_smith_proprio();
        DelayedData delayed_smith_prediction_data =
            (std::abs(smith_predictor_delay - proprioceptive_afferent_delay) < 1e-5 && delayed_proprioceptive_afferent_data.record.valid)
                ? delayed_proprioceptive_afferent_data
                : getDelayedData(state.getTime(), smith_predictor_delay);
        if (delayed_smith_prediction_data.record.valid) {
            double predicted_head_roll  = delayed_smith_prediction_data.record.estimated_head_orientation[0];
            double predicted_head_yaw   = delayed_smith_prediction_data.record.estimated_head_orientation[1];
            double predicted_head_pitch = delayed_smith_prediction_data.record.estimated_head_orientation[2];
            proprioceptive_attitude_error_world = SimTK::Vec3(
                proprioceptive_head_in_world_roll  - predicted_head_roll,
                proprioceptive_head_in_world_yaw   - predicted_head_yaw,
                proprioceptive_head_in_world_pitch - predicted_head_pitch);
        } else {
            proprioceptive_attitude_error_world = SimTK::Vec3(
                proprioceptive_head_in_world_roll  - estimated_head_roll,
                proprioceptive_head_in_world_yaw   - estimated_head_yaw,
                proprioceptive_head_in_world_pitch - estimated_head_pitch);
        }
    } else {
        // Original Mahony without delay compensation:
        // Compares arriving delayed measurement against current un-delayed estimate
        proprioceptive_attitude_error_world = SimTK::Vec3(
            proprioceptive_head_in_world_roll  - estimated_head_roll,
            proprioceptive_head_in_world_yaw   - estimated_head_yaw,
            proprioceptive_head_in_world_pitch - estimated_head_pitch);
    }
    // Rotate into Head frame
    SimTK::Vec3 proprioceptive_attitude_error_head = (~estimated_rotation_head_to_world) * proprioceptive_attitude_error_world;

    // Multisensory gains:
    // Otolith gravity tether (Kp_otolith_attitude ~ 0.05 rad/s so linear acceleration does not corrupt pitch/roll)
    double Kp_otolith_attitude = 0.05;
    // Proprioceptive gains: anchor low-frequency 3D attitude to Luenberger proprioceptive state
    double K_proprioceptive_attitude_roll  = 2.0;
    double K_proprioceptive_attitude_yaw   = 2.0;
    double K_proprioceptive_attitude_pitch = 2.0;
    SimTK::Vec3 weighted_proprioceptive_attitude_error(
        K_proprioceptive_attitude_roll  * proprioceptive_attitude_error_head[0],
        K_proprioceptive_attitude_yaw   * proprioceptive_attitude_error_head[1],
        K_proprioceptive_attitude_pitch * proprioceptive_attitude_error_head[2]);

    // Fused angular velocity for quaternion kinematics
    SimTK::Vec3 fused_angular_velocity_head =
        head_frame_angular_velocity +
        Kp_otolith_attitude * otolith_attitude_error_head +
        weighted_proprioceptive_attitude_error;

    double fused_angular_velocity_x = fused_angular_velocity_head[0];
    double fused_angular_velocity_y = fused_angular_velocity_head[1];
    double fused_angular_velocity_z = fused_angular_velocity_head[2];

    double quaternion_0_derivative = 0.5 * (-quaternion_1 * fused_angular_velocity_x - quaternion_2 * fused_angular_velocity_y - quaternion_3 * fused_angular_velocity_z);
    double quaternion_1_derivative = 0.5 * ( quaternion_0 * fused_angular_velocity_x + quaternion_2 * fused_angular_velocity_z - quaternion_3 * fused_angular_velocity_y);
    double quaternion_2_derivative = 0.5 * ( quaternion_0 * fused_angular_velocity_y - quaternion_1 * fused_angular_velocity_z + quaternion_3 * fused_angular_velocity_x);
    double quaternion_3_derivative = 0.5 * ( quaternion_0 * fused_angular_velocity_z + quaternion_1 * fused_angular_velocity_y - quaternion_2 * fused_angular_velocity_x);

    // 5. TRANSLATION DERIVATIVES
    // Reconstruct pure linear acceleration in WORLD frame: linear_acceleration_world = estimated_rotation_head_to_world * otolith_specific_force_head + gravity_vector_world
    SimTK::Vec3 linear_acceleration_world = (estimated_rotation_head_to_world * otolith_specific_force_head) + gravity_vector_world;

    // Biological Leaky Integrator (Velocity Storage & Egocentric tether)
    double velocity_leak_time_constant = 15.0; // Decay in dark
    double position_leak_time_constant = 5.0;

    SimTK::Vec3 linear_velocity_derivative = linear_acceleration_world - (1.0 / velocity_leak_time_constant) * estimated_linear_velocity;
    SimTK::Vec3 linear_position_derivative = estimated_linear_velocity - (1.0 / position_leak_time_constant) * estimated_linear_position;

    // Write to state derivatives
    state_variable_observer_quaternion[0]->setDerivative(state, quaternion_0_derivative);
    state_variable_observer_quaternion[1]->setDerivative(state, quaternion_1_derivative);
    state_variable_observer_quaternion[2]->setDerivative(state, quaternion_2_derivative);
    state_variable_observer_quaternion[3]->setDerivative(state, quaternion_3_derivative);

    state_variable_observer_velocity[0]->setDerivative(state, linear_velocity_derivative[0]);
    state_variable_observer_velocity[1]->setDerivative(state, linear_velocity_derivative[1]);
    state_variable_observer_velocity[2]->setDerivative(state, linear_velocity_derivative[2]);

    state_variable_observer_position[0]->setDerivative(state, linear_position_derivative[0]);
    state_variable_observer_position[1]->setDerivative(state, linear_position_derivative[1]);
    state_variable_observer_position[2]->setDerivative(state, linear_position_derivative[2]);

    // --- NEURAL TRANSFER FUNCTION DERIVATIVES ---
    double vestibulo_collic_efferent_delay = std::max(0.0, get_delay_vcr() - vestibular_afferent_delay); // 8 ms efferent MVST delay (2ms afferent + 8ms efferent = 10ms total)
    DelayedData delayed_vestibulo_collic_efferent_data = getDelayedData(state.getTime(), vestibulo_collic_efferent_delay);
    if (delayed_vestibulo_collic_data.record.valid) {
        const HistoryRecord& tonic_otolith_record =
            delayed_vestibulo_collic_efferent_data.record.valid
                ? delayed_vestibulo_collic_efferent_data.record
                : delayed_vestibulo_collic_data.record;
        double delayed_estimated_roll  = tonic_otolith_record.estimated_head_orientation[0];
        double delayed_estimated_yaw   = tonic_otolith_record.estimated_head_orientation[1];
        double delayed_estimated_pitch = tonic_otolith_record.estimated_head_orientation[2];

        double descending_traj_pos_pitch = (!history.empty() && delayed_descending_voluntary_data.record.valid) ? delayed_descending_voluntary_data.record.trajectory_position[0] : trajectory_position_pitch;
        double descending_traj_pos_roll  = (!history.empty() && delayed_descending_voluntary_data.record.valid) ? delayed_descending_voluntary_data.record.trajectory_position[1] : trajectory_position_roll;
        double descending_traj_pos_yaw   = (!history.empty() && delayed_descending_voluntary_data.record.valid) ? delayed_descending_voluntary_data.record.trajectory_position[2] : trajectory_position_yaw;
        double descending_traj_vel_pitch = (!history.empty() && delayed_descending_voluntary_data.record.valid) ? delayed_descending_voluntary_data.record.trajectory_velocity[0] : trajectory_velocity_pitch;
        double descending_traj_vel_roll  = (!history.empty() && delayed_descending_voluntary_data.record.valid) ? delayed_descending_voluntary_data.record.trajectory_velocity[1] : trajectory_velocity_roll;
        double descending_traj_vel_yaw   = (!history.empty() && delayed_descending_voluntary_data.record.valid) ? delayed_descending_voluntary_data.record.trajectory_velocity[2] : trajectory_velocity_yaw;
        double descending_traj_acc_pitch = (!history.empty() && delayed_descending_voluntary_data.record.valid) ? delayed_descending_voluntary_data.record.trajectory_acceleration[0] : trajectory_acceleration_pitch;
        double descending_traj_acc_roll  = (!history.empty() && delayed_descending_voluntary_data.record.valid) ? delayed_descending_voluntary_data.record.trajectory_acceleration[1] : trajectory_acceleration_roll;

        double tonic_otolith_input_pitch = (delayed_estimated_pitch - baseline_spatial_orientation[2]) - descending_traj_pos_pitch;
        double tonic_otolith_input_roll  = (delayed_estimated_roll  - baseline_spatial_orientation[0]) - descending_traj_pos_roll;
        double tonic_otolith_input_yaw   = (delayed_estimated_yaw   - baseline_spatial_orientation[1]) - descending_traj_pos_yaw;

        double semicircular_canal_input_pitch = delayed_vestibulo_collic_data.record.reconstructed_canal_angular_velocity[2] - descending_traj_vel_pitch;
        double semicircular_canal_input_roll  = delayed_vestibulo_collic_data.record.reconstructed_canal_angular_velocity[0] - descending_traj_vel_roll;
        double semicircular_canal_input_yaw   = delayed_vestibulo_collic_data.record.reconstructed_canal_angular_velocity[1] - descending_traj_vel_yaw;

        SimTK::Vec3 delayed_otolith_specific_force_head =
            delayed_vestibulo_collic_data.record.reconstructed_otolith_specific_force;
        SimTK::Vec3 estimated_linear_acceleration_head =
            delayed_otolith_specific_force_head + ((~tonic_otolith_record.estimated_rotation_head_to_world) * gravity_vector_world);

        double head_pivot_radius = 0.15;
        double expected_tangential_acceleration_x = descending_traj_acc_pitch * head_pivot_radius;
        double expected_tangential_acceleration_z = descending_traj_acc_roll  * head_pivot_radius;

        double phasic_otolith_input_pitch = estimated_linear_acceleration_head[0] - expected_tangential_acceleration_x;
        double phasic_otolith_input_roll  = estimated_linear_acceleration_head[2] - expected_tangential_acceleration_z;
        double phasic_otolith_input_yaw   = 0.0;

        double tonic_otolith_state_pitch = state_variable_tonic_otolith[0]->getValue(state);
        double tonic_otolith_state_roll  = state_variable_tonic_otolith[1]->getValue(state);
        double tonic_otolith_state_yaw   = state_variable_tonic_otolith[2]->getValue(state);

        double semicircular_canal_state_pitch = state_variable_semicircular_canal_reflex[0]->getValue(state);
        double semicircular_canal_state_roll  = state_variable_semicircular_canal_reflex[1]->getValue(state);
        double semicircular_canal_state_yaw   = state_variable_semicircular_canal_reflex[2]->getValue(state);

        double phasic_otolith_state_pitch = state_variable_phasic_otolith[0]->getValue(state);
        double phasic_otolith_state_roll  = state_variable_phasic_otolith[1]->getValue(state);
        double phasic_otolith_state_yaw   = state_variable_phasic_otolith[2]->getValue(state);

        double phasic_otolith_stage2_state_pitch = state_variable_phasic_otolith_stage2[0]->getValue(state);
        double phasic_otolith_stage2_state_roll  = state_variable_phasic_otolith_stage2[1]->getValue(state);
        double phasic_otolith_stage2_state_yaw   = state_variable_phasic_otolith_stage2[2]->getValue(state);

        state_variable_tonic_otolith[0]->setDerivative(state, -get_p_ton() * tonic_otolith_state_pitch + tonic_otolith_input_pitch);
        state_variable_tonic_otolith[1]->setDerivative(state, -get_p_ton() * tonic_otolith_state_roll  + tonic_otolith_input_roll);
        state_variable_tonic_otolith[2]->setDerivative(state, -get_p_ton() * tonic_otolith_state_yaw   + tonic_otolith_input_yaw);

        state_variable_semicircular_canal_reflex[0]->setDerivative(state, -get_p_sc() * semicircular_canal_state_pitch + semicircular_canal_input_pitch);
        state_variable_semicircular_canal_reflex[1]->setDerivative(state, -get_p_sc() * semicircular_canal_state_roll  + semicircular_canal_input_roll);
        state_variable_semicircular_canal_reflex[2]->setDerivative(state, -get_p_sc() * semicircular_canal_state_yaw   + semicircular_canal_input_yaw);

        state_variable_phasic_otolith[0]->setDerivative(state, -get_p_phas() * phasic_otolith_state_pitch + phasic_otolith_input_pitch);
        state_variable_phasic_otolith[1]->setDerivative(state, -get_p_phas() * phasic_otolith_state_roll  + phasic_otolith_input_roll);
        state_variable_phasic_otolith[2]->setDerivative(state, -get_p_phas() * phasic_otolith_state_yaw   + phasic_otolith_input_yaw);

        double phasic_otolith_zero = get_z_phas();
        double phasic_otolith_pole = get_p_phas();
        double second_phasic_otolith_pole = get_p_phas2();
        if (phasic_otolith_zero > 0.0) {
            state_variable_phasic_otolith_stage2[0]->setDerivative(state, -phasic_otolith_pole * phasic_otolith_stage2_state_pitch + phasic_otolith_state_pitch);
            state_variable_phasic_otolith_stage2[1]->setDerivative(state, -phasic_otolith_pole * phasic_otolith_stage2_state_roll  + phasic_otolith_state_roll);
            state_variable_phasic_otolith_stage2[2]->setDerivative(state, -phasic_otolith_pole * phasic_otolith_stage2_state_yaw   + phasic_otolith_state_yaw);
        } else if (second_phasic_otolith_pole > 0.0) {
            double phasic_otolith_stage1_output_pitch = phasic_otolith_pole * phasic_otolith_state_pitch;
            double phasic_otolith_stage1_output_roll  = phasic_otolith_pole * phasic_otolith_state_roll;
            double phasic_otolith_stage1_output_yaw   = phasic_otolith_pole * phasic_otolith_state_yaw;
            state_variable_phasic_otolith_stage2[0]->setDerivative(state, -second_phasic_otolith_pole * phasic_otolith_stage2_state_pitch + phasic_otolith_stage1_output_pitch);
            state_variable_phasic_otolith_stage2[1]->setDerivative(state, -second_phasic_otolith_pole * phasic_otolith_stage2_state_roll  + phasic_otolith_stage1_output_roll);
            state_variable_phasic_otolith_stage2[2]->setDerivative(state, -second_phasic_otolith_pole * phasic_otolith_stage2_state_yaw   + phasic_otolith_stage1_output_yaw);
        } else {
            state_variable_phasic_otolith_stage2[0]->setDerivative(state, 0.0);
            state_variable_phasic_otolith_stage2[1]->setDerivative(state, 0.0);
            state_variable_phasic_otolith_stage2[2]->setDerivative(state, 0.0);
        }
    } else {
        state_variable_tonic_otolith[0]->setDerivative(state, 0.0);
        state_variable_tonic_otolith[1]->setDerivative(state, 0.0);
        state_variable_tonic_otolith[2]->setDerivative(state, 0.0);
        state_variable_semicircular_canal_reflex[0]->setDerivative(state, 0.0);
        state_variable_semicircular_canal_reflex[1]->setDerivative(state, 0.0);
        state_variable_semicircular_canal_reflex[2]->setDerivative(state, 0.0);
        state_variable_phasic_otolith[0]->setDerivative(state, 0.0);
        state_variable_phasic_otolith[1]->setDerivative(state, 0.0);
        state_variable_phasic_otolith[2]->setDerivative(state, 0.0);
        state_variable_phasic_otolith_stage2[0]->setDerivative(state, 0.0);
        state_variable_phasic_otolith_stage2[1]->setDerivative(state, 0.0);
        state_variable_phasic_otolith_stage2[2]->setDerivative(state, 0.0);
    }

    double p_ccr = get_p_ccr();
    DelayedData delayed_cervico_collic_data = getDelayedData(state.getTime(), get_delay_ccr(), true);
    const HistoryRecord& cervico_collic_record = delayed_cervico_collic_data.record;
    if (cervico_collic_record.primary_afferent_firing_rate.size() == active_muscle_indices.size() && active_muscle_indices.size() > 0) {
        for (size_t muscle_index = 0; muscle_index < active_muscle_indices.size(); ++muscle_index) {
            double cervico_collic_state = state_variable_cervico_collic[muscle_index]->getValue(state);
            double delayed_primary_afferent_rate = cervico_collic_record.primary_afferent_firing_rate[muscle_index];
            state_variable_cervico_collic[muscle_index]->setDerivative(state, -p_ccr * cervico_collic_state + delayed_primary_afferent_rate);
        }
    } else {
        for (size_t muscle_index = 0; muscle_index < active_muscle_indices.size(); ++muscle_index) {
            state_variable_cervico_collic[muscle_index]->setDerivative(state, 0.0);
        }
    }
}

void HeadNeckNeuralController::computeControls(const SimTK::State& state, SimTK::Vector& controls) const {
    if (!is_initialized) {
        initializeGeometryAndBaseline(state);
    }
    double trajectory_time_constant = get_trajectory_tau();
    if (trajectory_time_constant < 1e-4) trajectory_time_constant = 1e-4;

    double trajectory_position_pitch = state_variable_trajectory_position[0]->getValue(state);
    double trajectory_velocity_pitch = state_variable_trajectory_velocity[0]->getValue(state);
    double trajectory_acceleration_pitch =
        (1.0 / (trajectory_time_constant * trajectory_time_constant)) * (get_desired_pitch() - trajectory_position_pitch) -
        (2.0 / trajectory_time_constant) * trajectory_velocity_pitch;

    double trajectory_position_roll = state_variable_trajectory_position[1]->getValue(state);
    double trajectory_velocity_roll = state_variable_trajectory_velocity[1]->getValue(state);
    double trajectory_acceleration_roll =
        (1.0 / (trajectory_time_constant * trajectory_time_constant)) * (get_desired_roll() - trajectory_position_roll) -
        (2.0 / trajectory_time_constant) * trajectory_velocity_roll;

    double trajectory_position_yaw = state_variable_trajectory_position[2]->getValue(state);
    double trajectory_velocity_yaw = state_variable_trajectory_velocity[2]->getValue(state);
    double trajectory_acceleration_yaw =
        (1.0 / (trajectory_time_constant * trajectory_time_constant)) * (get_desired_yaw() - trajectory_position_yaw) -
        (2.0 / trajectory_time_constant) * trajectory_velocity_yaw;

    const Model& model = getModel();
    const auto& muscles = model.getMuscles();
    int number_of_active_muscles = active_muscle_indices.size();

    // 1. Update Jacobian slowly
    if (state.getTime() - last_jacobian_update_time > 0.05) {
        for (int muscle_index = 0; muscle_index < number_of_active_muscles; ++muscle_index) {
            const Muscle& muscle = muscles.get(active_muscle_indices[muscle_index]);
            double moment_arm_pitch1 = cached_coordinate_pitch1 ? muscle.computeMomentArm(state, *const_cast<Coordinate*>(cached_coordinate_pitch1)) : 0.0;
            double moment_arm_pitch2 = cached_coordinate_pitch2 ? muscle.computeMomentArm(state, *const_cast<Coordinate*>(cached_coordinate_pitch2)) : 0.0;
            double moment_arm_roll1  = cached_coordinate_roll1  ? muscle.computeMomentArm(state, *const_cast<Coordinate*>(cached_coordinate_roll1))  : 0.0;
            double moment_arm_roll2  = cached_coordinate_roll2  ? muscle.computeMomentArm(state, *const_cast<Coordinate*>(cached_coordinate_roll2))  : 0.0;
            double moment_arm_yaw1   = cached_coordinate_yaw1   ? muscle.computeMomentArm(state, *const_cast<Coordinate*>(cached_coordinate_yaw1))   : 0.0;
            double moment_arm_yaw2   = cached_coordinate_yaw2   ? muscle.computeMomentArm(state, *const_cast<Coordinate*>(cached_coordinate_yaw2))   : 0.0;

            moment_arm_jacobian_3dof(0, muscle_index) = (moment_arm_pitch1 + moment_arm_pitch2) / 2.0;
            moment_arm_jacobian_3dof(1, muscle_index) = (moment_arm_roll1  + moment_arm_roll2)  / 2.0;
            moment_arm_jacobian_3dof(2, muscle_index) = (moment_arm_yaw1   + moment_arm_yaw2)   / 2.0;
            moment_arm_jacobian_6dof(0, muscle_index) = moment_arm_pitch1;
            moment_arm_jacobian_6dof(1, muscle_index) = moment_arm_pitch2;
            moment_arm_jacobian_6dof(2, muscle_index) = moment_arm_roll1;
            moment_arm_jacobian_6dof(3, muscle_index) = moment_arm_roll2;
            moment_arm_jacobian_6dof(4, muscle_index) = moment_arm_yaw1;
            moment_arm_jacobian_6dof(5, muscle_index) = moment_arm_yaw2;
        }
        updateDecoderMatrix();
        last_jacobian_update_time = state.getTime();
    }

    double speed_pitch1 = cached_coordinate_pitch1 ? cached_coordinate_pitch1->getSpeedValue(state) : 0.0;
    double speed_pitch2 = cached_coordinate_pitch2 ? cached_coordinate_pitch2->getSpeedValue(state) : 0.0;
    double speed_roll1  = cached_coordinate_roll1  ? cached_coordinate_roll1->getSpeedValue(state)  : 0.0;
    double speed_roll2  = cached_coordinate_roll2  ? cached_coordinate_roll2->getSpeedValue(state)  : 0.0;
    double speed_yaw1   = cached_coordinate_yaw1   ? cached_coordinate_yaw1->getSpeedValue(state)   : 0.0;
    double speed_yaw2   = cached_coordinate_yaw2   ? cached_coordinate_yaw2->getSpeedValue(state)   : 0.0;

    double angle_ground_pitch = cached_coordinate_ground_pitch ? cached_coordinate_ground_pitch->getValue(state) : 0.0;
    double angle_ground_roll  = cached_coordinate_ground_roll  ? cached_coordinate_ground_roll->getValue(state)  : 0.0;
    double angle_ground_yaw   = cached_coordinate_ground_yaw   ? cached_coordinate_ground_yaw->getValue(state)   : 0.0;
    if (std::abs(angle_ground_pitch) < 1e-12) angle_ground_pitch = 0.0;
    if (std::abs(angle_ground_roll)  < 1e-12) angle_ground_roll  = 0.0;
    if (std::abs(angle_ground_yaw)   < 1e-12) angle_ground_yaw   = 0.0;

    double speed_ground_pitch = cached_coordinate_ground_pitch ? cached_coordinate_ground_pitch->getSpeedValue(state) : 0.0;
    double speed_ground_roll  = cached_coordinate_ground_roll  ? cached_coordinate_ground_roll->getSpeedValue(state)  : 0.0;
    double speed_ground_yaw   = cached_coordinate_ground_yaw   ? cached_coordinate_ground_yaw->getSpeedValue(state)   : 0.0;
    if (std::abs(speed_ground_pitch) < 1e-12) speed_ground_pitch = 0.0;
    if (std::abs(speed_ground_roll)  < 1e-12) speed_ground_roll  = 0.0;
    if (std::abs(speed_ground_yaw)   < 1e-12) speed_ground_yaw   = 0.0;

    double head_pitch_dot_global = (speed_pitch1 + speed_pitch2) + speed_ground_pitch;
    double head_roll_dot_global  = (speed_roll1  + speed_roll2)  + speed_ground_roll;
    double head_yaw_dot_global   = (speed_yaw1   + speed_yaw2)   + speed_ground_yaw;

    // --- VESTIBULAR RECONSTRUCTION AND HISTORY ---
    SimTK::Vec3 head_global_angular_velocity(head_roll_dot_global, head_yaw_dot_global, head_pitch_dot_global);
    SimTK::Vec3 right_canal_projection = right_canal_projection_matrix * head_global_angular_velocity;
    SimTK::Vec3 left_canal_projection  = left_canal_projection_matrix  * head_global_angular_velocity;
    double canal_angular_velocity_inputs[6] = {
        right_canal_projection[0], right_canal_projection[1], right_canal_projection[2],
        left_canal_projection[0],  left_canal_projection[1],  left_canal_projection[2]};

    double regular_canal_afferent_outputs[6]   = {0.0};
    double irregular_canal_afferent_outputs[6] = {0.0};
    for (int canal_index = 0; canal_index < 6; ++canal_index) {
        const int regular_filter_index = 0;
        double reg_state_1 = state_variable_semicircular_canal[canal_index][regular_filter_index][0]->getValue(state);
        double reg_state_2 = state_variable_semicircular_canal[canal_index][regular_filter_index][1]->getValue(state);
        regular_canal_afferent_outputs[canal_index] =
            canal_filter_parameters[regular_filter_index].state_space_output_c0 * reg_state_1 +
            canal_filter_parameters[regular_filter_index].state_space_output_c1 * reg_state_2 +
            canal_filter_parameters[regular_filter_index].direct_feedthrough_d  * canal_angular_velocity_inputs[canal_index];

        const int irregular_filter_index = 1;
        double irr_state_1 = state_variable_semicircular_canal[canal_index][irregular_filter_index][0]->getValue(state);
        double irr_state_2 = state_variable_semicircular_canal[canal_index][irregular_filter_index][1]->getValue(state);
        irregular_canal_afferent_outputs[canal_index] =
            canal_filter_parameters[irregular_filter_index].state_space_output_c0 * irr_state_1 +
            canal_filter_parameters[irregular_filter_index].state_space_output_c1 * irr_state_2 +
            canal_filter_parameters[irregular_filter_index].direct_feedthrough_d  * canal_angular_velocity_inputs[canal_index];
    }

    SimTK::Vec3 filtered_right_regular_canal_output(regular_canal_afferent_outputs[0], regular_canal_afferent_outputs[1], regular_canal_afferent_outputs[2]);
    SimTK::Vec3 filtered_left_regular_canal_output(regular_canal_afferent_outputs[3], regular_canal_afferent_outputs[4], regular_canal_afferent_outputs[5]);
    const double regular_canal_passband_gain      = 0.436629; // G_pass,reg = 2.83 * (0.0027 / 0.0175)
    const double regular_canal_lead_time_constant = 0.0175;
    const double regular_canal_pole_time_constant = 0.0027;
    const double luenberger_direct_feedthrough    = regular_canal_pole_time_constant / regular_canal_lead_time_constant; // 0.154286
    const double luenberger_state_weight          = 1.0 - luenberger_direct_feedthrough;                                 // 0.845714
    SimTK::Vec3 reconstructed_regular_canal_velocity =
        (0.5 / regular_canal_passband_gain) * (right_canal_inverse_matrix * filtered_right_regular_canal_output + left_canal_inverse_matrix * filtered_left_regular_canal_output);

    SimTK::Vec3 filtered_right_irregular_canal_output(irregular_canal_afferent_outputs[0], irregular_canal_afferent_outputs[1], irregular_canal_afferent_outputs[2]);
    SimTK::Vec3 filtered_left_irregular_canal_output(irregular_canal_afferent_outputs[3], irregular_canal_afferent_outputs[4], irregular_canal_afferent_outputs[5]);
    const double irregular_canal_passband_gain = 0.541800; // G_pass,irr = 27.09 * (0.0006 / 0.0300)
    SimTK::Vec3 reconstructed_canal_angular_velocity =
        (0.5 / irregular_canal_passband_gain) * (right_canal_inverse_matrix * filtered_right_irregular_canal_output + left_canal_inverse_matrix * filtered_left_irregular_canal_output);

    // Observer estimated attitude for vestibular history buffer
    double quaternion_0 = state_variable_observer_quaternion[0]->getValue(state);
    double quaternion_1 = state_variable_observer_quaternion[1]->getValue(state);
    double quaternion_2 = state_variable_observer_quaternion[2]->getValue(state);
    double quaternion_3 = state_variable_observer_quaternion[3]->getValue(state);
    double quaternion_norm = std::sqrt(
        quaternion_0 * quaternion_0 + quaternion_1 * quaternion_1 +
        quaternion_2 * quaternion_2 + quaternion_3 * quaternion_3);
    if (quaternion_norm > 1e-4) {
        quaternion_0 /= quaternion_norm;
        quaternion_1 /= quaternion_norm;
        quaternion_2 /= quaternion_norm;
        quaternion_3 /= quaternion_norm;
    }
    double estimated_head_roll = std::atan2(
        2.0 * (quaternion_0 * quaternion_1 + quaternion_2 * quaternion_3),
        1.0 - 2.0 * (quaternion_1 * quaternion_1 + quaternion_2 * quaternion_2));
    double estimated_head_yaw = std::asin(
        std::clamp(2.0 * (quaternion_0 * quaternion_2 - quaternion_3 * quaternion_1), -1.0, 1.0));
    double estimated_head_pitch = std::atan2(
        2.0 * (quaternion_0 * quaternion_3 + quaternion_1 * quaternion_2),
        1.0 - 2.0 * (quaternion_2 * quaternion_2 + quaternion_3 * quaternion_3));
    mahony_estimated_angle = SimTK::Vec3(estimated_head_pitch, estimated_head_roll, estimated_head_yaw);
    SimTK::Rotation current_estimated_rotation_head_to_world(SimTK::Quaternion(quaternion_0, quaternion_1, quaternion_2, quaternion_3));
    SimTK::Vec3 luenberger_state_vector(
        state_variable_luenberger_angular_velocity[0]->getValue(state),
        state_variable_luenberger_angular_velocity[1]->getValue(state),
        state_variable_luenberger_angular_velocity[2]->getValue(state));
    SimTK::Vec3 current_world_angular_velocity =
        luenberger_state_weight * luenberger_state_vector + luenberger_direct_feedthrough * reconstructed_regular_canal_velocity;
    SimTK::Vec3 current_head_frame_angular_velocity = (~current_estimated_rotation_head_to_world) * current_world_angular_velocity;
    mahony_estimated_angular_velocity = SimTK::Vec3(
        current_head_frame_angular_velocity[2],
        current_head_frame_angular_velocity[0],
        current_head_frame_angular_velocity[1]); // pitch, roll, yaw
    SimTK::Vec3 current_cortical_voluntary_torque(
        state_variable_voluntary_filter_stage1[0]->getValue(state),
        state_variable_voluntary_filter_stage1[1]->getValue(state),
        state_variable_voluntary_filter_stage1[2]->getValue(state));

    // --- SUBSYSTEM 2 OTOLITH RECONSTRUCTION FOR HISTORY BUFFER (ADJUSTED SCHNEIDER 2015 / HAPPEE 2017) ---
    SimTK::Rotation current_rotation_world_to_head = cached_skull->getMobilizedBody().getBodyTransform(state).R().invert();
    SimTK::Vec3 gravity_vector_world_hist(0.0, -9.81, 0.0);
    DelayedData current_otolith_kinematics = getDelayedData(state.getTime(), 0.0005);
    SimTK::Vec3 current_gravitoinertial_force_head =
        current_rotation_world_to_head * (current_otolith_kinematics.linear_acceleration - gravity_vector_world_hist);

    const double reg_oto_adapt_ratio        = 37.4 / 66.0;   // 0.566667
    const double irr_oto_adapt_ratio        = 37.4 / 245.0;  // 0.152653
    const double reg_oto_adapt_tau          = 1.0000;
    const double reg_oto_lead_tau1          = 0.0276;
    const double reg_oto_pole_b             = 0.0175;
    const double irr_oto_lead_tau1          = 0.0272;
    const double irr_oto_pole_b             = 0.0070;
    const double regular_otolith_passband_gain   = 66.0 / 9.81;  // 6.727829 (imp/s)/(m/s^2)
    const double irregular_otolith_passband_gain = 245.0 / 9.81; // 24.974516 (imp/s)/(m/s^2)
    const double reg_macular_feedthrough    = reg_oto_lead_tau1 / reg_oto_pole_b;
    const double reg_macular_state_weight   = 1.0 - reg_macular_feedthrough;
    const double irr_macular_feedthrough    = irr_oto_lead_tau1 / irr_oto_pole_b;
    const double irr_macular_state_weight   = 1.0 - irr_macular_feedthrough;
    const double luen_macular_feedthrough   = reg_oto_pole_b / reg_oto_lead_tau1;
    const double luen_macular_state_weight  = 1.0 - luen_macular_feedthrough;
    const double luen_adapt_state_weight    = (1.0 / reg_oto_adapt_ratio) - 1.0;

    SimTK::Vec3 reconstructed_otolith_specific_force(0.0);
    SimTK::Vec3 luenberger_otolith_specific_force(0.0);
    for (int axis_index = 0; axis_index < 3; ++axis_index) {
        double u_gif           = current_gravitoinertial_force_head[axis_index];
        double reg_adapt_state = state_variable_regular_otolith_adapt[axis_index]->getValue(state);
        double reg_mac_state   = state_variable_regular_otolith[axis_index]->getValue(state);
        double irr_adapt_state = state_variable_irregular_otolith_adapt[axis_index]->getValue(state);
        double irr_mac_state   = state_variable_irregular_otolith[axis_index]->getValue(state);
        double luen_mac_state  = state_variable_luenberger_otolith[axis_index]->getValue(state);
        double luen_adp_state  = state_variable_luenberger_otolith_adapt[axis_index]->getValue(state);

        double irr_stage1_out = (irr_oto_adapt_ratio - 1.0) * irr_adapt_state + u_gif;
        double irregular_otolith_firing_rate =
            irregular_otolith_passband_gain * (irr_macular_state_weight * irr_mac_state + irr_macular_feedthrough * irr_stage1_out);
        reconstructed_otolith_specific_force[axis_index] = irregular_otolith_firing_rate / irregular_otolith_passband_gain;

        double reg_stage1_out = (reg_oto_adapt_ratio - 1.0) * reg_adapt_state + u_gif;
        double regular_otolith_firing_rate =
            regular_otolith_passband_gain * (reg_macular_state_weight * reg_mac_state + reg_macular_feedthrough * reg_stage1_out);
        double reconstructed_regular_otolith_force = regular_otolith_firing_rate / regular_otolith_passband_gain;
        double luen_stage1_out =
            luen_macular_state_weight * luen_mac_state + luen_macular_feedthrough * reconstructed_regular_otolith_force;
        luenberger_otolith_specific_force[axis_index] =
            luen_adapt_state_weight * luen_adp_state + luen_stage1_out;
    }

    HistoryRecord current_history_record;
    current_history_record.valid = true;
    current_history_record.estimated_head_orientation = SimTK::Vec3(estimated_head_roll, estimated_head_yaw, estimated_head_pitch);
    current_history_record.reconstructed_canal_angular_velocity = reconstructed_canal_angular_velocity;
    current_history_record.luenberger_angular_velocity = current_world_angular_velocity;
    current_history_record.reconstructed_otolith_specific_force = reconstructed_otolith_specific_force;
    current_history_record.luenberger_otolith_specific_force = luenberger_otolith_specific_force;
    current_history_record.mahony_estimated_angular_velocity = mahony_estimated_angular_velocity;
    current_history_record.cortical_voluntary_torque = current_cortical_voluntary_torque;
    current_history_record.trajectory_position = SimTK::Vec3(trajectory_position_pitch, trajectory_position_roll, trajectory_position_yaw);
    current_history_record.trajectory_velocity = SimTK::Vec3(trajectory_velocity_pitch, trajectory_velocity_roll, trajectory_velocity_yaw);
    current_history_record.trajectory_acceleration = SimTK::Vec3(trajectory_acceleration_pitch, trajectory_acceleration_roll, trajectory_acceleration_yaw);
    current_history_record.rotation_world_to_head = cached_skull->getMobilizedBody().getBodyTransform(state).R().invert().asMat33();
    current_history_record.estimated_rotation_head_to_world = current_estimated_rotation_head_to_world.asMat33();
    current_history_record.trunk_orientation = SimTK::Vec3(angle_ground_roll, angle_ground_yaw, angle_ground_pitch);
    current_history_record.trunk_angular_velocity = SimTK::Vec3(speed_ground_roll, speed_ground_yaw, speed_ground_pitch);
    current_history_record.head_linear_velocity = cached_skull->getVelocityInGround(state)[1];

    if (history.empty()) {
        for (int muscle_index = 0; muscle_index < number_of_active_muscles; ++muscle_index) {
            baseline_primary_afferent_rates[muscle_index] =
                cached_primary_afferent_outputs[muscle_index] ? cached_primary_afferent_outputs[muscle_index]->getValue(state) : 0.0;
            baseline_secondary_afferent_rates[muscle_index] =
                cached_secondary_afferent_outputs[muscle_index] ? cached_secondary_afferent_outputs[muscle_index]->getValue(state) : 0.0;
        }
    }
    current_history_record.primary_afferent_firing_rate.resize(number_of_active_muscles);
    std::vector<double> estimated_fiber_length_changes(number_of_active_muscles, 0.0);
    std::vector<double> estimated_fiber_velocities(number_of_active_muscles, 0.0);
    for (int muscle_index = 0; muscle_index < number_of_active_muscles; ++muscle_index) {
        estimated_fiber_length_changes[muscle_index] = state_variable_spindle_length_change[muscle_index]->getValue(state);
        estimated_fiber_velocities[muscle_index]     = state_variable_spindle_velocity[muscle_index]->getValue(state);
        double primary_afferent_value =
            cached_primary_afferent_outputs[muscle_index]
                ? cached_primary_afferent_outputs[muscle_index]->getValue(state)
                : baseline_primary_afferent_rates[muscle_index];
        double primary_afferent_above_baseline = std::max(0.0, primary_afferent_value - baseline_primary_afferent_rates[muscle_index]);
        current_history_record.primary_afferent_firing_rate[muscle_index] = std::clamp(primary_afferent_above_baseline, 0.0, 500.0);
    }

    // --- STAGE 2: 6-DOF PROPRIOCEPTIVE DECODER FROM 72 PER-SPINDLE LUENBERGER STATES ---
    if (proprioceptive_decoder_matrix.nrow() == 3 && proprioceptive_decoder_matrix.ncol() == number_of_active_muscles) {
        for (int row = 0; row < 3; ++row) {
            double sum_angle = 0.0;
            double sum_speed = 0.0;
            for (int muscle_index = 0; muscle_index < number_of_active_muscles; ++muscle_index) {
                sum_angle += proprioceptive_decoder_matrix(row, muscle_index) * estimated_fiber_length_changes[muscle_index];
                sum_speed += proprioceptive_decoder_matrix(row, muscle_index) * estimated_fiber_velocities[muscle_index];
            }
            estimated_neck_relative_angle[row] = sum_angle;
            estimated_neck_relative_speed[row] = sum_speed;
        }
    }
    current_history_record.estimated_neck_relative_angle = estimated_neck_relative_angle;
    current_history_record.estimated_neck_relative_speed = estimated_neck_relative_speed;

    double current_simulation_time = state.getTime();
    while (!history.empty() && history.back().first >= current_simulation_time) {
        history.pop_back();
    }
    history.emplace_back(current_simulation_time, std::move(current_history_record));

    double oldest_retained_time = current_simulation_time - 0.100; // keep 100ms
    while (!history.empty() && history.front().first < oldest_retained_time) {
        history.pop_front();
    }

    const double vestibular_afferent_delay = 0.002;
    double vestibulo_collic_efferent_delay = std::max(0.0, get_delay_vcr() - vestibular_afferent_delay);
    double descending_voluntary_delay      = 0.5 * get_delay_voluntary(); // 20 ms descending corticospinal motor delay
    DelayedData delayed_vestibulo_collic_efferent_data = getDelayedData(current_simulation_time, vestibulo_collic_efferent_delay);
    DelayedData delayed_vestibulo_collic_data          = getDelayedData(current_simulation_time, get_delay_vcr());
    DelayedData delayed_cervico_collic_data            = getDelayedData(current_simulation_time, get_delay_ccr(), true);
    DelayedData delayed_descending_voluntary_data      = getDelayedData(current_simulation_time, descending_voluntary_delay);

    const HistoryRecord& vestibulo_collic_record = delayed_vestibulo_collic_data.record;
    const HistoryRecord& cervico_collic_record   = delayed_cervico_collic_data.record;

    // --- DESCENDING TRACKS (Voluntary) vs ASCENDING TRACKS (Reflex) ---
    Vector vestibulo_collic_torque(3, 0.0); // Restorative Vestibular Reflexes
    Vector voluntary_torque(3, 0.0);        // Descending Voluntary Task Tracking

    // 1. Vestibular Reflex with Efference Copy Cancellation (expected vestibular firing from voluntary action)
    if (vestibulo_collic_record.valid) {
        const HistoryRecord& tonic_otolith_record =
            delayed_vestibulo_collic_efferent_data.record.valid
                ? delayed_vestibulo_collic_efferent_data.record
                : vestibulo_collic_record;
        double delayed_estimated_roll  = tonic_otolith_record.estimated_head_orientation[0];
        double delayed_estimated_yaw   = tonic_otolith_record.estimated_head_orientation[1];
        double delayed_estimated_pitch = tonic_otolith_record.estimated_head_orientation[2];

        double descending_traj_pos_pitch = (!history.empty() && delayed_descending_voluntary_data.record.valid) ? delayed_descending_voluntary_data.record.trajectory_position[0] : trajectory_position_pitch;
        double descending_traj_pos_roll  = (!history.empty() && delayed_descending_voluntary_data.record.valid) ? delayed_descending_voluntary_data.record.trajectory_position[1] : trajectory_position_roll;
        double descending_traj_pos_yaw   = (!history.empty() && delayed_descending_voluntary_data.record.valid) ? delayed_descending_voluntary_data.record.trajectory_position[2] : trajectory_position_yaw;
        double descending_traj_vel_pitch = (!history.empty() && delayed_descending_voluntary_data.record.valid) ? delayed_descending_voluntary_data.record.trajectory_velocity[0] : trajectory_velocity_pitch;
        double descending_traj_vel_roll  = (!history.empty() && delayed_descending_voluntary_data.record.valid) ? delayed_descending_voluntary_data.record.trajectory_velocity[1] : trajectory_velocity_roll;
        double descending_traj_vel_yaw   = (!history.empty() && delayed_descending_voluntary_data.record.valid) ? delayed_descending_voluntary_data.record.trajectory_velocity[2] : trajectory_velocity_yaw;
        double descending_traj_acc_pitch = (!history.empty() && delayed_descending_voluntary_data.record.valid) ? delayed_descending_voluntary_data.record.trajectory_acceleration[0] : trajectory_acceleration_pitch;
        double descending_traj_acc_roll  = (!history.empty() && delayed_descending_voluntary_data.record.valid) ? delayed_descending_voluntary_data.record.trajectory_acceleration[1] : trajectory_acceleration_roll;

        double tonic_otolith_input_pitch = (delayed_estimated_pitch - baseline_spatial_orientation[2]) - descending_traj_pos_pitch;
        double tonic_otolith_input_roll  = (delayed_estimated_roll  - baseline_spatial_orientation[0]) - descending_traj_pos_roll;
        double tonic_otolith_input_yaw   = (delayed_estimated_yaw   - baseline_spatial_orientation[1]) - descending_traj_pos_yaw;

        double semicircular_canal_input_pitch = vestibulo_collic_record.reconstructed_canal_angular_velocity[2] - descending_traj_vel_pitch;
        double semicircular_canal_input_roll  = vestibulo_collic_record.reconstructed_canal_angular_velocity[0] - descending_traj_vel_roll;
        double semicircular_canal_input_yaw   = vestibulo_collic_record.reconstructed_canal_angular_velocity[1] - descending_traj_vel_yaw;

        SimTK::Vec3 gravity_vector_world(0.0, -9.81, 0.0);
        SimTK::Vec3 delayed_otolith_specific_force_head =
            vestibulo_collic_record.reconstructed_otolith_specific_force;
        SimTK::Vec3 estimated_linear_acceleration_head =
            delayed_otolith_specific_force_head + ((~tonic_otolith_record.estimated_rotation_head_to_world) * gravity_vector_world);

        double head_pivot_radius = 0.15;
        double expected_tangential_acceleration_x = descending_traj_acc_pitch * head_pivot_radius;
        double expected_tangential_acceleration_z = descending_traj_acc_roll  * head_pivot_radius;

        double phasic_otolith_input_pitch = estimated_linear_acceleration_head[0] - expected_tangential_acceleration_x;
        double phasic_otolith_input_roll  = estimated_linear_acceleration_head[2] - expected_tangential_acceleration_z;
        double phasic_otolith_input_yaw   = 0.0;

        double tonic_otolith_state_pitch = state_variable_tonic_otolith[0]->getValue(state);
        double tonic_otolith_state_roll  = state_variable_tonic_otolith[1]->getValue(state);
        double tonic_otolith_state_yaw   = state_variable_tonic_otolith[2]->getValue(state);

        double semicircular_canal_state_pitch = state_variable_semicircular_canal_reflex[0]->getValue(state);
        double semicircular_canal_state_roll  = state_variable_semicircular_canal_reflex[1]->getValue(state);
        double semicircular_canal_state_yaw   = state_variable_semicircular_canal_reflex[2]->getValue(state);

        double phasic_otolith_state_pitch = state_variable_phasic_otolith[0]->getValue(state);
        double phasic_otolith_state_roll  = state_variable_phasic_otolith[1]->getValue(state);
        double phasic_otolith_state_yaw   = state_variable_phasic_otolith[2]->getValue(state);

        double tonic_otolith_output_pitch = get_K_ton() * get_p_ton() * tonic_otolith_state_pitch;
        double tonic_otolith_output_roll  = get_K_ton() * get_p_ton() * tonic_otolith_state_roll;
        double tonic_otolith_output_yaw   = get_K_ton() * get_p_ton() * tonic_otolith_state_yaw;

        // Center-frequency normalized lead filter: |H_sc(j*sqrt(z_sc*p_sc))| = K_sc (decouples gain from lead ratio)
        double p_sc = get_p_sc();
        double z_sc = get_z_sc();
        if (std::abs(z_sc) < 1e-4) z_sc = (z_sc >= 0.0 ? 1e-4 : -1e-4);
        double semicircular_canal_lead_ratio_sqrt = std::sqrt(std::clamp(std::abs(z_sc) / std::max(1e-4, std::abs(p_sc)), 1e-4, 100.0));
        double semicircular_canal_direct_feedthrough = 1.0 / semicircular_canal_lead_ratio_sqrt;
        double semicircular_canal_state_gain = (z_sc - p_sc) / semicircular_canal_lead_ratio_sqrt;

        double semicircular_canal_output_pitch =
            get_K_sc() * (semicircular_canal_direct_feedthrough * semicircular_canal_input_pitch + semicircular_canal_state_gain * semicircular_canal_state_pitch);
        double semicircular_canal_output_roll =
            get_K_sc() * (semicircular_canal_direct_feedthrough * semicircular_canal_input_roll  + semicircular_canal_state_gain * semicircular_canal_state_roll);
        double semicircular_canal_output_yaw =
            get_K_sc() * (semicircular_canal_direct_feedthrough * semicircular_canal_input_yaw   + semicircular_canal_state_gain * semicircular_canal_state_yaw);

        // Strictly-Proper Phasic Otolith Filter (Fernandez & Goldberg 1976 irregular unit adaptation + lead zero, D = 0):
        // When z_phas > 0: H_phas(s) = K_phas * p_phas * (s + z_phas) / (s + p_phas)^2
        //   = K_phas * p_phas * [ 1/(s + p_phas) + (z_phas - p_phas)/(s + p_phas)^2 ]
        // When z_phas <= 0 and p_phas2 > 0: 2nd-order low-pass H_phas(s) = K_phas * (p_phas / (s + p_phas)) * (p_phas2 / (s + p_phas2))
        // Otherwise: 1st-order low-pass H_phas(s) = K_phas * p_phas / (s + p_phas)
        double phasic_otolith_zero = get_z_phas();
        double phasic_otolith_pole = get_p_phas();
        double second_phasic_otolith_pole = get_p_phas2();
        double phasic_otolith_output_pitch = 0.0;
        double phasic_otolith_output_roll  = 0.0;
        double phasic_otolith_output_yaw   = 0.0;
        if (phasic_otolith_zero > 0.0) {
            double phasic_otolith_stage2_state_pitch = state_variable_phasic_otolith_stage2[0]->getValue(state);
            double phasic_otolith_stage2_state_roll  = state_variable_phasic_otolith_stage2[1]->getValue(state);
            double phasic_otolith_stage2_state_yaw   = state_variable_phasic_otolith_stage2[2]->getValue(state);
            double phasic_otolith_stage2_weight = phasic_otolith_zero - phasic_otolith_pole;
            phasic_otolith_output_pitch =
                get_K_phas() * phasic_otolith_pole * (phasic_otolith_state_pitch + phasic_otolith_stage2_weight * phasic_otolith_stage2_state_pitch);
            phasic_otolith_output_roll =
                get_K_phas() * phasic_otolith_pole * (phasic_otolith_state_roll  + phasic_otolith_stage2_weight * phasic_otolith_stage2_state_roll);
            phasic_otolith_output_yaw =
                get_K_phas() * phasic_otolith_pole * (phasic_otolith_state_yaw   + phasic_otolith_stage2_weight * phasic_otolith_stage2_state_yaw);
        } else if (second_phasic_otolith_pole > 0.0) {
            double phasic_otolith_stage2_state_pitch = state_variable_phasic_otolith_stage2[0]->getValue(state);
            double phasic_otolith_stage2_state_roll  = state_variable_phasic_otolith_stage2[1]->getValue(state);
            double phasic_otolith_stage2_state_yaw   = state_variable_phasic_otolith_stage2[2]->getValue(state);
            phasic_otolith_output_pitch = get_K_phas() * second_phasic_otolith_pole * phasic_otolith_stage2_state_pitch;
            phasic_otolith_output_roll  = get_K_phas() * second_phasic_otolith_pole * phasic_otolith_stage2_state_roll;
            phasic_otolith_output_yaw   = get_K_phas() * second_phasic_otolith_pole * phasic_otolith_stage2_state_yaw;
        } else {
            phasic_otolith_output_pitch = get_K_phas() * phasic_otolith_pole * phasic_otolith_state_pitch;
            phasic_otolith_output_roll  = get_K_phas() * phasic_otolith_pole * phasic_otolith_state_roll;
            phasic_otolith_output_yaw   = get_K_phas() * phasic_otolith_pole * phasic_otolith_state_yaw;
        }

        vestibulo_collic_torque[0] -= tonic_otolith_output_pitch + semicircular_canal_output_pitch + phasic_otolith_output_pitch;
        vestibulo_collic_torque[1] -= tonic_otolith_output_roll  + semicircular_canal_output_roll  + phasic_otolith_output_roll;
        vestibulo_collic_torque[2] -= tonic_otolith_output_yaw   + semicircular_canal_output_yaw   + phasic_otolith_output_yaw;
    }

    // 2. Filtered Voluntary Drive from Cortical Low-Pass Filter (20 ms Ascending + 2 Hz Cortical LPF + 20 ms Descending Delay)
    if (!history.empty() && delayed_descending_voluntary_data.record.valid) {
        voluntary_torque[0] = delayed_descending_voluntary_data.record.cortical_voluntary_torque[0];
        voluntary_torque[1] = delayed_descending_voluntary_data.record.cortical_voluntary_torque[1];
        voluntary_torque[2] = delayed_descending_voluntary_data.record.cortical_voluntary_torque[2];
    } else {
        voluntary_torque[0] = current_cortical_voluntary_torque[0];
        voluntary_torque[1] = current_cortical_voluntary_torque[1];
        voluntary_torque[2] = current_cortical_voluntary_torque[2];
    }

    // --- TOTAL MOTOR CORTEX DRIVE TO SPINAL CORD ---
    Vector desired_total_torque(3, 0.0);
    desired_total_torque[0] = voluntary_torque[0] + vestibulo_collic_torque[0];
    desired_total_torque[1] = voluntary_torque[1] + vestibulo_collic_torque[1];
    desired_total_torque[2] = voluntary_torque[2] + vestibulo_collic_torque[2];

    desired_total_torque[0] = std::clamp(desired_total_torque[0], -150.0, 150.0);
    desired_total_torque[1] = std::clamp(desired_total_torque[1], -100.0, 100.0);
    desired_total_torque[2] = std::clamp(desired_total_torque[2], -100.0, 100.0);

    if (last_desired_torque.size() != 3) last_desired_torque.resize(3);
    last_desired_torque[0] = desired_total_torque[0];
    last_desired_torque[1] = desired_total_torque[1];
    last_desired_torque[2] = desired_total_torque[2];

    // --- QP SOLVER ---
    const std::vector<double>& maximum_isometric_forces = cached_maximum_isometric_forces;

    std::vector<double> alpha_motor_excitations(number_of_active_muscles);
    for (int muscle_index = 0; muscle_index < number_of_active_muscles; ++muscle_index) {
        alpha_motor_excitations[muscle_index] = cached_minimum_excitations[muscle_index];
    }
    std::vector<bool> is_muscle_free(number_of_active_muscles, true);

    Matrix gram_matrix_3dof(3, 3);
    Vector remaining_torque_vector(3);
    Vector lagrange_multiplier_vector(3);
    for (int iteration = 0; iteration < 15; ++iteration) {
        SimTK::Vec3 fixed_muscle_torque(0.0);
        for (int muscle_index = 0; muscle_index < number_of_active_muscles; ++muscle_index) {
            if (!is_muscle_free[muscle_index]) {
                for (int axis_index = 0; axis_index < 3; ++axis_index) {
                    fixed_muscle_torque[axis_index] +=
                        (moment_arm_jacobian_3dof(axis_index, muscle_index) * maximum_isometric_forces[muscle_index]) *
                        alpha_motor_excitations[muscle_index];
                }
            }
        }

        SimTK::Vec3 remaining_desired_torque(
            desired_total_torque[0] - fixed_muscle_torque[0],
            desired_total_torque[1] - fixed_muscle_torque[1],
            desired_total_torque[2] - fixed_muscle_torque[2]);

        gram_matrix_3dof.setToZero();
        for (int muscle_index = 0; muscle_index < number_of_active_muscles; ++muscle_index) {
            if (is_muscle_free[muscle_index]) {
                for (int row = 0; row < 3; ++row) {
                    for (int column = 0; column < 3; ++column) {
                        gram_matrix_3dof(row, column) +=
                            (moment_arm_jacobian_3dof(row, muscle_index)    * maximum_isometric_forces[muscle_index]) *
                            (moment_arm_jacobian_3dof(column, muscle_index) * maximum_isometric_forces[muscle_index]);
                    }
                }
            }
        }

        const double qp_tikhonov_regularization = 1e-3;
        gram_matrix_3dof(0, 0) += qp_tikhonov_regularization;
        gram_matrix_3dof(1, 1) += qp_tikhonov_regularization;
        gram_matrix_3dof(2, 2) += qp_tikhonov_regularization;

        FactorLU lu_factorization(gram_matrix_3dof);
        for (int axis_index = 0; axis_index < 3; ++axis_index) {
            remaining_torque_vector[axis_index] = remaining_desired_torque[axis_index];
        }

        lu_factorization.solve(remaining_torque_vector, lagrange_multiplier_vector);

        bool bounds_violated = false;
        for (int muscle_index = 0; muscle_index < number_of_active_muscles; ++muscle_index) {
            if (is_muscle_free[muscle_index]) {
                double unconstrained_excitation = 0.0;
                for (int axis_index = 0; axis_index < 3; ++axis_index) {
                    unconstrained_excitation +=
                        (moment_arm_jacobian_3dof(axis_index, muscle_index) * maximum_isometric_forces[muscle_index]) *
                        lagrange_multiplier_vector[axis_index];
                }

                if (unconstrained_excitation < cached_minimum_excitations[muscle_index]) {
                    alpha_motor_excitations[muscle_index] = cached_minimum_excitations[muscle_index];
                    is_muscle_free[muscle_index] = false;
                    bounds_violated = true;
                } else if (unconstrained_excitation > 1.0) {
                    alpha_motor_excitations[muscle_index] = 1.0;
                    is_muscle_free[muscle_index] = false;
                    bounds_violated = true;
                } else {
                    alpha_motor_excitations[muscle_index] = unconstrained_excitation;
                }
            }
        }
        if (!bounds_violated) break;
    }
    last_alpha_motor_excitations = alpha_motor_excitations;

    // --- CCR AND FINAL ASSEMBLY ---
    double p_ccr = get_p_ccr();
    double z_ccr = get_z_ccr();
    if (std::abs(z_ccr) < 1e-4) z_ccr = (z_ccr >= 0.0 ? 1e-4 : -1e-4);
    double cervico_collic_lead_ratio_sqrt = std::sqrt(std::clamp(std::abs(z_ccr) / std::max(1e-4, std::abs(p_ccr)), 1e-4, 100.0));
    double cervico_collic_direct_feedthrough = 1.0 / cervico_collic_lead_ratio_sqrt;
    double cervico_collic_state_gain = (z_ccr - p_ccr) / cervico_collic_lead_ratio_sqrt;
    double voluntary_activity_gate = std::clamp(
        (std::abs(trajectory_position_pitch) + std::abs(trajectory_position_roll) + std::abs(trajectory_position_yaw) +
         0.2 * (std::abs(trajectory_velocity_pitch) + std::abs(trajectory_velocity_roll) + std::abs(trajectory_velocity_yaw))) / 0.02,
        0.0, 1.0);
    double reciprocal_inhibition_gain = 1.0 - voluntary_activity_gate * (1.0 - get_K_recip_inhib());
    double proprioceptive_gain = get_Kp_proprioception();
    bool has_cervico_collic_afferents = (cervico_collic_record.primary_afferent_firing_rate.size() == number_of_active_muscles);
    Vector muscle_control_vector;

    for (int muscle_index = 0; muscle_index < number_of_active_muscles; ++muscle_index) {
        const Muscle& muscle = muscles.get(active_muscle_indices[muscle_index]);

        double excitation = alpha_motor_excitations[muscle_index];

        if (has_cervico_collic_afferents) {
            // Cervico-Collic Reflex (CCR) - using biological firing rate
            double cervico_collic_state = state_variable_cervico_collic[muscle_index]->getValue(state);
            double delayed_primary_afferent_rate = cervico_collic_record.primary_afferent_firing_rate[muscle_index];

            double filtered_primary_afferent_rate =
                cervico_collic_direct_feedthrough * delayed_primary_afferent_rate +
                cervico_collic_state_gain * cervico_collic_state;

            // Rectify in case lead filter output goes negative
            double rectified_primary_afferent_rate = std::max(0.0, filtered_primary_afferent_rate);

            // Synergy-Based Reciprocal Inhibition (Descending Presynaptic Inhibition)
            // Gated by active voluntary trajectory vigor so all 72 muscles retain full anti-buckling CCR stiffness (pitch1 vs pitch2) during postural stabilization:
            double inhibition_factor = 1.0;
            double minimum_muscle_excitation = cached_minimum_excitations[muscle_index];
            double maximum_transition_excitation = minimum_muscle_excitation + 0.05;

            if (alpha_motor_excitations[muscle_index] <= minimum_muscle_excitation) {
                inhibition_factor = reciprocal_inhibition_gain;
            } else if (alpha_motor_excitations[muscle_index] >= maximum_transition_excitation) {
                inhibition_factor = 1.0;
            } else {
                double transition_fraction =
                    (alpha_motor_excitations[muscle_index] - minimum_muscle_excitation) /
                    (maximum_transition_excitation - minimum_muscle_excitation);
                double smooth_step = transition_fraction * transition_fraction * (3.0 - 2.0 * transition_fraction);
                inhibition_factor = reciprocal_inhibition_gain + smooth_step * (1.0 - reciprocal_inhibition_gain);
            }

            // Scale the firing rate (0-500 Hz) into excitation (0-1) using Kp_proprioception
            excitation += proprioceptive_gain * (rectified_primary_afferent_rate / 100.0) * inhibition_factor;
        }

        excitation = std::clamp(excitation, cached_minimum_excitations[muscle_index], 1.0);

        int number_of_controls = muscle.numControls();
        if (muscle_control_vector.size() != number_of_controls) {
            muscle_control_vector.resize(number_of_controls);
        }
        muscle_control_vector[0] = excitation; // Alpha motor command

        // Supraspinal alpha-gamma coactivation driven by descending command alpha_motor_excitations[muscle_index] (no Ia->gamma loop)
        if (number_of_controls >= 3) {
            muscle_control_vector[1] = 100.0 * get_K_gamma_dyn()  * alpha_motor_excitations[muscle_index]; // Gamma dynamic (pulses/s)
            muscle_control_vector[2] = 100.0 * get_K_gamma_stat() * alpha_motor_excitations[muscle_index]; // Gamma static (pulses/s)
        }

        muscle.addInControls(muscle_control_vector, controls);
    }
}
