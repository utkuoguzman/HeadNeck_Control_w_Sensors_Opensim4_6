#include "HeadNeckNeuralControllerAdaptive.h"
#include "RegisterTypes_osimHeadNeckNeuralAdaptive.h"
#include <complex>
#include <sstream>
#include <OpenSim/Simulation/Model/Muscle.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <iostream>
#include <algorithm>

using namespace OpenSim;
using namespace SimTK;


static const char* const ADAPT_12_NAMES[12] = {
    "adapt_K_ton",
    "adapt_p_ton",
    "adapt_K_sc",
    "adapt_z_sc",
    "adapt_p_sc",
    "adapt_K_phas",
    "adapt_p_phas",
    "adapt_kp_prop",
    "adapt_z_ccr",
    "adapt_p_ccr",
    "adapt_K_gamma_dyn",
    "adapt_K_gamma_stat"
};

static void getBaselineBodeAnchor(double out_gain[6], double out_phase_deg[6]) {
    // Default to measured OpenSim multisine response at Phase 2.4 Gen 3 Champion (W18_P24G3)
    const double default_gain[6]      = {112.50, 168.40, 210.80, 515.20, 710.50, 412.00};
    const double default_phase_deg[6] = {-105.0, -93.50, -108.0, -166.0, -276.0, -351.0};
    for (int k = 0; k < 6; ++k) {
        out_gain[k] = default_gain[k];
        out_phase_deg[k] = default_phase_deg[k];
    }
    const char* env_g = std::getenv("OSIM_BASE_GAINS");
    if (env_g) {
        std::stringstream ss(env_g);
        std::string item;
        int idx = 0;
        while (std::getline(ss, item, ',') && idx < 6) {
            try { out_gain[idx] = std::stod(item); } catch (...) {}
            idx++;
        }
    }
    const char* env_p = std::getenv("OSIM_BASE_PHASES");
    if (env_p) {
        std::stringstream ss(env_p);
        std::string item;
        int idx = 0;
        while (std::getline(ss, item, ',') && idx < 6) {
            try { out_phase_deg[idx] = std::stod(item); } catch (...) {}
            idx++;
        }
    }
}

static std::complex<double> evalRawClosedLoopBode(double freq_hz, const double th[12], double kp_task, double kd_task, double ki_task) {
    const double omega = 2.0 * SimTK::Pi * freq_hz;
    const std::complex<double> s(0.0, omega);
    const std::complex<double> s2(-omega * omega, 0.0);

    const std::complex<double> D_vcr = std::exp(-s * 0.010);
    const std::complex<double> D_ccr = std::exp(-s * 0.015);
    const std::complex<double> D_vol = std::exp(-s * 0.040);
    const std::complex<double> H_act = 1.0 / ((1.0 + 0.012 * s) * (1.0 + 0.035 * s));

    const double I_pivot = 0.0953;
    const double B_pass  = 0.35;
    const double K_pass  = 3.80;
    const double omega_vol = 2.0 * SimTK::Pi * 2.0;
    const std::complex<double> Z_vol = (kp_task + kd_task * s + (ki_task / s)) * (omega_vol / (s + omega_vol)) * D_vol * H_act;
    const std::complex<double> D_mech = I_pivot * s2 + B_pass * s + K_pass + Z_vol;

    const double K_ton = std::max(1e-4, th[0]);
    const double p_ton = std::max(1.0,  th[1]);
    const std::complex<double> Z_ton = K_ton * (p_ton / (s + p_ton)) * D_vcr * H_act;

    const double K_sc = std::max(1e-4, th[2]);
    const double z_sc = std::max(1e-3, th[3]);
    const double p_sc = std::max(1.0,  th[4]);
    const double norm_sc = 1.0 / std::sqrt(std::clamp(z_sc / p_sc, 1e-4, 100.0));
    const std::complex<double> H_canal = 1.0 + 0.02442 * s;
    const std::complex<double> Z_sc = 18.0 * K_sc * norm_sc * ((s + z_sc) / (s + p_sc)) * H_canal * s * D_vcr * H_act;

    const double K_phas = std::max(1e-4, th[5]);
    const double p_phas = std::max(0.5,  th[6]);
    const double r_neck = 0.13;
    const std::complex<double> H_phas = K_phas * (p_phas / (s + p_phas)) * D_vcr * H_act;
    const std::complex<double> Z_phas_rot = -r_neck * s2 * H_phas;

    const double kp_prop = std::max(1e-4, th[7]);
    const double z_ccr   = std::max(1e-3, th[8]);
    const double p_ccr   = std::max(1.0,  th[9]);
    const double k_gdyn  = std::max(0.01, th[10]);
    const double k_gstat = std::max(0.01, th[11]);
    const double norm_ccr = 1.0 / std::sqrt(std::clamp(z_ccr / p_ccr, 1e-4, 100.0));
    const std::complex<double> H_spindle = 9.5 * ((1.0 + 0.35 * k_gstat) + (0.055 + 0.045 * k_gdyn) * s);
    const std::complex<double> Z_ccr = kp_prop * norm_ccr * ((s + z_ccr) / (s + p_ccr)) * H_spindle * D_ccr * H_act;

    const std::complex<double> D_CL = D_mech + Z_ton + Z_sc + Z_phas_rot + Z_ccr;
    const double m_head_r = 4.30 * 0.13;
    const std::complex<double> N_CL = (-s2) * (m_head_r + 0.025 * H_phas) * (180.0 / SimTK::Pi);

    return N_CL / D_CL;
}

static std::complex<double> evalSpaceMappedBode(int k_idx, const double th[12], const double th0[12], double kp_task, double kd_task, double ki_task) {
    const double freqs[6] = {0.324, 0.783, 1.099, 2.446, 5.583, 7.927};
    double osim_base_gain[6], osim_base_phase_deg[6];
    getBaselineBodeAnchor(osim_base_gain, osim_base_phase_deg);

    double ph_rad = osim_base_phase_deg[k_idx] * SimTK::Pi / 180.0;
    std::complex<double> G_osim_0 = std::polar(osim_base_gain[k_idx], ph_rad);
    std::complex<double> G_raw_0  = evalRawClosedLoopBode(freqs[k_idx], th0, kp_task, kd_task, ki_task);
    std::complex<double> G_raw_th = evalRawClosedLoopBode(freqs[k_idx], th,  kp_task, kd_task, ki_task);

    return G_osim_0 * (G_raw_th / G_raw_0);
}

static double computeReferencePitchDeg(double t, double multisine_scale) {
    const double t_delay = 1.5;
    const double t_ramp = 1.5;
    if (t < t_delay) return 0.0;
    double t_rel = t - t_delay;
    double w = 1.0;
    if (t_rel < t_ramp) {
        double tau = t_rel / t_ramp;
        w = 10.0 * std::pow(tau, 3) - 15.0 * std::pow(tau, 4) + 6.0 * std::pow(tau, 5);
    }
    const double freqs[6] = {0.324, 0.783, 1.099, 2.446, 5.583, 7.927};
    const double targets_gain[6] = {86.55, 236.93, 319.74, 423.34, 503.30, 572.69};
    const double targets_phase_deg[6] = {-7.03, -96.96, -109.23, -161.34, -201.38, -247.62};

    double y_ref_deg = 0.0;
    for (int k = 0; k < 6; ++k) {
        double f = freqs[k];
        double phi_k = SimTK::Pi * k * (k - 1) / 6.0;
        double base_amp = 0.0215 * std::pow(0.835 / f, 2);
        double A_k = base_amp * multisine_scale;
        double phase_rad = phi_k + targets_phase_deg[k] * SimTK::Pi / 180.0;
        y_ref_deg += A_k * targets_gain[k] * std::sin(2.0 * SimTK::Pi * f * t_rel + phase_rad);
    }
    return w * y_ref_deg;
}

HeadNeckNeuralControllerAdaptive::HeadNeckNeuralControllerAdaptive() {

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
    constructProperty_delay_smith_proprio(0.005);
    constructProperty_delay_afferent_proprio(0.007);
    constructProperty_delay_voluntary(0.040);
    constructProperty_fc_voluntary(2.0);
    constructProperty_delay_afferent_vestibular(0.002);
    constructProperty_enable_adaptation(true);
    constructProperty_adaptation_rate_multiplier(1.0);
    constructProperty_sigma_leakage(0.02);
    constructProperty_adaptation_start_time(3.0);
    constructProperty_multisine_scale(0.142887);

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

void HeadNeckNeuralControllerAdaptive::extendAddToSystem(SimTK::MultibodySystem& system) const {
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
    for (int i = 0; i < 12; ++i) {
        addStateVariable(ADAPT_12_NAMES[i], SimTK::Stage::Velocity);
        addStateVariable("psif_x1_" + std::to_string(i), SimTK::Stage::Velocity);
        addStateVariable("psif_x2_" + std::to_string(i), SimTK::Stage::Velocity);
    }
}

double HeadNeckNeuralControllerAdaptive::getMahonyPitch(const SimTK::State& state) const {
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

double HeadNeckNeuralControllerAdaptive::getMahonyRoll(const SimTK::State& state) const {
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

double HeadNeckNeuralControllerAdaptive::getMahonyYaw(const SimTK::State& state) const {
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

double HeadNeckNeuralControllerAdaptive::getMahonyPitchSpeed(const SimTK::State& state) const {
    return mahony_estimated_angular_velocity[0];
}

double HeadNeckNeuralControllerAdaptive::getMahonyRollSpeed(const SimTK::State& state) const {
    return mahony_estimated_angular_velocity[1];
}

double HeadNeckNeuralControllerAdaptive::getMahonyYawSpeed(const SimTK::State& state) const {
    return mahony_estimated_angular_velocity[2];
}

double HeadNeckNeuralControllerAdaptive::getAdaptKton(const SimTK::State& s) const {
    return sv_adapt_theta[0] ? sv_adapt_theta[0]->getValue(s) : getStateVariableValue(s, "adapt_K_ton");
}
double HeadNeckNeuralControllerAdaptive::getAdaptPton(const SimTK::State& s) const {
    return sv_adapt_theta[1] ? sv_adapt_theta[1]->getValue(s) : getStateVariableValue(s, "adapt_p_ton");
}
double HeadNeckNeuralControllerAdaptive::getAdaptKsc(const SimTK::State& s) const {
    return sv_adapt_theta[2] ? sv_adapt_theta[2]->getValue(s) : getStateVariableValue(s, "adapt_K_sc");
}
double HeadNeckNeuralControllerAdaptive::getAdaptZsc(const SimTK::State& s) const {
    return sv_adapt_theta[3] ? sv_adapt_theta[3]->getValue(s) : getStateVariableValue(s, "adapt_z_sc");
}
double HeadNeckNeuralControllerAdaptive::getAdaptPsc(const SimTK::State& s) const {
    return sv_adapt_theta[4] ? sv_adapt_theta[4]->getValue(s) : getStateVariableValue(s, "adapt_p_sc");
}
double HeadNeckNeuralControllerAdaptive::getAdaptKphas(const SimTK::State& s) const {
    return sv_adapt_theta[5] ? sv_adapt_theta[5]->getValue(s) : getStateVariableValue(s, "adapt_K_phas");
}
double HeadNeckNeuralControllerAdaptive::getAdaptPphas(const SimTK::State& s) const {
    return sv_adapt_theta[6] ? sv_adapt_theta[6]->getValue(s) : getStateVariableValue(s, "adapt_p_phas");
}
double HeadNeckNeuralControllerAdaptive::getAdaptKpprop(const SimTK::State& s) const {
    return sv_adapt_theta[7] ? sv_adapt_theta[7]->getValue(s) : getStateVariableValue(s, "adapt_kp_prop");
}
double HeadNeckNeuralControllerAdaptive::getAdaptZccr(const SimTK::State& s) const {
    return sv_adapt_theta[8] ? sv_adapt_theta[8]->getValue(s) : getStateVariableValue(s, "adapt_z_ccr");
}
double HeadNeckNeuralControllerAdaptive::getAdaptPccr(const SimTK::State& s) const {
    return sv_adapt_theta[9] ? sv_adapt_theta[9]->getValue(s) : getStateVariableValue(s, "adapt_p_ccr");
}
double HeadNeckNeuralControllerAdaptive::getAdaptKgammaDyn(const SimTK::State& s) const {
    return sv_adapt_theta[10] ? sv_adapt_theta[10]->getValue(s) : getStateVariableValue(s, "adapt_K_gamma_dyn");
}
double HeadNeckNeuralControllerAdaptive::getAdaptKgammaStat(const SimTK::State& s) const {
    return sv_adapt_theta[11] ? sv_adapt_theta[11]->getValue(s) : getStateVariableValue(s, "adapt_K_gamma_stat");
}
double HeadNeckNeuralControllerAdaptive::getActualPitchDeg(const SimTK::State& s) const {
    double p1 = cached_coordinate_pitch1 ? cached_coordinate_pitch1->getValue(s) : 0.0;
    double p2 = cached_coordinate_pitch2 ? cached_coordinate_pitch2->getValue(s) : 0.0;
    return (p1 + p2) * 180.0 / SimTK::Pi;
}
double HeadNeckNeuralControllerAdaptive::getRefPitchDeg(const SimTK::State& s) const {
    return computeReferencePitchDeg(s.getTime(), get_multisine_scale());
}
double HeadNeckNeuralControllerAdaptive::getAdaptErrorDeg(const SimTK::State& s) const {
    return getActualPitchDeg(s) - getRefPitchDeg(s);
}


void HeadNeckNeuralControllerAdaptive::extendConnectToModel(Model& model) {
    Super::extendConnectToModel(model);
    is_initialized = false;
    last_jacobian_update_time = -1.0;
}

void HeadNeckNeuralControllerAdaptive::extendInitStateFromProperties(SimTK::State& state) const {
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
    // Matched irregular otolith gravity-adaptation states x_g,adapt in Head frame [x, y, z] (initialized to g_H(0) = [0, -9.81, 0])
    setStateVariableValue(state, "phas2_state_pitch", 0.0);
    setStateVariableValue(state, "phas2_state_roll", -9.81);
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

    setStateVariableValue(state, "adapt_K_ton", get_K_ton());
    setStateVariableValue(state, "adapt_p_ton", get_p_ton());
    setStateVariableValue(state, "adapt_K_sc", get_K_sc());
    setStateVariableValue(state, "adapt_z_sc", get_z_sc());
    setStateVariableValue(state, "adapt_p_sc", get_p_sc());
    setStateVariableValue(state, "adapt_K_phas", get_K_phas());
    setStateVariableValue(state, "adapt_p_phas", get_p_phas());
    setStateVariableValue(state, "adapt_kp_prop", get_Kp_proprioception());
    setStateVariableValue(state, "adapt_z_ccr", get_z_ccr());
    setStateVariableValue(state, "adapt_p_ccr", get_p_ccr());
    setStateVariableValue(state, "adapt_K_gamma_dyn", get_K_gamma_dyn());
    setStateVariableValue(state, "adapt_K_gamma_stat", get_K_gamma_stat());

    double osim_base_gain[6], osim_base_phase_deg[6];
    getBaselineBodeAnchor(osim_base_gain, osim_base_phase_deg);
    for (int k = 0; k < 6; ++k) {
        double ph_rad = osim_base_phase_deg[k] * SimTK::Pi / 180.0;
        double S0 = osim_base_gain[k] * std::cos(ph_rad);
        double C0 = osim_base_gain[k] * std::sin(ph_rad);
        setStateVariableValue(state, "psif_x1_" + std::to_string(k),     S0);
        setStateVariableValue(state, "psif_x1_" + std::to_string(k + 6), S0);
        setStateVariableValue(state, "psif_x2_" + std::to_string(k),     C0);
        setStateVariableValue(state, "psif_x2_" + std::to_string(k + 6), C0);
    }
}

void HeadNeckNeuralControllerAdaptive::initializeGeometryAndBaseline(const SimTK::State& state) const {
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
    for (int i = 0; i < 12; ++i) {
        sv_adapt_theta[i] = traverseToStateVariable(ADAPT_12_NAMES[i]);
        sv_psif_x1[i] = traverseToStateVariable("psif_x1_" + std::to_string(i));
        sv_psif_x2[i] = traverseToStateVariable("psif_x2_" + std::to_string(i));
    }

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

        // Per-Muscle Literature-Centered Static Equilibrium Co-Contraction
        // Superficial Flexors (1.0-2.5% MVC), Deep/Upper Flexors (2.0-4.0% MVC),
        // Superficial Extensors (2.0-5.0% MVC), Deep Extensors (3.0-7.0% MVC)
        double minimum_excitation_value = 0.030;
        const char* env_mode = std::getenv("GROUP_MODE");
        bool use_orig_groups = (env_mode && std::string(env_mode) == "ORIG");
        if (use_orig_groups) {
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
                const char* env_lf = std::getenv("LF_CO");
                minimum_excitation_value = env_lf ? std::stod(env_lf) : 0.025;
            } else if (muscle_name.find("long_cap") != std::string::npos ||
                       muscle_name.find("long_col") != std::string::npos ||
                       muscle_name.find("scalenus") != std::string::npos) {
                const char* env_uf = std::getenv("UF_CO");
                minimum_excitation_value = env_uf ? std::stod(env_uf) : 0.040;
            } else if (muscle_name.find("trap_") != std::string::npos ||
                       muscle_name.find("splen_cap") != std::string::npos ||
                       muscle_name.find("splen_cerv") != std::string::npos ||
                       muscle_name.find("levator_scap") != std::string::npos) {
                const char* env_se = std::getenv("SUP_EXT_CO");
                minimum_excitation_value = env_se ? std::stod(env_se) : 0.050;
            } else if (muscle_name.find("semi_cap") != std::string::npos ||
                       muscle_name.find("semi_cerv") != std::string::npos ||
                       muscle_name.find("longissi_") != std::string::npos ||
                       muscle_name.find("iliocost_") != std::string::npos ||
                       muscle_name.find("rectcap_") != std::string::npos ||
                       muscle_name.find("obl_cap")  != std::string::npos) {
                const char* env_de = std::getenv("DEEP_EXT_CO");
                minimum_excitation_value = env_de ? std::stod(env_de) : 0.070;
            }
        } else {
            // 72-Muscle Literature-Centered 6-DOF Static Equilibrium Co-Contraction Table
            // Group 1: Superficial Flexors (1.0% - 2.5% MVC)
            if      (muscle_name == "Sterno_hyoid")          minimum_excitation_value = 0.0139074664;
            else if (muscle_name == "Sterno_hyoid_L")        minimum_excitation_value = 0.0138869448;
            else if (muscle_name == "SternoThyroid")         minimum_excitation_value = 0.0139580126;
            else if (muscle_name == "SternoThyroid_L")       minimum_excitation_value = 0.0139146060;
            else if (muscle_name == "Omo_hyoid")             minimum_excitation_value = 0.0140401442;
            else if (muscle_name == "Omo_hyoid_L")           minimum_excitation_value = 0.0140511883;
            else if (muscle_name == "stern_mast")            minimum_excitation_value = 0.0137600533;
            else if (muscle_name == "stern_mast_L")          minimum_excitation_value = 0.0138882584;
            else if (muscle_name == "cleid_mast")            minimum_excitation_value = 0.0140218931;
            else if (muscle_name == "cleid_mast_L")          minimum_excitation_value = 0.0140604434;
            else if (muscle_name == "cleid_occ")             minimum_excitation_value = 0.0141125391;
            else if (muscle_name == "cleid_occ_L")           minimum_excitation_value = 0.0141761987;
            else if (muscle_name == "scalenus_ant")          minimum_excitation_value = 0.0145401926;
            else if (muscle_name == "scalenus_ant_L")        minimum_excitation_value = 0.0144270113;
            // Group 2: Deep / Upper Cervical Flexors (2.0% - 4.0% MVC)
            else if (muscle_name == "digastric_post")        minimum_excitation_value = 0.0257708683;
            else if (muscle_name == "digastric_post_L")      minimum_excitation_value = 0.0260257723;
            else if (muscle_name == "digastric_ant")         minimum_excitation_value = 0.0351123801;
            else if (muscle_name == "digastric_ant_L")       minimum_excitation_value = 0.0352215535;
            else if (muscle_name == "Geniohyoid")            minimum_excitation_value = 0.0358435442;
            else if (muscle_name == "Geniohyoid_L")          minimum_excitation_value = 0.0357879142;
            else if (muscle_name == "Mylohyoid_Post")        minimum_excitation_value = 0.0353440759;
            else if (muscle_name == "Mylohyoid_Post_L")      minimum_excitation_value = 0.0354227939;
            else if (muscle_name == "Mylohyoid_Ant")         minimum_excitation_value = 0.0357049120;
            else if (muscle_name == "Mylohyoid_Ant_L")       minimum_excitation_value = 0.0359162302;
            else if (muscle_name == "Stylohyoid_Lat")        minimum_excitation_value = 0.0326655729;
            else if (muscle_name == "Stylohyoid_Lat_L")      minimum_excitation_value = 0.0327162539;
            else if (muscle_name == "Stylohyoid_Med")        minimum_excitation_value = 0.0327072420;
            else if (muscle_name == "Stylohyoid_Med_L")      minimum_excitation_value = 0.0327579685;
            else if (muscle_name == "scalenus_med")          minimum_excitation_value = 0.0326803693;
            else if (muscle_name == "scalenus_med_L")        minimum_excitation_value = 0.0325170091;
            else if (muscle_name == "scalenus_post")         minimum_excitation_value = 0.0317368734;
            else if (muscle_name == "scalenus_post_L")       minimum_excitation_value = 0.0316464182;
            else if (muscle_name == "long_cap_sklc4")        minimum_excitation_value = 0.0353953794;
            else if (muscle_name == "long_cap_sklc4_L")      minimum_excitation_value = 0.0353956169;
            else if (muscle_name == "long_col_c1thx")        minimum_excitation_value = 0.0295473898;
            else if (muscle_name == "long_col_c1thx_L")      minimum_excitation_value = 0.0295025776;
            else if (muscle_name == "long_col_c1c5")         minimum_excitation_value = 0.0334943909;
            else if (muscle_name == "long_col_c1c5_L")       minimum_excitation_value = 0.0334361024;
            else if (muscle_name == "long_col_c5thx")        minimum_excitation_value = 0.0298660580;
            else if (muscle_name == "long_col_c5thx_L")      minimum_excitation_value = 0.0298181117;
            // Group 3: Superficial Extensors (2.0% - 5.0% MVC)
            else if (muscle_name == "trap_cl")               minimum_excitation_value = 0.0222030957;
            else if (muscle_name == "trap_cl_L")             minimum_excitation_value = 0.0221611600;
            else if (muscle_name == "trap_acr")              minimum_excitation_value = 0.0391430837;
            else if (muscle_name == "trap_acr_L")            minimum_excitation_value = 0.0389694904;
            else if (muscle_name == "splen_cap_sklc6")       minimum_excitation_value = 0.0232840744;
            else if (muscle_name == "splen_cap_sklc6_L")     minimum_excitation_value = 0.0233107645;
            else if (muscle_name == "splen_cap_sklthx")      minimum_excitation_value = 0.0256466659;
            else if (muscle_name == "splen_cap_sklthx_L")    minimum_excitation_value = 0.0256818910;
            else if (muscle_name == "splen_cerv_c3thx")      minimum_excitation_value = 0.0487024411;
            else if (muscle_name == "splen_cerv_c3thx_L")    minimum_excitation_value = 0.0486376139;
            else if (muscle_name == "semi_cap_sklc5")        minimum_excitation_value = 0.0221057900;
            else if (muscle_name == "semi_cap_sklc5_L")      minimum_excitation_value = 0.0222267488;
            else if (muscle_name == "semi_cap_sklthx")       minimum_excitation_value = 0.0230974719;
            else if (muscle_name == "semi_cap_sklthx_L")     minimum_excitation_value = 0.0231614554;
            else if (muscle_name == "levator_scap")          minimum_excitation_value = 0.0494874183;
            else if (muscle_name == "levator_scap_L")        minimum_excitation_value = 0.0494206141;
            // Group 4: Deep Cervical Extensors (3.0% - 7.0% MVC)
            else if (muscle_name == "semi_cerv_c3thx")       minimum_excitation_value = 0.0694999818;
            else if (muscle_name == "semi_cerv_c3thx_L")     minimum_excitation_value = 0.0694159181;
            else if (muscle_name == "longissi_cap_sklc6")    minimum_excitation_value = 0.0305173104;
            else if (muscle_name == "longissi_cap_sklc6_L")  minimum_excitation_value = 0.0305802837;
            else if (muscle_name == "longissi_cerv_c4thx")   minimum_excitation_value = 0.0687748785;
            else if (muscle_name == "longissi_cerv_c4thx_L") minimum_excitation_value = 0.0686880523;
            else if (muscle_name == "iliocost_cerv_c5rib")   minimum_excitation_value = 0.0665554157;
            else if (muscle_name == "iliocost_cerv_c5rib_L") minimum_excitation_value = 0.0664675579;
            else if (muscle_name == "rectcap_post_maj")      minimum_excitation_value = 0.0305236751;
            else if (muscle_name == "rectcap_post_maj_L")    minimum_excitation_value = 0.0305448742;
            else if (muscle_name == "rectcap_post_min")      minimum_excitation_value = 0.0307383191;
            else if (muscle_name == "rectcap_post_min_L")    minimum_excitation_value = 0.0308234885;
            else if (muscle_name == "obl_cap_sup")           minimum_excitation_value = 0.0320057410;
            else if (muscle_name == "obl_cap_sup_L")         minimum_excitation_value = 0.0321181604;
            else if (muscle_name == "obl_cap_inf")           minimum_excitation_value = 0.0358454823;
            else if (muscle_name == "obl_cap_inf_L")         minimum_excitation_value = 0.0358149574;
        }
        cached_minimum_excitations[active_index] = minimum_excitation_value;
        last_alpha_motor_excitations[active_index] = minimum_excitation_value;
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

void HeadNeckNeuralControllerAdaptive::updateDecoderMatrix() const {
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

HeadNeckNeuralControllerAdaptive::DelayedData HeadNeckNeuralControllerAdaptive::getDelayedData(double current_time, double delay_duration, bool interpolate_spindle_afferents) const {
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
        delayed_result.record.luenberger_angular_acceleration = source_record.luenberger_angular_acceleration;
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
            delayed_result.record.spinal_motor_excitations = source_record.spinal_motor_excitations;
            delayed_result.record.spinal_alpha_excitations = source_record.spinal_alpha_excitations;
        }
        return delayed_result;
    }
    if (iterator_upper == history.begin()) {
        const HistoryRecord& source_record = iterator_upper->second;
        delayed_result.record.valid = source_record.valid;
        delayed_result.record.estimated_head_orientation = source_record.estimated_head_orientation;
        delayed_result.record.reconstructed_canal_angular_velocity = source_record.reconstructed_canal_angular_velocity;
        delayed_result.record.luenberger_angular_velocity = source_record.luenberger_angular_velocity;
        delayed_result.record.luenberger_angular_acceleration = source_record.luenberger_angular_acceleration;
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
            delayed_result.record.spinal_motor_excitations = source_record.spinal_motor_excitations;
            delayed_result.record.spinal_alpha_excitations = source_record.spinal_alpha_excitations;
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
    interpolated_record.luenberger_angular_acceleration =
        iterator_lower->second.luenberger_angular_acceleration * (1.0 - interpolation_weight) +
        iterator_upper->second.luenberger_angular_acceleration * interpolation_weight;
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

    if (interpolate_spindle_afferents) {
        if (iterator_lower->second.primary_afferent_firing_rate.size() > 0 &&
            iterator_upper->second.primary_afferent_firing_rate.size() == iterator_lower->second.primary_afferent_firing_rate.size()) {
            interpolated_record.primary_afferent_firing_rate.resize(iterator_lower->second.primary_afferent_firing_rate.size());
            for (int muscle_index = 0; muscle_index < interpolated_record.primary_afferent_firing_rate.size(); ++muscle_index) {
                interpolated_record.primary_afferent_firing_rate[muscle_index] =
                    iterator_lower->second.primary_afferent_firing_rate[muscle_index] * (1.0 - interpolation_weight) +
                    iterator_upper->second.primary_afferent_firing_rate[muscle_index] * interpolation_weight;
            }
        }
        if (iterator_lower->second.spinal_motor_excitations.size() > 0 &&
            iterator_upper->second.spinal_motor_excitations.size() == iterator_lower->second.spinal_motor_excitations.size()) {
            interpolated_record.spinal_motor_excitations.resize(iterator_lower->second.spinal_motor_excitations.size());
            for (int muscle_index = 0; muscle_index < interpolated_record.spinal_motor_excitations.size(); ++muscle_index) {
                interpolated_record.spinal_motor_excitations[muscle_index] =
                    iterator_lower->second.spinal_motor_excitations[muscle_index] * (1.0 - interpolation_weight) +
                    iterator_upper->second.spinal_motor_excitations[muscle_index] * interpolation_weight;
            }
        }
        if (iterator_lower->second.spinal_alpha_excitations.size() > 0 &&
            iterator_upper->second.spinal_alpha_excitations.size() == iterator_lower->second.spinal_alpha_excitations.size()) {
            interpolated_record.spinal_alpha_excitations.resize(iterator_lower->second.spinal_alpha_excitations.size());
            for (int muscle_index = 0; muscle_index < interpolated_record.spinal_alpha_excitations.size(); ++muscle_index) {
                interpolated_record.spinal_alpha_excitations[muscle_index] =
                    iterator_lower->second.spinal_alpha_excitations[muscle_index] * (1.0 - interpolation_weight) +
                    iterator_upper->second.spinal_alpha_excitations[muscle_index] * interpolation_weight;
            }
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

void HeadNeckNeuralControllerAdaptive::computeStateVariableDerivatives(const SimTK::State& state) const {
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
    double luenberger_angular_acceleration_roll  = (reconstructed_regular_canal_velocity[0] - luenberger_state_roll)  / regular_canal_lead_time_constant;
    double luenberger_angular_acceleration_yaw   = (reconstructed_regular_canal_velocity[1] - luenberger_state_yaw)   / regular_canal_lead_time_constant;
    double luenberger_angular_acceleration_pitch = (reconstructed_regular_canal_velocity[2] - luenberger_state_pitch) / regular_canal_lead_time_constant;
    state_variable_luenberger_angular_velocity[0]->setDerivative(state, luenberger_angular_acceleration_roll);
    state_variable_luenberger_angular_velocity[1]->setDerivative(state, luenberger_angular_acceleration_yaw);
    state_variable_luenberger_angular_velocity[2]->setDerivative(state, luenberger_angular_acceleration_pitch);
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
    const double irr_macular_feedthrough    = irr_oto_lead_tau1 / irr_oto_pole_b;  // 3.885714
    const double irr_macular_state_weight   = 1.0 - irr_macular_feedthrough;       // -2.885714
    const double luen_macular_feedthrough   = reg_oto_pole_b / reg_oto_lead_tau1;  // 0.634058
    const double luen_macular_state_weight  = 1.0 - luen_macular_feedthrough;      // 0.365942
    const double luen_adapt_tau             = reg_oto_adapt_tau / reg_oto_adapt_ratio; // 1.764706 s
    const double luen_adapt_state_weight    = (1.0 / reg_oto_adapt_ratio) - 1.0;       // 0.764706

    SimTK::Vec3 current_luenberger_otolith_force_head(0.0);
    SimTK::Vec3 current_irregular_otolith_force_head(0.0);
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
        current_irregular_otolith_force_head[axis_index] =
            irr_macular_state_weight * irr_mac_state + irr_macular_feedthrough * irr_stage1_out;

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
        const double eff_k_gdyn = (sv_adapt_theta[10] && get_enable_adaptation()) ? sv_adapt_theta[10]->getValue(state) : get_K_gamma_dyn();
        const double eff_k_gstat = (sv_adapt_theta[11] && get_enable_adaptation()) ? sv_adapt_theta[11]->getValue(state) : get_K_gamma_stat();
        const double dynamic_gamma_gain_scaled        = 100.0 * eff_k_gdyn;
        const double static_gamma_gain_scaled         = 100.0 * eff_k_gstat;
        const double base_dyn_gamma_0                 = 100.0 * get_K_gamma_dyn();
        const double base_stat_gamma_0                = 100.0 * get_K_gamma_stat();
        const double length_observer_time_constant    = 0.010;
        const double secondary_afferent_time_constant = 0.055;
        const double velocity_observer_time_constant  = 0.005;
        for (int muscle_index = 0; muscle_index < number_of_active_muscles; ++muscle_index) {
            double spindle_length_change        = state_variable_spindle_length_change[muscle_index]->getValue(state);
            double spindle_velocity             = state_variable_spindle_velocity[muscle_index]->getValue(state);
            double dynamic_fusimotor_activation = state_variable_spindle_dynamic_fusimotor[muscle_index]->getValue(state);
            double static_fusimotor_activation  = state_variable_spindle_static_fusimotor[muscle_index]->getValue(state);

            double u0_alpha =
                (muscle_index < (int)cached_minimum_excitations.size()) ? cached_minimum_excitations[muscle_index] : 0.03;
            double alpha_motor_excitation =
                (muscle_index < (int)last_alpha_motor_excitations.size()) ? last_alpha_motor_excitations[muscle_index] : u0_alpha;
            double dynamic_gamma_firing_rate = dynamic_gamma_gain_scaled * alpha_motor_excitation;
            double static_gamma_firing_rate  = static_gamma_gain_scaled  * alpha_motor_excitation;
            double steady_state_dynamic_fusimotor =
                (dynamic_gamma_firing_rate * dynamic_gamma_firing_rate) / (dynamic_gamma_firing_rate * dynamic_gamma_firing_rate + 3600.0);
            double steady_state_static_fusimotor =
                (static_gamma_firing_rate * static_gamma_firing_rate) / (static_gamma_firing_rate * static_gamma_firing_rate + 3600.0);
            double instantaneous_nuclear_chain_activation =
                (static_gamma_firing_rate * static_gamma_firing_rate) / (static_gamma_firing_rate * static_gamma_firing_rate + 8100.0);

            double base_dyn_gamma = base_dyn_gamma_0 * u0_alpha;
            double base_stat_gamma = base_stat_gamma_0 * u0_alpha;
            double base_fdyn = (base_dyn_gamma * base_dyn_gamma) / (base_dyn_gamma * base_dyn_gamma + 3600.0);
            double base_fstat = (base_stat_gamma * base_stat_gamma) / (base_stat_gamma * base_stat_gamma + 3600.0);
            double base_fchain = (base_stat_gamma * base_stat_gamma) / (base_stat_gamma * base_stat_gamma + 8100.0);

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

            double delta_fdyn   = std::max(0.0, dynamic_fusimotor_activation - base_fdyn);
            double delta_fstat  = std::max(0.0, static_fusimotor_activation  - base_fstat);
            double delta_fchain = std::max(0.0, instantaneous_nuclear_chain_activation - base_fchain);
            double fusimotor_secondary_afferent_bias = 29.72138 * delta_fstat + 44.58207 * delta_fchain;
            double fusimotor_primary_afferent_bias   = 54.45176 * delta_fdyn  + 0.21517 * fusimotor_secondary_afferent_bias;
            double net_secondary_afferent_change     = (secondary_afferent_value - baseline_secondary_afferent_rates[muscle_index]) - fusimotor_secondary_afferent_bias;
            double net_primary_afferent_change       = (primary_afferent_value   - baseline_primary_afferent_rates[muscle_index])   - fusimotor_primary_afferent_bias;

            // Physiological spindle sensory threshold (prevents sub-threshold |Lp|^0.3 small-signal clonus around rest)
            static const double spindle_obs_threshold = std::getenv("SPINDLE_OBS_THRESH") ? std::stod(std::getenv("SPINDLE_OBS_THRESH")) : 0.5;
            if (spindle_obs_threshold > 0.0) {
                if (std::abs(net_primary_afferent_change) <= spindle_obs_threshold) {
                    net_primary_afferent_change = 0.0;
                } else {
                    net_primary_afferent_change = (net_primary_afferent_change > 0.0)
                        ? (net_primary_afferent_change - spindle_obs_threshold)
                        : (net_primary_afferent_change + spindle_obs_threshold);
                }
                const double sec_thresh = 0.5 * spindle_obs_threshold;
                if (std::abs(net_secondary_afferent_change) <= sec_thresh) {
                    net_secondary_afferent_change = 0.0;
                } else {
                    net_secondary_afferent_change = (net_secondary_afferent_change > 0.0)
                        ? (net_secondary_afferent_change - sec_thresh)
                        : (net_secondary_afferent_change + sec_thresh);
                }
            }

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

    // --- SE_2(3) SPATIAL OBSERVER (Tilt-Translation Disambiguation) ---
    // 1. Arriving Delayed Vestibular Readings (Input 1 & Input 2 at t - τ_vest = t - 2 ms from Subsystem 2)
    SimTK::Vec3 arriving_world_angular_velocity =
        (!history.empty() && delayed_vestibular_afferent_data.record.valid)
            ? delayed_vestibular_afferent_data.record.luenberger_angular_velocity
            : SimTK::Vec3(luenberger_angular_velocity_roll, luenberger_angular_velocity_yaw, luenberger_angular_velocity_pitch);
    SimTK::Vec3 arriving_world_angular_acceleration =
        (!history.empty() && delayed_vestibular_afferent_data.record.valid)
            ? delayed_vestibular_afferent_data.record.luenberger_angular_acceleration
            : SimTK::Vec3(luenberger_angular_acceleration_roll, luenberger_angular_acceleration_yaw, luenberger_angular_acceleration_pitch);
    SimTK::Vec3 otolith_specific_force_head =
        (!history.empty() && delayed_vestibular_afferent_data.record.valid)
            ? delayed_vestibular_afferent_data.record.luenberger_otolith_specific_force
            : current_luenberger_otolith_force_head; // Time-aligned at t - 2ms from Subsystem 2 Regular Otolith + Luenberger Inverse

    // 2. Current Observer Beliefs at NOW (t)
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

    // Input 1 Smith Predictor: Forward-predict arriving Canal Luenberger ω̂_Luen(t - τ_vest) by τ_vest = 2 ms
    // to NOW (t) using its derivative-free Luenberger angular acceleration α̂_Luen(t - τ_vest):
    double vestibular_prediction_dt = get_use_smith_predictor() ? vestibular_afferent_delay : 0.0;
    SimTK::Vec3 world_angular_velocity =
        arriving_world_angular_velocity + vestibular_prediction_dt * arriving_world_angular_acceleration;
    SimTK::Vec3 arriving_head_frame_angular_velocity = (~estimated_rotation_head_to_world) * arriving_world_angular_velocity;
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
    double pitch_integral_limit = std::max(0.6, 25.0 / std::max(0.05, ki_pitch));
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

    state_variable_voluntary_filter_stage1[0]->setDerivative(
        state, voluntary_cutoff_angular_frequency * (unfiltered_voluntary_torque_pitch - voluntary_filtered_stage1_pitch));
    state_variable_voluntary_filter_stage1[1]->setDerivative(
        state, voluntary_cutoff_angular_frequency * (unfiltered_voluntary_torque_roll  - voluntary_filtered_stage1_roll));
    state_variable_voluntary_filter_stage1[2]->setDerivative(
        state, voluntary_cutoff_angular_frequency * (unfiltered_voluntary_torque_yaw   - voluntary_filtered_stage1_yaw));

    // --- STAGE 1: MULTISENSORY SMITH PREDICTOR (Forward-Predicting Inputs 1, 2, 3 to NOW t Using Rate/Speed Estimates) ---
    // Input 1 (Canal ω̂_Luen) is already forward-predicted above by τ_vest = 2ms using α̂_Luen(t - 2ms) -> head_frame_angular_velocity.

    // Input 2 Smith Predictor (Otolith Gravity UP Vector, τ_vest = 2ms):
    // Forward-predict arriving u_meas(t - τ_vest) by τ_vest = 2ms to NOW (t) using rotational rate du/dt = -ω_head × u_meas:
    SimTK::Vec3 predicted_up_direction_head;
    if (otolith_specific_force_head.norm() > 1e-4) {
        SimTK::Vec3 arriving_up_direction_head = otolith_specific_force_head.normalize();
        SimTK::Vec3 extrapolated_up_direction_head =
            arriving_up_direction_head - vestibular_prediction_dt * (arriving_head_frame_angular_velocity % arriving_up_direction_head);
        double extrap_norm = extrapolated_up_direction_head.norm();
        predicted_up_direction_head = (extrap_norm > 1e-6) ? (extrapolated_up_direction_head / extrap_norm) : arriving_up_direction_head;
    } else {
        predicted_up_direction_head = (~estimated_rotation_head_to_world) * SimTK::Vec3(0.0, 1.0, 0.0);
    }

    // Input 3 Smith Predictor (Proprioceptive Head-in-Space Posture, τ_prop = 7ms):
    // In human anatomy, spindle action potentials take ~7 ms (delay_afferent_proprio)
    // to travel along dorsal roots and cuneate fasciculus to the brainstem/cerebellum.
    double arriving_relative_neck_pitch = estimated_neck_relative_angle[0];
    double arriving_relative_neck_roll  = estimated_neck_relative_angle[1];
    double arriving_relative_neck_yaw   = estimated_neck_relative_angle[2];
    double arriving_relative_neck_pitch_speed = estimated_neck_relative_speed[0];
    double arriving_relative_neck_roll_speed  = estimated_neck_relative_speed[1];
    double arriving_relative_neck_yaw_speed   = estimated_neck_relative_speed[2];
    double arriving_trunk_pitch = angle_ground_pitch;
    double arriving_trunk_roll  = angle_ground_roll;
    double arriving_trunk_yaw   = angle_ground_yaw;
    double arriving_trunk_pitch_speed = speed_ground_pitch;
    double arriving_trunk_roll_speed  = speed_ground_roll;
    double arriving_trunk_yaw_speed   = speed_ground_yaw;

    double proprioceptive_afferent_delay = get_delay_afferent_proprio();
    DelayedData delayed_proprioceptive_afferent_data;
    if (proprioceptive_afferent_delay > 1e-5 && !history.empty()) {
        delayed_proprioceptive_afferent_data = getDelayedData(state.getTime(), proprioceptive_afferent_delay);
        if (delayed_proprioceptive_afferent_data.record.valid) {
            arriving_relative_neck_pitch = delayed_proprioceptive_afferent_data.record.estimated_neck_relative_angle[0];
            arriving_relative_neck_roll  = delayed_proprioceptive_afferent_data.record.estimated_neck_relative_angle[1];
            arriving_relative_neck_yaw   = delayed_proprioceptive_afferent_data.record.estimated_neck_relative_angle[2];
            arriving_relative_neck_pitch_speed = delayed_proprioceptive_afferent_data.record.estimated_neck_relative_speed[0];
            arriving_relative_neck_roll_speed  = delayed_proprioceptive_afferent_data.record.estimated_neck_relative_speed[1];
            arriving_relative_neck_yaw_speed   = delayed_proprioceptive_afferent_data.record.estimated_neck_relative_speed[2];
            arriving_trunk_pitch = delayed_proprioceptive_afferent_data.record.trunk_orientation[2];
            arriving_trunk_roll  = delayed_proprioceptive_afferent_data.record.trunk_orientation[0];
            arriving_trunk_yaw   = delayed_proprioceptive_afferent_data.record.trunk_orientation[1];
            arriving_trunk_pitch_speed = delayed_proprioceptive_afferent_data.record.trunk_angular_velocity[2];
            arriving_trunk_roll_speed  = delayed_proprioceptive_afferent_data.record.trunk_angular_velocity[0];
            arriving_trunk_yaw_speed   = delayed_proprioceptive_afferent_data.record.trunk_angular_velocity[1];
        }
    }

    double proprioceptive_head_in_world_pitch = arriving_trunk_pitch + arriving_relative_neck_pitch;
    double proprioceptive_head_in_world_roll  = arriving_trunk_roll  + arriving_relative_neck_roll;
    double proprioceptive_head_in_world_yaw   = arriving_trunk_yaw   + arriving_relative_neck_yaw;

    double proprioceptive_head_in_world_pitch_speed = arriving_trunk_pitch_speed + arriving_relative_neck_pitch_speed;
    double proprioceptive_head_in_world_roll_speed  = arriving_trunk_roll_speed  + arriving_relative_neck_roll_speed;
    double proprioceptive_head_in_world_yaw_speed   = arriving_trunk_yaw_speed   + arriving_relative_neck_yaw_speed;

    // Forward-predict arriving θ_prop(t - τ_prop) by τ_prop = 7ms to NOW (t) using ω̂_prop(t - τ_prop):
    double proprioceptive_prediction_dt = get_use_smith_predictor() ? std::max(0.0, proprioceptive_afferent_delay) : 0.0;
    double predicted_proprioceptive_pitch =
        proprioceptive_head_in_world_pitch + proprioceptive_prediction_dt * proprioceptive_head_in_world_pitch_speed;
    double predicted_proprioceptive_roll =
        proprioceptive_head_in_world_roll + proprioceptive_prediction_dt * proprioceptive_head_in_world_roll_speed;
    double predicted_proprioceptive_yaw =
        proprioceptive_head_in_world_yaw + proprioceptive_prediction_dt * proprioceptive_head_in_world_yaw_speed;

    // --- STAGE 2: ATTITUDE INNOVATION BLOCKS (Comparing Forward-Predicted Signals at NOW t Against Observer State at NOW t) ---
    // 2a. Otolith Gravity Innovation at NOW (t) (Pitch & Roll):
    SimTK::Vec3 expected_up_direction_head = (~estimated_rotation_head_to_world) * SimTK::Vec3(0.0, 1.0, 0.0);
    SimTK::Vec3 otolith_attitude_error_head = predicted_up_direction_head % expected_up_direction_head;
    double Kp_otolith_attitude = 0.05; // Otolith gravity tether (0.05 rad/s so linear acceleration does not corrupt pitch/roll)
    SimTK::Vec3 weighted_otolith_attitude_error = Kp_otolith_attitude * otolith_attitude_error_head;

    // 2b. Proprioceptive Attitude Innovation at NOW (t) [Roll=0, Yaw=1, Pitch=2]:
    SimTK::Vec3 proprioceptive_attitude_error_world(
        predicted_proprioceptive_roll  - estimated_head_roll,
        predicted_proprioceptive_yaw   - estimated_head_yaw,
        predicted_proprioceptive_pitch - estimated_head_pitch);
    SimTK::Vec3 proprioceptive_attitude_error_head = (~estimated_rotation_head_to_world) * proprioceptive_attitude_error_world;
    double K_proprioceptive_attitude_roll  = 2.0;
    double K_proprioceptive_attitude_yaw   = 2.0;
    double K_proprioceptive_attitude_pitch = 2.0;
    SimTK::Vec3 weighted_proprioceptive_attitude_error(
        K_proprioceptive_attitude_roll  * proprioceptive_attitude_error_head[0],
        K_proprioceptive_attitude_yaw   * proprioceptive_attitude_error_head[1],
        K_proprioceptive_attitude_pitch * proprioceptive_attitude_error_head[2]);

    // --- STAGE 3: FUSED ANGULAR VELOCITY SUMMING JUNCTION (at NOW t) ---
    SimTK::Vec3 fused_angular_velocity_head =
        head_frame_angular_velocity +
        weighted_otolith_attitude_error +
        weighted_proprioceptive_attitude_error;

    double fused_angular_velocity_x = fused_angular_velocity_head[0];
    double fused_angular_velocity_y = fused_angular_velocity_head[1];
    double fused_angular_velocity_z = fused_angular_velocity_head[2];

    double quaternion_0_derivative = 0.5 * (-quaternion_1 * fused_angular_velocity_x - quaternion_2 * fused_angular_velocity_y - quaternion_3 * fused_angular_velocity_z);
    double quaternion_1_derivative = 0.5 * ( quaternion_0 * fused_angular_velocity_x + quaternion_2 * fused_angular_velocity_z - quaternion_3 * fused_angular_velocity_y);
    double quaternion_2_derivative = 0.5 * ( quaternion_0 * fused_angular_velocity_y - quaternion_1 * fused_angular_velocity_z + quaternion_3 * fused_angular_velocity_x);
    double quaternion_3_derivative = 0.5 * ( quaternion_0 * fused_angular_velocity_z + quaternion_1 * fused_angular_velocity_y - quaternion_2 * fused_angular_velocity_x);

    // 5. TRANSLATION DERIVATIVES
    // Since otolith_specific_force_head arrives from CN VIII with τ_vest = 2 ms delay (measured in Head frame at t - 2ms),
    // rotate it to World frame using the time-matched delayed rotation matrix R̂_H^W(t - 2ms):
    SimTK::Mat33 arriving_estimated_rotation_head_to_world =
        (!history.empty() && delayed_vestibular_afferent_data.record.valid)
            ? delayed_vestibular_afferent_data.record.estimated_rotation_head_to_world
            : estimated_rotation_head_to_world.asMat33();
    SimTK::Vec3 linear_acceleration_world = (arriving_estimated_rotation_head_to_world * otolith_specific_force_head) + gravity_vector_world;

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

    // --- SUBSYSTEM 5: BRAINSTEM VESTIBULO-COLLIC REFLEX (VCR) NEURAL FILTER DERIVATIVES (All Inputs Time-Aligned at t - 2 ms) ---
    // Causal 5-Delay Anatomical Structure (Cullen 2012; Roy & Cullen 2004; Brooks & Cullen 2013, 2014):
    //   - Input 1 (From Subsystem 4 via Cerebellovestibular Juxtarestiform Tract): τ_vest = 2 ms -> θ̂_head(t - 2ms)
    //   - Input 2 & 3 (From Subsystem 2 Inner Ear via CN VIII + Sub 4 Delayed R̂_H^W): τ_vest = 2 ms -> ω̂_irr(t - 2ms), f̂_oto,irr(t - 2ms), R̂_H^W(t - 2ms)
    //   - Input 4 (From Subsystem 1 M1 via Unified Descending Pyramidal Delay τ_desc = 12 ms + Cerebellar Forward Internal Model):
    //     Arrives at t - τ_desc = t - 12 ms and is forward-predicted by Δt_fwd = τ_desc - τ_vest = 10 ms to t - 2 ms:
    //       x̃_1,desc(t - 2ms) = x_1,desc(t - 12ms) + Δt_fwd * x_2,desc(t - 12ms)
    //       x̃_2,desc(t - 2ms) = x_2,desc(t - 12ms) + Δt_fwd * a_desc(t - 12ms)
    //   - Output of Subsystem 5 feeds directly (0 ms extra delay) into Subsystem 7, where the shared τ_spinal = 8 ms post-QP/post-pool efferent delay
    //     yields the exact 2 ms + 8 ms = 10 ms total VCR loop latency.
    double spinal_execution_delay          = std::max(0.0, get_delay_vcr() - vestibular_afferent_delay); // 8 ms shared spinal/ventral-root delay at Subsystem 7 output
    double unified_descending_delay        = std::max(0.0, descending_voluntary_delay - spinal_execution_delay); // 12 ms single descending delay from M1 to Brainstem & Spinal QP
    double efference_forward_prediction_dt = get_use_smith_predictor() ? std::max(0.0, unified_descending_delay - vestibular_afferent_delay) : 0.0; // 10 ms forward prediction to t - 2ms
    DelayedData delayed_corticobulbar_voluntary_data = getDelayedData(state.getTime(), unified_descending_delay);
    {
        double descending_traj_pos_pitch = (!history.empty() && delayed_corticobulbar_voluntary_data.record.valid) ? delayed_corticobulbar_voluntary_data.record.trajectory_position[0] : trajectory_position_pitch;
        double descending_traj_pos_roll  = (!history.empty() && delayed_corticobulbar_voluntary_data.record.valid) ? delayed_corticobulbar_voluntary_data.record.trajectory_position[1] : trajectory_position_roll;
        double descending_traj_pos_yaw   = (!history.empty() && delayed_corticobulbar_voluntary_data.record.valid) ? delayed_corticobulbar_voluntary_data.record.trajectory_position[2] : trajectory_position_yaw;
        double descending_traj_vel_pitch = (!history.empty() && delayed_corticobulbar_voluntary_data.record.valid) ? delayed_corticobulbar_voluntary_data.record.trajectory_velocity[0] : trajectory_velocity_pitch;
        double descending_traj_vel_roll  = (!history.empty() && delayed_corticobulbar_voluntary_data.record.valid) ? delayed_corticobulbar_voluntary_data.record.trajectory_velocity[1] : trajectory_velocity_roll;
        double descending_traj_vel_yaw   = (!history.empty() && delayed_corticobulbar_voluntary_data.record.valid) ? delayed_corticobulbar_voluntary_data.record.trajectory_velocity[2] : trajectory_velocity_yaw;
        double descending_traj_acc_pitch = (!history.empty() && delayed_corticobulbar_voluntary_data.record.valid) ? delayed_corticobulbar_voluntary_data.record.trajectory_acceleration[0] : trajectory_acceleration_pitch;
        double descending_traj_acc_roll  = (!history.empty() && delayed_corticobulbar_voluntary_data.record.valid) ? delayed_corticobulbar_voluntary_data.record.trajectory_acceleration[1] : trajectory_acceleration_roll;
        double descending_traj_acc_yaw   = (!history.empty() && delayed_corticobulbar_voluntary_data.record.valid) ? delayed_corticobulbar_voluntary_data.record.trajectory_acceleration[2] : trajectory_acceleration_yaw;

        // Dedicated Cerebellar Efference Forward Predictor Block on Input 4 (Δt_fwd = 10 ms: advances t - 12ms -> t - 2ms):
        double predicted_descending_traj_pos_pitch = descending_traj_pos_pitch + efference_forward_prediction_dt * descending_traj_vel_pitch;
        double predicted_descending_traj_pos_roll  = descending_traj_pos_roll  + efference_forward_prediction_dt * descending_traj_vel_roll;
        double predicted_descending_traj_pos_yaw   = descending_traj_pos_yaw   + efference_forward_prediction_dt * descending_traj_vel_yaw;

        double predicted_descending_traj_vel_pitch = descending_traj_vel_pitch + efference_forward_prediction_dt * descending_traj_acc_pitch;
        double predicted_descending_traj_vel_roll  = descending_traj_vel_roll  + efference_forward_prediction_dt * descending_traj_acc_roll;
        double predicted_descending_traj_vel_yaw   = descending_traj_vel_yaw   + efference_forward_prediction_dt * descending_traj_acc_yaw;

        // Input 1 from Subsystem 4 (arriving via Cerebellovestibular projection at t - 2 ms, aligned with Inputs 2, 3, and predicted Input 4):
        SimTK::Vec3 arriving_estimated_head_orientation =
            (!history.empty() && delayed_vestibular_afferent_data.record.valid)
                ? delayed_vestibular_afferent_data.record.estimated_head_orientation
                : SimTK::Vec3(estimated_head_roll, estimated_head_yaw, estimated_head_pitch);
        double tonic_otolith_input_pitch = (arriving_estimated_head_orientation[2] - baseline_spatial_orientation[2]) - predicted_descending_traj_pos_pitch;
        double tonic_otolith_input_roll  = (arriving_estimated_head_orientation[0] - baseline_spatial_orientation[0]) - predicted_descending_traj_pos_roll;
        double tonic_otolith_input_yaw   = (arriving_estimated_head_orientation[1] - baseline_spatial_orientation[1]) - predicted_descending_traj_pos_yaw;

        // Input 2 from Subsystem 2 Irregular Canals (arriving at Brainstem VN after τ_vest = 2 ms CN VIII afferent delay, aligned at t - 2 ms):
        SimTK::Vec3 arriving_irregular_canal_velocity =
            (!history.empty() && delayed_vestibular_afferent_data.record.valid)
                ? delayed_vestibular_afferent_data.record.reconstructed_canal_angular_velocity
                : head_global_angular_velocity;
        double semicircular_canal_input_pitch = arriving_irregular_canal_velocity[2] - predicted_descending_traj_vel_pitch;
        double semicircular_canal_input_roll  = arriving_irregular_canal_velocity[0] - predicted_descending_traj_vel_roll;
        double semicircular_canal_input_yaw   = arriving_irregular_canal_velocity[1] - predicted_descending_traj_vel_yaw;

        // Input 3 from Subsystem 2 Irregular Otoliths (arriving at Brainstem VN after τ_vest = 2 ms CN VIII afferent delay)
        // + time-matched delayed R̂_H^W(t - 2 ms) passed through the matched Irregular Adaptation Operator H_adapt,irr(s) = (α_irr + c_irr s) / (1 + c_irr s)
        // so gravity is cancelled across all frequencies (static & dynamic head tilt yield exact 0.0 m/s^2) while preserving the irregular otolith's
        // adaptation zero (c_irr = 0.25 s) and 27.2 ms hair-cell phase lead on true linear acceleration a_lin,H:
        SimTK::Vec3 arriving_otolith_specific_force_head =
            (!history.empty() && delayed_vestibular_afferent_data.record.valid)
                ? delayed_vestibular_afferent_data.record.reconstructed_otolith_specific_force
                : current_irregular_otolith_force_head;
        SimTK::Vec3 arriving_gravity_head = (~arriving_estimated_rotation_head_to_world) * gravity_vector_world;
        SimTK::Vec3 gravity_adapt_state(
            state_variable_phasic_otolith_stage2[0]->getValue(state),
            state_variable_phasic_otolith_stage2[1]->getValue(state),
            state_variable_phasic_otolith_stage2[2]->getValue(state));
        for (int axis_index = 0; axis_index < 3; ++axis_index) {
            state_variable_phasic_otolith_stage2[axis_index]->setDerivative(
                state, (arriving_gravity_head[axis_index] - gravity_adapt_state[axis_index]) / irr_oto_adapt_tau);
        }
        SimTK::Vec3 adapted_gravity_head =
            (irr_oto_adapt_ratio - 1.0) * gravity_adapt_state + arriving_gravity_head;
        SimTK::Vec3 estimated_linear_acceleration_head =
            arriving_otolith_specific_force_head + adapted_gravity_head;

        double head_pivot_radius = 0.15;
        double expected_tangential_acceleration_x = descending_traj_acc_pitch * head_pivot_radius;
        double expected_tangential_acceleration_z = descending_traj_acc_roll  * head_pivot_radius;

        double phasic_otolith_input_pitch = estimated_linear_acceleration_head[0] - expected_tangential_acceleration_x;
        double phasic_otolith_input_roll  = estimated_linear_acceleration_head[2] - expected_tangential_acceleration_z;
        double phasic_otolith_input_yaw   = 0.0;

        static const double vest_tilt_thresh = std::getenv("VEST_TILT_THRESH") ? std::stod(std::getenv("VEST_TILT_THRESH")) : 0.002;
        static const double vest_scc_thresh  = std::getenv("VEST_SCC_THRESH")  ? std::stod(std::getenv("VEST_SCC_THRESH"))  : 0.005;
        static const double vest_oto_thresh  = std::getenv("VEST_OTO_THRESH")  ? std::stod(std::getenv("VEST_OTO_THRESH"))  : 0.020;
        auto deadzone = [](double v, double th) {
            return (th <= 0.0) ? v : ((std::abs(v) <= th) ? 0.0 : ((v > 0.0) ? (v - th) : (v + th)));
        };
        tonic_otolith_input_pitch = deadzone(tonic_otolith_input_pitch, vest_tilt_thresh);
        tonic_otolith_input_roll  = deadzone(tonic_otolith_input_roll,  vest_tilt_thresh);
        tonic_otolith_input_yaw   = deadzone(tonic_otolith_input_yaw,   vest_tilt_thresh);
        semicircular_canal_input_pitch = deadzone(semicircular_canal_input_pitch, vest_scc_thresh);
        semicircular_canal_input_roll  = deadzone(semicircular_canal_input_roll,  vest_scc_thresh);
        semicircular_canal_input_yaw   = deadzone(semicircular_canal_input_yaw,   vest_scc_thresh);
        phasic_otolith_input_pitch = deadzone(phasic_otolith_input_pitch, vest_oto_thresh);
        phasic_otolith_input_roll  = deadzone(phasic_otolith_input_roll,  vest_oto_thresh);

        double tonic_otolith_state_pitch = state_variable_tonic_otolith[0]->getValue(state);
        double tonic_otolith_state_roll  = state_variable_tonic_otolith[1]->getValue(state);
        double tonic_otolith_state_yaw   = state_variable_tonic_otolith[2]->getValue(state);

        double semicircular_canal_state_pitch = state_variable_semicircular_canal_reflex[0]->getValue(state);
        double semicircular_canal_state_roll  = state_variable_semicircular_canal_reflex[1]->getValue(state);
        double semicircular_canal_state_yaw   = state_variable_semicircular_canal_reflex[2]->getValue(state);

        double phasic_otolith_state_pitch = state_variable_phasic_otolith[0]->getValue(state);
        double phasic_otolith_state_roll  = state_variable_phasic_otolith[1]->getValue(state);
        double phasic_otolith_state_yaw   = state_variable_phasic_otolith[2]->getValue(state);

        double eff_p_ton  = (sv_adapt_theta[1] && get_enable_adaptation()) ? sv_adapt_theta[1]->getValue(state) : get_p_ton();
        double eff_p_sc   = (sv_adapt_theta[4] && get_enable_adaptation()) ? sv_adapt_theta[4]->getValue(state) : get_p_sc();
        double eff_p_phas = (sv_adapt_theta[6] && get_enable_adaptation()) ? sv_adapt_theta[6]->getValue(state) : get_p_phas();

        state_variable_tonic_otolith[0]->setDerivative(state, -eff_p_ton * tonic_otolith_state_pitch + tonic_otolith_input_pitch);
        state_variable_tonic_otolith[1]->setDerivative(state, -eff_p_ton * tonic_otolith_state_roll  + tonic_otolith_input_roll);
        state_variable_tonic_otolith[2]->setDerivative(state, -eff_p_ton * tonic_otolith_state_yaw   + tonic_otolith_input_yaw);

        state_variable_semicircular_canal_reflex[0]->setDerivative(state, -eff_p_sc * semicircular_canal_state_pitch + semicircular_canal_input_pitch);
        state_variable_semicircular_canal_reflex[1]->setDerivative(state, -eff_p_sc * semicircular_canal_state_roll  + semicircular_canal_input_roll);
        state_variable_semicircular_canal_reflex[2]->setDerivative(state, -eff_p_sc * semicircular_canal_state_yaw   + semicircular_canal_input_yaw);

        state_variable_phasic_otolith[0]->setDerivative(state, -eff_p_phas * phasic_otolith_state_pitch + phasic_otolith_input_pitch);
        state_variable_phasic_otolith[1]->setDerivative(state, -eff_p_phas * phasic_otolith_state_roll  + phasic_otolith_input_roll);
        state_variable_phasic_otolith[2]->setDerivative(state, -eff_p_phas * phasic_otolith_state_yaw   + phasic_otolith_input_yaw);
    }

    // --- SUBSYSTEM 6: SPINAL CERVICO-COLLIC REFLEX (CCR) LEAD-LAG FILTERS (72 Muscles, Zero Extra Delay: Driven by Sub 3's τ_prop = 7 ms) ---
    // Total CCR loop latency = τ_prop (7 ms, Sub 3 output) + 0 ms (Sub 6) + τ_spinal (8 ms, Sub 7 output) = 15 ms.
    double p_ccr = (sv_adapt_theta[9] && get_enable_adaptation()) ? sv_adapt_theta[9]->getValue(state) : get_p_ccr();
    DelayedData delayed_proprioceptive_ccr_data = getDelayedData(state.getTime(), get_delay_afferent_proprio(), true);
    const HistoryRecord& cervico_collic_record = delayed_proprioceptive_ccr_data.record;
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

    // --- CONTINUOUS 12-PARAMETER LYAPUNOV HARMONIC OBSERVER & SPACE-MAPPED BODE ADAPTIVE CONTROL ---
    double t_now = state.getTime();
    double p1_val = cached_coordinate_pitch1 ? cached_coordinate_pitch1->getValue(state) : 0.0;
    double p2_val = cached_coordinate_pitch2 ? cached_coordinate_pitch2->getValue(state) : 0.0;
    double head_pitch_rad = p1_val + p2_val;
    double head_pitch_deg = head_pitch_rad * 180.0 / SimTK::Pi;
    double ref_pitch_deg = computeReferencePitchDeg(t_now, get_multisine_scale());
    double ref_pitch_rad = ref_pitch_deg * SimTK::Pi / 180.0;

    double err_pos_rad = head_pitch_rad - ref_pitch_rad;
    last_error_deg = err_pos_rad * 180.0 / SimTK::Pi;
    last_ref_pitch_deg = ref_pitch_deg;
    last_actual_pitch_deg = head_pitch_deg;

    const double freqs[6] = {0.324, 0.783, 1.099, 2.446, 5.583, 7.927};
    const double targets_gain[6] = {86.55, 236.93, 319.74, 423.34, 503.30, 572.69};
    const double targets_phase_deg[6] = {-7.03, -96.96, -109.23, -161.34, -201.38, -247.62};
    const double m_scale = get_multisine_scale();
    const double t_delay = 1.5;
    const double t_ramp  = 1.5;

    if (t_now < t_delay + t_ramp) {
        for (int k = 0; k < 6; ++k) {
            sv_psif_x1[k]->setDerivative(state, 0.0);
            sv_psif_x1[k + 6]->setDerivative(state, 0.0);
            sv_psif_x2[k]->setDerivative(state, 0.0);
            sv_psif_x2[k + 6]->setDerivative(state, 0.0);
        }
    } else {
        double t_rel = t_now - t_delay;
        double A_arr[6], psi_arr[6];
        double theta_recon_deg = 0.0;

        for (int k = 0; k < 6; ++k) {
            double f = freqs[k];
            double omega_k = 2.0 * SimTK::Pi * f;
            double phi_k = SimTK::Pi * k * (k - 1) / 6.0;
            double base_amp = 0.0215 * std::pow(0.835 / f, 2);
            A_arr[k] = std::max(1e-6, base_amp * m_scale);
            psi_arr[k] = omega_k * t_rel + phi_k;

            double S_hat_k = sv_psif_x1[k + 6]->getValue(state);
            double C_hat_k = sv_psif_x2[k + 6]->getValue(state);
            theta_recon_deg += A_arr[k] * (S_hat_k * std::sin(psi_arr[k]) + C_hat_k * std::cos(psi_arr[k]));
        }

        double e_resid_deg = head_pitch_deg - theta_recon_deg;

        for (int k = 0; k < 6; ++k) {
            double s1 = sv_psif_x1[k]->getValue(state);
            double s2 = sv_psif_x1[k + 6]->getValue(state);
            double c1 = sv_psif_x2[k]->getValue(state);
            double c2 = sv_psif_x2[k + 6]->getValue(state);

            double A_eff = std::max(A_arr[k], 0.0005);
            double dS_innov = 2.0 * e_resid_deg * std::sin(psi_arr[k]) / A_eff;
            double dC_innov = 2.0 * e_resid_deg * std::cos(psi_arr[k]) / A_eff;

            double omega_lp = 0.25 * (2.0 * SimTK::Pi * freqs[k]);
            sv_psif_x1[k]->setDerivative(state,     omega_lp * (s2 + dS_innov - s1));
            sv_psif_x1[k + 6]->setDerivative(state, omega_lp * (s1 - s2));
            sv_psif_x2[k]->setDerivative(state,     omega_lp * (c2 + dC_innov - c1));
            sv_psif_x2[k + 6]->setDerivative(state, omega_lp * (c1 - c2));
        }
    }

    if (get_enable_adaptation() && t_now >= get_adaptation_start_time()) {
        double mult = get_adaptation_rate_multiplier();
        if (mult < 0.01) mult = 0.01;

        double taper = 1.0;
        if (t_now >= 18.0) {
            taper = 0.0;
        } else if (t_now > 13.0) {
            double u_tap = (t_now - 13.0) / 5.0;
            taper = 0.5 * (1.0 + std::cos(SimTK::Pi * u_tap));
        }

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

        // Scale-invariant physical bounds relative to initial seed (with absolute floors/ceilings)
        const double theta_min[12] = {
            std::max(0.01, 0.15 * theta_0[0]),
            5.0,
            std::max(0.01, 0.15 * theta_0[2]),
            0.5,
            15.0,
            std::max(0.10, 0.15 * theta_0[5]),
            1.0,
            std::max(0.02, 0.15 * theta_0[7]),
            0.5,
            20.0,
            std::max(0.05, 0.20 * theta_0[10]),
            std::max(0.05, 0.20 * theta_0[11])
        };

        const double theta_max[12] = {
            std::max(3.00,  4.0 * theta_0[0]),
            180.0,
            std::max(1.50,  4.0 * theta_0[2]),
            120.0,
            200.0,
            std::max(28.00, 3.0 * theta_0[5]),
            80.0,
            std::max(2.00,  4.0 * theta_0[7]),
            100.0,
            300.0,
            std::max(1.80,  2.5 * theta_0[10]),
            std::max(1.80,  2.5 * theta_0[11])
        };

        double th_curr[12];
        for (int i = 0; i < 12; ++i) {
            th_curr[i] = sv_adapt_theta[i] ? sv_adapt_theta[i]->getValue(state) : theta_0[i];
        }

        double kp_task_eff = (get_Kp_task_pitch() >= 0.0) ? get_Kp_task_pitch() : 1.5;
        double kd_task_eff = (get_Kd_task_pitch() >= 0.0) ? get_Kd_task_pitch() : 0.15;
        double ki_task_eff = (get_Ki_task_pitch() >= 0.0) ? get_Ki_task_pitch() : get_Ki_task();

        double err_gain_norm[6]  = {0.0};
        double err_phase_norm[6] = {0.0};

        for (int k = 0; k < 6; ++k) {
            double S_hat = sv_psif_x1[k + 6]->getValue(state);
            double C_hat = sv_psif_x2[k + 6]->getValue(state);
            double sim_gain_demod  = std::sqrt(S_hat * S_hat + C_hat * C_hat);
            double sim_phase_demod = std::atan2(C_hat, S_hat) * 180.0 / SimTK::Pi;

            std::complex<double> G_sm = evalSpaceMappedBode(k, th_curr, theta_0, kp_task_eff, kd_task_eff, ki_task_eff);
            double sm_gain  = std::abs(G_sm);
            double sm_phase = std::arg(G_sm) * 180.0 / SimTK::Pi;

            const double w_demod[6] = {0.55, 0.50, 0.50, 0.35, 0.15, 0.10};
            double alpha_k = w_demod[k];

            double eff_gain = (sim_gain_demod > 5.0)
                              ? std::pow(sim_gain_demod, alpha_k) * std::pow(std::max(5.0, sm_gain), 1.0 - alpha_k)
                              : std::max(5.0, sm_gain);

            double d_demod_tgt = std::remainder(sim_phase_demod - targets_phase_deg[k], 360.0);
            double d_sm_tgt    = std::remainder(sm_phase        - targets_phase_deg[k], 360.0);
            double phase_diff_deg = alpha_k * d_demod_tgt + (1.0 - alpha_k) * d_sm_tgt;

            double gain_db_diff = 20.0 * std::log10(std::max(1e-3, eff_gain) / targets_gain[k]);
            err_gain_norm[k]  = gain_db_diff / 1.0;
            err_phase_norm[k] = phase_diff_deg / 10.0;
        }

        const double gamma_hbac = 0.035 * mult * taper;

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

            double grad_i  = 0.0;
            double hess_ii = 0.0;

            for (int k = 0; k < 6; ++k) {
                std::complex<double> G_p = evalSpaceMappedBode(k, th_plus,  theta_0, kp_task_eff, kd_task_eff, ki_task_eff);
                std::complex<double> G_m = evalSpaceMappedBode(k, th_minus, theta_0, kp_task_eff, kd_task_eff, ki_task_eff);

                std::complex<double> d_lnG = std::log(G_p / G_m) / (2.0 * d_th);
                double s_gain_k  = theta_0[i] * (20.0 / std::log(10.0)) * d_lnG.real() / 1.0;
                double s_phase_k = theta_0[i] * (180.0 / SimTK::Pi)     * d_lnG.imag() / 10.0;

                grad_i  += s_gain_k * err_gain_norm[k] + s_phase_k * err_phase_norm[k];
                hess_ii += s_gain_k * s_gain_k + s_phase_k * s_phase_k;
            }

            double norm_step = grad_i / (std::sqrt(hess_ii) + 0.35);
            norm_step = std::clamp(norm_step, -1.5, 1.5);

            double dth = -gamma_hbac * theta_0[i] * norm_step - taper * get_sigma_leakage() * (th_i - theta_0[i]);

            if (th_i < theta_min[i]) {
                double viol = (theta_min[i] - th_i) / std::max(1e-3, theta_0[i]);
                dth += 1000.0 * theta_0[i] * viol * viol * viol;
            } else if (th_i > theta_max[i]) {
                double viol = (th_i - theta_max[i]) / std::max(1e-3, theta_0[i]);
                dth -= 1000.0 * theta_0[i] * viol * viol * viol;
            }
            sv_adapt_theta[i]->setDerivative(state, dth);
        }
    } else {
        for (int i = 0; i < 12; ++i) {
            if (sv_adapt_theta[i]) sv_adapt_theta[i]->setDerivative(state, 0.0);
        }
    }
}

void HeadNeckNeuralControllerAdaptive::computeControls(const SimTK::State& state, SimTK::Vector& controls) const {
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
    SimTK::Vec3 current_world_angular_acceleration =
        (reconstructed_regular_canal_velocity - luenberger_state_vector) / regular_canal_lead_time_constant;
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
    current_history_record.luenberger_angular_acceleration = current_world_angular_acceleration;
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
        static const double spindle_ccr_threshold = std::getenv("SPINDLE_CCR_THRESH") ? std::stod(std::getenv("SPINDLE_CCR_THRESH")) : 0.5;
        double primary_afferent_above_baseline = std::max(0.0, (primary_afferent_value - baseline_primary_afferent_rates[muscle_index]) - spindle_ccr_threshold);
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
    const double vestibular_afferent_delay = 0.002; // 2 ms CN VIII afferent delay from Sub 2 to Sub 4 & Sub 5
    double spinal_execution_delay          = std::max(0.0, get_delay_vcr() - vestibular_afferent_delay); // 8 ms shared spinal/ventral-root delay at Sub 7 output
    double unified_descending_delay        = std::max(0.0, (0.5 * get_delay_voluntary()) - spinal_execution_delay); // 12 ms single descending delay from M1 (Sub 1) to Sub 5 & Sub 7

    DelayedData delayed_vestibular_afferent_data     = getDelayedData(current_simulation_time, vestibular_afferent_delay);
    DelayedData delayed_corticobulbar_voluntary_data = getDelayedData(current_simulation_time, unified_descending_delay);
    DelayedData delayed_proprioceptive_ccr_data      = getDelayedData(current_simulation_time, get_delay_afferent_proprio(), true);

    // --- SUBSYSTEM 5: BRAINSTEM VESTIBULO-COLLIC REFLEX (VCR) COMPUTATION (All Inputs Time-Aligned at t - 2 ms, Zero Extra Output Delay) ---
    double efference_forward_prediction_dt = get_use_smith_predictor() ? std::max(0.0, unified_descending_delay - vestibular_afferent_delay) : 0.0; // 10 ms forward prediction to t - 2ms
    SimTK::Vec3 current_brainstem_vcr_torque(0.0);
    {
        double descending_traj_pos_pitch = (!history.empty() && delayed_corticobulbar_voluntary_data.record.valid) ? delayed_corticobulbar_voluntary_data.record.trajectory_position[0] : trajectory_position_pitch;
        double descending_traj_pos_roll  = (!history.empty() && delayed_corticobulbar_voluntary_data.record.valid) ? delayed_corticobulbar_voluntary_data.record.trajectory_position[1] : trajectory_position_roll;
        double descending_traj_pos_yaw   = (!history.empty() && delayed_corticobulbar_voluntary_data.record.valid) ? delayed_corticobulbar_voluntary_data.record.trajectory_position[2] : trajectory_position_yaw;
        double descending_traj_vel_pitch = (!history.empty() && delayed_corticobulbar_voluntary_data.record.valid) ? delayed_corticobulbar_voluntary_data.record.trajectory_velocity[0] : trajectory_velocity_pitch;
        double descending_traj_vel_roll  = (!history.empty() && delayed_corticobulbar_voluntary_data.record.valid) ? delayed_corticobulbar_voluntary_data.record.trajectory_velocity[1] : trajectory_velocity_roll;
        double descending_traj_vel_yaw   = (!history.empty() && delayed_corticobulbar_voluntary_data.record.valid) ? delayed_corticobulbar_voluntary_data.record.trajectory_velocity[2] : trajectory_velocity_yaw;
        double descending_traj_acc_pitch = (!history.empty() && delayed_corticobulbar_voluntary_data.record.valid) ? delayed_corticobulbar_voluntary_data.record.trajectory_acceleration[0] : trajectory_acceleration_pitch;
        double descending_traj_acc_roll  = (!history.empty() && delayed_corticobulbar_voluntary_data.record.valid) ? delayed_corticobulbar_voluntary_data.record.trajectory_acceleration[1] : trajectory_acceleration_roll;
        double descending_traj_acc_yaw   = (!history.empty() && delayed_corticobulbar_voluntary_data.record.valid) ? delayed_corticobulbar_voluntary_data.record.trajectory_acceleration[2] : trajectory_acceleration_yaw;

        // Dedicated Cerebellar Efference Forward Predictor Block on Input 4 (Δt_fwd = 10 ms: advances t - 12ms -> t - 2ms):
        double predicted_descending_traj_pos_pitch = descending_traj_pos_pitch + efference_forward_prediction_dt * descending_traj_vel_pitch;
        double predicted_descending_traj_pos_roll  = descending_traj_pos_roll  + efference_forward_prediction_dt * descending_traj_vel_roll;
        double predicted_descending_traj_pos_yaw   = descending_traj_pos_yaw   + efference_forward_prediction_dt * descending_traj_vel_yaw;

        double predicted_descending_traj_vel_pitch = descending_traj_vel_pitch + efference_forward_prediction_dt * descending_traj_acc_pitch;
        double predicted_descending_traj_vel_roll  = descending_traj_vel_roll  + efference_forward_prediction_dt * descending_traj_acc_roll;
        double predicted_descending_traj_vel_yaw   = descending_traj_vel_yaw   + efference_forward_prediction_dt * descending_traj_acc_yaw;

        // Input 1 from Subsystem 4 (arriving via Cerebellovestibular projection at t - 2 ms, aligned with Inputs 2, 3, and predicted Input 4):
        SimTK::Vec3 arriving_estimated_head_orientation =
            (!history.empty() && delayed_vestibular_afferent_data.record.valid)
                ? delayed_vestibular_afferent_data.record.estimated_head_orientation
                : SimTK::Vec3(estimated_head_roll, estimated_head_yaw, estimated_head_pitch);
        double tonic_otolith_input_pitch = (arriving_estimated_head_orientation[2] - baseline_spatial_orientation[2]) - predicted_descending_traj_pos_pitch;
        double tonic_otolith_input_roll  = (arriving_estimated_head_orientation[0] - baseline_spatial_orientation[0]) - predicted_descending_traj_pos_roll;
        double tonic_otolith_input_yaw   = (arriving_estimated_head_orientation[1] - baseline_spatial_orientation[1]) - predicted_descending_traj_pos_yaw;

        // Input 2 from Subsystem 2 Irregular Canals (arriving at Brainstem VN after τ_vest = 2 ms CN VIII afferent delay, aligned at t - 2 ms):
        SimTK::Vec3 arriving_irregular_canal_velocity =
            (!history.empty() && delayed_vestibular_afferent_data.record.valid)
                ? delayed_vestibular_afferent_data.record.reconstructed_canal_angular_velocity
                : reconstructed_canal_angular_velocity;
        double semicircular_canal_input_pitch = arriving_irregular_canal_velocity[2] - predicted_descending_traj_vel_pitch;
        double semicircular_canal_input_roll  = arriving_irregular_canal_velocity[0] - predicted_descending_traj_vel_roll;
        double semicircular_canal_input_yaw   = arriving_irregular_canal_velocity[1] - predicted_descending_traj_vel_yaw;

        // Input 3 from Subsystem 2 Irregular Otoliths (arriving at Brainstem VN after τ_vest = 2 ms CN VIII afferent delay)
        // + time-matched delayed R̂_H^W(t - 2 ms) passed through the matched Irregular Adaptation Operator H_adapt,irr(s) = (α_irr + c_irr s) / (1 + c_irr s)
        // so gravity is cancelled across all frequencies (static & dynamic head tilt yield exact 0.0 m/s^2) while preserving the irregular otolith's
        // adaptation zero (c_irr = 0.25 s) and 27.2 ms hair-cell phase lead on true linear acceleration a_lin,H:
        SimTK::Vec3 gravity_vector_world(0.0, -9.81, 0.0);
        SimTK::Vec3 arriving_otolith_specific_force_head =
            (!history.empty() && delayed_vestibular_afferent_data.record.valid)
                ? delayed_vestibular_afferent_data.record.reconstructed_otolith_specific_force
                : reconstructed_otolith_specific_force;
        SimTK::Mat33 arriving_estimated_rotation_head_to_world =
            (!history.empty() && delayed_vestibular_afferent_data.record.valid)
                ? delayed_vestibular_afferent_data.record.estimated_rotation_head_to_world
                : current_estimated_rotation_head_to_world.asMat33();
        SimTK::Vec3 arriving_gravity_head = (~arriving_estimated_rotation_head_to_world) * gravity_vector_world;
        SimTK::Vec3 gravity_adapt_state(
            state_variable_phasic_otolith_stage2[0]->getValue(state),
            state_variable_phasic_otolith_stage2[1]->getValue(state),
            state_variable_phasic_otolith_stage2[2]->getValue(state));
        SimTK::Vec3 adapted_gravity_head =
            (irr_oto_adapt_ratio - 1.0) * gravity_adapt_state + arriving_gravity_head;
        SimTK::Vec3 estimated_linear_acceleration_head =
            arriving_otolith_specific_force_head + adapted_gravity_head;

        double head_pivot_radius = 0.15;
        double expected_tangential_acceleration_x = descending_traj_acc_pitch * head_pivot_radius;
        double expected_tangential_acceleration_z = descending_traj_acc_roll  * head_pivot_radius;

        double phasic_otolith_input_pitch = estimated_linear_acceleration_head[0] - expected_tangential_acceleration_x;
        double phasic_otolith_input_roll  = estimated_linear_acceleration_head[2] - expected_tangential_acceleration_z;
        double phasic_otolith_input_yaw   = 0.0;

        static const double vest_tilt_thresh = std::getenv("VEST_TILT_THRESH") ? std::stod(std::getenv("VEST_TILT_THRESH")) : 0.002;
        static const double vest_scc_thresh  = std::getenv("VEST_SCC_THRESH")  ? std::stod(std::getenv("VEST_SCC_THRESH"))  : 0.005;
        static const double vest_oto_thresh  = std::getenv("VEST_OTO_THRESH")  ? std::stod(std::getenv("VEST_OTO_THRESH"))  : 0.020;
        auto deadzone = [](double v, double th) {
            return (th <= 0.0) ? v : ((std::abs(v) <= th) ? 0.0 : ((v > 0.0) ? (v - th) : (v + th)));
        };
        tonic_otolith_input_pitch = deadzone(tonic_otolith_input_pitch, vest_tilt_thresh);
        tonic_otolith_input_roll  = deadzone(tonic_otolith_input_roll,  vest_tilt_thresh);
        tonic_otolith_input_yaw   = deadzone(tonic_otolith_input_yaw,   vest_tilt_thresh);
        semicircular_canal_input_pitch = deadzone(semicircular_canal_input_pitch, vest_scc_thresh);
        semicircular_canal_input_roll  = deadzone(semicircular_canal_input_roll,  vest_scc_thresh);
        semicircular_canal_input_yaw   = deadzone(semicircular_canal_input_yaw,   vest_scc_thresh);
        phasic_otolith_input_pitch = deadzone(phasic_otolith_input_pitch, vest_oto_thresh);
        phasic_otolith_input_roll  = deadzone(phasic_otolith_input_roll,  vest_oto_thresh);

        double tonic_otolith_state_pitch = state_variable_tonic_otolith[0]->getValue(state);
        double tonic_otolith_state_roll  = state_variable_tonic_otolith[1]->getValue(state);
        double tonic_otolith_state_yaw   = state_variable_tonic_otolith[2]->getValue(state);

        double semicircular_canal_state_pitch = state_variable_semicircular_canal_reflex[0]->getValue(state);
        double semicircular_canal_state_roll  = state_variable_semicircular_canal_reflex[1]->getValue(state);
        double semicircular_canal_state_yaw   = state_variable_semicircular_canal_reflex[2]->getValue(state);

        double phasic_otolith_state_pitch = state_variable_phasic_otolith[0]->getValue(state);
        double phasic_otolith_state_roll  = state_variable_phasic_otolith[1]->getValue(state);
        double phasic_otolith_state_yaw   = state_variable_phasic_otolith[2]->getValue(state);

        double eff_K_ton  = (sv_adapt_theta[0] && get_enable_adaptation()) ? sv_adapt_theta[0]->getValue(state) : get_K_ton();
        double eff_p_ton  = (sv_adapt_theta[1] && get_enable_adaptation()) ? sv_adapt_theta[1]->getValue(state) : get_p_ton();
        double eff_K_sc   = (sv_adapt_theta[2] && get_enable_adaptation()) ? sv_adapt_theta[2]->getValue(state) : get_K_sc();
        double z_sc       = (sv_adapt_theta[3] && get_enable_adaptation()) ? sv_adapt_theta[3]->getValue(state) : get_z_sc();
        double p_sc       = (sv_adapt_theta[4] && get_enable_adaptation()) ? sv_adapt_theta[4]->getValue(state) : get_p_sc();
        double eff_K_phas = (sv_adapt_theta[5] && get_enable_adaptation()) ? sv_adapt_theta[5]->getValue(state) : get_K_phas();
        double phasic_otolith_pole = (sv_adapt_theta[6] && get_enable_adaptation()) ? sv_adapt_theta[6]->getValue(state) : get_p_phas();

        double tonic_otolith_output_pitch = eff_K_ton * eff_p_ton * tonic_otolith_state_pitch;
        double tonic_otolith_output_roll  = eff_K_ton * eff_p_ton * tonic_otolith_state_roll;
        double tonic_otolith_output_yaw   = eff_K_ton * eff_p_ton * tonic_otolith_state_yaw;

        if (std::abs(z_sc) < 1e-4) z_sc = (z_sc >= 0.0 ? 1e-4 : -1e-4);
        double semicircular_canal_lead_ratio_sqrt = std::sqrt(std::clamp(std::abs(z_sc) / std::max(1e-4, std::abs(p_sc)), 1e-4, 100.0));
        double semicircular_canal_direct_feedthrough = 1.0 / semicircular_canal_lead_ratio_sqrt;
        double semicircular_canal_state_gain = (z_sc - p_sc) / semicircular_canal_lead_ratio_sqrt;

        double semicircular_canal_output_pitch =
            eff_K_sc * (semicircular_canal_direct_feedthrough * semicircular_canal_input_pitch + semicircular_canal_state_gain * semicircular_canal_state_pitch);
        double semicircular_canal_output_roll =
            eff_K_sc * (semicircular_canal_direct_feedthrough * semicircular_canal_input_roll  + semicircular_canal_state_gain * semicircular_canal_state_roll);
        double semicircular_canal_output_yaw =
            eff_K_sc * (semicircular_canal_direct_feedthrough * semicircular_canal_input_yaw   + semicircular_canal_state_gain * semicircular_canal_state_yaw);

        double phasic_otolith_output_pitch = eff_K_phas * phasic_otolith_pole * phasic_otolith_state_pitch;
        double phasic_otolith_output_roll  = eff_K_phas * phasic_otolith_pole * phasic_otolith_state_roll;
        double phasic_otolith_output_yaw   = eff_K_phas * phasic_otolith_pole * phasic_otolith_state_yaw;

        current_brainstem_vcr_torque[0] = -(tonic_otolith_output_pitch + semicircular_canal_output_pitch + phasic_otolith_output_pitch);
        current_brainstem_vcr_torque[1] = -(tonic_otolith_output_roll  + semicircular_canal_output_roll  + phasic_otolith_output_roll);
        current_brainstem_vcr_torque[2] = -(tonic_otolith_output_yaw   + semicircular_canal_output_yaw   + phasic_otolith_output_yaw);
    }

    // --- SUBSYSTEM 6: SPINAL CERVICO-COLLIC REFLEX (CCR) LEAD-LAG FILTERS & STRETCH RECTIFICATION (72 Muscles, Zero Extra Delay: Driven by Sub 3's τ_prop = 7 ms) ---
    const HistoryRecord& cervico_collic_record = delayed_proprioceptive_ccr_data.record;
    double p_ccr = (sv_adapt_theta[9] && get_enable_adaptation()) ? sv_adapt_theta[9]->getValue(state) : get_p_ccr();
    double z_ccr = (sv_adapt_theta[8] && get_enable_adaptation()) ? sv_adapt_theta[8]->getValue(state) : get_z_ccr();
    if (std::abs(z_ccr) < 1e-4) z_ccr = (z_ccr >= 0.0 ? 1e-4 : -1e-4);
    double cervico_collic_lead_ratio_sqrt = std::sqrt(std::clamp(std::abs(z_ccr) / std::max(1e-4, std::abs(p_ccr)), 1e-4, 100.0));
    double cervico_collic_direct_feedthrough = 1.0 / cervico_collic_lead_ratio_sqrt;
    double cervico_collic_state_gain = (z_ccr - p_ccr) / cervico_collic_lead_ratio_sqrt;
    double proprioceptive_gain = (sv_adapt_theta[7] && get_enable_adaptation()) ? sv_adapt_theta[7]->getValue(state) : get_Kp_proprioception();
    bool has_cervico_collic_afferents = (cervico_collic_record.primary_afferent_firing_rate.size() == number_of_active_muscles);
    std::vector<double> spinal_ccr_drive(number_of_active_muscles, 0.0);
    if (has_cervico_collic_afferents) {
        for (int muscle_index = 0; muscle_index < number_of_active_muscles; ++muscle_index) {
            double cervico_collic_state = state_variable_cervico_collic[muscle_index]->getValue(state);
            double delayed_primary_afferent_rate = cervico_collic_record.primary_afferent_firing_rate[muscle_index];
            double filtered_primary_afferent_rate =
                cervico_collic_direct_feedthrough * delayed_primary_afferent_rate +
                cervico_collic_state_gain * cervico_collic_state;
            double rectified_primary_afferent_rate = std::max(0.0, filtered_primary_afferent_rate);
            spinal_ccr_drive[muscle_index] = proprioceptive_gain * (rectified_primary_afferent_rate / 100.0);
        }
    }

    // --- SUBSYSTEM 7: CONVERGING MOTOR COMMANDS (VOLUNTARY, VCR, CCR), ACTIVE-SET QP, LAMINA IX POOL & 8 MS SPINAL EFFERENT DELAY ---
    Vector vestibulo_collic_torque(3, 0.0); // Input 2: Restorative Vestibular Reflexes (directly from Sub 5; 2 ms input + 8 ms Sub 7 output = 10 ms total VCR)
    Vector voluntary_torque(3, 0.0);        // Input 1: Descending Voluntary Task Tracking (arriving from Sub 1 after 12 ms + 8 ms Sub 7 output = 20 ms total desc)

    // 1. Filtered VCR Drive from Subsystem 5 (0 ms extra delay between Sub 5 and Sub 7)
    vestibulo_collic_torque[0] = current_brainstem_vcr_torque[0];
    vestibulo_collic_torque[1] = current_brainstem_vcr_torque[1];
    vestibulo_collic_torque[2] = current_brainstem_vcr_torque[2];

    // 2. Filtered Voluntary Drive from Subsystem 1 (20 ms Ascending + 2 Hz Cortical LPF + 12 ms Unified Descending Delay to Sub 7)
    if (!history.empty() && delayed_corticobulbar_voluntary_data.record.valid) {
        voluntary_torque[0] = delayed_corticobulbar_voluntary_data.record.cortical_voluntary_torque[0];
        voluntary_torque[1] = delayed_corticobulbar_voluntary_data.record.cortical_voluntary_torque[1];
        voluntary_torque[2] = delayed_corticobulbar_voluntary_data.record.cortical_voluntary_torque[2];
    } else {
        voluntary_torque[0] = current_cortical_voluntary_torque[0];
        voluntary_torque[1] = current_cortical_voluntary_torque[1];
        voluntary_torque[2] = current_cortical_voluntary_torque[2];
    }

    // --- STAGE 1: DESCENDING TASK-TORQUE SUMMING JUNCTION & CLAMP ---
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

    // --- STAGE 2: 15-ITERATION ACTIVE-SET QP SYNERGY SOLVER ---
    const std::vector<double>& maximum_isometric_forces = cached_maximum_isometric_forces;

    std::vector<double> alpha_motor_excitations(number_of_active_muscles);
    for (int muscle_index = 0; muscle_index < number_of_active_muscles; ++muscle_index) {
        alpha_motor_excitations[muscle_index] = cached_minimum_excitations[muscle_index];
    }
    std::vector<bool> is_muscle_free(number_of_active_muscles, true);

    const char* env_6dof_qp = std::getenv("USE_6DOF_QP");
    const bool use_6dof_qp = (env_6dof_qp != nullptr && std::string(env_6dof_qp) == "1");
    const int qp_dof = use_6dof_qp ? 6 : 3;
    const Matrix& active_jacobian = use_6dof_qp ? moment_arm_jacobian_6dof : moment_arm_jacobian_3dof;
    Vector target_torque_qp(qp_dof, 0.0);
    if (use_6dof_qp) {
        target_torque_qp[0] = desired_total_torque[0]; // pitch1
        target_torque_qp[1] = desired_total_torque[0]; // pitch2
        target_torque_qp[2] = desired_total_torque[1]; // roll1
        target_torque_qp[3] = desired_total_torque[1]; // roll2
        target_torque_qp[4] = desired_total_torque[2]; // yaw1
        target_torque_qp[5] = desired_total_torque[2]; // yaw2
    } else {
        target_torque_qp[0] = desired_total_torque[0];
        target_torque_qp[1] = desired_total_torque[1];
        target_torque_qp[2] = desired_total_torque[2];
    }

    Matrix gram_matrix_qp(qp_dof, qp_dof);
    Vector remaining_torque_vector(qp_dof);
    Vector lagrange_multiplier_vector(qp_dof);
    for (int iteration = 0; iteration < 15; ++iteration) {
        Vector fixed_muscle_torque(qp_dof, 0.0);
        for (int muscle_index = 0; muscle_index < number_of_active_muscles; ++muscle_index) {
            if (!is_muscle_free[muscle_index]) {
                double delta_u_fixed = alpha_motor_excitations[muscle_index] - cached_minimum_excitations[muscle_index];
                if (delta_u_fixed > 0.0) {
                    for (int axis_index = 0; axis_index < qp_dof; ++axis_index) {
                        fixed_muscle_torque[axis_index] +=
                            (active_jacobian(axis_index, muscle_index) * maximum_isometric_forces[muscle_index]) *
                            delta_u_fixed;
                    }
                }
            }
        }

        for (int axis_index = 0; axis_index < qp_dof; ++axis_index) {
            remaining_torque_vector[axis_index] = target_torque_qp[axis_index] - fixed_muscle_torque[axis_index];
        }

        gram_matrix_qp.setToZero();
        for (int muscle_index = 0; muscle_index < number_of_active_muscles; ++muscle_index) {
            if (is_muscle_free[muscle_index]) {
                for (int row = 0; row < qp_dof; ++row) {
                    for (int column = 0; column < qp_dof; ++column) {
                        gram_matrix_qp(row, column) +=
                            (active_jacobian(row, muscle_index)    * maximum_isometric_forces[muscle_index]) *
                            (active_jacobian(column, muscle_index) * maximum_isometric_forces[muscle_index]);
                    }
                }
            }
        }

        const double qp_tikhonov_regularization = 1e-3;
        for (int axis_index = 0; axis_index < qp_dof; ++axis_index) {
            gram_matrix_qp(axis_index, axis_index) += qp_tikhonov_regularization;
        }

        FactorLU lu_factorization(gram_matrix_qp);
        lu_factorization.solve(remaining_torque_vector, lagrange_multiplier_vector);

        bool bounds_violated = false;
        for (int muscle_index = 0; muscle_index < number_of_active_muscles; ++muscle_index) {
            if (is_muscle_free[muscle_index]) {
                double unconstrained_delta_u = 0.0;
                for (int axis_index = 0; axis_index < qp_dof; ++axis_index) {
                    unconstrained_delta_u +=
                        (active_jacobian(axis_index, muscle_index) * maximum_isometric_forces[muscle_index]) *
                        lagrange_multiplier_vector[axis_index];
                }
                double unconstrained_excitation = cached_minimum_excitations[muscle_index] + unconstrained_delta_u;

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

    // --- STAGE 3 & 4: VOLUNTARY-GATED SHERRINGTON RECIPROCAL INHIBITION & LAMINA IX MOTONEURON POOL SUM ---
    double voluntary_activity_gate = std::clamp(
        (std::abs(trajectory_velocity_pitch) + std::abs(trajectory_velocity_roll) + std::abs(trajectory_velocity_yaw)) / 0.10,
        0.0, 1.0);
    double reciprocal_inhibition_gain = 1.0 - voluntary_activity_gate * (1.0 - get_K_recip_inhib());

    current_history_record.spinal_motor_excitations.resize(number_of_active_muscles);
    current_history_record.spinal_alpha_excitations.resize(number_of_active_muscles);
    for (int muscle_index = 0; muscle_index < number_of_active_muscles; ++muscle_index) {
        double excitation = alpha_motor_excitations[muscle_index];

        if (has_cervico_collic_afferents) {
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

            excitation += spinal_ccr_drive[muscle_index] * inhibition_factor;
        }

        excitation = std::clamp(excitation, cached_minimum_excitations[muscle_index], 1.0);
        current_history_record.spinal_motor_excitations[muscle_index] = excitation;
        current_history_record.spinal_alpha_excitations[muscle_index] = alpha_motor_excitations[muscle_index];
    }

    while (!history.empty() && history.back().first >= current_simulation_time) {
        history.pop_back();
    }
    history.emplace_back(current_simulation_time, std::move(current_history_record));

    double oldest_retained_time = current_simulation_time - 0.100; // keep 100ms
    while (!history.empty() && history.front().first < oldest_retained_time) {
        history.pop_front();
    }

    // --- STAGE 5 & 6: SPINAL CIRCUIT & VENTRAL ROOT EFFERENT DELAY (τ_spinal = 8 ms) + ALPHA-GAMMA MUX ---
    DelayedData delayed_spinal_execution_data = getDelayedData(current_simulation_time, spinal_execution_delay, true);
    const HistoryRecord& spinal_record = delayed_spinal_execution_data.record;
    bool has_delayed_spinal_excitations =
        (spinal_record.valid &&
         spinal_record.spinal_motor_excitations.size() == number_of_active_muscles &&
         spinal_record.spinal_alpha_excitations.size() == number_of_active_muscles);

    Vector muscle_control_vector;
    for (int muscle_index = 0; muscle_index < number_of_active_muscles; ++muscle_index) {
        const Muscle& muscle = muscles.get(active_muscle_indices[muscle_index]);

        double arriving_motor_excitation = has_delayed_spinal_excitations
            ? spinal_record.spinal_motor_excitations[muscle_index]
            : alpha_motor_excitations[muscle_index];
        double arriving_alpha_excitation = has_delayed_spinal_excitations
            ? spinal_record.spinal_alpha_excitations[muscle_index]
            : alpha_motor_excitations[muscle_index];

        arriving_motor_excitation = std::clamp(arriving_motor_excitation, cached_minimum_excitations[muscle_index], 1.0);

        int number_of_controls = muscle.numControls();
        if (muscle_control_vector.size() != number_of_controls) {
            muscle_control_vector.resize(number_of_controls);
        }
        muscle_control_vector[0] = arriving_motor_excitation; // Alpha motor command (arriving at NMJ after τ_spinal = 8 ms)

        // Supraspinal alpha-gamma coactivation driven by descending command arriving_alpha_excitation (no Ia->gamma loop)
        if (number_of_controls >= 3) {
            double eff_k_gdyn  = (sv_adapt_theta[10] && get_enable_adaptation()) ? sv_adapt_theta[10]->getValue(state) : get_K_gamma_dyn();
            double eff_k_gstat = (sv_adapt_theta[11] && get_enable_adaptation()) ? sv_adapt_theta[11]->getValue(state) : get_K_gamma_stat();
            muscle_control_vector[1] = 100.0 * eff_k_gdyn  * arriving_alpha_excitation; // Gamma dynamic (pulses/s)
            muscle_control_vector[2] = 100.0 * eff_k_gstat * arriving_alpha_excitation; // Gamma static (pulses/s)
        }

        muscle.addInControls(muscle_control_vector, controls);
    }
}
