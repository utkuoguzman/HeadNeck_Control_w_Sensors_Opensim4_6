#include "HeadNeckNeuralControllerAdaptive.h"
#include "RegisterTypes_osimHeadNeckNeuralAdaptive.h"
#include <complex>
#include <sstream>
#include <fstream>
#include <OpenSim/Simulation/Model/Muscle.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <iostream>
#include <algorithm>

using namespace OpenSim;
using namespace SimTK;


static const char* const ADAPT_15_NAMES[15] = {
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
    "adapt_K_gamma_stat",
    "adapt_Kp_task_pitch",
    "adapt_Kd_task_pitch",
    "adapt_Ki_task_pitch"
};

static double g_cached_base_gain[6]      = {36.32, 144.09, 226.52, 425.36, 754.36, 366.71};
static double g_cached_base_phase_deg[6] = {46.53, -77.11, -109.85, 173.60, 76.81, 27.67};

static void reloadBaselineBodeAnchor() {
    std::string line_g, line_p;
    std::ifstream fin("logs/current_adaptive_bode_anchor.txt");
    if (fin.is_open()) {
        std::getline(fin, line_g);
        std::getline(fin, line_p);
        fin.close();
    }
    if (line_g.empty()) {
        const char* env_g = std::getenv("OSIM_BASE_GAINS");
        if (!env_g) env_g = std::getenv("ADAPT_BASE_GAINS");
        if (env_g) line_g = std::string(env_g);
    }
    if (line_p.empty()) {
        const char* env_p = std::getenv("OSIM_BASE_PHASES");
        if (!env_p) env_p = std::getenv("ADAPT_BASE_PHASES");
        if (env_p) line_p = std::string(env_p);
    }
    if (!line_g.empty()) {
        std::stringstream ss(line_g);
        std::string item;
        int idx = 0;
        while (std::getline(ss, item, ',') && idx < 6) {
            try { g_cached_base_gain[idx] = std::stod(item); } catch (...) {}
            idx++;
        }
    }
    if (!line_p.empty()) {
        std::stringstream ss(line_p);
        std::string item;
        int idx = 0;
        while (std::getline(ss, item, ',') && idx < 6) {
            try { g_cached_base_phase_deg[idx] = std::stod(item); } catch (...) {}
            idx++;
        }
    }
}

static void getBaselineBodeAnchor(double out_gain[6], double out_phase_deg[6]) {
    for (int k = 0; k < 6; ++k) {
        out_gain[k]      = g_cached_base_gain[k];
        out_phase_deg[k] = g_cached_base_phase_deg[k];
    }
}

static std::complex<double> evalRawClosedLoopBode(double freq_hz, const double th[15], const double th0[15]) {
    const double omega = 2.0 * SimTK::Pi * freq_hz;
    const std::complex<double> s(0.0, omega);
    const std::complex<double> s2(-omega * omega, 0.0);

    const std::complex<double> D_vcr = std::exp(-s * 0.010);
    const std::complex<double> D_ccr = std::exp(-s * 0.015);
    const std::complex<double> D_vol = std::exp(-s * 0.040);
    const std::complex<double> H_act = 1.0 / ((1.0 + 0.015 * s) * (1.0 + 0.040 * s) * (1.0 + 0.025 * s));

    const double K_ton   = std::max(1e-4, th[0]);
    const double p_ton   = std::max(1.0,  th[1]);
    const double K_sc    = std::max(1e-4, th[2]);
    const double tau_5   = (th[3] > 0.0 && th[3] <= 0.050) ? th[3] : 0.0025;
    const double p_sc    = std::max(1.0,  th[4]);
    const double K_phas  = std::max(1e-4, th[5]);
    const double p_phas  = std::max(0.5,  th[6]);
    const double kp_prop = std::max(1e-4, th[7]);
    const double tau_6   = (th[8] > 0.0 && th[8] <= 0.050) ? th[8] : 0.0040;
    const double p_ccr   = std::max(1.0,  th[9]);
    const double k_gdyn  = std::clamp(th[10], 0.01, 1.0);
    const double k_gstat = std::clamp(th[11], 0.01, 1.0);
    const double kp_task = std::max(0.0,  th[12]);
    const double kd_task = std::max(0.0,  th[13]);
    const double ki_task = std::max(0.1,  th[14]);

    const double I_pivot     = 0.0953;
    const double B_pass_base = 0.95;
    const double K_pass_base = 18.00;
    const std::complex<double> s_post_norm = s / (2.0 * SimTK::Pi * 1.8);
    const std::complex<double> H_post_lp = 1.0 / (1.0 + s_post_norm * s_post_norm + 1.4 * s_post_norm);
    // Activation-dependent tonic co-contraction stiffness (rolls off above 1.8 Hz so D_CL enters 2nd-quadrant inertia regime at 5.583 Hz)
    const double K_cocontr = 36.0 * kp_prop * k_gstat + 22.0 * K_ton + 1.65 * ki_task + 18.0 * kp_task;
    const double B_cocontr = 3.2 * K_sc + 14.0 * kd_task;

    const double omega_vol = 2.0 * SimTK::Pi * 2.0;
    const std::complex<double> H_vol_lpf = (omega_vol / (s + omega_vol)) * (omega_vol / (s + omega_vol));
    const std::complex<double> Z_vol = 12.0 * (kp_task + kd_task * s + (0.35 * ki_task / s)) * H_vol_lpf * D_vol * H_act;
    const std::complex<double> D_mech = I_pivot * s2 + (B_pass_base + B_cocontr) * s + K_pass_base + K_cocontr * H_post_lp + Z_vol;

    const std::complex<double> C_5 = 1.0 + tau_5 * s + 0.5 * tau_5 * tau_5 * s2;
    const std::complex<double> C_6 = 1.0 + tau_6 * s + 0.5 * tau_6 * tau_6 * s2;
    const std::complex<double> H_ton = (p_ton / (s + p_ton)) * (p_ton / (s + p_ton));
    const std::complex<double> H_sc  = (p_sc  / (s + p_sc))  * (p_sc  / (s + p_sc));
    const std::complex<double> H_ccr = (p_ccr / (s + p_ccr)) * (p_ccr / (s + p_ccr));

    const std::complex<double> Z_ton = 18.0 * K_ton * H_ton * C_5 * D_vcr * H_act;

    const std::complex<double> H_canal = 1.0 + 0.02442 * s;
    const std::complex<double> Z_sc = 12.0 * K_sc * H_sc * C_5 * H_canal * s * D_vcr * H_act;

    const std::complex<double> H_spindle = 9.5 * ((1.0 + 0.45 * k_gstat) + (0.045 + 0.065 * k_gdyn) * s);
    const std::complex<double> Z_ccr = 4.2 * kp_prop * H_ccr * C_6 * H_spindle * D_ccr * H_act;

    const std::complex<double> D_CL = D_mech + Z_ton + Z_sc + Z_ccr;
    const double m_head_r = 4.30 * 0.13;
    const double H_phas_amp = K_phas * ((p_phas * p_phas) / (omega * omega + p_phas * p_phas));
    const std::complex<double> H_phas_mag = std::polar(H_phas_amp, -0.08 * omega / (omega + p_phas));
    const std::complex<double> N_CL = (-s2) * (m_head_r + 0.16 * H_phas_mag) * (180.0 / SimTK::Pi);

    return N_CL / D_CL;
}

static std::complex<double> evalSpaceMappedBode(int k_idx, const double th[15], const double th0[15]) {
    const double freqs[6] = {0.324, 0.783, 1.099, 2.446, 5.583, 7.927};
    double osim_base_gain[6], osim_base_phase_deg[6];
    getBaselineBodeAnchor(osim_base_gain, osim_base_phase_deg);

    double ph_rad = osim_base_phase_deg[k_idx] * SimTK::Pi / 180.0;
    std::complex<double> G_osim_0 = std::polar(osim_base_gain[k_idx], ph_rad);
    std::complex<double> G_raw_0  = evalRawClosedLoopBode(freqs[k_idx], th0, th0);
    std::complex<double> G_raw_th = evalRawClosedLoopBode(freqs[k_idx], th,  th0);

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

static double computeReferencePitchVelDeg(double t, double multisine_scale) {
    const double t_delay = 1.5;
    const double t_ramp = 1.5;
    if (t < t_delay) return 0.0;
    double t_rel = t - t_delay;
    double w = 1.0, dw_dt = 0.0;
    if (t_rel < t_ramp) {
        double tau = t_rel / t_ramp;
        w = 10.0 * std::pow(tau, 3) - 15.0 * std::pow(tau, 4) + 6.0 * std::pow(tau, 5);
        dw_dt = (30.0 * std::pow(tau, 2) - 60.0 * std::pow(tau, 3) + 30.0 * std::pow(tau, 4)) / t_ramp;
    }
    const double freqs[6] = {0.324, 0.783, 1.099, 2.446, 5.583, 7.927};
    const double targets_gain[6] = {86.55, 236.93, 319.74, 423.34, 503.30, 572.69};
    const double targets_phase_deg[6] = {-7.03, -96.96, -109.23, -161.34, -201.38, -247.62};

    double y_ref_deg = 0.0, dy_ref_deg = 0.0;
    for (int k = 0; k < 6; ++k) {
        double f = freqs[k];
        double omega = 2.0 * SimTK::Pi * f;
        double phi_k = SimTK::Pi * k * (k - 1) / 6.0;
        double base_amp = 0.0215 * std::pow(0.835 / f, 2);
        double A_k = base_amp * multisine_scale;
        double phase_rad = phi_k + targets_phase_deg[k] * SimTK::Pi / 180.0;
        y_ref_deg  += A_k * targets_gain[k] * std::sin(omega * t_rel + phase_rad);
        dy_ref_deg += A_k * targets_gain[k] * omega * std::cos(omega * t_rel + phase_rad);
    }
    return dw_dt * y_ref_deg + w * dy_ref_deg;
}

HeadNeckNeuralControllerAdaptive::HeadNeckNeuralControllerAdaptive() {

    constructProperty_delay_vcr(0.010);
    constructProperty_delay_ccr(0.015);
    constructProperty_Kp_proprioception(2.5030);
    constructProperty_z_ccr(0.0150);
    constructProperty_p_ccr(100.0);
    constructProperty_K_recip_inhib(0.0); // 0.0 means complete reciprocal inhibition (no antagonist reflex)
    constructProperty_Kg_proprioception(0.0);
    constructProperty_K_vestibular(2.0);
    constructProperty_K_otolith(4.0);
    constructProperty_desired_yaw(0.0);
    constructProperty_desired_pitch(0.0);
    constructProperty_desired_roll(0.0);
    constructProperty_K_ton(74.3907);
    constructProperty_p_ton(15.0);
    constructProperty_K_sc(49.3723);
    constructProperty_z_sc(0.0150);
    constructProperty_p_sc(50.0);
    constructProperty_K_phas(4.9396);
    constructProperty_z_phas(-1.0);
    constructProperty_p_phas(4.0);
    constructProperty_p_phas2(-1.0);
    constructProperty_k_p(2.5030);
    constructProperty_K_gamma_dyn(0.9964);
    constructProperty_K_gamma_stat(0.9951);
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
    addStateVariable("ton_vel_pitch", SimTK::Stage::Velocity);
    addStateVariable("ton_vel_roll", SimTK::Stage::Velocity);
    addStateVariable("ton_vel_yaw", SimTK::Stage::Velocity);
    addStateVariable("sc_state_pitch", SimTK::Stage::Velocity);
    addStateVariable("sc_state_roll", SimTK::Stage::Velocity);
    addStateVariable("sc_state_yaw", SimTK::Stage::Velocity);
    addStateVariable("sc_vel_pitch", SimTK::Stage::Velocity);
    addStateVariable("sc_vel_roll", SimTK::Stage::Velocity);
    addStateVariable("sc_vel_yaw", SimTK::Stage::Velocity);
    addStateVariable("phas_state_pitch", SimTK::Stage::Velocity);
    addStateVariable("phas_state_roll", SimTK::Stage::Velocity);
    addStateVariable("phas_state_yaw", SimTK::Stage::Velocity);
    addStateVariable("phas_vel_pitch", SimTK::Stage::Velocity);
    addStateVariable("phas_vel_roll", SimTK::Stage::Velocity);
    addStateVariable("phas_vel_yaw", SimTK::Stage::Velocity);
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
        addStateVariable("ccr_vel_" + muscle_name, SimTK::Stage::Velocity);
        addStateVariable("sp_dL_" + muscle_name, SimTK::Stage::Velocity);
        addStateVariable("sp_v_" + muscle_name, SimTK::Stage::Velocity);
        addStateVariable("sp_fdyn_" + muscle_name, SimTK::Stage::Velocity);
        addStateVariable("sp_fstat_" + muscle_name, SimTK::Stage::Velocity);
    }
    for (int i = 0; i < 15; ++i) {
        addStateVariable(ADAPT_15_NAMES[i], SimTK::Stage::Velocity);
    }
    for (int i = 0; i < 12; ++i) {
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
double HeadNeckNeuralControllerAdaptive::getAdaptKpTaskPitch(const SimTK::State& s) const {
    return sv_adapt_theta[12] ? sv_adapt_theta[12]->getValue(s) : getStateVariableValue(s, "adapt_Kp_task_pitch");
}
double HeadNeckNeuralControllerAdaptive::getAdaptKdTaskPitch(const SimTK::State& s) const {
    return sv_adapt_theta[13] ? sv_adapt_theta[13]->getValue(s) : getStateVariableValue(s, "adapt_Kd_task_pitch");
}
double HeadNeckNeuralControllerAdaptive::getAdaptKiTaskPitch(const SimTK::State& s) const {
    return sv_adapt_theta[14] ? sv_adapt_theta[14]->getValue(s) : getStateVariableValue(s, "adapt_Ki_task_pitch");
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
    setStateVariableValue(state, "ton_state_pitch", 0.0);
    setStateVariableValue(state, "ton_state_roll", 0.0);
    setStateVariableValue(state, "ton_state_yaw", 0.0);
    setStateVariableValue(state, "ton_vel_pitch", 0.0);
    setStateVariableValue(state, "ton_vel_roll", 0.0);
    setStateVariableValue(state, "ton_vel_yaw", 0.0);
    setStateVariableValue(state, "sc_state_pitch", 0.0);
    setStateVariableValue(state, "sc_state_roll", 0.0);
    setStateVariableValue(state, "sc_state_yaw", 0.0);
    setStateVariableValue(state, "sc_vel_pitch", 0.0);
    setStateVariableValue(state, "sc_vel_roll", 0.0);
    setStateVariableValue(state, "sc_vel_yaw", 0.0);
    setStateVariableValue(state, "phas_state_pitch", 0.0);
    setStateVariableValue(state, "phas_state_roll", 0.0);
    setStateVariableValue(state, "phas_state_yaw", 0.0);
    setStateVariableValue(state, "phas_vel_pitch", 0.0);
    setStateVariableValue(state, "phas_vel_roll", 0.0);
    setStateVariableValue(state, "phas_vel_yaw", 0.0);
    // Matched irregular otolith gravity-adaptation states x_g,adapt in Head frame [x, y, z] (initialized to g_H(0) = [0, -9.81, 0])
    setStateVariableValue(state, "phas2_state_pitch", 0.0);
    setStateVariableValue(state, "phas2_state_roll", -9.81);
    setStateVariableValue(state, "phas2_state_yaw", 0.0);

    const auto& muscles = getModel().getMuscles();
    for (int muscle_index = 0; muscle_index < muscles.getSize(); ++muscle_index) {
        std::string muscle_name = muscles.get(muscle_index).getName();
        if ((muscle_name.rfind("r_", 0) == 0 || muscle_name.rfind("l_", 0) == 0) && !get_enable_eye_afferents()) continue;
        setStateVariableValue(state, "ccr_state_" + muscle_name, 0.0);
        setStateVariableValue(state, "ccr_vel_" + muscle_name, 0.0);
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
    setStateVariableValue(state, "adapt_K_gamma_dyn", std::clamp(get_K_gamma_dyn(), 0.0, 1.0));
    setStateVariableValue(state, "adapt_K_gamma_stat", std::clamp(get_K_gamma_stat(), 0.0, 1.0));
    setStateVariableValue(state, "adapt_Kp_task_pitch", (get_Kp_task_pitch() >= 0.0) ? get_Kp_task_pitch() : 1.5);
    setStateVariableValue(state, "adapt_Kd_task_pitch", (get_Kd_task_pitch() >= 0.0) ? get_Kd_task_pitch() : 0.15);
    setStateVariableValue(state, "adapt_Ki_task_pitch", (get_Ki_task_pitch() >= 0.0) ? get_Ki_task_pitch() : get_Ki_task());

    reloadBaselineBodeAnchor();
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
    state_variable_tonic_otolith_vel[0]   = traverseToStateVariable("ton_vel_pitch");
    state_variable_tonic_otolith_vel[1]   = traverseToStateVariable("ton_vel_roll");
    state_variable_tonic_otolith_vel[2]   = traverseToStateVariable("ton_vel_yaw");
    state_variable_semicircular_canal_reflex[0]     = traverseToStateVariable("sc_state_pitch");
    state_variable_semicircular_canal_reflex[1]     = traverseToStateVariable("sc_state_roll");
    state_variable_semicircular_canal_reflex[2]     = traverseToStateVariable("sc_state_yaw");
    state_variable_semicircular_canal_reflex_vel[0] = traverseToStateVariable("sc_vel_pitch");
    state_variable_semicircular_canal_reflex_vel[1] = traverseToStateVariable("sc_vel_roll");
    state_variable_semicircular_canal_reflex_vel[2] = traverseToStateVariable("sc_vel_yaw");
    state_variable_phasic_otolith[0]      = traverseToStateVariable("phas_state_pitch");
    state_variable_phasic_otolith[1]      = traverseToStateVariable("phas_state_roll");
    state_variable_phasic_otolith[2]      = traverseToStateVariable("phas_state_yaw");
    state_variable_phasic_otolith_vel[0]  = traverseToStateVariable("phas_vel_pitch");
    state_variable_phasic_otolith_vel[1]  = traverseToStateVariable("phas_vel_roll");
    state_variable_phasic_otolith_vel[2]  = traverseToStateVariable("phas_vel_yaw");
    state_variable_phasic_otolith_stage2[0] = traverseToStateVariable("phas2_state_pitch");
    state_variable_phasic_otolith_stage2[1] = traverseToStateVariable("phas2_state_roll");
    state_variable_phasic_otolith_stage2[2] = traverseToStateVariable("phas2_state_yaw");
    for (int i = 0; i < 15; ++i) {
        sv_adapt_theta[i] = traverseToStateVariable(ADAPT_15_NAMES[i]);
    }
    for (int i = 0; i < 12; ++i) {
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
    last_voluntary_alpha_excitations.assign(number_of_active_muscles, 0.03);
    cervico_collic_state_names.resize(number_of_active_muscles);
    cervico_collic_vel_state_names.resize(number_of_active_muscles);
    spindle_length_change_state_names.resize(number_of_active_muscles);
    spindle_velocity_state_names.resize(number_of_active_muscles);
    spindle_dynamic_fusimotor_state_names.resize(number_of_active_muscles);
    spindle_static_fusimotor_state_names.resize(number_of_active_muscles);
    state_variable_cervico_collic.resize(number_of_active_muscles, nullptr);
    state_variable_cervico_collic_vel.resize(number_of_active_muscles, nullptr);
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
    cached_optimal_fiber_lengths.resize(number_of_active_muscles, 0.05);
    cached_tendon_slack_lengths.resize(number_of_active_muscles, 0.02);
    cached_cosine_pennation_angles.resize(number_of_active_muscles, 1.0);
    baseline_normalized_fiber_lengths.resize(number_of_active_muscles, 1.0);
    baseline_reference_ia_rates.resize(number_of_active_muscles, 0.0);
    baseline_reference_ii_rates.resize(number_of_active_muscles, 0.0);

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
        last_voluntary_alpha_excitations[active_index] = minimum_excitation_value;
        cervico_collic_state_names[active_index] = "ccr_state_" + muscle_name;
        cervico_collic_vel_state_names[active_index] = "ccr_vel_" + muscle_name;
        spindle_length_change_state_names[active_index] = "sp_dL_" + muscle_name;
        spindle_velocity_state_names[active_index] = "sp_v_" + muscle_name;
        spindle_dynamic_fusimotor_state_names[active_index] = "sp_fdyn_" + muscle_name;
        spindle_static_fusimotor_state_names[active_index] = "sp_fstat_" + muscle_name;
        state_variable_cervico_collic[active_index] = traverseToStateVariable(cervico_collic_state_names[active_index]);
        state_variable_cervico_collic_vel[active_index] = traverseToStateVariable(cervico_collic_vel_state_names[active_index]);
        state_variable_spindle_length_change[active_index] = traverseToStateVariable(spindle_length_change_state_names[active_index]);
        state_variable_spindle_velocity[active_index] = traverseToStateVariable(spindle_velocity_state_names[active_index]);
        state_variable_spindle_dynamic_fusimotor[active_index] = traverseToStateVariable(spindle_dynamic_fusimotor_state_names[active_index]);
        state_variable_spindle_static_fusimotor[active_index] = traverseToStateVariable(spindle_static_fusimotor_state_names[active_index]);

        double optimal_fiber_length = muscle.getOptimalFiberLength();
        double tendon_slack_length  = muscle.getTendonSlackLength();
        double cosine_pennation_angle = std::max(0.5, std::cos(muscle.getPennationAngleAtOptimalFiberLength()));
        cached_optimal_fiber_lengths[active_index] = optimal_fiber_length;
        cached_tendon_slack_lengths[active_index]  = tendon_slack_length;
        cached_cosine_pennation_angles[active_index] = cosine_pennation_angle;
        cached_fiber_length_scales[active_index] = optimal_fiber_length / (369.1427 * cosine_pennation_angle);
        cached_primary_afferent_velocity_scales[active_index] = optimal_fiber_length / (35000.0 * cosine_pennation_angle);
        cached_maximum_isometric_forces[active_index] = muscle.getMaxIsometricForce();
        baseline_muscle_activations[active_index] = muscle.getActivation(state);
        double resting_mt_length = muscle.getLength(state);
        double resting_fiber_length = std::max(0.45 * optimal_fiber_length, (resting_mt_length - tendon_slack_length) / cosine_pennation_angle);
        baseline_normalized_fiber_lengths[active_index] = std::clamp(resting_fiber_length / std::max(1e-4, optimal_fiber_length), 0.50, 1.50);

        double k_gdyn_init  = std::clamp(get_K_gamma_dyn(),  0.0, 1.0);
        double k_gstat_init = std::clamp(get_K_gamma_stat(), 0.0, 1.0);
        double base_gdyn  = 100.0 * k_gdyn_init  * minimum_excitation_value;
        double base_gstat = 100.0 * k_gstat_init * minimum_excitation_value;
        double base_fdyn   = (base_gdyn  * base_gdyn)  / (base_gdyn  * base_gdyn  + 3600.0);
        double base_fstat  = (base_gstat * base_gstat) / (base_gstat * base_gstat + 3600.0);
        double base_fchain = (base_gstat * base_gstat) / (base_gstat * base_gstat + 8100.0);
        double r_ia_ref0 = 0.0, r_ii_ref0 = 0.0, kappa_ref0 = 1.0;
        evaluateReferenceMileusnicSpindle(
            static_cast<int>(active_index),
            baseline_normalized_fiber_lengths[active_index],
            0.0,
            base_fdyn,
            base_fstat,
            base_fchain,
            r_ia_ref0,
            r_ii_ref0,
            kappa_ref0);
        baseline_reference_ia_rates[active_index] = r_ia_ref0;
        baseline_reference_ii_rates[active_index] = r_ii_ref0;

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

    // Precompute C^infty Smooth 6-DOF Quadratic Taylor Moment-Arm Polynomial J(q) around q0 (eliminates 50 ms staircase)
    jacobian_6dof_q0 = moment_arm_jacobian_6dof;
    const Coordinate* coords_6dof[6] = {
        cached_coordinate_pitch1, cached_coordinate_pitch2,
        cached_coordinate_roll1,  cached_coordinate_roll2,
        cached_coordinate_yaw1,   cached_coordinate_yaw2
    };
    for (int d = 0; d < 6; ++d) {
        q0_6dof[d] = coords_6dof[d] ? coords_6dof[d]->getValue(state) : 0.0;
        jacobian_6dof_d1[d].resize(6, number_of_active_muscles);
        jacobian_6dof_d1[d].setToZero();
        jacobian_6dof_d2[d].resize(6, number_of_active_muscles);
        jacobian_6dof_d2[d].setToZero();
    }
    try {
        SimTK::State work_state = state;
        const double dq_step = 0.05; // ~2.86 deg central finite difference step
        for (int d = 0; d < 6; ++d) {
            if (!coords_6dof[d]) continue;
            Coordinate* mut_coord_d = const_cast<Coordinate*>(coords_6dof[d]);

            mut_coord_d->setValue(work_state, q0_6dof[d] + dq_step, true);
            model.realizePosition(work_state);
            SimTK::Matrix J_plus(6, number_of_active_muscles);
            for (int m = 0; m < number_of_active_muscles; ++m) {
                const Muscle& mus = muscles.get(active_muscle_indices[m]);
                for (int c = 0; c < 6; ++c) {
                    J_plus(c, m) = coords_6dof[c] ? mus.computeMomentArm(work_state, *const_cast<Coordinate*>(coords_6dof[c])) : 0.0;
                }
            }

            mut_coord_d->setValue(work_state, q0_6dof[d] - dq_step, true);
            model.realizePosition(work_state);
            SimTK::Matrix J_minus(6, number_of_active_muscles);
            for (int m = 0; m < number_of_active_muscles; ++m) {
                const Muscle& mus = muscles.get(active_muscle_indices[m]);
                for (int c = 0; c < 6; ++c) {
                    J_minus(c, m) = coords_6dof[c] ? mus.computeMomentArm(work_state, *const_cast<Coordinate*>(coords_6dof[c])) : 0.0;
                }
            }

            mut_coord_d->setValue(work_state, q0_6dof[d], true);
            for (int m = 0; m < number_of_active_muscles; ++m) {
                for (int c = 0; c < 6; ++c) {
                    jacobian_6dof_d1[d](c, m) = (J_plus(c, m) - J_minus(c, m)) / (2.0 * dq_step);
                    jacobian_6dof_d2[d](c, m) = (J_plus(c, m) - 2.0 * jacobian_6dof_q0(c, m) + J_minus(c, m)) / (dq_step * dq_step);
                }
            }
        }
        model.realizePosition(state);
    } catch (...) {
        // Fallback to constant J0 if work_state perturbation fails
    }

    updateDecoderMatrix();
    history.clear();
    is_initialized = true;
    last_jacobian_update_time = state.getTime();
}

void HeadNeckNeuralControllerAdaptive::evaluateReferenceMileusnicSpindle(
    int muscle_index,
    double L_hat,
    double Ldot_hat,
    double f_dyn_hat,
    double f_stat_hat,
    double f_chain_hat,
    double& r_ia_hat,
    double& r_ii_hat,
    double& kappa_fd) const
{
    // Exact Mileusnic et al. (2006) spindle model constants matching proprioception_plugin/Mileusnic06Spindle.cpp
    constexpr double Ksr   = 10.4649;
    constexpr double Kpr   = 0.15;
    constexpr double R_len = 0.46;
    constexpr double L0sr  = 0.04;
    constexpr double L0pr  = 0.76;
    constexpr double a_pow = 0.3;
    constexpr double S_occ = 0.156;
    constexpr double X_sec = 0.7;

    constexpr double beta0_b1 = 0.0605;
    constexpr double beta1_b1 = 0.2592;
    constexpr double Gamma1   = 0.0289;
    constexpr double G_pri_b1 = 20000.0;
    constexpr double Lns_b1   = 0.0423;

    constexpr double beta0_b2   = 0.0822;
    constexpr double beta2_b2   = -0.046;
    constexpr double Gamma2     = 0.0636;
    constexpr double G_pri_b2   = 10000.0;
    constexpr double G_sec_b2   = 7250.0;
    constexpr double Lns_b2     = 0.0423;
    constexpr double Lnsec_b2   = 0.89;

    constexpr double beta0_ch   = 0.0822;
    constexpr double beta2_ch   = -0.069;
    constexpr double Gamma_ch   = 0.0954;
    constexpr double G_pri_ch   = 10000.0;
    constexpr double G_sec_ch   = 7250.0;
    constexpr double Lns_ch     = 0.0423;
    constexpr double Lnsec_ch   = 0.89;

    const double fd = std::clamp(f_dyn_hat,   0.0, 1.0);
    const double fs = std::clamp(f_stat_hat,  0.0, 1.0);
    const double fc = std::clamp(f_chain_hat, 0.0, 1.0);

    const double C_asym = (Ldot_hat >= 0.0) ? 1.0 : 0.42;
    const double phi_v  = Ldot_hat * std::pow(Ldot_hat * Ldot_hat + 1e-4, 0.5 * (a_pow - 1.0));
    const double dL_R   = std::max(0.0, L_hat - R_len);
    const double k_sum  = Ksr + Kpr;
    const double k_rat  = Ksr / k_sum; // Ksr / (Ksr + Kpr)

    // 1. Intrafusal bag1 quasi-steady tension T_bag1(L_hat, Ldot_hat, f_dyn_hat)
    const double beta_b1 = beta0_b1 + beta1_b1 * fd;
    const double T_b1    = k_rat * (Kpr * (L_hat - L0sr - L0pr) + C_asym * beta_b1 * dL_R * phi_v + Gamma1 * fd);

    // 2. Intrafusal bag2 quasi-steady tension T_bag2(L_hat, Ldot_hat, f_stat_hat)
    const double beta_b2 = beta0_b2 + beta2_b2 * fs;
    const double T_b2    = k_rat * (Kpr * (L_hat - L0sr - L0pr) + C_asym * beta_b2 * dL_R * phi_v + Gamma2 * fs);

    // 3. Intrafusal nuclear chain quasi-steady tension T_chain(L_hat, Ldot_hat, f_chain_hat)
    const double beta_ch = beta0_ch + beta2_ch * fc;
    const double T_ch    = k_rat * (Kpr * (L_hat - L0sr - L0pr) + C_asym * beta_ch * dL_R * phi_v + Gamma_ch * fc);

    // 4. Primary (Ia) and Secondary (II) afferent firing rates matching Mileusnic06Spindle::getIaOutput & getIIOutput
    const double stretch_pri_b1 = (T_b1 / Ksr) - (Lns_b1 - L0sr);
    const double AP_bag1        = G_pri_b1 * 0.5 * (stretch_pri_b1 + std::sqrt(stretch_pri_b1 * stretch_pri_b1 + 1e-8));

    const double term1_b2 = X_sec * (Lns_b2 / L0sr) * ((T_b2 / Ksr) - (Lns_b2 - L0sr));
    const double term2_b2 = (1.0 - X_sec) * (Lns_b2 / L0pr) * (L_hat - (T_b2 / Ksr) - L0sr - Lnsec_b2);
    const double raw_b2   = term1_b2 + term2_b2;
    const double AP_bag2  = 0.5 * (raw_b2 + std::sqrt(raw_b2 * raw_b2 + 1e-8));

    const double term1_ch = X_sec * (Lns_ch / L0sr) * ((T_ch / Ksr) - (Lns_ch - L0sr));
    const double term2_ch = (1.0 - X_sec) * (Lns_ch / L0pr) * (L_hat - (T_ch / Ksr) - L0sr - Lnsec_ch);
    const double raw_ch   = term1_ch + term2_ch;
    const double AP_chain = 0.5 * (raw_ch + std::sqrt(raw_ch * raw_ch + 1e-8));

    const double pri_stat = G_pri_b2 * AP_bag2 + G_pri_ch * AP_chain;
    const double diff_ia  = AP_bag1 - pri_stat;
    const double abs_diff = std::sqrt(diff_ia * diff_ia + 1e-4);
    const double s_max    = 0.5 * (AP_bag1 + pri_stat + abs_diff);
    const double s_min    = 0.5 * (AP_bag1 + pri_stat - abs_diff);

    r_ia_hat = std::max(0.0, s_max + S_occ * s_min);
    r_ii_hat = std::max(0.0, G_sec_b2 * AP_bag2 + G_sec_ch * AP_chain);

    // 5. Dynamic-Gamma Velocity-Gain Normalization factor kappa(f_d_hat):
    // Removes multiplicative velocity sensitivity inflation of gamma_d on unexpected stretch perturbations:
    //   kappa(f_d_hat) = 1 + (beta_1 * f_d_hat) / (beta_0 + beta_2 * f_s_hat)
    const double denom_kappa = std::max(0.015, beta0_b1 + beta2_b2 * fs);
    kappa_fd = 1.0 + (beta1_b1 * fd) / denom_kappa;
}

void HeadNeckNeuralControllerAdaptive::updateDecoderMatrix() const {
    int number_of_active_muscles = active_muscle_indices.size();
    if (number_of_active_muscles == 0) return;

    // Full 6-DOF cervical spine Jacobian inversion on stack (zero heap allocations): gram_matrix_6dof = J6 * J6^T (6 x 6)
    SimTK::Mat<6, 6> gram_matrix_6dof(0.0);
    for (int muscle_index = 0; muscle_index < number_of_active_muscles; ++muscle_index) {
        for (int row = 0; row < 6; ++row) {
            const double j_row = moment_arm_jacobian_6dof(row, muscle_index);
            for (int column = 0; column < 6; ++column) {
                gram_matrix_6dof(row, column) += j_row * moment_arm_jacobian_6dof(column, muscle_index);
            }
        }
    }
    for (int row = 0; row < 6; ++row) gram_matrix_6dof(row, row) += 1e-8;

    SimTK::Mat<6, 6> gram_matrix_6dof_inverse = gram_matrix_6dof.invert();

    // K6 = -(J6 * J6^T)^{-1} * J6 (6 x number_of_active_muscles), then sum (pitch1+pitch2, roll1+roll2, yaw1+yaw2) into 3 x number_of_active_muscles
    if (proprioceptive_decoder_matrix.nrow() != 3 || proprioceptive_decoder_matrix.ncol() != number_of_active_muscles) {
        proprioceptive_decoder_matrix.resize(3, number_of_active_muscles);
    }
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
        delayed_result.record.estimated_head_orientation_4a = source_record.estimated_head_orientation_4a;
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
        delayed_result.record.mahony_estimated_angular_velocity_4a = source_record.mahony_estimated_angular_velocity_4a;
        delayed_result.record.cortical_voluntary_torque = source_record.cortical_voluntary_torque;
        delayed_result.record.cortical_voluntary_torque_dot = source_record.cortical_voluntary_torque_dot;
        delayed_result.record.cortical_voluntary_torque_ddot = source_record.cortical_voluntary_torque_ddot;
        delayed_result.record.trajectory_position = source_record.trajectory_position;
        delayed_result.record.trajectory_velocity = source_record.trajectory_velocity;
        delayed_result.record.trajectory_acceleration = source_record.trajectory_acceleration;
        delayed_result.record.trajectory_jerk = source_record.trajectory_jerk;
        delayed_result.record.trajectory_position_1a = source_record.trajectory_position_1a;
        delayed_result.record.trajectory_velocity_1b = source_record.trajectory_velocity_1b;
        delayed_result.record.trajectory_acceleration_1b = source_record.trajectory_acceleration_1b;
        delayed_result.record.brainstem_vcr_torque = source_record.brainstem_vcr_torque;
        delayed_result.record.brainstem_vcr_torque_dot = source_record.brainstem_vcr_torque_dot;
        delayed_result.record.brainstem_vcr_torque_ddot = source_record.brainstem_vcr_torque_ddot;
        if (interpolate_spindle_afferents) {
            delayed_result.record.primary_afferent_firing_rate = source_record.primary_afferent_firing_rate;
            delayed_result.record.spinal_ccr_drive = source_record.spinal_ccr_drive;
            delayed_result.record.spinal_ccr_drive_dot = source_record.spinal_ccr_drive_dot;
            delayed_result.record.spinal_ccr_drive_ddot = source_record.spinal_ccr_drive_ddot;
            delayed_result.record.spinal_motor_excitations = source_record.spinal_motor_excitations;
            delayed_result.record.spinal_alpha_excitations = source_record.spinal_alpha_excitations;
            delayed_result.record.spinal_gamma_dyn_commands = source_record.spinal_gamma_dyn_commands;
            delayed_result.record.spinal_gamma_stat_commands = source_record.spinal_gamma_stat_commands;
            delayed_result.record.expected_normalized_fiber_lengths = source_record.expected_normalized_fiber_lengths;
            delayed_result.record.expected_normalized_fiber_velocities = source_record.expected_normalized_fiber_velocities;
        }
        return delayed_result;
    }
    if (iterator_upper == history.begin()) {
        const HistoryRecord& source_record = iterator_upper->second;
        delayed_result.record.valid = source_record.valid;
        delayed_result.record.estimated_head_orientation = source_record.estimated_head_orientation;
        delayed_result.record.estimated_head_orientation_4a = source_record.estimated_head_orientation_4a;
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
        delayed_result.record.mahony_estimated_angular_velocity_4a = source_record.mahony_estimated_angular_velocity_4a;
        delayed_result.record.cortical_voluntary_torque = source_record.cortical_voluntary_torque;
        delayed_result.record.cortical_voluntary_torque_dot = source_record.cortical_voluntary_torque_dot;
        delayed_result.record.cortical_voluntary_torque_ddot = source_record.cortical_voluntary_torque_ddot;
        delayed_result.record.trajectory_position = source_record.trajectory_position;
        delayed_result.record.trajectory_velocity = source_record.trajectory_velocity;
        delayed_result.record.trajectory_acceleration = source_record.trajectory_acceleration;
        delayed_result.record.trajectory_jerk = source_record.trajectory_jerk;
        delayed_result.record.trajectory_position_1a = source_record.trajectory_position_1a;
        delayed_result.record.trajectory_velocity_1b = source_record.trajectory_velocity_1b;
        delayed_result.record.trajectory_acceleration_1b = source_record.trajectory_acceleration_1b;
        delayed_result.record.brainstem_vcr_torque = source_record.brainstem_vcr_torque;
        delayed_result.record.brainstem_vcr_torque_dot = source_record.brainstem_vcr_torque_dot;
        delayed_result.record.brainstem_vcr_torque_ddot = source_record.brainstem_vcr_torque_ddot;
        if (interpolate_spindle_afferents) {
            delayed_result.record.primary_afferent_firing_rate = source_record.primary_afferent_firing_rate;
            delayed_result.record.spinal_ccr_drive = source_record.spinal_ccr_drive;
            delayed_result.record.spinal_ccr_drive_dot = source_record.spinal_ccr_drive_dot;
            delayed_result.record.spinal_ccr_drive_ddot = source_record.spinal_ccr_drive_ddot;
            delayed_result.record.spinal_motor_excitations = source_record.spinal_motor_excitations;
            delayed_result.record.spinal_alpha_excitations = source_record.spinal_alpha_excitations;
            delayed_result.record.spinal_gamma_dyn_commands = source_record.spinal_gamma_dyn_commands;
            delayed_result.record.spinal_gamma_stat_commands = source_record.spinal_gamma_stat_commands;
            delayed_result.record.expected_normalized_fiber_lengths = source_record.expected_normalized_fiber_lengths;
            delayed_result.record.expected_normalized_fiber_velocities = source_record.expected_normalized_fiber_velocities;
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
    interpolated_record.estimated_head_orientation_4a =
        iterator_lower->second.estimated_head_orientation_4a * (1.0 - interpolation_weight) +
        iterator_upper->second.estimated_head_orientation_4a * interpolation_weight;
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
    interpolated_record.mahony_estimated_angular_velocity_4a =
        iterator_lower->second.mahony_estimated_angular_velocity_4a * (1.0 - interpolation_weight) +
        iterator_upper->second.mahony_estimated_angular_velocity_4a * interpolation_weight;
    interpolated_record.cortical_voluntary_torque =
        iterator_lower->second.cortical_voluntary_torque * (1.0 - interpolation_weight) +
        iterator_upper->second.cortical_voluntary_torque * interpolation_weight;
    interpolated_record.cortical_voluntary_torque_dot =
        iterator_lower->second.cortical_voluntary_torque_dot * (1.0 - interpolation_weight) +
        iterator_upper->second.cortical_voluntary_torque_dot * interpolation_weight;
    interpolated_record.cortical_voluntary_torque_ddot =
        iterator_lower->second.cortical_voluntary_torque_ddot * (1.0 - interpolation_weight) +
        iterator_upper->second.cortical_voluntary_torque_ddot * interpolation_weight;
    interpolated_record.trajectory_position =
        iterator_lower->second.trajectory_position * (1.0 - interpolation_weight) +
        iterator_upper->second.trajectory_position * interpolation_weight;
    interpolated_record.trajectory_velocity =
        iterator_lower->second.trajectory_velocity * (1.0 - interpolation_weight) +
        iterator_upper->second.trajectory_velocity * interpolation_weight;
    interpolated_record.trajectory_acceleration =
        iterator_lower->second.trajectory_acceleration * (1.0 - interpolation_weight) +
        iterator_upper->second.trajectory_acceleration * interpolation_weight;
    interpolated_record.trajectory_jerk =
        iterator_lower->second.trajectory_jerk * (1.0 - interpolation_weight) +
        iterator_upper->second.trajectory_jerk * interpolation_weight;
    interpolated_record.trajectory_position_1a =
        iterator_lower->second.trajectory_position_1a * (1.0 - interpolation_weight) +
        iterator_upper->second.trajectory_position_1a * interpolation_weight;
    interpolated_record.trajectory_velocity_1b =
        iterator_lower->second.trajectory_velocity_1b * (1.0 - interpolation_weight) +
        iterator_upper->second.trajectory_velocity_1b * interpolation_weight;
    interpolated_record.trajectory_acceleration_1b =
        iterator_lower->second.trajectory_acceleration_1b * (1.0 - interpolation_weight) +
        iterator_upper->second.trajectory_acceleration_1b * interpolation_weight;
    interpolated_record.brainstem_vcr_torque =
        iterator_lower->second.brainstem_vcr_torque * (1.0 - interpolation_weight) +
        iterator_upper->second.brainstem_vcr_torque * interpolation_weight;
    interpolated_record.brainstem_vcr_torque_dot =
        iterator_lower->second.brainstem_vcr_torque_dot * (1.0 - interpolation_weight) +
        iterator_upper->second.brainstem_vcr_torque_dot * interpolation_weight;
    interpolated_record.brainstem_vcr_torque_ddot =
        iterator_lower->second.brainstem_vcr_torque_ddot * (1.0 - interpolation_weight) +
        iterator_upper->second.brainstem_vcr_torque_ddot * interpolation_weight;

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
        if (iterator_lower->second.spinal_ccr_drive.size() > 0 &&
            iterator_upper->second.spinal_ccr_drive.size() == iterator_lower->second.spinal_ccr_drive.size()) {
            interpolated_record.spinal_ccr_drive.resize(iterator_lower->second.spinal_ccr_drive.size());
            for (int muscle_index = 0; muscle_index < interpolated_record.spinal_ccr_drive.size(); ++muscle_index) {
                interpolated_record.spinal_ccr_drive[muscle_index] =
                    iterator_lower->second.spinal_ccr_drive[muscle_index] * (1.0 - interpolation_weight) +
                    iterator_upper->second.spinal_ccr_drive[muscle_index] * interpolation_weight;
            }
        }
        if (iterator_lower->second.spinal_ccr_drive_dot.size() > 0 &&
            iterator_upper->second.spinal_ccr_drive_dot.size() == iterator_lower->second.spinal_ccr_drive_dot.size()) {
            interpolated_record.spinal_ccr_drive_dot.resize(iterator_lower->second.spinal_ccr_drive_dot.size());
            for (int muscle_index = 0; muscle_index < interpolated_record.spinal_ccr_drive_dot.size(); ++muscle_index) {
                interpolated_record.spinal_ccr_drive_dot[muscle_index] =
                    iterator_lower->second.spinal_ccr_drive_dot[muscle_index] * (1.0 - interpolation_weight) +
                    iterator_upper->second.spinal_ccr_drive_dot[muscle_index] * interpolation_weight;
            }
        }
        if (iterator_lower->second.spinal_ccr_drive_ddot.size() > 0 &&
            iterator_upper->second.spinal_ccr_drive_ddot.size() == iterator_lower->second.spinal_ccr_drive_ddot.size()) {
            interpolated_record.spinal_ccr_drive_ddot.resize(iterator_lower->second.spinal_ccr_drive_ddot.size());
            for (int muscle_index = 0; muscle_index < interpolated_record.spinal_ccr_drive_ddot.size(); ++muscle_index) {
                interpolated_record.spinal_ccr_drive_ddot[muscle_index] =
                    iterator_lower->second.spinal_ccr_drive_ddot[muscle_index] * (1.0 - interpolation_weight) +
                    iterator_upper->second.spinal_ccr_drive_ddot[muscle_index] * interpolation_weight;
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
        if (iterator_lower->second.spinal_gamma_dyn_commands.size() > 0 &&
            iterator_upper->second.spinal_gamma_dyn_commands.size() == iterator_lower->second.spinal_gamma_dyn_commands.size()) {
            interpolated_record.spinal_gamma_dyn_commands.resize(iterator_lower->second.spinal_gamma_dyn_commands.size());
            for (int muscle_index = 0; muscle_index < interpolated_record.spinal_gamma_dyn_commands.size(); ++muscle_index) {
                interpolated_record.spinal_gamma_dyn_commands[muscle_index] =
                    iterator_lower->second.spinal_gamma_dyn_commands[muscle_index] * (1.0 - interpolation_weight) +
                    iterator_upper->second.spinal_gamma_dyn_commands[muscle_index] * interpolation_weight;
            }
        }
        if (iterator_lower->second.spinal_gamma_stat_commands.size() > 0 &&
            iterator_upper->second.spinal_gamma_stat_commands.size() == iterator_lower->second.spinal_gamma_stat_commands.size()) {
            interpolated_record.spinal_gamma_stat_commands.resize(iterator_lower->second.spinal_gamma_stat_commands.size());
            for (int muscle_index = 0; muscle_index < interpolated_record.spinal_gamma_stat_commands.size(); ++muscle_index) {
                interpolated_record.spinal_gamma_stat_commands[muscle_index] =
                    iterator_lower->second.spinal_gamma_stat_commands[muscle_index] * (1.0 - interpolation_weight) +
                    iterator_upper->second.spinal_gamma_stat_commands[muscle_index] * interpolation_weight;
            }
        }
        if (iterator_lower->second.expected_normalized_fiber_lengths.size() > 0 &&
            iterator_upper->second.expected_normalized_fiber_lengths.size() == iterator_lower->second.expected_normalized_fiber_lengths.size()) {
            interpolated_record.expected_normalized_fiber_lengths.resize(iterator_lower->second.expected_normalized_fiber_lengths.size());
            for (int muscle_index = 0; muscle_index < interpolated_record.expected_normalized_fiber_lengths.size(); ++muscle_index) {
                interpolated_record.expected_normalized_fiber_lengths[muscle_index] =
                    iterator_lower->second.expected_normalized_fiber_lengths[muscle_index] * (1.0 - interpolation_weight) +
                    iterator_upper->second.expected_normalized_fiber_lengths[muscle_index] * interpolation_weight;
            }
        }
        if (iterator_lower->second.expected_normalized_fiber_velocities.size() > 0 &&
            iterator_upper->second.expected_normalized_fiber_velocities.size() == iterator_lower->second.expected_normalized_fiber_velocities.size()) {
            interpolated_record.expected_normalized_fiber_velocities.resize(iterator_lower->second.expected_normalized_fiber_velocities.size());
            for (int muscle_index = 0; muscle_index < interpolated_record.expected_normalized_fiber_velocities.size(); ++muscle_index) {
                interpolated_record.expected_normalized_fiber_velocities[muscle_index] =
                    iterator_lower->second.expected_normalized_fiber_velocities[muscle_index] * (1.0 - interpolation_weight) +
                    iterator_upper->second.expected_normalized_fiber_velocities[muscle_index] * interpolation_weight;
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
        const double k_gdyn_clamped  = std::clamp((sv_adapt_theta[10] && get_enable_adaptation()) ? sv_adapt_theta[10]->getValue(state) : get_K_gamma_dyn(),  0.0, 1.0);
        const double k_gstat_clamped = std::clamp((sv_adapt_theta[11] && get_enable_adaptation()) ? sv_adapt_theta[11]->getValue(state) : get_K_gamma_stat(), 0.0, 1.0);
        const double dynamic_gamma_gain_scaled        = 100.0 * k_gdyn_clamped;
        const double static_gamma_gain_scaled         = 100.0 * k_gstat_clamped;
        const double length_observer_time_constant    = 0.010;
        const double secondary_afferent_time_constant = 0.055;
        const double velocity_observer_time_constant  = 0.005;

        // Time-aligned 8 ms A-gamma efferent delay (Branch 7B: Sub 7 -> Sub 3) for the parallel reference Mileusnic spindle model:
        const double tau_7b_gamma_spindle_delay = 0.0080;
        DelayedData delayed_spinal_ref_data = getDelayedData(state.getTime(), tau_7b_gamma_spindle_delay, true);
        const HistoryRecord& ref_spinal_record = delayed_spinal_ref_data.record;
        const bool has_ref_spinal_commands =
            (ref_spinal_record.valid &&
             ref_spinal_record.spinal_gamma_dyn_commands.size() == number_of_active_muscles &&
             ref_spinal_record.spinal_gamma_stat_commands.size() == number_of_active_muscles &&
             ref_spinal_record.expected_normalized_fiber_lengths.size() == number_of_active_muscles &&
             ref_spinal_record.expected_normalized_fiber_velocities.size() == number_of_active_muscles);

        for (int muscle_index = 0; muscle_index < number_of_active_muscles; ++muscle_index) {
            double spindle_length_change        = state_variable_spindle_length_change[muscle_index]->getValue(state);
            double spindle_velocity             = state_variable_spindle_velocity[muscle_index]->getValue(state);
            double dynamic_fusimotor_activation = state_variable_spindle_dynamic_fusimotor[muscle_index]->getValue(state);
            double static_fusimotor_activation  = state_variable_spindle_static_fusimotor[muscle_index]->getValue(state);

            double u0_alpha =
                (muscle_index < (int)cached_minimum_excitations.size()) ? cached_minimum_excitations[muscle_index] : 0.03;
            double planned_alpha =
                (muscle_index < (int)last_voluntary_alpha_excitations.size()) ? last_voluntary_alpha_excitations[muscle_index] : u0_alpha;
            double dynamic_gamma_firing_rate = has_ref_spinal_commands
                ? ref_spinal_record.spinal_gamma_dyn_commands[muscle_index]
                : (dynamic_gamma_gain_scaled * planned_alpha);
            double static_gamma_firing_rate = has_ref_spinal_commands
                ? ref_spinal_record.spinal_gamma_stat_commands[muscle_index]
                : (static_gamma_gain_scaled * planned_alpha);
            double L_hat_ref = has_ref_spinal_commands
                ? ref_spinal_record.expected_normalized_fiber_lengths[muscle_index]
                : baseline_normalized_fiber_lengths[muscle_index];
            double Ldot_hat_ref = has_ref_spinal_commands
                ? ref_spinal_record.expected_normalized_fiber_velocities[muscle_index]
                : 0.0;

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

            // --- STAGE 2: PARALLEL REFERENCE MILEUSNIC SPINDLE (EFFERENCE COPY PREDICTOR) & DYNAMIC GAIN NORMALIZATION kappa(f_d_hat) ---
            double r_ia_hat = 0.0, r_ii_hat = 0.0, kappa_fd = 1.0;
            evaluateReferenceMileusnicSpindle(
                muscle_index,
                L_hat_ref,
                Ldot_hat_ref,
                dynamic_fusimotor_activation,
                static_fusimotor_activation,
                instantaneous_nuclear_chain_activation,
                r_ia_hat,
                r_ii_hat,
                kappa_fd);

            double r_ia_expected = baseline_primary_afferent_rates[muscle_index]   + (r_ia_hat - baseline_reference_ia_rates[muscle_index]);
            double r_ii_expected = baseline_secondary_afferent_rates[muscle_index] + (r_ii_hat - baseline_reference_ii_rates[muscle_index]);

            const double gd0_ref = dynamic_gamma_gain_scaled * (0.25 + 0.75 * u0_alpha);
            const double gs0_ref = static_gamma_gain_scaled * u0_alpha;
            const double fd0_ref = (gd0_ref * gd0_ref) / (gd0_ref * gd0_ref + 3600.0);
            const double fs0_ref = (gs0_ref * gs0_ref) / (gs0_ref * gs0_ref + 3600.0);
            const double denom_k0 = std::max(0.015, 0.0605 - 0.046 * fs0_ref);
            const double kappa_0  = 1.0 + (0.2592 * fd0_ref) / denom_k0;
            double net_secondary_afferent_change = secondary_afferent_value - r_ii_expected;
            double net_primary_afferent_change   = (primary_afferent_value  - r_ia_expected) * (std::max(1.0, kappa_0) / std::max(1.0, kappa_fd));

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

    // --- ATLAS ANATOMICAL INTER-BLOCK DELAYS (13 INDEPENDENT BRANCHES) ---
    const double tau_1a_m1_vn_tilt    = 0.0090; // Branch 1A (Sub 1 -> Sub 5 Brainstem VN Tonic Tilt): 9.0 ms
    const double tau_1b_m1_vn_canal   = 0.0100; // Branch 1B (Sub 1 -> Sub 5 Brainstem VN Canal/Phasic): 10.0 ms
    const double tau_2a_cn8_vn_delay  = 0.0020; // Branch 2A (Sub 2 -> Sub 5 Brainstem VN): 2.0 ms
    const double tau_2b_cn8_cb_delay  = 0.0030; // Branch 2B (Sub 2 -> Sub 4 Cerebellum): 3.0 ms
    const double tau_3a_drg_ccr_delay = 0.0055; // Branch 3A (Sub 3 -> Sub 6 Spinal CCR): 5.5 ms
    const double tau_3b_cuneate_delay = 0.0070; // Branch 3B (Sub 3 -> Sub 4 Cerebellum Cuneate): 7.0 ms
    const double tau_4a_fn_vn_delay   = 0.0015; // Branch 4A (Sub 4 -> Sub 5 Brainstem VN Uncinate Tract): 1.5 ms
    const double tau_4b_thal_m1_delay = 0.0130; // Branch 4B (Sub 4 -> VL/VA Thalamus -> Sub 1 M1 Cortex): 13.0 ms

    DelayedData delayed_2a_vn_data   = getDelayedData(state.getTime(), tau_2a_cn8_vn_delay);
    DelayedData delayed_2b_cb_data   = getDelayedData(state.getTime(), tau_2b_cn8_cb_delay);
    DelayedData delayed_3b_cb_data   = getDelayedData(state.getTime(), tau_3b_cuneate_delay);
    DelayedData delayed_4a_fn_vn     = getDelayedData(state.getTime(), tau_4a_fn_vn_delay);
    DelayedData delayed_4b_thal_m1   = getDelayedData(state.getTime(), tau_4b_thal_m1_delay);
    DelayedData delayed_1a_m1_vn     = getDelayedData(state.getTime(), tau_1a_m1_vn_tilt);
    DelayedData delayed_1b_m1_vn     = getDelayedData(state.getTime(), tau_1b_m1_vn_canal);

    // --- SE_2(3) SPATIAL OBSERVER (Sub 4 Cerebellum: Driven by Branch 2B [3.0 ms] & Branch 3B [7.0 ms], Both Pre-Compensated at Sender) ---
    SimTK::Vec3 arriving_world_angular_velocity =
        (!history.empty() && delayed_2b_cb_data.record.valid)
            ? delayed_2b_cb_data.record.luenberger_angular_velocity
            : SimTK::Vec3(luenberger_angular_velocity_roll, luenberger_angular_velocity_yaw, luenberger_angular_velocity_pitch);
    SimTK::Vec3 otolith_specific_force_head =
        (!history.empty() && delayed_2b_cb_data.record.valid)
            ? delayed_2b_cb_data.record.luenberger_otolith_specific_force
            : current_luenberger_otolith_force_head;

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
    SimTK::Vec3 world_angular_velocity = arriving_world_angular_velocity;
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

    // --- SUBSYSTEM 1: CORTICAL / VOLUNTARY 2ND-ORDER CRITICALLY DAMPED FILTER DYNAMICS (Driven by Branch 4B [13.0 ms], Pre-Compensated at Sub 4) ---
    double delayed_mahony_pitch =
        (!history.empty() && delayed_4b_thal_m1.record.valid)
            ? (delayed_4b_thal_m1.record.estimated_head_orientation[2] - baseline_spatial_orientation[2])
            : (estimated_head_pitch - baseline_spatial_orientation[2]);
    double delayed_mahony_roll =
        (!history.empty() && delayed_4b_thal_m1.record.valid)
            ? (delayed_4b_thal_m1.record.estimated_head_orientation[0] - baseline_spatial_orientation[0])
            : (estimated_head_roll - baseline_spatial_orientation[0]);
    double delayed_mahony_yaw =
        (!history.empty() && delayed_4b_thal_m1.record.valid)
            ? (delayed_4b_thal_m1.record.estimated_head_orientation[1] - baseline_spatial_orientation[1])
            : (estimated_head_yaw - baseline_spatial_orientation[1]);

    double delayed_mahony_pitch_speed =
        (!history.empty() && delayed_4b_thal_m1.record.valid)
            ? delayed_4b_thal_m1.record.mahony_estimated_angular_velocity[0]
            : mahony_estimated_angular_velocity[0];
    double delayed_mahony_roll_speed =
        (!history.empty() && delayed_4b_thal_m1.record.valid)
            ? delayed_4b_thal_m1.record.mahony_estimated_angular_velocity[1]
            : mahony_estimated_angular_velocity[1];
    double delayed_mahony_yaw_speed =
        (!history.empty() && delayed_4b_thal_m1.record.valid)
            ? delayed_4b_thal_m1.record.mahony_estimated_angular_velocity[2]
            : mahony_estimated_angular_velocity[2];

    double ki_pitch = (sv_adapt_theta[14] && get_enable_adaptation())
        ? sv_adapt_theta[14]->getValue(state)
        : ((get_Ki_task_pitch() >= 0.0) ? get_Ki_task_pitch() : get_Ki_task());
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
    double kp_pitch = (sv_adapt_theta[12] && get_enable_adaptation())
        ? sv_adapt_theta[12]->getValue(state)
        : ((get_Kp_task_pitch() >= 0.0) ? get_Kp_task_pitch() : std::sqrt(get_Wp_base_pitch() * vigor_pitch));
    double kd_pitch = (sv_adapt_theta[13] && get_enable_adaptation())
        ? sv_adapt_theta[13]->getValue(state)
        : ((get_Kd_task_pitch() >= 0.0) ? get_Kd_task_pitch() : std::sqrt(get_Wv_base_pitch() * vigor_pitch + 2.0 * kp_pitch));
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
    double p_vol = 2.0 * SimTK::Pi * voluntary_cutoff_frequency; // Literature M1 cortical LPF pole: 2 Hz -> 12.566 rad/s

    const double u_vol_raw[3] = {
        unfiltered_voluntary_torque_pitch,
        unfiltered_voluntary_torque_roll,
        unfiltered_voluntary_torque_yaw
    };
    for (int axis = 0; axis < 3; ++axis) {
        double y_vol = state_variable_voluntary_filter_stage1[axis]->getValue(state);
        double v_vol = state_variable_voluntary_filter_stage2[axis]->getValue(state);
        double a_vol = (p_vol * p_vol) * (u_vol_raw[axis] - y_vol) - (2.0 * p_vol) * v_vol;
        state_variable_voluntary_filter_stage1[axis]->setDerivative(state, v_vol);
        state_variable_voluntary_filter_stage2[axis]->setDerivative(state, a_vol);
    }

    // --- SUBSYSTEM 4 ATTITUDE INNOVATION BLOCKS (Literature Mahony Gains: K_oto = 0.20 rad/s, K_prop = 2.50 rad/s) ---
    SimTK::Vec3 predicted_up_direction_head;
    if (otolith_specific_force_head.norm() > 1e-4) {
        predicted_up_direction_head = otolith_specific_force_head.normalize();
    } else {
        predicted_up_direction_head = (~estimated_rotation_head_to_world) * SimTK::Vec3(0.0, 1.0, 0.0);
    }

    double arriving_relative_neck_pitch = estimated_neck_relative_angle[0];
    double arriving_relative_neck_roll  = estimated_neck_relative_angle[1];
    double arriving_relative_neck_yaw   = estimated_neck_relative_angle[2];
    double arriving_trunk_pitch = angle_ground_pitch;
    double arriving_trunk_roll  = angle_ground_roll;
    double arriving_trunk_yaw   = angle_ground_yaw;

    if (!history.empty() && delayed_3b_cb_data.record.valid) {
        arriving_relative_neck_pitch = delayed_3b_cb_data.record.estimated_neck_relative_angle[0];
        arriving_relative_neck_roll  = delayed_3b_cb_data.record.estimated_neck_relative_angle[1];
        arriving_relative_neck_yaw   = delayed_3b_cb_data.record.estimated_neck_relative_angle[2];
        arriving_trunk_pitch         = delayed_3b_cb_data.record.trunk_orientation[2];
        arriving_trunk_roll          = delayed_3b_cb_data.record.trunk_orientation[0];
        arriving_trunk_yaw           = delayed_3b_cb_data.record.trunk_orientation[1];
    }

    double predicted_proprioceptive_pitch = arriving_trunk_pitch + arriving_relative_neck_pitch;
    double predicted_proprioceptive_roll  = arriving_trunk_roll  + arriving_relative_neck_roll;
    double predicted_proprioceptive_yaw   = arriving_trunk_yaw   + arriving_relative_neck_yaw;

    // 2a. Otolith Gravity Innovation (Literature value: Angelaki et al. 1999; Merfeld et al. 1999; Laurens & Angelaki 2011: K_oto = 0.20 rad/s, tau_grav = 5.0 s)
    SimTK::Vec3 expected_up_direction_head = (~estimated_rotation_head_to_world) * SimTK::Vec3(0.0, 1.0, 0.0);
    SimTK::Vec3 otolith_attitude_error_head = predicted_up_direction_head % expected_up_direction_head;
    const double Kp_otolith_attitude = 0.20;
    SimTK::Vec3 weighted_otolith_attitude_error = Kp_otolith_attitude * otolith_attitude_error_head;

    // 2b. Proprioceptive Attitude Innovation (Literature value: Mergner et al. 1991, 1997; Brooks & Cullen 2009: K_prop = 2.50 rad/s, 12.5:1 ratio over K_oto)
    SimTK::Vec3 proprioceptive_attitude_error_world(
        predicted_proprioceptive_roll  - estimated_head_roll,
        predicted_proprioceptive_yaw   - estimated_head_yaw,
        predicted_proprioceptive_pitch - estimated_head_pitch);
    SimTK::Vec3 proprioceptive_attitude_error_head = (~estimated_rotation_head_to_world) * proprioceptive_attitude_error_world;
    const double K_proprioceptive_attitude_roll  = 2.50;
    const double K_proprioceptive_attitude_yaw   = 2.50;
    const double K_proprioceptive_attitude_pitch = 2.50;
    SimTK::Vec3 weighted_proprioceptive_attitude_error(
        K_proprioceptive_attitude_roll  * proprioceptive_attitude_error_head[0],
        K_proprioceptive_attitude_yaw   * proprioceptive_attitude_error_head[1],
        K_proprioceptive_attitude_pitch * proprioceptive_attitude_error_head[2]);

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

    SimTK::Mat33 arriving_estimated_rotation_head_to_world_cb =
        (!history.empty() && delayed_2b_cb_data.record.valid)
            ? delayed_2b_cb_data.record.estimated_rotation_head_to_world
            : estimated_rotation_head_to_world.asMat33();
    SimTK::Vec3 linear_acceleration_world = (arriving_estimated_rotation_head_to_world_cb * otolith_specific_force_head) + gravity_vector_world;

    double velocity_leak_time_constant = 15.0;
    double position_leak_time_constant = 5.0;

    SimTK::Vec3 linear_velocity_derivative = linear_acceleration_world - (1.0 / velocity_leak_time_constant) * estimated_linear_velocity;
    SimTK::Vec3 linear_position_derivative = estimated_linear_velocity - (1.0 / position_leak_time_constant) * estimated_linear_position;

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

    // --- SUBSYSTEM 5: BRAINSTEM VESTIBULO-COLLIC REFLEX (VCR) 2ND-ORDER NEURAL FILTER DERIVATIVES ---
    double vcr_ton_in_pitch = 0.0, vcr_ton_st_pitch = 0.0;
    double vcr_sc_in_pitch  = 0.0, vcr_sc_st_pitch  = 0.0;
    double vcr_phas_in_pitch = 0.0, vcr_phas_st_pitch = 0.0;
    {
        double descending_traj_pos_pitch = (!history.empty() && delayed_1a_m1_vn.record.valid) ? delayed_1a_m1_vn.record.trajectory_position_1a[0] : trajectory_position_pitch;
        double descending_traj_pos_roll  = (!history.empty() && delayed_1a_m1_vn.record.valid) ? delayed_1a_m1_vn.record.trajectory_position_1a[1] : trajectory_position_roll;
        double descending_traj_pos_yaw   = (!history.empty() && delayed_1a_m1_vn.record.valid) ? delayed_1a_m1_vn.record.trajectory_position_1a[2] : trajectory_position_yaw;

        double descending_traj_vel_pitch = (!history.empty() && delayed_1b_m1_vn.record.valid) ? delayed_1b_m1_vn.record.trajectory_velocity_1b[0] : trajectory_velocity_pitch;
        double descending_traj_vel_roll  = (!history.empty() && delayed_1b_m1_vn.record.valid) ? delayed_1b_m1_vn.record.trajectory_velocity_1b[1] : trajectory_velocity_roll;
        double descending_traj_vel_yaw   = (!history.empty() && delayed_1b_m1_vn.record.valid) ? delayed_1b_m1_vn.record.trajectory_velocity_1b[2] : trajectory_velocity_yaw;

        double descending_traj_acc_pitch = (!history.empty() && delayed_1b_m1_vn.record.valid) ? delayed_1b_m1_vn.record.trajectory_acceleration_1b[0] : trajectory_acceleration_pitch;
        double descending_traj_acc_roll  = (!history.empty() && delayed_1b_m1_vn.record.valid) ? delayed_1b_m1_vn.record.trajectory_acceleration_1b[1] : trajectory_acceleration_roll;

        SimTK::Vec3 arriving_estimated_head_orientation =
            (!history.empty() && delayed_4a_fn_vn.record.valid)
                ? delayed_4a_fn_vn.record.estimated_head_orientation_4a
                : SimTK::Vec3(estimated_head_roll, estimated_head_yaw, estimated_head_pitch);
        double tonic_otolith_input_pitch = (arriving_estimated_head_orientation[2] - baseline_spatial_orientation[2]) - descending_traj_pos_pitch;
        double tonic_otolith_input_roll  = (arriving_estimated_head_orientation[0] - baseline_spatial_orientation[0]) - descending_traj_pos_roll;
        double tonic_otolith_input_yaw   = (arriving_estimated_head_orientation[1] - baseline_spatial_orientation[1]) - descending_traj_pos_yaw;

        SimTK::Vec3 arriving_irregular_canal_velocity =
            (!history.empty() && delayed_2a_vn_data.record.valid)
                ? delayed_2a_vn_data.record.reconstructed_canal_angular_velocity
                : head_global_angular_velocity;
        double semicircular_canal_input_pitch = arriving_irregular_canal_velocity[2] - descending_traj_vel_pitch;
        double semicircular_canal_input_roll  = arriving_irregular_canal_velocity[0] - descending_traj_vel_roll;
        double semicircular_canal_input_yaw   = arriving_irregular_canal_velocity[1] - descending_traj_vel_yaw;

        SimTK::Vec3 arriving_otolith_specific_force_head =
            (!history.empty() && delayed_2a_vn_data.record.valid)
                ? delayed_2a_vn_data.record.reconstructed_otolith_specific_force
                : current_irregular_otolith_force_head;
        SimTK::Mat33 arriving_estimated_rotation_head_to_world_vn =
            (!history.empty() && delayed_4a_fn_vn.record.valid)
                ? delayed_4a_fn_vn.record.estimated_rotation_head_to_world
                : estimated_rotation_head_to_world.asMat33();
        SimTK::Vec3 arriving_gravity_head = (~arriving_estimated_rotation_head_to_world_vn) * gravity_vector_world;
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
            if (th <= 0.0) return v;
            double av = std::abs(v);
            if (av <= 0.75 * th) return 0.0;
            if (av < 1.25 * th) {
                double u = av - 0.75 * th;
                return (v >= 0.0 ? 1.0 : -1.0) * (u * u) / th;
            }
            return (v > 0.0) ? (v - th) : (v + th);
        };
        const double u_ton[3] = {
            deadzone(tonic_otolith_input_pitch, vest_tilt_thresh),
            deadzone(tonic_otolith_input_roll,  vest_tilt_thresh),
            deadzone(tonic_otolith_input_yaw,   vest_tilt_thresh)
        };
        const double u_sc[3] = {
            deadzone(semicircular_canal_input_pitch, vest_scc_thresh),
            deadzone(semicircular_canal_input_roll,  vest_scc_thresh),
            deadzone(semicircular_canal_input_yaw,   vest_scc_thresh)
        };
        const double u_phas[3] = {
            deadzone(phasic_otolith_input_pitch, vest_oto_thresh),
            deadzone(phasic_otolith_input_roll,  vest_oto_thresh),
            phasic_otolith_input_yaw
        };

        vcr_ton_in_pitch  = u_ton[0];
        vcr_ton_st_pitch  = state_variable_tonic_otolith[0]->getValue(state);
        vcr_sc_in_pitch   = u_sc[0];
        vcr_sc_st_pitch   = state_variable_semicircular_canal_reflex[0]->getValue(state);
        vcr_phas_in_pitch = u_phas[0];
        vcr_phas_st_pitch = state_variable_phasic_otolith[0]->getValue(state);

        // Literature Poles (Dominant pole p_i paired with fast secondary membrane pole p_fast = 250.0 rad/s):
        //   p_ton  = 15.0 rad/s (~2.4 Hz, Schor et al. 1984; Peng et al. 1996)
        //   p_sc   = 50.0 rad/s (~8.0 Hz, Shinoda et al. 2006; Cullen 2012)
        //   p_phas = 4.0 rad/s  (~0.64 Hz, Angelaki 2004; Raphan & Cohen 2002)
        const double p_fast = 250.0;
        const double p_ton  = std::max(0.5, get_p_ton());
        const double p_sc   = std::max(0.5, get_p_sc());
        const double p_phas = std::max(0.5, get_p_phas());

        for (int axis = 0; axis < 3; ++axis) {
            double y_ton = state_variable_tonic_otolith[axis]->getValue(state);
            double v_ton = state_variable_tonic_otolith_vel[axis]->getValue(state);
            double a_ton = (p_ton * p_fast) * (u_ton[axis] - y_ton) - (p_ton + p_fast) * v_ton;
            state_variable_tonic_otolith[axis]->setDerivative(state, v_ton);
            state_variable_tonic_otolith_vel[axis]->setDerivative(state, a_ton);

            double y_sc = state_variable_semicircular_canal_reflex[axis]->getValue(state);
            double v_sc = state_variable_semicircular_canal_reflex_vel[axis]->getValue(state);
            double a_sc = (p_sc * p_fast) * (u_sc[axis] - y_sc) - (p_sc + p_fast) * v_sc;
            state_variable_semicircular_canal_reflex[axis]->setDerivative(state, v_sc);
            state_variable_semicircular_canal_reflex_vel[axis]->setDerivative(state, a_sc);

            double y_phas = state_variable_phasic_otolith[axis]->getValue(state);
            double v_phas = state_variable_phasic_otolith_vel[axis]->getValue(state);
            double a_phas = (p_phas * p_fast) * (u_phas[axis] - y_phas) - (p_phas + p_fast) * v_phas;
            state_variable_phasic_otolith[axis]->setDerivative(state, v_phas);
            state_variable_phasic_otolith_vel[axis]->setDerivative(state, a_phas);
        }
    }

    // --- SUBSYSTEM 6: SPINAL CERVICO-COLLIC REFLEX (CCR) LITERATURE POLE (p_ccr = 100 rad/s, p_fast = 250 rad/s) ---
    // Literature Pole: p_ccr = 100.0 rad/s (~15.9 Hz, τ_syn = 10 ms spinal Lamina VI/VII EPSP filter; Peterson et al. 1985; Happee et al. 2017)
    const double p_fast_ccr = 250.0;
    const double p_ccr = std::max(0.5, get_p_ccr());
    DelayedData delayed_3a_ccr_data = getDelayedData(state.getTime(), tau_3a_drg_ccr_delay, true);
    const HistoryRecord& cervico_collic_record = delayed_3a_ccr_data.record;
    double ccr_u_pitch = 0.0, ccr_x_pitch = 0.0;
    if (cervico_collic_record.primary_afferent_firing_rate.size() == active_muscle_indices.size() && active_muscle_indices.size() > 0) {
        double tau_max_ext = 0.0;
        double tau_max_flex = 0.0;
        if (moment_arm_jacobian_3dof.ncol() == static_cast<int>(active_muscle_indices.size())) {
            for (size_t m = 0; m < active_muscle_indices.size(); ++m) {
                double r_pitch = moment_arm_jacobian_3dof(0, static_cast<int>(m)) * cached_maximum_isometric_forces[m];
                if (r_pitch > 0.0) tau_max_ext += r_pitch;
                else if (r_pitch < 0.0) tau_max_flex += -r_pitch;
            }
        }
        const double tau_max_sum = tau_max_ext + tau_max_flex;
        const double w_ccr_ext  = (tau_max_sum > 1e-6) ? (2.0 * tau_max_flex / tau_max_sum) : 1.0;
        const double w_ccr_flex = (tau_max_sum > 1e-6) ? (2.0 * tau_max_ext  / tau_max_sum) : 1.0;

        for (size_t muscle_index = 0; muscle_index < active_muscle_indices.size(); ++muscle_index) {
            double y_ccr = state_variable_cervico_collic[muscle_index]->getValue(state);
            double v_ccr = state_variable_cervico_collic_vel[muscle_index]->getValue(state);
            double u_ccr = cervico_collic_record.primary_afferent_firing_rate[muscle_index];
            double a_ccr = (p_ccr * p_fast_ccr) * (u_ccr - y_ccr) - (p_ccr + p_fast_ccr) * v_ccr;
            state_variable_cervico_collic[muscle_index]->setDerivative(state, v_ccr);
            state_variable_cervico_collic_vel[muscle_index]->setDerivative(state, a_ccr);
            if (moment_arm_jacobian_3dof.ncol() > static_cast<int>(muscle_index)) {
                double j_pitch_m = moment_arm_jacobian_3dof(0, static_cast<int>(muscle_index));
                double w_ccr_m = (j_pitch_m > 1e-6) ? w_ccr_ext : ((j_pitch_m < -1e-6) ? w_ccr_flex : 1.0);
                double r_pitch_m = w_ccr_m * j_pitch_m * cached_maximum_isometric_forces[muscle_index];
                ccr_u_pitch += r_pitch_m * u_ccr;
                ccr_x_pitch += r_pitch_m * y_ccr;
            }
        }
    } else {
        for (size_t muscle_index = 0; muscle_index < active_muscle_indices.size(); ++muscle_index) {
            state_variable_cervico_collic[muscle_index]->setDerivative(state, 0.0);
            state_variable_cervico_collic_vel[muscle_index]->setDerivative(state, 0.0);
        }
    }

    // --- CONTINUOUS 15-PARAMETER REAL-PLANT ADAPTIVE CONTROL (OPTION 1: 6-FREQ REAL-PLANT BODE vs OPTION 2: TIME-DOMAIN MRAC) ---
    double t_now = state.getTime();
    double p1_val = cached_coordinate_pitch1 ? cached_coordinate_pitch1->getValue(state) : 0.0;
    double p2_val = cached_coordinate_pitch2 ? cached_coordinate_pitch2->getValue(state) : 0.0;
    double head_pitch_rad = p1_val + p2_val;
    double head_pitch_deg = head_pitch_rad * 180.0 / SimTK::Pi;
    double head_pitch_vel_rad = speed_pitch1 + speed_pitch2;
    double ref_pitch_deg = computeReferencePitchDeg(t_now, get_multisine_scale());
    double ref_pitch_rad = ref_pitch_deg * SimTK::Pi / 180.0;
    double ref_pitch_vel_deg = computeReferencePitchVelDeg(t_now, get_multisine_scale());
    double ref_pitch_vel_rad = ref_pitch_vel_deg * SimTK::Pi / 180.0;

    double err_pos_rad = head_pitch_rad - ref_pitch_rad;
    double err_vel_rad = head_pitch_vel_rad - ref_pitch_vel_rad;
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

            double A_eff = std::max(A_arr[k], 5e-5);
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
        if (t_now >= 14.0) {
            taper = 0.0;
        } else if (t_now > 10.5) {
            double u_tap = (t_now - 10.5) / 3.5;
            taper = 0.5 * (1.0 + std::cos(SimTK::Pi * u_tap));
        }

        const double kp_task_0 = (get_Kp_task_pitch() >= 0.0) ? get_Kp_task_pitch() : 1.5;
        const double kd_task_0 = (get_Kd_task_pitch() >= 0.0) ? get_Kd_task_pitch() : 0.15;
        const double ki_task_0 = (get_Ki_task_pitch() >= 0.0) ? get_Ki_task_pitch() : get_Ki_task();

        const double theta_0[15] = {
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
            std::clamp(get_K_gamma_dyn(),  0.0, 1.0),
            std::clamp(get_K_gamma_stat(), 0.0, 1.0),
            kp_task_0,
            kd_task_0,
            ki_task_0
        };

        // Scale-invariant physical bounds relative to initial seed (with absolute floors/ceilings; gamma capped at 1.0 biological limit)
        // Poles (indices 1, 4, 6, 9) and anatomical delay compensators (indices 3, 8) are fixed to literature/anatomical values!
        const double theta_min[15] = {
            std::max(0.01, 0.15 * theta_0[0]),
            theta_0[1],
            std::max(0.01, 0.15 * theta_0[2]),
            theta_0[3],
            theta_0[4],
            std::max(0.10, 0.15 * theta_0[5]),
            theta_0[6],
            std::max(0.02, 0.15 * theta_0[7]),
            theta_0[8],
            theta_0[9],
            std::max(0.05, 0.20 * theta_0[10]),
            std::max(0.05, 0.20 * theta_0[11]),
            std::max(0.05, 0.20 * theta_0[12]),
            std::max(0.02, 0.20 * theta_0[13]),
            std::max(1.00, 0.20 * theta_0[14])
        };

        const double theta_max[15] = {
            std::max(3.00,  4.0 * theta_0[0]),
            theta_0[1],
            std::max(1.50,  4.0 * theta_0[2]),
            theta_0[3],
            theta_0[4],
            std::max(28.00, 3.0 * theta_0[5]),
            theta_0[6],
            std::max(2.00,  4.0 * theta_0[7]),
            theta_0[8],
            theta_0[9],
            1.00,
            1.00,
            std::max(15.0,  3.0 * theta_0[12]),
            std::max(5.0,   3.0 * theta_0[13]),
            std::max(65.0,  2.5 * theta_0[14])
        };

        double th_curr[15];
        for (int i = 0; i < 15; ++i) {
            th_curr[i] = sv_adapt_theta[i] ? sv_adapt_theta[i]->getValue(state) : theta_0[i];
        }

        const char* env_mrac_mode = std::getenv("ADAPT_MRAC_MODE");
        const bool use_timedomain_mrac = (env_mrac_mode && (std::string(env_mrac_mode) == "2" || std::string(env_mrac_mode) == "TIMEDOMAIN"));

        const double gamma_hbac = 0.032 * mult * taper;
        // Only the 9 controller gains adapt; the 4 literature poles and 2 anatomical delay compensators are locked (rate = 0.0):
        const double w_param_rate[15] = {
            1.6, // 0: adapt_K_ton
            0.0, // 1: adapt_p_ton (fixed literature pole = 15.0 rad/s)
            1.5, // 2: adapt_K_sc
            0.0, // 3: adapt_z_sc  (fixed MVST delay tau_5 = 0.0025 s)
            0.0, // 4: adapt_p_sc  (fixed literature pole = 50.0 rad/s)
            1.4, // 5: adapt_K_phas
            0.0, // 6: adapt_p_phas (fixed literature pole = 20.0 rad/s)
            1.3, // 7: adapt_kp_prop
            0.0, // 8: adapt_z_ccr (fixed propriospinal delay tau_6 = 0.0040 s)
            0.0, // 9: adapt_p_ccr (fixed literature pole = 100.0 rad/s)
            0.8, // 10: adapt_K_gamma_dyn
            1.5, // 11: adapt_K_gamma_stat
            1.8, // 12: adapt_Kp_task_pitch
            1.5, // 13: adapt_Kd_task_pitch
            1.8  // 14: adapt_Ki_task_pitch
        };

        if (!use_timedomain_mrac) {
            // =========================================================================
            // OPTION 1: 100% REAL-PLANT DEMODULATED 6-FREQUENCY BODE ADAPTATION
            // Error is 100% measured from the live OpenSim plant (sim_gain_demod, sim_phase_demod).
            // Equal frequency weights (1.0) and exact Happee Bode objective scaling (1 dB : 10 deg).
            // =========================================================================
            double err_gain_norm[6]  = {0.0};
            double err_phase_norm[6] = {0.0};
            const double w_freq_prio[6] = {1.00, 1.00, 1.00, 1.00, 1.00, 1.00};

            for (int k = 0; k < 6; ++k) {
                double S_hat = sv_psif_x1[k + 6]->getValue(state);
                double C_hat = sv_psif_x2[k + 6]->getValue(state);
                double sim_gain_demod  = std::sqrt(S_hat * S_hat + C_hat * C_hat);
                double sim_phase_demod = std::atan2(C_hat, S_hat) * 180.0 / SimTK::Pi;

                double eff_gain = std::max(5.0, sim_gain_demod);
                double phase_diff_deg = std::remainder(sim_phase_demod - targets_phase_deg[k], 360.0);

                double gain_db_diff = 20.0 * std::log10(std::max(1e-3, eff_gain) / targets_gain[k]);
                err_gain_norm[k]  = w_freq_prio[k] * (gain_db_diff / 1.00);
                err_phase_norm[k] = w_freq_prio[k] * (phase_diff_deg / 10.0);
            }

            for (int i = 0; i < 15; ++i) {
                if (!sv_adapt_theta[i]) continue;
                if (w_param_rate[i] <= 0.0) {
                    sv_adapt_theta[i]->setDerivative(state, 0.0);
                    continue;
                }
                double th_i = th_curr[i];
                double d_th = std::max(1e-5, 1e-4 * std::abs(theta_0[i]));

                double th_plus[15], th_minus[15];
                for (int j = 0; j < 15; ++j) {
                    th_plus[j]  = th_curr[j];
                    th_minus[j] = th_curr[j];
                }
                th_plus[i]  += d_th;
                th_minus[i] -= d_th;

                double grad_i  = 0.0;
                double hess_ii = 0.0;

                for (int k = 0; k < 6; ++k) {
                    std::complex<double> G_p = evalRawClosedLoopBode(freqs[k], th_plus,  theta_0);
                    std::complex<double> G_m = evalRawClosedLoopBode(freqs[k], th_minus, theta_0);

                    std::complex<double> d_lnG = std::log(G_p / G_m) / (2.0 * d_th);
                    double s_gain_k  = theta_0[i] * (20.0 / std::log(10.0)) * d_lnG.real() / 1.00;
                    double s_phase_k = theta_0[i] * (180.0 / SimTK::Pi)     * d_lnG.imag() / 10.0;

                    grad_i  += s_gain_k * err_gain_norm[k] + s_phase_k * err_phase_norm[k];
                    hess_ii += s_gain_k * s_gain_k + s_phase_k * s_phase_k;
                }

                double norm_step = grad_i / (std::sqrt(hess_ii) + 0.35);
                norm_step = std::clamp(norm_step, -1.5, 1.5);

                double dth = -gamma_hbac * w_param_rate[i] * theta_0[i] * norm_step - taper * get_sigma_leakage() * (th_i - theta_0[i]);

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
            // =========================================================================
            // OPTION 2: 100% PURE TIME-DOMAIN REAL-PLANT TRACKING REGRESSOR MRAC LAW
            // Zero Bode model, zero Fourier demodulator, zero surrogate.
            // Driven strictly by live real-plant composite tracking error e_s(t) and
            // live time-domain sensitivity regressors phi_i(t) = d(tau_pitch)/d(theta_i).
            // =========================================================================
            const double lambda_vel = 0.08; // Slotine & Li SPR composite error time constant (s)
            const double e_s = (err_pos_rad + lambda_vel * err_vel_rad) / 0.05; // normalized by ~2.86 deg

            const double kp_prop_c = std::max(1e-4, th_curr[7]);

            double phi_raw[15] = {0.0};
            // 0: Tonic Otolith Reflex Gain K_ton
            phi_raw[0] = -vcr_ton_st_pitch;
            // 2: Semicircular Canal Reflex Gain K_sc
            phi_raw[2] = -vcr_sc_st_pitch;
            // 5: Phasic Otolith Reflex Gain K_phas
            phi_raw[5] = -vcr_phas_st_pitch;
            // 7: Spinal CCR Reflex Gain kp_prop
            phi_raw[7] = ccr_x_pitch;
            // 10..11: Fusimotor Spindle Sensitivity (dynamic velocity & static position)
            phi_raw[10] = -kp_prop_c * estimated_neck_relative_speed[0];
            phi_raw[11] = -kp_prop_c * estimated_neck_relative_angle[0];
            // 12..14: Voluntary Task-Space PID
            phi_raw[12] = (trajectory_position_pitch - delayed_mahony_pitch);
            phi_raw[13] = (trajectory_velocity_pitch - delayed_mahony_pitch_speed);
            phi_raw[14] = pitch_error_integral;

            for (int i = 0; i < 15; ++i) {
                if (!sv_adapt_theta[i]) continue;
                if (w_param_rate[i] <= 0.0) {
                    sv_adapt_theta[i]->setDerivative(state, 0.0);
                    continue;
                }
                double th_i = th_curr[i];
                double phi_scaled = theta_0[i] * phi_raw[i];
                double norm_reg = phi_scaled / (0.25 + std::abs(phi_scaled));
                double norm_step = std::clamp(e_s * norm_reg, -1.5, 1.5);

                double dth = -gamma_hbac * w_param_rate[i] * theta_0[i] * norm_step - taper * get_sigma_leakage() * (th_i - theta_0[i]);

                if (th_i < theta_min[i]) {
                    double viol = (theta_min[i] - th_i) / std::max(1e-3, theta_0[i]);
                    dth += 1000.0 * theta_0[i] * viol * viol * viol;
                } else if (th_i > theta_max[i]) {
                    double viol = (th_i - theta_max[i]) / std::max(1e-3, theta_0[i]);
                    dth -= 1000.0 * theta_0[i] * viol * viol * viol;
                }
                sv_adapt_theta[i]->setDerivative(state, dth);
            }
        }
    } else {
        for (int i = 0; i < 15; ++i) {
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
    double trajectory_jerk_pitch =
        -(1.0 / (trajectory_time_constant * trajectory_time_constant)) * trajectory_velocity_pitch -
        (2.0 / trajectory_time_constant) * trajectory_acceleration_pitch;

    double trajectory_position_roll = state_variable_trajectory_position[1]->getValue(state);
    double trajectory_velocity_roll = state_variable_trajectory_velocity[1]->getValue(state);
    double trajectory_acceleration_roll =
        (1.0 / (trajectory_time_constant * trajectory_time_constant)) * (get_desired_roll() - trajectory_position_roll) -
        (2.0 / trajectory_time_constant) * trajectory_velocity_roll;
    double trajectory_jerk_roll =
        -(1.0 / (trajectory_time_constant * trajectory_time_constant)) * trajectory_velocity_roll -
        (2.0 / trajectory_time_constant) * trajectory_acceleration_roll;

    double trajectory_position_yaw = state_variable_trajectory_position[2]->getValue(state);
    double trajectory_velocity_yaw = state_variable_trajectory_velocity[2]->getValue(state);
    double trajectory_acceleration_yaw =
        (1.0 / (trajectory_time_constant * trajectory_time_constant)) * (get_desired_yaw() - trajectory_position_yaw) -
        (2.0 / trajectory_time_constant) * trajectory_velocity_yaw;
    double trajectory_jerk_yaw =
        -(1.0 / (trajectory_time_constant * trajectory_time_constant)) * trajectory_velocity_yaw -
        (2.0 / trajectory_time_constant) * trajectory_acceleration_yaw;

    const Model& model = getModel();
    const auto& muscles = model.getMuscles();
    int number_of_active_muscles = active_muscle_indices.size();

    // 1. Evaluate C^infty Smooth 6-DOF Quadratic Taylor Moment-Arm Jacobian continuously at every substep (zero staircases)
    if (jacobian_6dof_q0.nrow() == 6 && jacobian_6dof_q0.ncol() == number_of_active_muscles) {
        const double dq[6] = {
            (cached_coordinate_pitch1 ? cached_coordinate_pitch1->getValue(state) : 0.0) - q0_6dof[0],
            (cached_coordinate_pitch2 ? cached_coordinate_pitch2->getValue(state) : 0.0) - q0_6dof[1],
            (cached_coordinate_roll1  ? cached_coordinate_roll1->getValue(state)  : 0.0) - q0_6dof[2],
            (cached_coordinate_roll2  ? cached_coordinate_roll2->getValue(state)  : 0.0) - q0_6dof[3],
            (cached_coordinate_yaw1   ? cached_coordinate_yaw1->getValue(state)   : 0.0) - q0_6dof[4],
            (cached_coordinate_yaw2   ? cached_coordinate_yaw2->getValue(state)   : 0.0) - q0_6dof[5]
        };
        for (int muscle_index = 0; muscle_index < number_of_active_muscles; ++muscle_index) {
            for (int c = 0; c < 6; ++c) {
                double val = jacobian_6dof_q0(c, muscle_index);
                for (int d = 0; d < 6; ++d) {
                    val += (jacobian_6dof_d1[d](c, muscle_index) + 0.5 * jacobian_6dof_d2[d](c, muscle_index) * dq[d]) * dq[d];
                }
                moment_arm_jacobian_6dof(c, muscle_index) = val;
            }
            moment_arm_jacobian_3dof(0, muscle_index) = 0.5 * (moment_arm_jacobian_6dof(0, muscle_index) + moment_arm_jacobian_6dof(1, muscle_index));
            moment_arm_jacobian_3dof(1, muscle_index) = 0.5 * (moment_arm_jacobian_6dof(2, muscle_index) + moment_arm_jacobian_6dof(3, muscle_index));
            moment_arm_jacobian_3dof(2, muscle_index) = 0.5 * (moment_arm_jacobian_6dof(4, muscle_index) + moment_arm_jacobian_6dof(5, muscle_index));
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

    double regular_canal_afferent_outputs[6]        = {0.0};
    double regular_canal_afferent_outputs_dot[6]    = {0.0};
    double regular_canal_afferent_outputs_ddot[6]   = {0.0};
    double irregular_canal_afferent_outputs[6]      = {0.0};
    double irregular_canal_afferent_outputs_dot[6]  = {0.0};
    double irregular_canal_afferent_outputs_ddot[6] = {0.0};
    for (int canal_index = 0; canal_index < 6; ++canal_index) {
        const double u_canal = canal_angular_velocity_inputs[canal_index];

        const int regular_filter_index = 0;
        const CanalFilterParameters& reg_p = canal_filter_parameters[regular_filter_index];
        double reg_state_1 = state_variable_semicircular_canal[canal_index][regular_filter_index][0]->getValue(state);
        double reg_state_2 = state_variable_semicircular_canal[canal_index][regular_filter_index][1]->getValue(state);
        double reg_x1_dot  = reg_state_2;
        double reg_x2_dot  = -reg_p.denominator_constant_a0 * reg_state_1 - reg_p.denominator_linear_a1 * reg_state_2 + u_canal;
        double reg_x1_ddot = reg_x2_dot;
        double reg_x2_ddot = -reg_p.denominator_constant_a0 * reg_x1_dot - reg_p.denominator_linear_a1 * reg_x2_dot;
        regular_canal_afferent_outputs[canal_index] =
            reg_p.state_space_output_c0 * reg_state_1 + reg_p.state_space_output_c1 * reg_state_2 + reg_p.direct_feedthrough_d * u_canal;
        regular_canal_afferent_outputs_dot[canal_index] =
            reg_p.state_space_output_c0 * reg_x1_dot  + reg_p.state_space_output_c1 * reg_x2_dot;
        regular_canal_afferent_outputs_ddot[canal_index] =
            reg_p.state_space_output_c0 * reg_x1_ddot + reg_p.state_space_output_c1 * reg_x2_ddot;

        const int irregular_filter_index = 1;
        const CanalFilterParameters& irr_p = canal_filter_parameters[irregular_filter_index];
        double irr_state_1 = state_variable_semicircular_canal[canal_index][irregular_filter_index][0]->getValue(state);
        double irr_state_2 = state_variable_semicircular_canal[canal_index][irregular_filter_index][1]->getValue(state);
        double irr_x1_dot  = irr_state_2;
        double irr_x2_dot  = -irr_p.denominator_constant_a0 * irr_state_1 - irr_p.denominator_linear_a1 * irr_state_2 + u_canal;
        double irr_x1_ddot = irr_x2_dot;
        double irr_x2_ddot = -irr_p.denominator_constant_a0 * irr_x1_dot - irr_p.denominator_linear_a1 * irr_x2_dot;
        irregular_canal_afferent_outputs[canal_index] =
            irr_p.state_space_output_c0 * irr_state_1 + irr_p.state_space_output_c1 * irr_state_2 + irr_p.direct_feedthrough_d * u_canal;
        irregular_canal_afferent_outputs_dot[canal_index] =
            irr_p.state_space_output_c0 * irr_x1_dot  + irr_p.state_space_output_c1 * irr_x2_dot;
        irregular_canal_afferent_outputs_ddot[canal_index] =
            irr_p.state_space_output_c0 * irr_x1_ddot + irr_p.state_space_output_c1 * irr_x2_ddot;
    }

    SimTK::Vec3 filtered_right_regular_canal_output(regular_canal_afferent_outputs[0], regular_canal_afferent_outputs[1], regular_canal_afferent_outputs[2]);
    SimTK::Vec3 filtered_left_regular_canal_output(regular_canal_afferent_outputs[3], regular_canal_afferent_outputs[4], regular_canal_afferent_outputs[5]);
    SimTK::Vec3 filtered_right_regular_canal_dot(regular_canal_afferent_outputs_dot[0], regular_canal_afferent_outputs_dot[1], regular_canal_afferent_outputs_dot[2]);
    SimTK::Vec3 filtered_left_regular_canal_dot(regular_canal_afferent_outputs_dot[3], regular_canal_afferent_outputs_dot[4], regular_canal_afferent_outputs_dot[5]);
    SimTK::Vec3 filtered_right_regular_canal_ddot(regular_canal_afferent_outputs_ddot[0], regular_canal_afferent_outputs_ddot[1], regular_canal_afferent_outputs_ddot[2]);
    SimTK::Vec3 filtered_left_regular_canal_ddot(regular_canal_afferent_outputs_ddot[3], regular_canal_afferent_outputs_ddot[4], regular_canal_afferent_outputs_ddot[5]);
    const double regular_canal_passband_gain      = 0.436629; // G_pass,reg = 2.83 * (0.0027 / 0.0175)
    const double regular_canal_lead_time_constant = 0.0175;
    const double regular_canal_pole_time_constant = 0.0027;
    const double luenberger_direct_feedthrough    = regular_canal_pole_time_constant / regular_canal_lead_time_constant; // 0.154286
    const double luenberger_state_weight          = 1.0 - luenberger_direct_feedthrough;                                 // 0.845714
    SimTK::Vec3 reconstructed_regular_canal_velocity =
        (0.5 / regular_canal_passband_gain) * (right_canal_inverse_matrix * filtered_right_regular_canal_output + left_canal_inverse_matrix * filtered_left_regular_canal_output);
    SimTK::Vec3 reconstructed_regular_canal_velocity_dot =
        (0.5 / regular_canal_passband_gain) * (right_canal_inverse_matrix * filtered_right_regular_canal_dot + left_canal_inverse_matrix * filtered_left_regular_canal_dot);
    SimTK::Vec3 reconstructed_regular_canal_velocity_ddot =
        (0.5 / regular_canal_passband_gain) * (right_canal_inverse_matrix * filtered_right_regular_canal_ddot + left_canal_inverse_matrix * filtered_left_regular_canal_ddot);

    SimTK::Vec3 filtered_right_irregular_canal_output(irregular_canal_afferent_outputs[0], irregular_canal_afferent_outputs[1], irregular_canal_afferent_outputs[2]);
    SimTK::Vec3 filtered_left_irregular_canal_output(irregular_canal_afferent_outputs[3], irregular_canal_afferent_outputs[4], irregular_canal_afferent_outputs[5]);
    SimTK::Vec3 filtered_right_irregular_canal_dot(irregular_canal_afferent_outputs_dot[0], irregular_canal_afferent_outputs_dot[1], irregular_canal_afferent_outputs_dot[2]);
    SimTK::Vec3 filtered_left_irregular_canal_dot(irregular_canal_afferent_outputs_dot[3], irregular_canal_afferent_outputs_dot[4], irregular_canal_afferent_outputs_dot[5]);
    SimTK::Vec3 filtered_right_irregular_canal_ddot(irregular_canal_afferent_outputs_ddot[0], irregular_canal_afferent_outputs_ddot[1], irregular_canal_afferent_outputs_ddot[2]);
    SimTK::Vec3 filtered_left_irregular_canal_ddot(irregular_canal_afferent_outputs_ddot[3], irregular_canal_afferent_outputs_ddot[4], irregular_canal_afferent_outputs_ddot[5]);
    const double irregular_canal_passband_gain = 0.541800; // G_pass,irr = 27.09 * (0.0006 / 0.0300)
    SimTK::Vec3 reconstructed_canal_angular_velocity =
        (0.5 / irregular_canal_passband_gain) * (right_canal_inverse_matrix * filtered_right_irregular_canal_output + left_canal_inverse_matrix * filtered_left_irregular_canal_output);
    SimTK::Vec3 reconstructed_canal_angular_velocity_dot =
        (0.5 / irregular_canal_passband_gain) * (right_canal_inverse_matrix * filtered_right_irregular_canal_dot + left_canal_inverse_matrix * filtered_left_irregular_canal_dot);
    SimTK::Vec3 reconstructed_canal_angular_velocity_ddot =
        (0.5 / irregular_canal_passband_gain) * (right_canal_inverse_matrix * filtered_right_irregular_canal_ddot + left_canal_inverse_matrix * filtered_left_irregular_canal_ddot);

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
    SimTK::Vec3 luenberger_state_dot =
        (reconstructed_regular_canal_velocity - luenberger_state_vector) / regular_canal_lead_time_constant;
    SimTK::Vec3 luenberger_state_ddot =
        (reconstructed_regular_canal_velocity_dot - luenberger_state_dot) / regular_canal_lead_time_constant;
    SimTK::Vec3 current_world_angular_velocity =
        luenberger_state_weight * luenberger_state_vector + luenberger_direct_feedthrough * reconstructed_regular_canal_velocity;
    SimTK::Vec3 current_world_angular_acceleration =
        luenberger_state_weight * luenberger_state_dot + luenberger_direct_feedthrough * reconstructed_regular_canal_velocity_dot;
    SimTK::Vec3 current_world_angular_jerk =
        luenberger_state_weight * luenberger_state_ddot + luenberger_direct_feedthrough * reconstructed_regular_canal_velocity_ddot;
    SimTK::Vec3 current_head_frame_angular_velocity = (~current_estimated_rotation_head_to_world) * current_world_angular_velocity;
    SimTK::Vec3 current_head_frame_angular_acceleration = (~current_estimated_rotation_head_to_world) * current_world_angular_acceleration;
    SimTK::Vec3 current_head_frame_angular_jerk = (~current_estimated_rotation_head_to_world) * current_world_angular_jerk;
    mahony_estimated_angular_velocity = SimTK::Vec3(
        current_head_frame_angular_velocity[2],
        current_head_frame_angular_velocity[0],
        current_head_frame_angular_velocity[1]); // pitch, roll, yaw
    SimTK::Vec3 mahony_estimated_angular_acceleration(
        current_head_frame_angular_acceleration[2],
        current_head_frame_angular_acceleration[0],
        current_head_frame_angular_acceleration[1]); // pitch, roll, yaw
    SimTK::Vec3 mahony_estimated_angular_jerk(
        current_head_frame_angular_jerk[2],
        current_head_frame_angular_jerk[0],
        current_head_frame_angular_jerk[1]); // pitch, roll, yaw

    // --- ATLAS ANATOMICAL INTER-BLOCK DELAYS (13 INDEPENDENT BRANCHES) ---
    const double tau_1a_m1_vn_tilt    = 0.0090; // Branch 1A (Sub 1 -> Sub 5 Brainstem VN Tonic Tilt): 9.0 ms
    const double tau_1b_m1_vn_canal   = 0.0100; // Branch 1B (Sub 1 -> Sub 5 Brainstem VN Canal/Phasic): 10.0 ms
    const double tau_1c_m1_spinal_cst = 0.0145; // Branch 1C (Sub 1 -> Sub 7 Spinal Lamina IX QP & Fusimotor): 14.5 ms
    const double tau_2a_cn8_vn_delay  = 0.0020; // Branch 2A (Sub 2 -> Sub 5 Brainstem VN): 2.0 ms
    const double tau_2b_cn8_cb_delay  = 0.0030; // Branch 2B (Sub 2 -> Sub 4 Cerebellum): 3.0 ms
    const double tau_3a_drg_ccr_delay = 0.0055; // Branch 3A (Sub 3 -> Sub 6 Spinal CCR): 5.5 ms
    const double tau_3b_cuneate_delay = 0.0070; // Branch 3B (Sub 3 -> Sub 4 Cerebellum Cuneate): 7.0 ms
    const double tau_4a_fn_vn_delay   = 0.0015; // Branch 3->2B (Cerebellum -> Brainstem VN Uncinate Tract): 1.5 ms
    const double tau_4b_thal_m1_delay = 0.0130; // Branch 3->4 (Cerebellum -> VL/VA Thalamus -> M1 Cortex): 13.0 ms
    const double tau_5_mvst_vcr_delay = 0.0025; // Path 2B->5 (MVST): fixed 2.5 ms axonal delay
    const double tau_6_prop_ccr_delay = 0.0040; // Path 1B->5 (Propriospinal): fixed 4.0 ms axonal delay
    const double tau_7a_aalpha_delay  = 0.0055; // Branch 5->Muscles (72 Extrafusal Muscles A-alpha + NMJ): 5.5 ms
    const double tau_7b_agamma_delay  = 0.0080; // Branch 7B (Sub 7 -> Sub 3 72 Spindles A-gamma): 8.0 ms

    // --- SUBSYSTEM 1 SENDER-SIDE DELAY COMPENSATORS (1 + tau*s + 0.5*tau^2*s^2) FOR BRANCHES 1A, 1B, 1C ---
    DelayedData delayed_4b_thal_m1_ctrl = getDelayedData(state.getTime(), tau_4b_thal_m1_delay);
    double del_mahony_pitch = (!history.empty() && delayed_4b_thal_m1_ctrl.record.valid)
        ? (delayed_4b_thal_m1_ctrl.record.estimated_head_orientation[2] - baseline_spatial_orientation[2])
        : (estimated_head_pitch - baseline_spatial_orientation[2]);
    double del_mahony_roll = (!history.empty() && delayed_4b_thal_m1_ctrl.record.valid)
        ? (delayed_4b_thal_m1_ctrl.record.estimated_head_orientation[0] - baseline_spatial_orientation[0])
        : (estimated_head_roll - baseline_spatial_orientation[0]);
    double del_mahony_yaw = (!history.empty() && delayed_4b_thal_m1_ctrl.record.valid)
        ? (delayed_4b_thal_m1_ctrl.record.estimated_head_orientation[1] - baseline_spatial_orientation[1])
        : (estimated_head_yaw - baseline_spatial_orientation[1]);
    double del_mahony_pitch_spd = (!history.empty() && delayed_4b_thal_m1_ctrl.record.valid)
        ? delayed_4b_thal_m1_ctrl.record.mahony_estimated_angular_velocity[0]
        : mahony_estimated_angular_velocity[0];
    double del_mahony_roll_spd = (!history.empty() && delayed_4b_thal_m1_ctrl.record.valid)
        ? delayed_4b_thal_m1_ctrl.record.mahony_estimated_angular_velocity[1]
        : mahony_estimated_angular_velocity[1];
    double del_mahony_yaw_spd = (!history.empty() && delayed_4b_thal_m1_ctrl.record.valid)
        ? delayed_4b_thal_m1_ctrl.record.mahony_estimated_angular_velocity[2]
        : mahony_estimated_angular_velocity[2];

    const bool adapt_on = get_enable_adaptation();
    double ki_p_ctrl = (sv_adapt_theta[14] && adapt_on)
        ? sv_adapt_theta[14]->getValue(state)
        : ((get_Ki_task_pitch() >= 0.0) ? get_Ki_task_pitch() : get_Ki_task());
    double ki_r_ctrl = (get_Ki_task_roll()  >= 0.0) ? get_Ki_task_roll()  : 0.0;
    double ki_y_ctrl = (get_Ki_task_yaw()   >= 0.0) ? get_Ki_task_yaw()   : 0.0;
    double vig_p_ctrl = 1.0 + get_k_vigor() * std::abs(trajectory_velocity_pitch);
    double kp_p_ctrl = (sv_adapt_theta[12] && adapt_on)
        ? sv_adapt_theta[12]->getValue(state)
        : ((get_Kp_task_pitch() >= 0.0) ? get_Kp_task_pitch() : std::sqrt(get_Wp_base_pitch() * vig_p_ctrl));
    double kd_p_ctrl = (sv_adapt_theta[13] && adapt_on)
        ? sv_adapt_theta[13]->getValue(state)
        : ((get_Kd_task_pitch() >= 0.0) ? get_Kd_task_pitch() : std::sqrt(get_Wv_base_pitch() * vig_p_ctrl + 2.0 * kp_p_ctrl));
    double vig_r_ctrl = 1.0 + get_k_vigor() * std::abs(trajectory_velocity_roll);
    double kp_r_ctrl  = (get_Kp_task_roll() >= 0.0) ? get_Kp_task_roll() : std::sqrt(get_Wp_base_roll() * vig_r_ctrl);
    double kd_r_ctrl  = (get_Kd_task_roll() >= 0.0) ? get_Kd_task_roll() : std::sqrt(get_Wv_base_roll() * vig_r_ctrl + 2.0 * kp_r_ctrl);
    double vig_y_ctrl = 1.0 + get_k_vigor() * std::abs(trajectory_velocity_yaw);
    double kp_y_ctrl  = (get_Kp_task_yaw() >= 0.0) ? get_Kp_task_yaw() : std::sqrt(get_Wp_base_yaw() * vig_y_ctrl);
    double kd_y_ctrl  = (get_Kd_task_yaw() >= 0.0) ? get_Kd_task_yaw() : std::sqrt(get_Wv_base_yaw() * vig_y_ctrl + 2.0 * kp_y_ctrl);

    double u_pid_pitch = kp_p_ctrl * (trajectory_position_pitch - del_mahony_pitch) + kd_p_ctrl * (trajectory_velocity_pitch - del_mahony_pitch_spd) + ki_p_ctrl * state_variable_error_integral[0]->getValue(state) + 0.05 * trajectory_acceleration_pitch;
    double u_pid_roll  = kp_r_ctrl * (trajectory_position_roll  - del_mahony_roll)  + kd_r_ctrl * (trajectory_velocity_roll  - del_mahony_roll_spd)  + ki_r_ctrl * state_variable_error_integral[1]->getValue(state) + 0.05 * trajectory_acceleration_roll;
    double u_pid_yaw   = kp_y_ctrl * (trajectory_position_yaw   - del_mahony_yaw)   + kd_y_ctrl * (trajectory_velocity_yaw   - del_mahony_yaw_spd)   + ki_y_ctrl * state_variable_error_integral[2]->getValue(state) + 0.05 * trajectory_acceleration_yaw;

    double fc_vol_ctrl = std::max(0.1, get_fc_voluntary());
    double wc_vol_ctrl = 2.0 * SimTK::Pi * fc_vol_ctrl;
    double wc_vol_sq_ctrl = wc_vol_ctrl * wc_vol_ctrl;

    SimTK::Vec3 vol_y(
        state_variable_voluntary_filter_stage1[0]->getValue(state),
        state_variable_voluntary_filter_stage1[1]->getValue(state),
        state_variable_voluntary_filter_stage1[2]->getValue(state));
    SimTK::Vec3 vol_ydot(
        state_variable_voluntary_filter_stage2[0]->getValue(state),
        state_variable_voluntary_filter_stage2[1]->getValue(state),
        state_variable_voluntary_filter_stage2[2]->getValue(state));
    SimTK::Vec3 vol_yddot(
        wc_vol_sq_ctrl * (u_pid_pitch - vol_y[0]) - 2.0 * wc_vol_ctrl * vol_ydot[0],
        wc_vol_sq_ctrl * (u_pid_roll  - vol_y[1]) - 2.0 * wc_vol_ctrl * vol_ydot[1],
        wc_vol_sq_ctrl * (u_pid_yaw   - vol_y[2]) - 2.0 * wc_vol_ctrl * vol_ydot[2]);

    // Branch 1C (14.5 ms -> Sub 7): Compensated Cortical Voluntary Torque y + tau_1C*ydot + 0.5*tau_1C^2*yddot
    SimTK::Vec3 current_cortical_voluntary_torque =
        vol_y + tau_1c_m1_spinal_cst * vol_ydot + (0.5 * tau_1c_m1_spinal_cst * tau_1c_m1_spinal_cst) * vol_yddot;
    SimTK::Vec3 current_cortical_voluntary_torque_dot =
        vol_ydot + tau_1c_m1_spinal_cst * vol_yddot;

    // Trajectory higher derivatives (jerk, snap, crackle from 2nd-order critically damped planner)
    const double inv_tau_sq = 1.0 / (trajectory_time_constant * trajectory_time_constant);
    const double two_inv_tau = 2.0 / trajectory_time_constant;
    SimTK::Vec3 traj_pos_vec(trajectory_position_pitch, trajectory_position_roll, trajectory_position_yaw);
    SimTK::Vec3 traj_vel_vec(trajectory_velocity_pitch, trajectory_velocity_roll, trajectory_velocity_yaw);
    SimTK::Vec3 traj_acc_vec(trajectory_acceleration_pitch, trajectory_acceleration_roll, trajectory_acceleration_yaw);
    SimTK::Vec3 traj_jerk_vec(trajectory_jerk_pitch, trajectory_jerk_roll, trajectory_jerk_yaw);
    SimTK::Vec3 traj_snap_vec    = -inv_tau_sq * traj_acc_vec  - two_inv_tau * traj_jerk_vec;
    SimTK::Vec3 traj_crackle_vec = -inv_tau_sq * traj_jerk_vec - two_inv_tau * traj_snap_vec;

    // Branch 1A (9.0 ms -> Sub 5 Tonic Tilt):
    SimTK::Vec3 traj_pos_comp_1a =
        traj_pos_vec + tau_1a_m1_vn_tilt * traj_vel_vec + (0.5 * tau_1a_m1_vn_tilt * tau_1a_m1_vn_tilt) * traj_acc_vec;
    // Branch 1B (10.0 ms -> Sub 5 Canal & Phasic):
    SimTK::Vec3 traj_vel_comp_1b =
        traj_vel_vec + tau_1b_m1_vn_canal * traj_acc_vec + (0.5 * tau_1b_m1_vn_canal * tau_1b_m1_vn_canal) * traj_jerk_vec;
    SimTK::Vec3 traj_acc_comp_1b =
        traj_acc_vec + tau_1b_m1_vn_canal * traj_jerk_vec + (0.5 * tau_1b_m1_vn_canal * tau_1b_m1_vn_canal) * traj_snap_vec;
    // Branch 1C (14.5 ms -> Sub 7 Fusimotor Planner):
    SimTK::Vec3 traj_pos_comp_1c =
        traj_pos_vec + tau_1c_m1_spinal_cst * traj_vel_vec + (0.5 * tau_1c_m1_spinal_cst * tau_1c_m1_spinal_cst) * traj_acc_vec;
    SimTK::Vec3 traj_vel_comp_1c =
        traj_vel_vec + tau_1c_m1_spinal_cst * traj_acc_vec + (0.5 * tau_1c_m1_spinal_cst * tau_1c_m1_spinal_cst) * traj_jerk_vec;
    SimTK::Vec3 traj_acc_comp_1c =
        traj_acc_vec + tau_1c_m1_spinal_cst * traj_jerk_vec + (0.5 * tau_1c_m1_spinal_cst * tau_1c_m1_spinal_cst) * traj_snap_vec;
    SimTK::Vec3 traj_jerk_comp_1c =
        traj_jerk_vec + tau_1c_m1_spinal_cst * traj_snap_vec + (0.5 * tau_1c_m1_spinal_cst * tau_1c_m1_spinal_cst) * traj_crackle_vec;

    // --- SUBSYSTEM 2 OTOLITH RECONSTRUCTION & SENDER-SIDE DELAY COMPENSATORS (BRANCH 2A [2.0 ms] & BRANCH 2B [3.0 ms]) ---
    SimTK::Rotation current_rotation_world_to_head = cached_skull->getMobilizedBody().getBodyTransform(state).R().invert();
    SimTK::Vec3 gravity_vector_world_hist(0.0, -9.81, 0.0);
    DelayedData current_otolith_kinematics = getDelayedData(state.getTime(), 0.0005);
    SimTK::Vec3 current_gravitoinertial_force_head =
        current_rotation_world_to_head * (current_otolith_kinematics.linear_acceleration - gravity_vector_world_hist);

    const double reg_oto_adapt_ratio        = 37.4 / 66.0;   // 0.566667
    const double irr_oto_adapt_ratio        = 37.4 / 245.0;  // 0.152653
    const double reg_oto_adapt_tau          = 1.0000;
    const double irr_oto_adapt_tau          = 0.0760;
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
    SimTK::Vec3 irr_oto_state_dot_vec(0.0);
    SimTK::Vec3 irr_oto_state_ddot_vec(0.0);
    SimTK::Vec3 luenberger_otolith_specific_force(0.0);
    SimTK::Vec3 luen_oto_state_dot_vec(0.0);
    SimTK::Vec3 luen_oto_state_ddot_vec(0.0);
    for (int axis_index = 0; axis_index < 3; ++axis_index) {
        double u_gif           = current_gravitoinertial_force_head[axis_index];
        double reg_adapt_state = state_variable_regular_otolith_adapt[axis_index]->getValue(state);
        double reg_mac_state   = state_variable_regular_otolith[axis_index]->getValue(state);
        double irr_adapt_state = state_variable_irregular_otolith_adapt[axis_index]->getValue(state);
        double irr_mac_state   = state_variable_irregular_otolith[axis_index]->getValue(state);
        double luen_mac_state  = state_variable_luenberger_otolith[axis_index]->getValue(state);
        double luen_adp_state  = state_variable_luenberger_otolith_adapt[axis_index]->getValue(state);

        double irr_stage1_out  = (irr_oto_adapt_ratio - 1.0) * irr_adapt_state + u_gif;
        double irr_adapt_dot   = (u_gif - irr_adapt_state) / irr_oto_adapt_tau;
        double irr_stage1_dot  = (irr_oto_adapt_ratio - 1.0) * irr_adapt_dot;
        double irr_mac_dot     = (irr_stage1_out - irr_mac_state) / irr_oto_pole_b;
        double irr_adapt_ddot  = -irr_adapt_dot / irr_oto_adapt_tau;
        double irr_stage1_ddot = (irr_oto_adapt_ratio - 1.0) * irr_adapt_ddot;
        double irr_mac_ddot    = (irr_stage1_dot - irr_mac_dot) / irr_oto_pole_b;

        double irregular_otolith_firing_rate =
            irregular_otolith_passband_gain * (irr_macular_state_weight * irr_mac_state + irr_macular_feedthrough * irr_stage1_out);
        reconstructed_otolith_specific_force[axis_index] = irregular_otolith_firing_rate / irregular_otolith_passband_gain;
        irr_oto_state_dot_vec[axis_index]  = irr_macular_state_weight * irr_mac_dot  + irr_macular_feedthrough * irr_stage1_dot;
        irr_oto_state_ddot_vec[axis_index] = irr_macular_state_weight * irr_mac_ddot + irr_macular_feedthrough * irr_stage1_ddot;

        double reg_stage1_out  = (reg_oto_adapt_ratio - 1.0) * reg_adapt_state + u_gif;
        double reg_adapt_dot   = (u_gif - reg_adapt_state) / reg_oto_adapt_tau;
        double reg_stage1_dot  = (reg_oto_adapt_ratio - 1.0) * reg_adapt_dot;
        double reg_mac_dot     = (reg_stage1_out - reg_mac_state) / reg_oto_pole_b;
        double reg_adapt_ddot  = -reg_adapt_dot / reg_oto_adapt_tau;
        double reg_stage1_ddot = (reg_oto_adapt_ratio - 1.0) * reg_adapt_ddot;
        double reg_mac_ddot    = (reg_stage1_dot - reg_mac_dot) / reg_oto_pole_b;

        double regular_otolith_firing_rate =
            regular_otolith_passband_gain * (reg_macular_state_weight * reg_mac_state + reg_macular_feedthrough * reg_stage1_out);
        double reconstructed_regular_otolith_force      = regular_otolith_firing_rate / regular_otolith_passband_gain;
        double reconstructed_regular_otolith_force_dot  = reg_macular_state_weight * reg_mac_dot  + reg_macular_feedthrough * reg_stage1_dot;
        double reconstructed_regular_otolith_force_ddot = reg_macular_state_weight * reg_mac_ddot + reg_macular_feedthrough * reg_stage1_ddot;

        double luen_mac_dot    = (reconstructed_regular_otolith_force - luen_mac_state) / reg_oto_lead_tau1;
        double luen_stage1_out = luen_macular_state_weight * luen_mac_state + luen_macular_feedthrough * reconstructed_regular_otolith_force;
        double luen_stage1_dot = luen_macular_state_weight * luen_mac_dot   + luen_macular_feedthrough * reconstructed_regular_otolith_force_dot;
        double luen_mac_ddot   = (reconstructed_regular_otolith_force_dot - luen_mac_dot) / reg_oto_lead_tau1;
        double luen_stage1_ddot = luen_macular_state_weight * luen_mac_ddot + luen_macular_feedthrough * reconstructed_regular_otolith_force_ddot;

        double luen_adp_dot  = (luen_stage1_out - luen_adp_state) / (reg_oto_adapt_ratio * reg_oto_adapt_tau);
        double luen_adp_ddot = (luen_stage1_dot - luen_adp_dot)   / (reg_oto_adapt_ratio * reg_oto_adapt_tau);

        luenberger_otolith_specific_force[axis_index] = luen_adapt_state_weight * luen_adp_state + luen_stage1_out;
        luen_oto_state_dot_vec[axis_index]            = luen_adapt_state_weight * luen_adp_dot   + luen_stage1_dot;
        luen_oto_state_ddot_vec[axis_index]           = luen_adapt_state_weight * luen_adp_ddot  + luen_stage1_ddot;
    }

    // Branch 2A (2.0 ms -> Sub 5) & Branch 2B (3.0 ms -> Sub 4) Sender-Side Delay Compensators (1 + tau*s + 0.5*tau^2*s^2):
    SimTK::Vec3 canal_irr_comp_2a =
        reconstructed_canal_angular_velocity +
        tau_2a_cn8_vn_delay * reconstructed_canal_angular_velocity_dot +
        (0.5 * tau_2a_cn8_vn_delay * tau_2a_cn8_vn_delay) * reconstructed_canal_angular_velocity_ddot;
    SimTK::Vec3 canal_luen_comp_2b =
        current_world_angular_velocity +
        tau_2b_cn8_cb_delay * current_world_angular_acceleration +
        (0.5 * tau_2b_cn8_cb_delay * tau_2b_cn8_cb_delay) * current_world_angular_jerk;
    SimTK::Vec3 canal_luen_acc_comp_2b =
        current_world_angular_acceleration + tau_2b_cn8_cb_delay * current_world_angular_jerk;

    SimTK::Vec3 oto_irr_dot  = irr_oto_state_dot_vec - (current_head_frame_angular_velocity % reconstructed_otolith_specific_force);
    SimTK::Vec3 oto_irr_ddot = irr_oto_state_ddot_vec
                               - (current_head_frame_angular_acceleration % reconstructed_otolith_specific_force)
                               - (current_head_frame_angular_velocity % oto_irr_dot);
    SimTK::Vec3 oto_irr_comp_2a =
        reconstructed_otolith_specific_force +
        tau_2a_cn8_vn_delay * oto_irr_dot +
        (0.5 * tau_2a_cn8_vn_delay * tau_2a_cn8_vn_delay) * oto_irr_ddot;

    SimTK::Vec3 oto_luen_dot  = luen_oto_state_dot_vec - (current_head_frame_angular_velocity % luenberger_otolith_specific_force);
    SimTK::Vec3 oto_luen_ddot = luen_oto_state_ddot_vec
                                - (current_head_frame_angular_acceleration % luenberger_otolith_specific_force)
                                - (current_head_frame_angular_velocity % oto_luen_dot);
    SimTK::Vec3 oto_luen_comp_2b =
        luenberger_otolith_specific_force +
        tau_2b_cn8_cb_delay * oto_luen_dot +
        (0.5 * tau_2b_cn8_cb_delay * tau_2b_cn8_cb_delay) * oto_luen_ddot;

    // --- SUBSYSTEM 4 SENDER-SIDE DELAY COMPENSATORS FOR BRANCH 4A (1.5 ms -> Sub 5) & BRANCH 4B (13.0 ms -> Sub 1) ---
    SimTK::Vec3 est_head_ori_raw(estimated_head_roll, estimated_head_yaw, estimated_head_pitch);
    SimTK::Vec3 est_head_ori_comp_4a =
        est_head_ori_raw +
        tau_4a_fn_vn_delay * current_world_angular_velocity +
        (0.5 * tau_4a_fn_vn_delay * tau_4a_fn_vn_delay) * current_world_angular_acceleration;
    SimTK::Vec3 est_head_ori_comp_4b =
        est_head_ori_raw +
        tau_4b_thal_m1_delay * current_world_angular_velocity +
        (0.5 * tau_4b_thal_m1_delay * tau_4b_thal_m1_delay) * current_world_angular_acceleration;
    SimTK::Vec3 mahony_vel_comp_4b =
        mahony_estimated_angular_velocity +
        tau_4b_thal_m1_delay * mahony_estimated_angular_acceleration +
        (0.5 * tau_4b_thal_m1_delay * tau_4b_thal_m1_delay) * mahony_estimated_angular_jerk;
    SimTK::Rotation est_rot_comp_4a;
    est_rot_comp_4a.setRotationToBodyFixedXYZ(
        SimTK::Vec3(est_head_ori_comp_4a[0], est_head_ori_comp_4a[1], est_head_ori_comp_4a[2]));

    HistoryRecord current_history_record;
    current_history_record.valid = true;
    current_history_record.estimated_head_orientation = est_head_ori_comp_4b;
    current_history_record.estimated_head_orientation_4a = est_head_ori_comp_4a;
    current_history_record.reconstructed_canal_angular_velocity = canal_irr_comp_2a;
    current_history_record.luenberger_angular_velocity = canal_luen_comp_2b;
    current_history_record.luenberger_angular_acceleration = canal_luen_acc_comp_2b;
    current_history_record.reconstructed_otolith_specific_force = oto_irr_comp_2a;
    current_history_record.luenberger_otolith_specific_force = oto_luen_comp_2b;
    current_history_record.mahony_estimated_angular_velocity = mahony_vel_comp_4b;
    current_history_record.cortical_voluntary_torque = current_cortical_voluntary_torque;
    current_history_record.cortical_voluntary_torque_dot = current_cortical_voluntary_torque_dot;
    current_history_record.cortical_voluntary_torque_ddot = vol_yddot;
    current_history_record.trajectory_position_1a = traj_pos_comp_1a;
    current_history_record.trajectory_velocity_1b = traj_vel_comp_1b;
    current_history_record.trajectory_acceleration_1b = traj_acc_comp_1b;
    current_history_record.trajectory_position = traj_pos_comp_1c;
    current_history_record.trajectory_velocity = traj_vel_comp_1c;
    current_history_record.trajectory_acceleration = traj_acc_comp_1c;
    current_history_record.trajectory_jerk = traj_jerk_comp_1c;
    current_history_record.rotation_world_to_head = cached_skull->getMobilizedBody().getBodyTransform(state).R().invert().asMat33();
    current_history_record.estimated_rotation_head_to_world = est_rot_comp_4a.asMat33();
    current_history_record.trunk_orientation = SimTK::Vec3(angle_ground_roll, angle_ground_yaw, angle_ground_pitch)
        + tau_3b_cuneate_delay * SimTK::Vec3(speed_ground_roll, speed_ground_yaw, speed_ground_pitch);
    current_history_record.trunk_angular_velocity = SimTK::Vec3(speed_ground_roll, speed_ground_yaw, speed_ground_pitch);
    current_history_record.head_linear_velocity = cached_skull->getVelocityInGround(state)[1];

    const double k_gamma_dyn_eff  = std::clamp((sv_adapt_theta[10] && adapt_on) ? sv_adapt_theta[10]->getValue(state) : get_K_gamma_dyn(),  0.0, 1.0);
    const double k_gamma_stat_eff = std::clamp((sv_adapt_theta[11] && adapt_on) ? sv_adapt_theta[11]->getValue(state) : get_K_gamma_stat(), 0.0, 1.0);

    if (history.empty()) {
        for (int muscle_index = 0; muscle_index < number_of_active_muscles; ++muscle_index) {
            const Muscle& mus_init = muscles.get(active_muscle_indices[muscle_index]);
            const double l_opt_init = std::max(1e-4, cached_optimal_fiber_lengths[muscle_index]);
            double L0_norm_init = baseline_normalized_fiber_lengths[muscle_index];
            try {
                L0_norm_init = std::clamp(mus_init.getFiberLength(state) / l_opt_init, 0.2, 2.5);
            } catch (...) {}
            baseline_normalized_fiber_lengths[muscle_index] = L0_norm_init;
            baseline_primary_afferent_rates[muscle_index] =
                cached_primary_afferent_outputs[muscle_index] ? cached_primary_afferent_outputs[muscle_index]->getValue(state) : 0.0;
            baseline_secondary_afferent_rates[muscle_index] =
                cached_secondary_afferent_outputs[muscle_index] ? cached_secondary_afferent_outputs[muscle_index]->getValue(state) : 0.0;
            const double fd0_init = state_variable_spindle_dynamic_fusimotor[muscle_index]->getValue(state);
            const double fs0_init = state_variable_spindle_static_fusimotor[muscle_index]->getValue(state);
            const double u0_init  = (muscle_index < (int)cached_minimum_excitations.size()) ? cached_minimum_excitations[muscle_index] : 0.03;
            const double gs0_init = 100.0 * k_gamma_stat_eff * u0_init;
            const double fc0_init = (gs0_init * gs0_init) / (gs0_init * gs0_init + 8100.0);
            double ref_ia_0 = 0.0, ref_ii_0 = 0.0, kappa_0 = 1.0;
            evaluateReferenceMileusnicSpindle(muscle_index, L0_norm_init, 0.0, fd0_init, fs0_init, fc0_init, ref_ia_0, ref_ii_0, kappa_0);
            baseline_reference_ia_rates[muscle_index] = ref_ia_0;
            baseline_reference_ii_rates[muscle_index] = ref_ii_0;
        }
    }
    current_history_record.primary_afferent_firing_rate.resize(number_of_active_muscles);
    std::vector<double> estimated_fiber_length_changes(number_of_active_muscles, 0.0);
    std::vector<double> estimated_fiber_velocities(number_of_active_muscles, 0.0);
    std::vector<double> estimated_fiber_accelerations(number_of_active_muscles, 0.0);
    std::vector<double> estimated_fiber_jerks(number_of_active_muscles, 0.0);
    DelayedData delayed_spinal_for_ccr = getDelayedData(state.getTime(), tau_7b_agamma_delay, true);
    const HistoryRecord& sp_rec_ccr = delayed_spinal_for_ccr.record;
    const bool has_sp_gamma_ccr = (sp_rec_ccr.valid &&
        sp_rec_ccr.spinal_gamma_stat_commands.size() == number_of_active_muscles &&
        sp_rec_ccr.expected_normalized_fiber_lengths.size() == number_of_active_muscles &&
        sp_rec_ccr.expected_normalized_fiber_velocities.size() == number_of_active_muscles);

    for (int muscle_index = 0; muscle_index < number_of_active_muscles; ++muscle_index) {
        estimated_fiber_length_changes[muscle_index] = state_variable_spindle_length_change[muscle_index]->getValue(state);
        estimated_fiber_velocities[muscle_index]     = state_variable_spindle_velocity[muscle_index]->getValue(state);
        double primary_afferent_value =
            cached_primary_afferent_outputs[muscle_index]
                ? cached_primary_afferent_outputs[muscle_index]->getValue(state)
                : baseline_primary_afferent_rates[muscle_index];
        double secondary_afferent_value =
            cached_secondary_afferent_outputs[muscle_index]
                ? cached_secondary_afferent_outputs[muscle_index]->getValue(state)
                : baseline_secondary_afferent_rates[muscle_index];
        static const double spindle_ccr_threshold = std::getenv("SPINDLE_CCR_THRESH") ? std::stod(std::getenv("SPINDLE_CCR_THRESH")) : 0.5;
        double dynamic_fusimotor_activation = state_variable_spindle_dynamic_fusimotor[muscle_index]->getValue(state);
        double static_fusimotor_activation  = state_variable_spindle_static_fusimotor[muscle_index]->getValue(state);
        double u0_alpha = (muscle_index < (int)cached_minimum_excitations.size()) ? cached_minimum_excitations[muscle_index] : 0.03;
        double L0_norm  = (muscle_index < (int)baseline_normalized_fiber_lengths.size()) ? baseline_normalized_fiber_lengths[muscle_index] : 1.0;

        double arriving_gamma_stat = has_sp_gamma_ccr ? sp_rec_ccr.spinal_gamma_stat_commands[muscle_index] : (100.0 * k_gamma_stat_eff * u0_alpha);
        double instantaneous_nuclear_chain_activation =
            (arriving_gamma_stat * arriving_gamma_stat) / (arriving_gamma_stat * arriving_gamma_stat + 8100.0);
        double arriving_L_hat      = has_sp_gamma_ccr ? sp_rec_ccr.expected_normalized_fiber_lengths[muscle_index]    : L0_norm;
        double arriving_Lp_hat     = has_sp_gamma_ccr ? sp_rec_ccr.expected_normalized_fiber_velocities[muscle_index] : 0.0;

        double ref_ia = 0.0, ref_ii = 0.0, kappa_gain = 1.0;
        evaluateReferenceMileusnicSpindle(
            muscle_index,
            arriving_L_hat,
            arriving_Lp_hat,
            dynamic_fusimotor_activation,
            static_fusimotor_activation,
            instantaneous_nuclear_chain_activation,
            ref_ia,
            ref_ii,
            kappa_gain);
        double expected_ia = baseline_primary_afferent_rates[muscle_index] + (ref_ia - baseline_reference_ia_rates[muscle_index]);
        double expected_ii = baseline_secondary_afferent_rates[muscle_index] + (ref_ii - baseline_reference_ii_rates[muscle_index]);
        const double gd0_ref_c = 100.0 * k_gamma_dyn_eff * (0.25 + 0.75 * u0_alpha);
        const double gs0_ref_c = 100.0 * k_gamma_stat_eff * u0_alpha;
        const double fd0_ref_c = (gd0_ref_c * gd0_ref_c) / (gd0_ref_c * gd0_ref_c + 3600.0);
        const double fs0_ref_c = (gs0_ref_c * gs0_ref_c) / (gs0_ref_c * gs0_ref_c + 3600.0);
        const double denom_k0_c = std::max(0.015, 0.0605 - 0.046 * fs0_ref_c);
        const double kappa_0_c  = 1.0 + (0.2592 * fd0_ref_c) / denom_k0_c;
        double delta_ia = (primary_afferent_value - expected_ia) * (std::max(1.0, kappa_0_c) / std::max(1.0, kappa_gain));
        double delta_ii = (secondary_afferent_value - expected_ii);

        double net_ia_obs = delta_ia;
        double net_ii_obs = delta_ii;
        static const double spindle_obs_thresh_c = std::getenv("SPINDLE_OBS_THRESH") ? std::stod(std::getenv("SPINDLE_OBS_THRESH")) : 0.5;
        if (spindle_obs_thresh_c > 0.0) {
            if (std::abs(net_ia_obs) <= spindle_obs_thresh_c) {
                net_ia_obs = 0.0;
            } else {
                net_ia_obs = (net_ia_obs > 0.0) ? (net_ia_obs - spindle_obs_thresh_c) : (net_ia_obs + spindle_obs_thresh_c);
            }
            const double sec_th_c = 0.5 * spindle_obs_thresh_c;
            if (std::abs(net_ii_obs) <= sec_th_c) {
                net_ii_obs = 0.0;
            } else {
                net_ii_obs = (net_ii_obs > 0.0) ? (net_ii_obs - sec_th_c) : (net_ii_obs + sec_th_c);
            }
        }

        // Analytical spindle velocity & acceleration derivatives from spindle Luenberger state equations:
        double recon_len_sec = cached_fiber_length_scales[muscle_index] * net_ii_obs;
        double dyn_pri_comp  = net_ia_obs - 1.37931034 * net_ii_obs;
        double vel_len_err   = (recon_len_sec - estimated_fiber_length_changes[muscle_index]) / 0.055;
        double vel_pri_aff   = cached_primary_afferent_velocity_scales[muscle_index] * dyn_pri_comp;
        double sp_accel      = (vel_len_err + vel_pri_aff - estimated_fiber_velocities[muscle_index]) / 0.005;
        double sp_jerk       = (-estimated_fiber_velocities[muscle_index] / 0.055 - sp_accel) / 0.005;
        estimated_fiber_accelerations[muscle_index] = sp_accel;
        estimated_fiber_jerks[muscle_index]         = sp_jerk;

        double primary_afferent_above_baseline = 0.0;
        if (spindle_ccr_threshold <= 0.0) {
            primary_afferent_above_baseline = std::max(0.0, delta_ia);
        } else if (delta_ia > 0.75 * spindle_ccr_threshold) {
            if (delta_ia < 1.25 * spindle_ccr_threshold) {
                double u = delta_ia - 0.75 * spindle_ccr_threshold;
                primary_afferent_above_baseline = (u * u) / spindle_ccr_threshold;
            } else {
                primary_afferent_above_baseline = delta_ia - spindle_ccr_threshold;
            }
        }
        // Branch 3A (5.5 ms -> Sub 6 Spinal CCR) Sender-Side Delay Compensator (1 + tau_3A*s + 0.5*tau_3A^2*s^2):
        double inv_len_scale = (cached_fiber_length_scales[muscle_index] > 1e-8) ? (1.37931034 / cached_fiber_length_scales[muscle_index]) : 0.0;
        double r_ia_dot  = (primary_afferent_above_baseline > 0.0) ? (inv_len_scale * estimated_fiber_velocities[muscle_index]) : 0.0;
        double r_ia_ddot = (primary_afferent_above_baseline > 0.0) ? (inv_len_scale * sp_accel) : 0.0;
        double r_ia_comp_3a = primary_afferent_above_baseline
            + tau_3a_drg_ccr_delay * r_ia_dot
            + (0.5 * tau_3a_drg_ccr_delay * tau_3a_drg_ccr_delay) * r_ia_ddot;
        current_history_record.primary_afferent_firing_rate[muscle_index] = std::clamp(r_ia_comp_3a, 0.0, 500.0);
    }

    // --- SUBSYSTEM 3 STAGE 2: 6-DOF PROPRIOCEPTIVE DECODER & BRANCH 3B (7.0 ms -> Sub 4) DELAY COMPENSATOR ---
    SimTK::Vec3 estimated_neck_relative_accel(0.0);
    SimTK::Vec3 estimated_neck_relative_jerk(0.0);
    if (proprioceptive_decoder_matrix.nrow() == 3 && proprioceptive_decoder_matrix.ncol() == number_of_active_muscles) {
        for (int row = 0; row < 3; ++row) {
            double sum_angle = 0.0;
            double sum_speed = 0.0;
            double sum_accel = 0.0;
            double sum_jerk  = 0.0;
            for (int muscle_index = 0; muscle_index < number_of_active_muscles; ++muscle_index) {
                const double dec_w = proprioceptive_decoder_matrix(row, muscle_index);
                sum_angle += dec_w * estimated_fiber_length_changes[muscle_index];
                sum_speed += dec_w * estimated_fiber_velocities[muscle_index];
                sum_accel += dec_w * estimated_fiber_accelerations[muscle_index];
                sum_jerk  += dec_w * estimated_fiber_jerks[muscle_index];
            }
            estimated_neck_relative_angle[row] = sum_angle;
            estimated_neck_relative_speed[row] = sum_speed;
            estimated_neck_relative_accel[row] = sum_accel;
            estimated_neck_relative_jerk[row]  = sum_jerk;
        }
    }
    // Branch 3B (7.0 ms -> Sub 4 Cerebellum) Sender-Side Delay Compensator (1 + tau_3B*s + 0.5*tau_3B^2*s^2):
    current_history_record.estimated_neck_relative_angle =
        estimated_neck_relative_angle +
        tau_3b_cuneate_delay * estimated_neck_relative_speed +
        (0.5 * tau_3b_cuneate_delay * tau_3b_cuneate_delay) * estimated_neck_relative_accel;
    current_history_record.estimated_neck_relative_speed =
        estimated_neck_relative_speed +
        tau_3b_cuneate_delay * estimated_neck_relative_accel +
        (0.5 * tau_3b_cuneate_delay * tau_3b_cuneate_delay) * estimated_neck_relative_jerk;

    double current_simulation_time = state.getTime();
    DelayedData delayed_1a_m1_vn_data      = getDelayedData(current_simulation_time, tau_1a_m1_vn_tilt);
    DelayedData delayed_1b_m1_vn_data      = getDelayedData(current_simulation_time, tau_1b_m1_vn_canal);
    DelayedData delayed_1c_m1_spinal_data  = getDelayedData(current_simulation_time, tau_1c_m1_spinal_cst);
    DelayedData delayed_2a_cn8_vn_data     = getDelayedData(current_simulation_time, tau_2a_cn8_vn_delay);
    DelayedData delayed_4a_fn_vn_data      = getDelayedData(current_simulation_time, tau_4a_fn_vn_delay);
    DelayedData delayed_3a_drg_ccr_data    = getDelayedData(current_simulation_time, tau_3a_drg_ccr_delay, true);

    // --- SUBSYSTEM 5: BRAINSTEM VCR 2ND-ORDER LITERATURE POLES + PATH 5 (2.5 ms MVST -> Sub 7) DELAY COMPENSATOR ---
    SimTK::Vec3 current_brainstem_vcr_torque(0.0);
    SimTK::Vec3 current_brainstem_vcr_torque_dot(0.0);
    SimTK::Vec3 current_brainstem_vcr_torque_ddot(0.0);
    {
        double eff_traj_pos_pitch = (!history.empty() && delayed_1a_m1_vn_data.record.valid) ? delayed_1a_m1_vn_data.record.trajectory_position_1a[0] : trajectory_position_pitch;
        double eff_traj_pos_roll  = (!history.empty() && delayed_1a_m1_vn_data.record.valid) ? delayed_1a_m1_vn_data.record.trajectory_position_1a[1] : trajectory_position_roll;
        double eff_traj_pos_yaw   = (!history.empty() && delayed_1a_m1_vn_data.record.valid) ? delayed_1a_m1_vn_data.record.trajectory_position_1a[2] : trajectory_position_yaw;

        double eff_traj_vel_pitch = (!history.empty() && delayed_1b_m1_vn_data.record.valid) ? delayed_1b_m1_vn_data.record.trajectory_velocity_1b[0] : trajectory_velocity_pitch;
        double eff_traj_vel_roll  = (!history.empty() && delayed_1b_m1_vn_data.record.valid) ? delayed_1b_m1_vn_data.record.trajectory_velocity_1b[1] : trajectory_velocity_roll;
        double eff_traj_vel_yaw   = (!history.empty() && delayed_1b_m1_vn_data.record.valid) ? delayed_1b_m1_vn_data.record.trajectory_velocity_1b[2] : trajectory_velocity_yaw;

        double eff_traj_acc_pitch = (!history.empty() && delayed_1b_m1_vn_data.record.valid) ? delayed_1b_m1_vn_data.record.trajectory_acceleration_1b[0] : trajectory_acceleration_pitch;
        double eff_traj_acc_roll  = (!history.empty() && delayed_1b_m1_vn_data.record.valid) ? delayed_1b_m1_vn_data.record.trajectory_acceleration_1b[1] : trajectory_acceleration_roll;

        SimTK::Vec3 arriving_estimated_head_orientation =
            (!history.empty() && delayed_4a_fn_vn_data.record.valid)
                ? delayed_4a_fn_vn_data.record.estimated_head_orientation_4a
                : SimTK::Vec3(estimated_head_roll, estimated_head_yaw, estimated_head_pitch);
        double tonic_otolith_input_pitch = (arriving_estimated_head_orientation[2] - baseline_spatial_orientation[2]) - eff_traj_pos_pitch;
        double tonic_otolith_input_roll  = (arriving_estimated_head_orientation[0] - baseline_spatial_orientation[0]) - eff_traj_pos_roll;
        double tonic_otolith_input_yaw   = (arriving_estimated_head_orientation[1] - baseline_spatial_orientation[1]) - eff_traj_pos_yaw;

        SimTK::Vec3 arriving_irregular_canal_velocity =
            (!history.empty() && delayed_2a_cn8_vn_data.record.valid)
                ? delayed_2a_cn8_vn_data.record.reconstructed_canal_angular_velocity
                : reconstructed_canal_angular_velocity;
        double semicircular_canal_input_pitch = arriving_irregular_canal_velocity[2] - eff_traj_vel_pitch;
        double semicircular_canal_input_roll  = arriving_irregular_canal_velocity[0] - eff_traj_vel_roll;
        double semicircular_canal_input_yaw   = arriving_irregular_canal_velocity[1] - eff_traj_vel_yaw;

        SimTK::Vec3 gravity_vector_world(0.0, -9.81, 0.0);
        SimTK::Vec3 arriving_otolith_specific_force_head =
            (!history.empty() && delayed_2a_cn8_vn_data.record.valid)
                ? delayed_2a_cn8_vn_data.record.reconstructed_otolith_specific_force
                : reconstructed_otolith_specific_force;
        SimTK::Mat33 arriving_rot_4a =
            (!history.empty() && delayed_4a_fn_vn_data.record.valid)
                ? delayed_4a_fn_vn_data.record.estimated_rotation_head_to_world
                : current_estimated_rotation_head_to_world.asMat33();
        SimTK::Vec3 arriving_gravity_head = (~arriving_rot_4a) * gravity_vector_world;
        SimTK::Vec3 gravity_adapt_state(
            state_variable_phasic_otolith_stage2[0]->getValue(state),
            state_variable_phasic_otolith_stage2[1]->getValue(state),
            state_variable_phasic_otolith_stage2[2]->getValue(state));
        SimTK::Vec3 adapted_gravity_head =
            (irr_oto_adapt_ratio - 1.0) * gravity_adapt_state + arriving_gravity_head;
        SimTK::Vec3 estimated_linear_acceleration_head =
            arriving_otolith_specific_force_head + adapted_gravity_head;

        double head_pivot_radius = 0.15;
        double expected_tangential_acceleration_x = eff_traj_acc_pitch * head_pivot_radius;
        double expected_tangential_acceleration_z = eff_traj_acc_roll  * head_pivot_radius;

        double phasic_otolith_input_pitch = estimated_linear_acceleration_head[0] - expected_tangential_acceleration_x;
        double phasic_otolith_input_roll  = estimated_linear_acceleration_head[2] - expected_tangential_acceleration_z;
        double phasic_otolith_input_yaw   = 0.0;

        static const double vest_tilt_thresh = std::getenv("VEST_TILT_THRESH") ? std::stod(std::getenv("VEST_TILT_THRESH")) : 0.002;
        static const double vest_scc_thresh  = std::getenv("VEST_SCC_THRESH")  ? std::stod(std::getenv("VEST_SCC_THRESH"))  : 0.005;
        static const double vest_oto_thresh  = std::getenv("VEST_OTO_THRESH")  ? std::stod(std::getenv("VEST_OTO_THRESH"))  : 0.020;
        auto deadzone = [](double v, double th) {
            if (th <= 0.0) return v;
            double av = std::abs(v);
            if (av <= 0.75 * th) return 0.0;
            if (av < 1.25 * th) {
                double u = av - 0.75 * th;
                return (v >= 0.0 ? 1.0 : -1.0) * (u * u) / th;
            }
            return (v > 0.0) ? (v - th) : (v + th);
        };
        tonic_otolith_input_pitch = deadzone(tonic_otolith_input_pitch, vest_tilt_thresh);
        tonic_otolith_input_roll  = deadzone(tonic_otolith_input_roll,  vest_tilt_thresh);
        tonic_otolith_input_yaw   = deadzone(tonic_otolith_input_yaw,   vest_tilt_thresh);
        semicircular_canal_input_pitch = deadzone(semicircular_canal_input_pitch, vest_scc_thresh);
        semicircular_canal_input_roll  = deadzone(semicircular_canal_input_roll,  vest_scc_thresh);
        semicircular_canal_input_yaw   = deadzone(semicircular_canal_input_yaw,   vest_scc_thresh);
        phasic_otolith_input_pitch = deadzone(phasic_otolith_input_pitch, vest_oto_thresh);
        phasic_otolith_input_roll  = deadzone(phasic_otolith_input_roll,  vest_oto_thresh);

        const double p_fast = 250.0;
        const double p_ton  = std::max(0.5, get_p_ton());
        const double p_sc   = std::max(0.5, get_p_sc());
        const double p_phas = std::max(0.5, get_p_phas());
        const double raw_z_sc = get_z_sc();
        const double tau_vcr_plant = (raw_z_sc > 0.25) ? 0.0150 : std::clamp(raw_z_sc, 0.0, 0.25);
        const double k_ton  = (sv_adapt_theta[0] && adapt_on) ? sv_adapt_theta[0]->getValue(state) : get_K_ton();
        const double k_sc   = (sv_adapt_theta[2] && adapt_on) ? sv_adapt_theta[2]->getValue(state) : get_K_sc();
        const double k_phas = (sv_adapt_theta[5] && adapt_on) ? sv_adapt_theta[5]->getValue(state) : get_K_phas();

        for (int axis_idx = 0; axis_idx < 3; ++axis_idx) {
            const double u_ton  = (axis_idx == 0) ? tonic_otolith_input_pitch      : ((axis_idx == 1) ? tonic_otolith_input_roll      : tonic_otolith_input_yaw);
            const double u_sc   = (axis_idx == 0) ? semicircular_canal_input_pitch : ((axis_idx == 1) ? semicircular_canal_input_roll : semicircular_canal_input_yaw);
            const double u_phas = (axis_idx == 0) ? phasic_otolith_input_pitch     : ((axis_idx == 1) ? phasic_otolith_input_roll     : phasic_otolith_input_yaw);

            const double x_ton  = state_variable_tonic_otolith[axis_idx]->getValue(state);
            const double v_ton  = state_variable_tonic_otolith_vel[axis_idx]->getValue(state);
            const double a_ton  = (p_ton * p_fast) * (u_ton - x_ton) - (p_ton + p_fast) * v_ton;

            const double x_sc   = state_variable_semicircular_canal_reflex[axis_idx]->getValue(state);
            const double v_sc   = state_variable_semicircular_canal_reflex_vel[axis_idx]->getValue(state);
            const double a_sc   = (p_sc * p_fast) * (u_sc - x_sc) - (p_sc + p_fast) * v_sc;

            const double y_sc     = x_sc + tau_vcr_plant * v_sc + (0.5 * tau_vcr_plant * tau_vcr_plant) * a_sc;
            const double ydot_sc  = v_sc + tau_vcr_plant * a_sc;
            const double yddot_sc = a_sc;

            const double x_phas = state_variable_phasic_otolith[axis_idx]->getValue(state);
            const double v_phas = state_variable_phasic_otolith_vel[axis_idx]->getValue(state);
            const double a_phas = (p_phas * p_fast) * (u_phas - x_phas) - (p_phas + p_fast) * v_phas;

            // Exact output, 1st derivative, and 2nd derivative of Subsystem 5:
            const double y_vcr     = -(k_ton * x_ton + k_sc * y_sc     + k_phas * x_phas);
            const double ydot_vcr  = -(k_ton * v_ton + k_sc * ydot_sc  + k_phas * v_phas);
            const double yddot_vcr = -(k_ton * a_ton + k_sc * yddot_sc + k_phas * a_phas);

            // Path 5 (2.5 ms MVST -> Sub 7) Sender-Side Delay Compensator: 1 + tau_5*s + 0.5*tau_5^2*s^2
            current_brainstem_vcr_torque[axis_idx] =
                y_vcr + tau_5_mvst_vcr_delay * ydot_vcr + (0.5 * tau_5_mvst_vcr_delay * tau_5_mvst_vcr_delay) * yddot_vcr;
            current_brainstem_vcr_torque_dot[axis_idx]  = ydot_vcr + tau_5_mvst_vcr_delay * yddot_vcr;
            current_brainstem_vcr_torque_ddot[axis_idx] = yddot_vcr;
        }
    }
    current_history_record.brainstem_vcr_torque      = current_brainstem_vcr_torque;
    current_history_record.brainstem_vcr_torque_dot  = current_brainstem_vcr_torque_dot;
    current_history_record.brainstem_vcr_torque_ddot = current_brainstem_vcr_torque_ddot;

    // --- SUBSYSTEM 6: SPINAL CCR LITERATURE POLE (p_ccr = 100, p_fast = 250) + PLANT DELAY COMPENSATOR + PATH 6 (4.0 ms -> Sub 7) ---
    const HistoryRecord& cervico_collic_record = delayed_3a_drg_ccr_data.record;
    const double p_fast_ccr = 250.0;
    const double p_ccr = std::max(0.5, get_p_ccr());
    const double raw_z_ccr = get_z_ccr();
    const double tau_ccr_plant = (raw_z_ccr > 0.25) ? 0.0150 : std::clamp(raw_z_ccr, 0.0, 0.25);
    double proprioceptive_gain = (sv_adapt_theta[7] && adapt_on) ? sv_adapt_theta[7]->getValue(state) : get_Kp_proprioception();
    bool has_cervico_collic_afferents = (cervico_collic_record.primary_afferent_firing_rate.size() == number_of_active_muscles);
    current_history_record.spinal_ccr_drive.resize(number_of_active_muscles);
    current_history_record.spinal_ccr_drive_dot.resize(number_of_active_muscles);
    current_history_record.spinal_ccr_drive_ddot.resize(number_of_active_muscles);
    current_history_record.spinal_ccr_drive.setToZero();
    current_history_record.spinal_ccr_drive_dot.setToZero();
    current_history_record.spinal_ccr_drive_ddot.setToZero();
    if (has_cervico_collic_afferents) {
        const int ccr_balance_mode = std::getenv("CCR_BALANCE_MODE") ? std::stoi(std::getenv("CCR_BALANCE_MODE")) : 1;
        const double w_ccr_ext  = std::getenv("CCR_W_EXT")  ? std::stod(std::getenv("CCR_W_EXT"))  : 0.70;
        const double w_ccr_flex = std::getenv("CCR_W_FLEX") ? std::stod(std::getenv("CCR_W_FLEX")) : 1.15;

        for (int muscle_index = 0; muscle_index < number_of_active_muscles; ++muscle_index) {
            double x_ccr = state_variable_cervico_collic[muscle_index]->getValue(state);
            double v_ccr = state_variable_cervico_collic_vel[muscle_index]->getValue(state);
            double delayed_primary_afferent_rate = cervico_collic_record.primary_afferent_firing_rate[muscle_index];
            double a_ccr = (p_ccr * p_fast_ccr) * (delayed_primary_afferent_rate - x_ccr) - (p_ccr + p_fast_ccr) * v_ccr;
            // Gate CCR lead derivatives by active stretch afferent input so relaxing/shortening muscles do not receive rebound co-contraction:
            double stretch_gate = std::clamp(delayed_primary_afferent_rate / 2.0, 0.0, 1.0);
            double v_ccr_lead = stretch_gate * v_ccr;
            double a_ccr_lead = stretch_gate * a_ccr;

            double y_ccr_plant     = x_ccr + tau_ccr_plant * v_ccr_lead + (0.5 * tau_ccr_plant * tau_ccr_plant) * a_ccr_lead;
            double ydot_ccr_plant  = v_ccr_lead + tau_ccr_plant * a_ccr_lead;
            double yddot_ccr_plant = a_ccr_lead;

            // Path 6 (4.0 ms Propriospinal -> Sub 7) Sender-Side Delay Compensator: 1 + tau_6*s + 0.5*tau_6^2*s^2
            double filtered_primary_afferent_rate =
                y_ccr_plant + tau_6_prop_ccr_delay * ydot_ccr_plant + (0.5 * tau_6_prop_ccr_delay * tau_6_prop_ccr_delay) * yddot_ccr_plant;
            double filtered_rate_dot  = ydot_ccr_plant + tau_6_prop_ccr_delay * yddot_ccr_plant;
            double filtered_rate_ddot = yddot_ccr_plant;

            const double ccr_rect_knee = 0.02;
            double rectified_primary_afferent_rate = 0.0;
            double rect_slope = 0.0;
            if (filtered_primary_afferent_rate > 0.0) {
                if (filtered_primary_afferent_rate < 2.0 * ccr_rect_knee) {
                    rectified_primary_afferent_rate = (filtered_primary_afferent_rate * filtered_primary_afferent_rate) / (4.0 * ccr_rect_knee);
                    rect_slope = filtered_primary_afferent_rate / (2.0 * ccr_rect_knee);
                } else {
                    rectified_primary_afferent_rate = filtered_primary_afferent_rate - ccr_rect_knee;
                    rect_slope = 1.0;
                }
            }
            double w_ccr_m = 1.0;
            if (ccr_balance_mode == 1 && moment_arm_jacobian_3dof.ncol() > muscle_index) {
                double r_p = moment_arm_jacobian_3dof(0, muscle_index);
                double r_r = moment_arm_jacobian_3dof(1, muscle_index);
                double r_y = moment_arm_jacobian_3dof(2, muscle_index);
                double eta_pitch = (r_p * r_p) / (r_p * r_p + r_r * r_r + r_y * r_y + 1e-8);
                double w_target = (r_p > 1e-6) ? w_ccr_ext : ((r_p < -1e-6) ? w_ccr_flex : 1.0);
                w_ccr_m = 1.0 + eta_pitch * (w_target - 1.0);
            } else if (ccr_balance_mode == 2 && moment_arm_jacobian_3dof.ncol() > muscle_index) {
                double r_p = moment_arm_jacobian_3dof(0, muscle_index);
                w_ccr_m = (r_p > 1e-6) ? w_ccr_ext : ((r_p < -1e-6) ? w_ccr_flex : 1.0);
            }
            const double scale_ccr = w_ccr_m * proprioceptive_gain / 100.0;
            current_history_record.spinal_ccr_drive[muscle_index]      = scale_ccr * rectified_primary_afferent_rate;
            current_history_record.spinal_ccr_drive_dot[muscle_index]  = scale_ccr * rect_slope * filtered_rate_dot;
            current_history_record.spinal_ccr_drive_ddot[muscle_index] = scale_ccr * rect_slope * filtered_rate_ddot;
        }
    }

    // --- SUBSYSTEM 7: CONVERGING MOTOR COMMANDS (BRANCH 1C [14.5 ms], PATH 5 [2.5 ms], PATH 6 [4.0 ms]) ---
    DelayedData delayed_5_mvst_vcr_data  = getDelayedData(current_simulation_time, tau_5_mvst_vcr_delay);
    DelayedData delayed_6_prop_ccr_data  = getDelayedData(current_simulation_time, tau_6_prop_ccr_delay, true);

    Vector vestibulo_collic_torque(3, 0.0);
    Vector voluntary_torque(3, 0.0);
    SimTK::Vec3 arriving_vcr_dot  = current_brainstem_vcr_torque_dot;
    SimTK::Vec3 arriving_vcr_ddot = current_brainstem_vcr_torque_ddot;
    SimTK::Vec3 arriving_vol_dot  = vol_ydot;
    SimTK::Vec3 arriving_vol_ddot = vol_yddot;

    // 1. Pre-Compensated VCR Drive arriving from Subsystem 5 via Path 5 (2.5 ms MVST delay)
    if (!history.empty() && delayed_5_mvst_vcr_data.record.valid) {
        vestibulo_collic_torque[0] = delayed_5_mvst_vcr_data.record.brainstem_vcr_torque[0];
        vestibulo_collic_torque[1] = delayed_5_mvst_vcr_data.record.brainstem_vcr_torque[1];
        vestibulo_collic_torque[2] = delayed_5_mvst_vcr_data.record.brainstem_vcr_torque[2];
        arriving_vcr_dot  = delayed_5_mvst_vcr_data.record.brainstem_vcr_torque_dot;
        arriving_vcr_ddot = delayed_5_mvst_vcr_data.record.brainstem_vcr_torque_ddot;
    } else {
        vestibulo_collic_torque[0] = current_brainstem_vcr_torque[0];
        vestibulo_collic_torque[1] = current_brainstem_vcr_torque[1];
        vestibulo_collic_torque[2] = current_brainstem_vcr_torque[2];
    }

    // 2. Pre-Compensated Voluntary Drive arriving from Subsystem 1 via Branch 1C (14.5 ms Lateral CST delay)
    if (!history.empty() && delayed_1c_m1_spinal_data.record.valid) {
        voluntary_torque[0] = delayed_1c_m1_spinal_data.record.cortical_voluntary_torque[0];
        voluntary_torque[1] = delayed_1c_m1_spinal_data.record.cortical_voluntary_torque[1];
        voluntary_torque[2] = delayed_1c_m1_spinal_data.record.cortical_voluntary_torque[2];
        arriving_vol_dot  = delayed_1c_m1_spinal_data.record.cortical_voluntary_torque_dot;
        arriving_vol_ddot = delayed_1c_m1_spinal_data.record.cortical_voluntary_torque_ddot;
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

    // Solve 1st and 2nd time derivatives of QP alpha motor excitations on the active free muscle set
    SimTK::Vec3 desired_total_torque_dot  = arriving_vol_dot  + arriving_vcr_dot;
    SimTK::Vec3 desired_total_torque_ddot = arriving_vol_ddot + arriving_vcr_ddot;
    for (int axis = 0; axis < 3; ++axis) {
        const double max_tau = (axis == 0) ? 150.0 : 100.0;
        if (std::abs(desired_total_torque[axis]) >= max_tau - 1e-6) {
            desired_total_torque_dot[axis]  = 0.0;
            desired_total_torque_ddot[axis] = 0.0;
        }
    }
    Vector target_torque_dot_qp(qp_dof, 0.0);
    Vector target_torque_ddot_qp(qp_dof, 0.0);
    if (use_6dof_qp) {
        target_torque_dot_qp[0]  = desired_total_torque_dot[0];  target_torque_dot_qp[1]  = desired_total_torque_dot[0];
        target_torque_dot_qp[2]  = desired_total_torque_dot[1];  target_torque_dot_qp[3]  = desired_total_torque_dot[1];
        target_torque_dot_qp[4]  = desired_total_torque_dot[2];  target_torque_dot_qp[5]  = desired_total_torque_dot[2];
        target_torque_ddot_qp[0] = desired_total_torque_ddot[0]; target_torque_ddot_qp[1] = desired_total_torque_ddot[0];
        target_torque_ddot_qp[2] = desired_total_torque_ddot[1]; target_torque_ddot_qp[3] = desired_total_torque_ddot[1];
        target_torque_ddot_qp[4] = desired_total_torque_ddot[2]; target_torque_ddot_qp[5] = desired_total_torque_ddot[2];
    } else {
        for (int axis = 0; axis < 3; ++axis) {
            target_torque_dot_qp[axis]  = desired_total_torque_dot[axis];
            target_torque_ddot_qp[axis] = desired_total_torque_ddot[axis];
        }
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
    for (int axis_index = 0; axis_index < qp_dof; ++axis_index) {
        gram_matrix_qp(axis_index, axis_index) += 1e-3;
    }
    FactorLU lu_factorization_deriv(gram_matrix_qp);
    Vector lambda_dot_qp(qp_dof, 0.0);
    Vector lambda_ddot_qp(qp_dof, 0.0);
    lu_factorization_deriv.solve(target_torque_dot_qp,  lambda_dot_qp);
    lu_factorization_deriv.solve(target_torque_ddot_qp, lambda_ddot_qp);

    std::vector<double> alpha_motor_excitations_dot(number_of_active_muscles, 0.0);
    std::vector<double> alpha_motor_excitations_ddot(number_of_active_muscles, 0.0);
    for (int muscle_index = 0; muscle_index < number_of_active_muscles; ++muscle_index) {
        if (is_muscle_free[muscle_index]) {
            double u_dot = 0.0;
            double u_ddot = 0.0;
            for (int axis_index = 0; axis_index < qp_dof; ++axis_index) {
                const double jf = active_jacobian(axis_index, muscle_index) * maximum_isometric_forces[muscle_index];
                u_dot  += jf * lambda_dot_qp[axis_index];
                u_ddot += jf * lambda_ddot_qp[axis_index];
            }
            alpha_motor_excitations_dot[muscle_index]  = u_dot;
            alpha_motor_excitations_ddot[muscle_index] = u_ddot;
        }
    }

    // --- STAGE 3 & 4: VOLUNTARY-GATED SHERRINGTON RECIPROCAL INHIBITION, LAMINA IX MOTONEURON POOL & TWO-STAGE FUSIMOTOR CONTROLLER ---
    // Planned trajectory arriving at spinal cord from M1 via Branch 1C (14.5 ms corticospinal projection):
    const bool has_arriving_1c = (!history.empty() && delayed_1c_m1_spinal_data.record.valid);
    const double plan_pos_pitch  = has_arriving_1c ? delayed_1c_m1_spinal_data.record.trajectory_position[0]     : trajectory_position_pitch;
    const double plan_pos_roll   = has_arriving_1c ? delayed_1c_m1_spinal_data.record.trajectory_position[1]     : trajectory_position_roll;
    const double plan_pos_yaw    = has_arriving_1c ? delayed_1c_m1_spinal_data.record.trajectory_position[2]     : trajectory_position_yaw;
    const double plan_vel_pitch  = has_arriving_1c ? delayed_1c_m1_spinal_data.record.trajectory_velocity[0]     : trajectory_velocity_pitch;
    const double plan_vel_roll   = has_arriving_1c ? delayed_1c_m1_spinal_data.record.trajectory_velocity[1]     : trajectory_velocity_roll;
    const double plan_vel_yaw    = has_arriving_1c ? delayed_1c_m1_spinal_data.record.trajectory_velocity[2]     : trajectory_velocity_yaw;
    const double plan_acc_pitch  = has_arriving_1c ? delayed_1c_m1_spinal_data.record.trajectory_acceleration[0] : trajectory_acceleration_pitch;
    const double plan_acc_roll   = has_arriving_1c ? delayed_1c_m1_spinal_data.record.trajectory_acceleration[1] : trajectory_acceleration_roll;
    const double plan_acc_yaw    = has_arriving_1c ? delayed_1c_m1_spinal_data.record.trajectory_acceleration[2] : trajectory_acceleration_yaw;
    const double plan_jerk_pitch = has_arriving_1c ? delayed_1c_m1_spinal_data.record.trajectory_jerk[0]         : trajectory_jerk_pitch;
    const double plan_jerk_roll  = has_arriving_1c ? delayed_1c_m1_spinal_data.record.trajectory_jerk[1]         : trajectory_jerk_roll;
    const double plan_jerk_yaw   = has_arriving_1c ? delayed_1c_m1_spinal_data.record.trajectory_jerk[2]         : trajectory_jerk_yaw;

    double voluntary_activity_gate = std::clamp(
        (std::abs(plan_vel_pitch) + std::abs(plan_vel_roll) + std::abs(plan_vel_yaw)) / 0.10,
        0.0, 1.0);
    double reciprocal_inhibition_gain = 1.0 - voluntary_activity_gate * (1.0 - get_K_recip_inhib());

    // Planned voluntary cortical recruitment (free of high-frequency VCR/CCR reflex oscillations):
    SimTK::Mat<3, 3> gram_vol(0.0);
    for (int m = 0; m < number_of_active_muscles; ++m) {
        const double f_max = maximum_isometric_forces[m];
        for (int r = 0; r < 3; ++r) {
            const double jr = moment_arm_jacobian_3dof(r, m) * f_max;
            for (int c = 0; c < 3; ++c) {
                gram_vol(r, c) += jr * (moment_arm_jacobian_3dof(c, m) * f_max);
            }
        }
    }
    for (int r = 0; r < 3; ++r) gram_vol(r, r) += 1e-3;
    const SimTK::Mat<3, 3> gram_vol_inv = gram_vol.invert();
    const SimTK::Vec3 vol_tau_vec(voluntary_torque[0], voluntary_torque[1], voluntary_torque[2]);
    const SimTK::Vec3 lambda_vol      = gram_vol_inv * vol_tau_vec;
    const SimTK::Vec3 lambda_vol_dot  = gram_vol_inv * arriving_vol_dot;
    const SimTK::Vec3 lambda_vol_ddot = gram_vol_inv * arriving_vol_ddot;

    // Mileusnic (2006) bag2 / bag1 constants for Stage 1A & Stage 1B Feedforward Inversion:
    constexpr double K_SR_sp       = 10.4649;
    constexpr double K_PR_sp       = 0.15;
    constexpr double L_0SR_sp      = 0.04;
    constexpr double L_0PR_sp      = 0.76;
    constexpr double R_sp          = 0.46;
    constexpr double a_pow_sp      = 0.3;
    constexpr double C_L_sp        = 1.0;
    constexpr double C_S_sp        = 0.42;
    constexpr double beta_0_b2_sp  = 0.0822;
    constexpr double beta_2_b2_sp  = -0.046;
    constexpr double Gamma_2_b2_sp = 0.0636;
    constexpr double tau_s_sp      = 0.205;
    constexpr double freq_b2_sp    = 60.0;
    constexpr double freq_b1_sp    = 60.0;
    constexpr double sigma_v_sp    = 0.50; // Velocity gating threshold (L_0 / s)

    const bool has_arriving_6_ccr = (
        !history.empty() &&
        delayed_6_prop_ccr_data.record.valid &&
        delayed_6_prop_ccr_data.record.spinal_ccr_drive.size() == number_of_active_muscles);

    current_history_record.spinal_motor_excitations.resize(number_of_active_muscles);
    current_history_record.spinal_alpha_excitations.resize(number_of_active_muscles);
    current_history_record.spinal_gamma_dyn_commands.resize(number_of_active_muscles);
    current_history_record.spinal_gamma_stat_commands.resize(number_of_active_muscles);
    current_history_record.expected_normalized_fiber_lengths.resize(number_of_active_muscles);
    current_history_record.expected_normalized_fiber_velocities.resize(number_of_active_muscles);

    for (int muscle_index = 0; muscle_index < number_of_active_muscles; ++muscle_index) {
        double excitation      = alpha_motor_excitations[muscle_index];
        double excitation_dot  = alpha_motor_excitations_dot[muscle_index];
        double excitation_ddot = alpha_motor_excitations_ddot[muscle_index];
        double minimum_muscle_excitation = cached_minimum_excitations[muscle_index];

        if (has_cervico_collic_afferents) {
            double inhibition_factor = 1.0;
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

            // Pre-Compensated CCR Drive arriving from Subsystem 6 via Path 6 (4.0 ms Propriospinal delay)
            const double ccr_u = has_arriving_6_ccr
                ? delayed_6_prop_ccr_data.record.spinal_ccr_drive[muscle_index]
                : current_history_record.spinal_ccr_drive[muscle_index];
            const double ccr_udot = (has_arriving_6_ccr && delayed_6_prop_ccr_data.record.spinal_ccr_drive_dot.size() == number_of_active_muscles)
                ? delayed_6_prop_ccr_data.record.spinal_ccr_drive_dot[muscle_index]
                : current_history_record.spinal_ccr_drive_dot[muscle_index];
            const double ccr_uddot = (has_arriving_6_ccr && delayed_6_prop_ccr_data.record.spinal_ccr_drive_ddot.size() == number_of_active_muscles)
                ? delayed_6_prop_ccr_data.record.spinal_ccr_drive_ddot[muscle_index]
                : current_history_record.spinal_ccr_drive_ddot[muscle_index];

            excitation      += ccr_u     * inhibition_factor;
            excitation_dot  += ccr_udot  * inhibition_factor;
            excitation_ddot += ccr_uddot * inhibition_factor;
        }

        if (excitation <= minimum_muscle_excitation || excitation >= 1.0) {
            excitation_dot  = 0.0;
            excitation_ddot = 0.0;
        }
        excitation = std::clamp(excitation, minimum_muscle_excitation, 1.0);

        // --- BRANCH 7A DELAY COMPENSATOR (Sub 7 -> 72 Extrafusal Neck Muscles, tau_7A = 5.5 ms) ---
        // C_7A(s) = 1 + tau_7A*s + 0.5*tau_7A^2*s^2
        const double excitation_comp_7a = std::clamp(
            excitation +
            tau_7a_aalpha_delay * excitation_dot +
            0.5 * tau_7a_aalpha_delay * tau_7a_aalpha_delay * excitation_ddot,
            minimum_muscle_excitation, 1.0);
        current_history_record.spinal_motor_excitations[muscle_index] = excitation_comp_7a;
        current_history_record.spinal_alpha_excitations[muscle_index] = alpha_motor_excitations[muscle_index];

        // --- TWO-STAGE FUSIMOTOR CONTROLLER (Stage 1A: Model-Inversion gamma_s; Stage 1B: Velocity-Gated gamma_d) ---
        double delta_u_vol      = 0.0;
        double delta_u_vol_dot  = 0.0;
        double delta_u_vol_ddot = 0.0;
        for (int axis_idx = 0; axis_idx < 3; ++axis_idx) {
            const double jf = moment_arm_jacobian_3dof(axis_idx, muscle_index) * maximum_isometric_forces[muscle_index];
            delta_u_vol      += jf * lambda_vol[axis_idx];
            delta_u_vol_dot  += jf * lambda_vol_dot[axis_idx];
            delta_u_vol_ddot += jf * lambda_vol_ddot[axis_idx];
        }
        double u_plan_alpha = std::clamp(minimum_muscle_excitation + std::max(0.0, delta_u_vol), minimum_muscle_excitation, 1.0);
        double u_plan_alpha_dot  = (delta_u_vol > 0.0 && u_plan_alpha < 1.0) ? delta_u_vol_dot  : 0.0;
        double u_plan_alpha_ddot = (delta_u_vol > 0.0 && u_plan_alpha < 1.0) ? delta_u_vol_ddot : 0.0;
        last_voluntary_alpha_excitations[muscle_index] = u_plan_alpha;

        // Apply Branch 7B (Sub 7 -> Sub 3 72 Intrafusal Spindles, tau_7B = 8.0 ms) 1 + tau_7B*s + 0.5*tau_7B^2*s^2
        const double u_plan_alpha_7b = std::clamp(
            u_plan_alpha +
            tau_7b_agamma_delay * u_plan_alpha_dot +
            0.5 * tau_7b_agamma_delay * tau_7b_agamma_delay * u_plan_alpha_ddot,
            minimum_muscle_excitation, 1.0);

        const double L0_eq = baseline_normalized_fiber_lengths[muscle_index];
        const double l_opt = cached_optimal_fiber_lengths[muscle_index];
        const double l_ten = cached_tendon_slack_lengths[muscle_index];
        const double cos_p = cached_cosine_pennation_angles[muscle_index];
        const double r_p   = moment_arm_jacobian_3dof(0, muscle_index);
        const double r_r   = moment_arm_jacobian_3dof(1, muscle_index);
        const double r_y   = moment_arm_jacobian_3dof(2, muscle_index);
        const double eff_scale = std::max(1e-4, l_opt * cos_p);

        const double dL_kin   = -(r_p * plan_pos_pitch  + r_r * plan_pos_roll  + r_y * plan_pos_yaw)  / eff_scale;
        const double dL_ten   = -(0.049 * l_ten * std::max(0.0, u_plan_alpha - minimum_muscle_excitation)) / eff_scale;
        const double L_hat    = std::clamp(L0_eq + dL_kin + dL_ten, 0.2, 2.5);
        const double Lp_hat   = std::clamp(-(r_p * plan_vel_pitch  + r_r * plan_vel_roll  + r_y * plan_vel_yaw)  / eff_scale, -20.0, 20.0);
        const double Lpp_hat  = std::clamp(-(r_p * plan_acc_pitch  + r_r * plan_acc_roll  + r_y * plan_acc_yaw)  / eff_scale, -200.0, 200.0);
        const double Lppp_hat = std::clamp(-(r_p * plan_jerk_pitch + r_r * plan_jerk_roll + r_y * plan_jerk_yaw) / eff_scale, -2000.0, 2000.0);

        // --- BRANCH 7B DELAY COMPENSATOR (Sub 7 -> Sub 3 72 Intrafusal Spindles, tau_7B = 8.0 ms) ---
        // C_7B(s) = 1 + tau_7B*s + 0.5*tau_7B^2*s^2
        const double L_hat_7b  = std::clamp(L_hat  + tau_7b_agamma_delay * Lp_hat  + 0.5 * tau_7b_agamma_delay * tau_7b_agamma_delay * Lpp_hat,  0.2, 2.5);
        const double Lp_hat_7b = std::clamp(Lp_hat + tau_7b_agamma_delay * Lpp_hat + 0.5 * tau_7b_agamma_delay * tau_7b_agamma_delay * Lppp_hat, -20.0, 20.0);

        // Stage 1A: Inverse Quasi-Static Law for gamma_s(t) with Lead Compensation (tau_s = 0.205 s)
        const double gamma_s_base = 100.0 * k_gamma_stat_eff * u_plan_alpha_7b;
        const double f_s_base     = (gamma_s_base * gamma_s_base) / (gamma_s_base * gamma_s_base + 3600.0);
        const double T_base       = (K_SR_sp / (K_SR_sp + K_PR_sp)) * (K_PR_sp * (L0_eq - L_0SR_sp - L_0PR_sp) + Gamma_2_b2_sp * f_s_base);

        const double C_2      = (Lp_hat_7b >= 0.0) ? C_L_sp : C_S_sp;
        const double phi_v    = Lp_hat_7b * std::pow(std::sqrt(Lp_hat_7b * Lp_hat_7b + 1e-4), a_pow_sp - 1.0);
        const double len_term = std::max(0.0, L_hat_7b - L_0SR_sp - (T_base / K_SR_sp) - R_sp);
        const double v_bag2   = C_2 * len_term * phi_v;
        const double num_s    = T_base - K_PR_sp * (L_hat_7b - L_0SR_sp - (T_base / K_SR_sp) - L_0PR_sp) - beta_0_b2_sp * v_bag2;
        const double den_s    = std::max(0.015, Gamma_2_b2_sp + beta_2_b2_sp * v_bag2);
        const double f_s_star = std::clamp(num_s / den_s, 0.0, 0.99);
        const double df_s_dt  = std::clamp((-K_PR_sp * Lp_hat_7b) / den_s, -2.0, 2.0);
        const double u_s      = std::clamp(f_s_star + tau_s_sp * df_s_dt, 0.0, 0.99);
        const double gamma_s_cmd = std::clamp(freq_b2_sp * std::sqrt(u_s / std::max(1e-6, 1.0 - u_s)), 0.0, 100.0);

        // Stage 1B: Task- & Velocity-Gated Law for gamma_d(t)
        const double gamma_d_task = 100.0 * k_gamma_dyn_eff * (0.25 + 0.75 * u_plan_alpha_7b);
        const double f_d_task     = (gamma_d_task * gamma_d_task) / (gamma_d_task * gamma_d_task + 3600.0);
        const double v_stretch    = std::max(0.0, Lp_hat_7b);
        const double f_d_star     = f_d_task * std::exp(-(v_stretch * v_stretch) / (2.0 * sigma_v_sp * sigma_v_sp));
        const double u_d          = std::clamp(f_d_star, 0.0, 0.99);
        const double gamma_d_cmd  = std::clamp(freq_b1_sp * std::sqrt(u_d / std::max(1e-6, 1.0 - u_d)), 0.0, 100.0);

        current_history_record.spinal_gamma_dyn_commands[muscle_index]            = gamma_d_cmd;
        current_history_record.spinal_gamma_stat_commands[muscle_index]           = gamma_s_cmd;
        current_history_record.expected_normalized_fiber_lengths[muscle_index]    = L_hat_7b;
        current_history_record.expected_normalized_fiber_velocities[muscle_index] = Lp_hat_7b;
    }

    while (!history.empty() && history.back().first >= current_simulation_time) {
        history.pop_back();
    }
    history.emplace_back(current_simulation_time, std::move(current_history_record));

    double oldest_retained_time = current_simulation_time - 0.100; // keep 100ms
    while (!history.empty() && history.front().first < oldest_retained_time) {
        history.pop_front();
    }

    // --- STAGE 5 & 6: VENTRAL ROOT EFFERENT DELAYS (BRANCH 7A: tau_7A = 5.5 ms A-alpha; BRANCH 7B: tau_7B = 8.0 ms A-gamma) ---
    DelayedData delayed_7a_alpha_data = getDelayedData(current_simulation_time, tau_7a_aalpha_delay, true);
    DelayedData delayed_7b_gamma_data = getDelayedData(current_simulation_time, tau_7b_agamma_delay, true);
    const HistoryRecord& record_7a = delayed_7a_alpha_data.record;
    const HistoryRecord& record_7b = delayed_7b_gamma_data.record;

    const bool has_delayed_7a_excitations =
        (record_7a.valid &&
         record_7a.spinal_motor_excitations.size() == number_of_active_muscles);
    const bool has_delayed_7b_gamma_commands =
        (record_7b.valid &&
         record_7b.spinal_gamma_dyn_commands.size() == number_of_active_muscles &&
         record_7b.spinal_gamma_stat_commands.size() == number_of_active_muscles);

    Vector muscle_control_vector;
    for (int muscle_index = 0; muscle_index < number_of_active_muscles; ++muscle_index) {
        const Muscle& muscle = muscles.get(active_muscle_indices[muscle_index]);

        double arriving_motor_excitation = has_delayed_7a_excitations
            ? record_7a.spinal_motor_excitations[muscle_index]
            : alpha_motor_excitations[muscle_index];

        arriving_motor_excitation = std::clamp(arriving_motor_excitation, cached_minimum_excitations[muscle_index], 1.0);

        int number_of_controls = muscle.numControls();
        if (muscle_control_vector.size() != number_of_controls) {
            muscle_control_vector.resize(number_of_controls);
        }
        // Alpha motor command arriving at extrafusal NMJ after Branch 7A delay (tau_7A = 5.5 ms)
        muscle_control_vector[0] = arriving_motor_excitation;

        // Two-Stage Fusimotor commands (gamma_d, gamma_s) arriving at intrafusal fibers after Branch 7B delay (tau_7B = 8.0 ms)
        if (number_of_controls >= 3) {
            muscle_control_vector[1] = has_delayed_7b_gamma_commands ? record_7b.spinal_gamma_dyn_commands[muscle_index]  : (100.0 * k_gamma_dyn_eff * (0.25 + 0.75 * last_voluntary_alpha_excitations[muscle_index]));
            muscle_control_vector[2] = has_delayed_7b_gamma_commands ? record_7b.spinal_gamma_stat_commands[muscle_index] : (100.0 * k_gamma_stat_eff * last_voluntary_alpha_excitations[muscle_index]);
        }

        muscle.addInControls(muscle_control_vector, controls);
    }
}
