#include "HeadNeckNeuralController.h"
#include <OpenSim/Simulation/Model/BushingForce.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <OpenSim/Simulation/Model/Muscle.h>
#include <OpenSim/Simulation/Model/PhysicalFrame.h>
#include <OpenSim/Simulation/SimbodyEngine/CoordinateCouplerConstraint.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <set>

using namespace OpenSim;
using namespace SimTK;

namespace {

// C1-smooth biological sensory recruitment deadband
inline double applyBiologicalDeadband(double v, double th) {
    if (th <= 0.0) return v;
    const double av = std::abs(v);
    if (av <= 0.75 * th) return 0.0;
    if (av < 1.25 * th) {
        const double u = av - 0.75 * th;
        return (v >= 0.0 ? 1.0 : -1.0) * (u * u) / th;
    }
    return (v >= 0.0) ? (v - th) : (v + th);
}

inline SimTK::Vec3 applyDeadbandVec3(const SimTK::Vec3& v, double th) {
    if (th <= 0.0) return v;
    const double n = v.norm();
    if (n < 1e-12) return SimTK::Vec3(0.0);
    return (applyBiologicalDeadband(n, th) / n) * v;
}

inline double smoothstep01(double x) {
    const double u = std::clamp(x, 0.0, 1.0);
    return u * u * (3.0 - 2.0 * u);
}

inline double computeSmoothIntegratorExcess(double val, double lim) {
    const double knee = 0.95 * std::max(1e-6, lim);
    const double a = std::abs(val);
    if (a <= knee) return 0.0;
    const double ex = a - knee;
    const double w = std::max(1e-4, 0.05 * lim);
    const double mag = (ex * ex) / (ex + w);
    return (val >= 0.0) ? mag : -mag;
}

inline double applyLinearKneeSaturation(double val, double knee, double ceiling) {
    if (ceiling <= 1e-6) return val;
    const double k = std::clamp(knee, 0.0, ceiling * 0.999);
    const double a = std::abs(val);
    if (a <= k) return val;
    const double span = std::max(1e-6, ceiling - k);
    const double mag = k + span * std::tanh((a - k) / span);
    return (val >= 0.0) ? mag : -mag;
}

inline SimTK::Vec3 applyRadialLinearKneeSat(const SimTK::Vec3& v, double knee, double ceiling) {
    if (ceiling <= 1e-6) return v;
    const double n = v.norm();
    if (n < 1e-12) return SimTK::Vec3(0.0);
    return (applyLinearKneeSaturation(n, knee, ceiling) / n) * v;
}

inline SimTK::Vec3 applyRadialTanhSat(const SimTK::Vec3& v, double sat) {
    if (sat <= 1e-6) return v;
    const double n = v.norm();
    if (n < 1e-12) return SimTK::Vec3(0.0);
    return (sat * std::tanh(n / sat) / n) * v;
}

inline double computeHighGAttenuation(double knee_mps2, double env_mps2, double width_mps2) {
    if (knee_mps2 <= 1e-6) return 1.0;
    const double r = std::max(0.05, width_mps2);
    const double x = std::max(0.0, env_mps2) / r;
    return 1.0 / (1.0 + x * x);
}

inline double computeVoluntaryEngagement(double urgency) {
    const double u = std::clamp(urgency, 0.0, 1.0);
    if (u <= 0.0) return 0.0;
    static const double kNorm = std::tanh(1.0 / 0.15);
    return std::clamp(std::tanh(u / 0.15) / kNorm, 0.0, 1.0);
}

inline double computeSupraspinalAlertness(double urgency) {
    if (urgency >= 0.0) return 1.0;
    const double a = std::clamp(1.0 + urgency, 0.0, 1.0);
    return a * a;
}

inline double computeSpinalAlertness(double urgency) {
    if (urgency >= 0.0) return 1.0;
    return std::clamp(1.0 + urgency, 0.0, 1.0);
}

// Isotropic 3rd-order critically-damped trajectory time constant on SE(3)
inline double computeEffectiveTrajectoryTau3(double tau0, double urgency) {
    const double u = std::clamp(urgency, 0.0, 1.0);
    const double tau_eff = std::max(1e-4, tau0) / (1.0 + 1.80 * u);
    return std::max(1e-4, 0.7535 * tau_eff);
}

// C^infty Lie-group logarithmic map log: SO(3) -> so(3)
inline SimTK::Vec3 logSO3(const SimTK::Mat33& R) {
    const SimTK::Vec3 v(
        0.5 * (R(2, 1) - R(1, 2)),
        0.5 * (R(0, 2) - R(2, 0)),
        0.5 * (R(1, 0) - R(0, 1)));
    const double s = v.norm();
    const double c = std::clamp(0.5 * (R(0, 0) + R(1, 1) + R(2, 2) - 1.0), -1.0, 1.0);
    if (s < 1e-7) {
        if (c > 0.0) {
            return (1.0 + (s * s) / 6.0) * v;
        }
    }
    const double theta = std::atan2(s, c);
    return (s > 1e-12) ? ((theta / s) * v) : SimTK::Vec3(0.0);
}

// C^infty Rodrigues exponential map exp: so(3) -> SO(3)
inline SimTK::Mat33 expSO3(const SimTK::Vec3& xi) {
    const double th2 = xi.normSqr();
    const double th = std::sqrt(th2);
    double a1 = 1.0 - th2 / 6.0;
    double a2 = 0.5 - th2 / 24.0;
    if (th >= 1e-5) {
        a1 = std::sin(th) / th;
        a2 = (1.0 - std::cos(th)) / th2;
    }
    const SimTK::Mat33 K(
         0.0,   -xi[2],  xi[1],
         xi[2],  0.0,   -xi[0],
        -xi[1],  xi[0],  0.0);
    return SimTK::Mat33(1.0) + a1 * K + a2 * (K * K);
}

// Exact 3x3 Jacobi eigendecomposition to project symmetric 3x3 matrix onto PSD cone (clips negative eigenvalues to 0)
inline SimTK::Mat33 projectSymmetric3x3PSD(const SimTK::Mat33& S_in) {
    SimTK::Mat33 A = 0.5 * (S_in + ~S_in);
    SimTK::Mat33 V(1.0);
    for (int sweep = 0; sweep < 15; ++sweep) {
        int p = 0, q = 1;
        double max_off = std::abs(A(0, 1));
        if (std::abs(A(0, 2)) > max_off) { max_off = std::abs(A(0, 2)); p = 0; q = 2; }
        if (std::abs(A(1, 2)) > max_off) { max_off = std::abs(A(1, 2)); p = 1; q = 2; }
        if (max_off < 1e-13) break;
        const double diff = A(q, q) - A(p, p);
        double t = 0.0;
        if (std::abs(A(p, q)) < 1e-30 * std::abs(diff)) {
            t = A(p, q) / diff;
        } else {
            const double phi = diff / (2.0 * A(p, q));
            t = 1.0 / (std::abs(phi) + std::sqrt(phi * phi + 1.0));
            if (phi < 0.0) t = -t;
        }
        const double c = 1.0 / std::sqrt(1.0 + t * t);
        const double s = t * c;
        SimTK::Mat33 G(1.0);
        G(p, p) = c;  G(q, q) = c;
        G(p, q) = s;  G(q, p) = -s;
        A = (~G) * A * G;
        V = V * G;
    }
    SimTK::Mat33 D(0.0);
    for (int i = 0; i < 3; ++i) {
        D(i, i) = std::max(0.0, A(i, i));
    }
    return V * D * (~V);
}

inline SimTK::Vec3 mulMatVec3(const SimTK::Matrix& M, const SimTK::Vector& v) {
    SimTK::Vec3 out(0.0);
    const int nc = std::min(M.ncol(), v.size());
    for (int r = 0; r < std::min(3, M.nrow()); ++r) {
        double acc = 0.0;
        for (int c = 0; c < nc; ++c) acc += M(r, c) * v[c];
        out[r] = acc;
    }
    return out;
}

inline SimTK::Vec3 mulMatPtr3(const SimTK::Matrix& M, const double* v, int len) {
    SimTK::Vec3 out(0.0);
    const int nc = std::min(M.ncol(), len);
    for (int r = 0; r < std::min(3, M.nrow()); ++r) {
        double acc = 0.0;
        for (int c = 0; c < nc; ++c) acc += M(r, c) * v[c];
        out[r] = acc;
    }
    return out;
}

inline SimTK::Vec3 mulBlendedJsta(
    double b, const SimTK::Matrix& J_c1, const SimTK::Matrix& J_com, const double* v, int len) {
    return (1.0 - b) * mulMatPtr3(J_c1, v, len) + b * mulMatPtr3(J_com, v, len);
}

inline SimTK::Vec3 mulBlendedJstaVec(
    double b, const SimTK::Matrix& J_c1, const SimTK::Matrix& J_com, const SimTK::Vector& v) {
    return (1.0 - b) * mulMatVec3(J_c1, v) + b * mulMatVec3(J_com, v);
}

inline double stdVecNorm(const std::vector<double>& v) {
    double s = 0.0;
    for (double x : v) s += x * x;
    return std::sqrt(s);
}

// Zero-heap stack Cholesky solver for small SPD systems H * sol = rhs (d <= 32, row-major H[d*d])
inline bool solveSPDCholesky(int d, const double* H, const double* rhs, double* sol) {
    if (d <= 0 || d > 32) return false;
    double L[32 * 32];
    for (int i = 0; i < d * d; ++i) L[i] = 0.0;
    for (int i = 0; i < d; ++i) {
        for (int j = 0; j <= i; ++j) {
            double sum = H[i * d + j];
            for (int k = 0; k < j; ++k) sum -= L[i * d + k] * L[j * d + k];
            if (i == j) {
                if (sum <= 1e-18) return false;
                L[i * d + j] = std::sqrt(sum);
            } else {
                L[i * d + j] = sum / L[j * d + j];
            }
        }
    }
    double y[32];
    for (int i = 0; i < d; ++i) {
        double sum = rhs[i];
        for (int k = 0; k < i; ++k) sum -= L[i * d + k] * y[k];
        y[i] = sum / L[i * d + i];
    }
    for (int i = d - 1; i >= 0; --i) {
        double sum = y[i];
        for (int k = i + 1; k < d; ++k) sum -= L[k * d + i] * sol[k];
        sol[i] = sum / L[i * d + i];
    }
    return true;
}

inline SimTK::Vec3 computeSkullBodyAngularVelocity(const OpenSim::Body& skull, const SimTK::State& state) {
    const SimTK::Transform& X_GS = skull.getMobilizedBody().getBodyTransform(state);
    const SimTK::Vec3 omega_world = skull.getMobilizedBody().getBodyAngularVelocity(state);
    return (~X_GS.R()) * omega_world;
}

inline double computeTendonCompensatedMtuLength(
    double fiber_dl_sec, double act_now, double act_base, double tendon_slack_len, bool eff_cancel_on) {
    if (!eff_cancel_on) return fiber_dl_sec;
    return fiber_dl_sec + 0.052 * tendon_slack_len * (act_now - act_base);
}

inline double evaluateCuneateNeutralReferenceII(
    double L0_norm, double Lp_est, double static_fusimotor_activation, double static_gamma_pps) {
    constexpr double K_SR = 10.4649, K_PR = 0.15, L_0SR = 0.04, L_0PR = 0.76;
    constexpr double L_NSR = 0.0423, L_NPR = 0.89, L_sec = 0.04, X = 0.7, R = 0.46;
    constexpr double a_pow = 0.3, C_L = 1.0, C_S = 0.42;
    constexpr double beta_0_b2 = 0.0822, beta_2_b2 = -0.046, Gamma_2_b2 = 0.0636, G_sec_b2 = 7250.0;
    constexpr double beta_0_ch = 0.0822, beta_2_ch = -0.069, Gamma_2_ch = 0.0954, G_sec_ch = 7250.0;

    const double L_hat  = std::clamp(L0_norm, 0.2, 2.5);
    const double Lp_hat = std::clamp(Lp_est, -20.0, 20.0);
    const double f_s    = std::clamp(static_fusimotor_activation, 0.0, 1.0);
    const double g_s    = std::max(0.0, static_gamma_pps);
    const double f_c    = (g_s * g_s) / (g_s * g_s + 8100.0);

    const double C_asym  = (Lp_hat > 0.0) ? C_L : C_S;
    const double phi_v   = Lp_hat * std::pow(std::sqrt(Lp_hat * Lp_hat + 1e-4), a_pow - 1.0);
    const double Phi_b2  = C_asym * (beta_0_b2 + beta_2_b2 * f_s) * phi_v;
    const double Phi_ch  = C_asym * (beta_0_ch + beta_2_ch * f_c) * phi_v;

    const double spring_disp = L_hat - L_0SR - L_0PR;
    const double damp_disp   = L_hat - L_0SR - R;
    const double T2_hat = (K_SR * (K_PR * spring_disp + Phi_b2 * damp_disp + Gamma_2_b2 * f_s)) / std::max(1.0, K_SR + K_PR + Phi_b2);
    const double Tc_hat = (K_SR * (K_PR * spring_disp + Phi_ch * damp_disp + Gamma_2_ch * f_c)) / std::max(1.0, K_SR + K_PR + Phi_ch);

    const double APbag2  = X * (L_sec / L_0SR) * ((T2_hat / K_SR) - (L_NSR - L_0SR))
                         + (1.0 - X) * (L_sec / L_0PR) * (L_hat - (T2_hat / K_SR) - L_0SR - L_NPR);
    const double APchain = X * (L_sec / L_0SR) * ((Tc_hat / K_SR) - (L_NSR - L_0SR))
                         + (1.0 - X) * (L_sec / L_0PR) * (L_hat - (Tc_hat / K_SR) - L_0SR - L_NPR);
    return G_sec_b2 * APbag2 + G_sec_ch * APchain;
}

} // namespace

HeadNeckNeuralController::HeadNeckNeuralController() {
    constructProperty_desired_rotation_vec(SimTK::Vec3(0.0));
    constructProperty_desired_translation_vec(SimTK::Vec3(0.0));
    constructProperty_w_translation(0.0);
    constructProperty_urgency(0.0);
    constructProperty_trajectory_tau(0.28);
    constructProperty_fc_voluntary(2.15);

    constructProperty_omega_task(12.0);
    constructProperty_zeta_task(1.05);
    constructProperty_c_grav_task(1.40);
    constructProperty_c_musc_task(2.20);
    constructProperty_c_disc_task(1.20);
    constructProperty_ki_task(2.20);

    constructProperty_omega_shift(1.20);
    constructProperty_zeta_shift(0.06);
    constructProperty_ki_shift(3.50);
    constructProperty_c_ff_shift(1.00);
    constructProperty_k_trans_obs_pos(0.0);
    constructProperty_k_trans_obs_vel(0.0);
    constructProperty_k_shift_obs_pos(0.0);
    constructProperty_k_shift_obs_vel(0.0);
    constructProperty_shift_station_blend(0.0);
    constructProperty_tau_obs_washout(4.0);

    constructProperty_alpha_grav_ton(0.65);
    constructProperty_omega_ton(5.0);
    constructProperty_gamma_musc_ton(0.08);
    constructProperty_c_disc_ton(0.15);
    constructProperty_p_ton(12.0);
    constructProperty_ki_grav_ton(0.35);
    constructProperty_tau_ton_int(4.0);

    constructProperty_omega_sc(16.0);
    constructProperty_beta_musc_sc(0.05);
    constructProperty_c_disc_sc(0.05);
    constructProperty_c_grav_sc(0.0);
    constructProperty_z_sc(0.010);
    constructProperty_p_sc(18.0);
    constructProperty_c_omega_p_sc(0.0);

    constructProperty_alpha_inert_phas(0.35);
    constructProperty_gamma_musc_phas(0.05);
    constructProperty_c_disc_phas(0.0);
    constructProperty_z_phas(0.010);
    constructProperty_p_phas(5.0);

    constructProperty_oto_reweight_a0_mps2(10.0);
    constructProperty_oto_reweight_tau_s(0.18);
    constructProperty_phasic_oto_tanh_mps2(16.0);
    constructProperty_phasic_oto_sat_mps2(0.0);
    constructProperty_phasic_torque_sat_nm(0.0);
    constructProperty_phasic_lead_sat_nm(0.0);
    constructProperty_phasic_torque_tanh_nm(0.0);
    constructProperty_highg_oto_knee_mps2(0.0);
    constructProperty_highg_transition_mps2(0.65);
    constructProperty_highg_irr_knee_ratio(1.90);
    constructProperty_highg_phas_pole_boost(0.0);
    constructProperty_highg_phas_sust_floor(0.0);
    constructProperty_highg_p_sc_floor(60.0);
    constructProperty_highg_tau_lead_add_s(0.0);
    constructProperty_highg_sc_gain_boost(0.0);
    constructProperty_highg_brace_phas_suppress(0.0);
    constructProperty_vcr_compliant_share(0.0);

    constructProperty_kappa_ia_local(0.003);
    constructProperty_kappa_ia_synergy(0.002);
    constructProperty_c_disc_ccr(0.10);
    constructProperty_gamma_musc_ccr(0.04);
    constructProperty_c_grav_ccr(0.0);
    constructProperty_z_ccr(0.008);
    constructProperty_p_ccr(85.0);
    constructProperty_K_recip_inhib(0.0);
    constructProperty_K_gamma_dyn(0.90);
    constructProperty_K_gamma_stat(0.70);

    constructProperty_kappa_buckle(0.15);
    constructProperty_beta_buckle(0.006);
    constructProperty_tau_lead_pole(0.012);

    constructProperty_K_gto_ib(0.00045);
    constructProperty_p_gto_ib(150.0);
    constructProperty_thresh_gto_ib(0.50);
    constructProperty_K_gto_vol_suppress(0.85);
    constructProperty_K_renshaw(0.16);
    constructProperty_tau_renshaw(0.022);
    constructProperty_K_lig_protective(0.09);
    constructProperty_p_lig_reflex(40.0);

    constructProperty_thresh_spindle_ia(0.50);
    constructProperty_thresh_spindle_ii(0.25);
    constructProperty_thresh_vest_tilt(0.002);
    constructProperty_thresh_vest_scc(0.005);
    constructProperty_thresh_vest_oto(0.005);

    constructProperty_Kp_mahony(0.25);
    constructProperty_Ki_mahony(0.008);
    constructProperty_w_oto_mahony(0.50);
    constructProperty_vs_leak_rate(0.05);
    constructProperty_tau_vel_leak(15.0);
    constructProperty_tau_pos_leak(5.0);

    constructProperty_enable_ccr(true);
    constructProperty_enable_ccr_spinal(true);
    constructProperty_enable_ccr_task(true);
    constructProperty_enable_vcr(true);
    constructProperty_enable_vcr_tonic(true);
    constructProperty_enable_vcr_canal(true);
    constructProperty_enable_vcr_phasic(true);
    constructProperty_enable_task_controller(true);
    constructProperty_enable_task_integral(true);
    constructProperty_enable_decoder(true);
    constructProperty_enable_mahony(true);
    constructProperty_enable_efference_cancellation(true);
    constructProperty_enable_fusimotor(true);
    constructProperty_enable_reciprocal_inhibition(true);
    constructProperty_enable_antibuckling(true);
    constructProperty_enable_lead_compensators(true);
    constructProperty_enable_eye_afferents(false);
    constructProperty_enable_gto_reflex(true);
    constructProperty_enable_renshaw(true);
    constructProperty_enable_ligament_reflex(true);
    constructProperty_enable_sensor_noise(false);
    constructProperty_enable_motor_noise(false);
    constructProperty_noise_seed(42);
    constructProperty_noise_scale(1.0);

    // Anatomical semicircular canal plane normals in skull frame
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

    const double reg_gain = 2.83, reg_t1 = 0.0175, reg_t2 = 0.0027, reg_tc = 5.7;
    canal_filter_parameters[0].denominator_constant_a0 = 1.0 / (reg_tc * reg_t2);
    canal_filter_parameters[0].denominator_linear_a1   = 1.0 / reg_tc + 1.0 / reg_t2;
    canal_filter_parameters[0].numerator_zero_b1       = 1.0 / reg_t1;
    canal_filter_parameters[0].direct_feedthrough_d    = reg_gain;
    canal_filter_parameters[0].state_space_output_c0   = -reg_gain * canal_filter_parameters[0].denominator_constant_a0;
    canal_filter_parameters[0].state_space_output_c1   = reg_gain * (canal_filter_parameters[0].numerator_zero_b1 - canal_filter_parameters[0].denominator_linear_a1);

    const double irr_gain = 27.09, irr_t1 = 0.03, irr_t2 = 0.0006, irr_tc = 5.7;
    canal_filter_parameters[1].denominator_constant_a0 = 1.0 / (irr_tc * irr_t2);
    canal_filter_parameters[1].denominator_linear_a1   = 1.0 / irr_tc + 1.0 / irr_t2;
    canal_filter_parameters[1].numerator_zero_b1       = 1.0 / irr_t1;
    canal_filter_parameters[1].direct_feedthrough_d    = irr_gain;
    canal_filter_parameters[1].state_space_output_c0   = -irr_gain * canal_filter_parameters[1].denominator_constant_a0;
    canal_filter_parameters[1].state_space_output_c1   = irr_gain * (canal_filter_parameters[1].numerator_zero_b1 - canal_filter_parameters[1].denominator_linear_a1);
}

SimTK::Vec3 HeadNeckNeuralController::getVestibularAngularVelocity(const SimTK::State& state) const {
    const double wx = state_variable_luen_omega[0] ? state_variable_luen_omega[0]->getValue(state) : getStateVariableValue(state, "luen_omega_x");
    const double wy = state_variable_luen_omega[1] ? state_variable_luen_omega[1]->getValue(state) : getStateVariableValue(state, "luen_omega_y");
    const double wz = state_variable_luen_omega[2] ? state_variable_luen_omega[2]->getValue(state) : getStateVariableValue(state, "luen_omega_z");
    return SimTK::Vec3(wx, wy, wz);
}

SimTK::Vec3 HeadNeckNeuralController::getMahonyRotationVector(const SimTK::State& state) const {
    if (!get_enable_mahony()) return SimTK::Vec3(0.0);
    double q0 = state_variable_observer_quaternion[0] ? state_variable_observer_quaternion[0]->getValue(state) : getStateVariableValue(state, "obs_q0");
    double q1 = state_variable_observer_quaternion[1] ? state_variable_observer_quaternion[1]->getValue(state) : getStateVariableValue(state, "obs_q1");
    double q2 = state_variable_observer_quaternion[2] ? state_variable_observer_quaternion[2]->getValue(state) : getStateVariableValue(state, "obs_q2");
    double q3 = state_variable_observer_quaternion[3] ? state_variable_observer_quaternion[3]->getValue(state) : getStateVariableValue(state, "obs_q3");
    const double qn = std::sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (qn > 1e-6) { q0 /= qn; q1 /= qn; q2 /= qn; q3 /= qn; }
    SimTK::Rotation R_est_HW(SimTK::Quaternion(q0, q1, q2, q3));
    const SimTK::Mat33 R_delta = R_est_HW.asMat33() * (~baseline_R_head_to_world.asMat33());
    return logSO3(R_delta);
}

SimTK::Vec3 HeadNeckNeuralController::getMahonyAngularVelocity(const SimTK::State&) const {
    return get_enable_mahony() ? mahony_omega_cached : SimTK::Vec3(0.0);
}

SimTK::Vec3 HeadNeckNeuralController::getCuneateRotationVector(const SimTK::State& state) const {
    if (!state_variable_cuneate_rot[0]) return cuneate_posture_rotvec;
    return SimTK::Vec3(
        state_variable_cuneate_rot[0]->getValue(state),
        state_variable_cuneate_rot[1]->getValue(state),
        state_variable_cuneate_rot[2]->getValue(state));
}

SimTK::Vec3 HeadNeckNeuralController::getCuneateShiftVector(const SimTK::State& state) const {
    if (!state_variable_cuneate_shift[0]) return cuneate_posture_shift_m;
    const SimTK::Vec3 xf(
        state_variable_cuneate_shift[0]->getValue(state),
        state_variable_cuneate_shift[1]->getValue(state),
        state_variable_cuneate_shift[2]->getValue(state));
    return (xf.normSqr() > 1e-20) ? xf : cuneate_posture_shift_m;
}

SimTK::Vec3 HeadNeckNeuralController::getObserverPositionVector(const SimTK::State& state) const {
    if (!get_enable_mahony()) return SimTK::Vec3(0.0);
    return SimTK::Vec3(
        state_variable_observer_position[0] ? state_variable_observer_position[0]->getValue(state) : getStateVariableValue(state, "obs_px"),
        state_variable_observer_position[1] ? state_variable_observer_position[1]->getValue(state) : getStateVariableValue(state, "obs_py"),
        state_variable_observer_position[2] ? state_variable_observer_position[2]->getValue(state) : getStateVariableValue(state, "obs_pz"));
}

SimTK::Vec3 HeadNeckNeuralController::getObserverVelocityVector(const SimTK::State& state) const {
    if (!get_enable_mahony()) return SimTK::Vec3(0.0);
    return SimTK::Vec3(
        state_variable_observer_velocity[0] ? state_variable_observer_velocity[0]->getValue(state) : getStateVariableValue(state, "obs_vx"),
        state_variable_observer_velocity[1] ? state_variable_observer_velocity[1]->getValue(state) : getStateVariableValue(state, "obs_vy"),
        state_variable_observer_velocity[2] ? state_variable_observer_velocity[2]->getValue(state) : getStateVariableValue(state, "obs_vz"));
}

void HeadNeckNeuralController::extendAddToSystem(SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);
    for (int ci = 0; ci < 6; ++ci)
        for (int fi = 0; fi < 2; ++fi)
            for (int si = 0; si < 2; ++si)
                addStateVariable("canal_" + std::to_string(ci) + "_filter_" + std::to_string(fi) + "_state_" + std::to_string(si), SimTK::Stage::Velocity);

    static const char* kAx[3] = {"x", "y", "z"};
    for (int i = 0; i < 3; ++i) {
        const std::string ax(kAx[i]);
        addStateVariable("traj_rot_" + ax, SimTK::Stage::Velocity);
        addStateVariable("traj_vel_" + ax, SimTK::Stage::Velocity);
        addStateVariable("traj_acc_" + ax, SimTK::Stage::Velocity);
        addStateVariable("traj_pos_" + ax, SimTK::Stage::Velocity);
        addStateVariable("traj_tvel_" + ax, SimTK::Stage::Velocity);
        addStateVariable("traj_tacc_" + ax, SimTK::Stage::Velocity);
        addStateVariable("rot_err_int_" + ax, SimTK::Stage::Velocity);
        addStateVariable("shift_err_int_" + ax, SimTK::Stage::Velocity);
        addStateVariable("vol_filt_pos_" + ax, SimTK::Stage::Velocity);
        addStateVariable("vol_filt_vel_" + ax, SimTK::Stage::Velocity);
        addStateVariable("vol_shift_filt_pos_" + ax, SimTK::Stage::Velocity);
        addStateVariable("vol_shift_filt_vel_" + ax, SimTK::Stage::Velocity);
        addStateVariable("luen_omega_" + ax, SimTK::Stage::Velocity);
        addStateVariable("vs_omega_" + ax, SimTK::Stage::Velocity);
        addStateVariable("cuneate_rot_" + ax, SimTK::Stage::Velocity);
        addStateVariable("cuneate_shift_" + ax, SimTK::Stage::Velocity);
        addStateVariable("obs_pos_wash_" + ax, SimTK::Stage::Velocity);
        addStateVariable("obs_vel_wash_" + ax, SimTK::Stage::Velocity);
        addStateVariable("otolith_reg_adapt_" + ax, SimTK::Stage::Velocity);
        addStateVariable("otolith_reg_state_" + ax, SimTK::Stage::Velocity);
        addStateVariable("otolith_irr_adapt_" + ax, SimTK::Stage::Velocity);
        addStateVariable("otolith_irr_state_" + ax, SimTK::Stage::Velocity);
        addStateVariable("luen_otolith_" + ax, SimTK::Stage::Velocity);
        addStateVariable("luen_otolith_adapt_" + ax, SimTK::Stage::Velocity);
        addStateVariable("obs_v" + ax, SimTK::Stage::Velocity);
        addStateVariable("obs_p" + ax, SimTK::Stage::Velocity);
        addStateVariable("ton_state_" + ax, SimTK::Stage::Velocity);
        addStateVariable("ton_vel_" + ax, SimTK::Stage::Velocity);
        addStateVariable("ton_int_" + ax, SimTK::Stage::Velocity);
        addStateVariable("sc_state_" + ax, SimTK::Stage::Velocity);
        addStateVariable("sc_vel_" + ax, SimTK::Stage::Velocity);
        addStateVariable("phas_state_" + ax, SimTK::Stage::Velocity);
        addStateVariable("phas_vel_" + ax, SimTK::Stage::Velocity);
        addStateVariable("cb_grav_adapt_" + ax, SimTK::Stage::Velocity);
    }
    addStateVariable("highg_acc_env", SimTK::Stage::Velocity);
    addStateVariable("oto_reweight_env", SimTK::Stage::Velocity);
    for (int qi = 0; qi < 4; ++qi) {
        addStateVariable("obs_q" + std::to_string(qi), SimTK::Stage::Velocity);
    }
    for (int s = 0; s < NUM_FIXED_LEAD_SLOTS; ++s) {
        addStateVariable("lead_fixed_" + std::to_string(s), SimTK::Stage::Velocity);
    }
    for (int r = 0; r < 4; ++r) {
        addStateVariable("lig_ref_" + std::to_string(r), SimTK::Stage::Velocity);
    }

    const auto& muscles = getModel().getMuscles();
    for (int mi = 0; mi < muscles.getSize(); ++mi) {
        const std::string mname = muscles.get(mi).getName();
        if ((mname.rfind("r_", 0) == 0 || mname.rfind("l_", 0) == 0) && !get_enable_eye_afferents()) continue;
        addStateVariable("ccr_state_" + mname, SimTK::Stage::Velocity);
        addStateVariable("ccr_vel_" + mname, SimTK::Stage::Velocity);
        addStateVariable("sp_dL_" + mname, SimTK::Stage::Velocity);
        addStateVariable("sp_v_" + mname, SimTK::Stage::Velocity);
        addStateVariable("sp_fdyn_" + mname, SimTK::Stage::Velocity);
        addStateVariable("sp_fstat_" + mname, SimTK::Stage::Velocity);
        addStateVariable("lead_5a_" + mname, SimTK::Stage::Velocity);
        addStateVariable("lead_5bu_" + mname, SimTK::Stage::Velocity);
        addStateVariable("lead_5bL_" + mname, SimTK::Stage::Velocity);
        addStateVariable("lead_5bLp_" + mname, SimTK::Stage::Velocity);
        addStateVariable("lead_1b_" + mname, SimTK::Stage::Velocity);
        addStateVariable("ib_state_" + mname, SimTK::Stage::Velocity);
        addStateVariable("ren_state_" + mname, SimTK::Stage::Velocity);
    }
}

void HeadNeckNeuralController::extendConnectToModel(Model& model) {
    Super::extendConnectToModel(model);
    is_initialized = false;
    last_jacobian_update_time = -1.0;
    last_exact_jacobian_time = -1.0;
}

void HeadNeckNeuralController::extendInitStateFromProperties(SimTK::State& state) const {
    Super::extendInitStateFromProperties(state);
    is_initialized = false;
    lead_u0_initialized = false;
    last_jacobian_update_time = -1.0;
    last_exact_jacobian_time = -1.0;
    while (!history.empty()) {
        if (history_pool.size() < 64) history_pool.push_back(std::move(history.back().second));
        history.pop_back();
    }
    estimated_neck_rotvec = SimTK::Vec3(0.0);
    estimated_neck_omega = SimTK::Vec3(0.0);
    cuneate_posture_rotvec = SimTK::Vec3(0.0);
    cuneate_posture_shift_m = SimTK::Vec3(0.0);
    mahony_omega_cached = SimTK::Vec3(0.0);

    static const char* kAx[3] = {"x", "y", "z"};
    const double reg_adapt_ratio = 37.4 / 66.0;
    const double irr_adapt_ratio = 37.4 / 245.0;
    for (int i = 0; i < 3; ++i) {
        const std::string ax(kAx[i]);
        setStateVariableValue(state, "traj_rot_" + ax, 0.0);
        setStateVariableValue(state, "traj_vel_" + ax, 0.0);
        setStateVariableValue(state, "traj_acc_" + ax, 0.0);
        setStateVariableValue(state, "traj_pos_" + ax, 0.0);
        setStateVariableValue(state, "traj_tvel_" + ax, 0.0);
        setStateVariableValue(state, "traj_tacc_" + ax, 0.0);
        setStateVariableValue(state, "rot_err_int_" + ax, 0.0);
        setStateVariableValue(state, "shift_err_int_" + ax, 0.0);
        setStateVariableValue(state, "vol_filt_pos_" + ax, 0.0);
        setStateVariableValue(state, "vol_filt_vel_" + ax, 0.0);
        setStateVariableValue(state, "vol_shift_filt_pos_" + ax, 0.0);
        setStateVariableValue(state, "vol_shift_filt_vel_" + ax, 0.0);
        setStateVariableValue(state, "luen_omega_" + ax, 0.0);
        setStateVariableValue(state, "vs_omega_" + ax, 0.0);
        setStateVariableValue(state, "cuneate_rot_" + ax, 0.0);
        setStateVariableValue(state, "cuneate_shift_" + ax, 0.0);
        setStateVariableValue(state, "obs_pos_wash_" + ax, 0.0);
        setStateVariableValue(state, "obs_vel_wash_" + ax, 0.0);
        setStateVariableValue(state, "obs_v" + ax, 0.0);
        setStateVariableValue(state, "obs_p" + ax, 0.0);
        setStateVariableValue(state, "ton_state_" + ax, 0.0);
        setStateVariableValue(state, "ton_vel_" + ax, 0.0);
        setStateVariableValue(state, "ton_int_" + ax, 0.0);
        setStateVariableValue(state, "sc_state_" + ax, 0.0);
        setStateVariableValue(state, "sc_vel_" + ax, 0.0);
        setStateVariableValue(state, "phas_state_" + ax, 0.0);
        setStateVariableValue(state, "phas_vel_" + ax, 0.0);
        const double g0 = (i == 1) ? 9.81 : 0.0;
        setStateVariableValue(state, "otolith_reg_adapt_" + ax, g0);
        setStateVariableValue(state, "otolith_reg_state_" + ax, reg_adapt_ratio * g0);
        setStateVariableValue(state, "otolith_irr_adapt_" + ax, g0);
        setStateVariableValue(state, "otolith_irr_state_" + ax, irr_adapt_ratio * g0);
        setStateVariableValue(state, "luen_otolith_" + ax, reg_adapt_ratio * g0);
        setStateVariableValue(state, "luen_otolith_adapt_" + ax, reg_adapt_ratio * g0);
        setStateVariableValue(state, "cb_grav_adapt_" + ax, -g0);
    }
    setStateVariableValue(state, "highg_acc_env", 0.0);
    setStateVariableValue(state, "oto_reweight_env", 0.0);

    // Initialize observer quaternion to exact resting skull orientation if skull exists
    SimTK::Quaternion q_init(1.0, 0.0, 0.0, 0.0);
    if (getModel().getBodySet().contains("skull")) {
        try {
            SimTK::State s_pos = state;
            getModel().realizePosition(s_pos);
            q_init = getModel().getBodySet().get("skull").getTransformInGround(s_pos).R().convertRotationToQuaternion();
            // Initialize cb_grav_adapt to exact resting gravity in skull frame
            const SimTK::Vec3 g_head0 = (~getModel().getBodySet().get("skull").getTransformInGround(s_pos).R()) * SimTK::Vec3(0.0, -9.81, 0.0);
            const SimTK::Vec3 f_head0 = -g_head0;
            for (int i = 0; i < 3; ++i) {
                const std::string ax(kAx[i]);
                setStateVariableValue(state, "otolith_reg_adapt_" + ax, f_head0[i]);
                setStateVariableValue(state, "otolith_reg_state_" + ax, reg_adapt_ratio * f_head0[i]);
                setStateVariableValue(state, "otolith_irr_adapt_" + ax, f_head0[i]);
                setStateVariableValue(state, "otolith_irr_state_" + ax, irr_adapt_ratio * f_head0[i]);
                setStateVariableValue(state, "luen_otolith_" + ax, reg_adapt_ratio * f_head0[i]);
                setStateVariableValue(state, "luen_otolith_adapt_" + ax, reg_adapt_ratio * f_head0[i]);
                setStateVariableValue(state, "cb_grav_adapt_" + ax, g_head0[i]);
            }
        } catch (...) {}
    }
    setStateVariableValue(state, "obs_q0", q_init[0]);
    setStateVariableValue(state, "obs_q1", q_init[1]);
    setStateVariableValue(state, "obs_q2", q_init[2]);
    setStateVariableValue(state, "obs_q3", q_init[3]);

    for (int s = 0; s < NUM_FIXED_LEAD_SLOTS; ++s) {
        setStateVariableValue(state, "lead_fixed_" + std::to_string(s), 0.0);
    }
    for (int r = 0; r < 4; ++r) {
        setStateVariableValue(state, "lig_ref_" + std::to_string(r), 0.0);
    }

    const auto& muscles = getModel().getMuscles();
    for (int mi = 0; mi < muscles.getSize(); ++mi) {
        const std::string mname = muscles.get(mi).getName();
        if ((mname.rfind("r_", 0) == 0 || mname.rfind("l_", 0) == 0) && !get_enable_eye_afferents()) continue;
        setStateVariableValue(state, "ccr_state_" + mname, 0.0);
        setStateVariableValue(state, "ccr_vel_" + mname, 0.0);
        setStateVariableValue(state, "sp_dL_" + mname, 0.0);
        setStateVariableValue(state, "sp_v_" + mname, 0.0);
        setStateVariableValue(state, "sp_fdyn_" + mname, 0.0);
        setStateVariableValue(state, "sp_fstat_" + mname, 0.0);
        setStateVariableValue(state, "lead_5a_" + mname, 0.0);
        setStateVariableValue(state, "lead_5bu_" + mname, 0.0);
        setStateVariableValue(state, "lead_5bL_" + mname, 0.0);
        setStateVariableValue(state, "lead_5bLp_" + mname, 0.0);
        setStateVariableValue(state, "lead_1b_" + mname, 0.0);
        setStateVariableValue(state, "ib_state_" + mname, 0.0);
        setStateVariableValue(state, "ren_state_" + mname, 0.0);
    }
}

void HeadNeckNeuralController::initializeGeometryAndBaseline(const SimTK::State& state) const {
    const Model& model = getModel();
    for (int ci = 0; ci < 6; ++ci)
        for (int fi = 0; fi < 2; ++fi)
            for (int si = 0; si < 2; ++si)
                state_variable_semicircular_canal[ci][fi][si] =
                    traverseToStateVariable("canal_" + std::to_string(ci) + "_filter_" + std::to_string(fi) + "_state_" + std::to_string(si));

    static const char* kAx[3] = {"x", "y", "z"};
    for (int i = 0; i < 3; ++i) {
        const std::string ax(kAx[i]);
        state_variable_traj_rot[i]            = traverseToStateVariable("traj_rot_" + ax);
        state_variable_traj_vel[i]            = traverseToStateVariable("traj_vel_" + ax);
        state_variable_traj_acc[i]            = traverseToStateVariable("traj_acc_" + ax);
        state_variable_traj_pos[i]            = traverseToStateVariable("traj_pos_" + ax);
        state_variable_traj_tvel[i]           = traverseToStateVariable("traj_tvel_" + ax);
        state_variable_traj_tacc[i]           = traverseToStateVariable("traj_tacc_" + ax);
        state_variable_rot_err_int[i]         = traverseToStateVariable("rot_err_int_" + ax);
        state_variable_shift_err_int[i]       = traverseToStateVariable("shift_err_int_" + ax);
        state_variable_vol_filt_pos[i]        = traverseToStateVariable("vol_filt_pos_" + ax);
        state_variable_vol_filt_vel[i]        = traverseToStateVariable("vol_filt_vel_" + ax);
        state_variable_vol_shift_filt_pos[i]  = traverseToStateVariable("vol_shift_filt_pos_" + ax);
        state_variable_vol_shift_filt_vel[i]  = traverseToStateVariable("vol_shift_filt_vel_" + ax);
        state_variable_luen_omega[i]          = traverseToStateVariable("luen_omega_" + ax);
        state_variable_vs_omega[i]            = traverseToStateVariable("vs_omega_" + ax);
        state_variable_cuneate_rot[i]         = traverseToStateVariable("cuneate_rot_" + ax);
        state_variable_cuneate_shift[i]       = traverseToStateVariable("cuneate_shift_" + ax);
        state_variable_obs_pos_wash[i]        = traverseToStateVariable("obs_pos_wash_" + ax);
        state_variable_obs_vel_wash[i]        = traverseToStateVariable("obs_vel_wash_" + ax);
        state_variable_regular_otolith_adapt[i]    = traverseToStateVariable("otolith_reg_adapt_" + ax);
        state_variable_regular_otolith[i]          = traverseToStateVariable("otolith_reg_state_" + ax);
        state_variable_irregular_otolith_adapt[i]  = traverseToStateVariable("otolith_irr_adapt_" + ax);
        state_variable_irregular_otolith[i]        = traverseToStateVariable("otolith_irr_state_" + ax);
        state_variable_luenberger_otolith[i]       = traverseToStateVariable("luen_otolith_" + ax);
        state_variable_luenberger_otolith_adapt[i] = traverseToStateVariable("luen_otolith_adapt_" + ax);
        state_variable_observer_velocity[i]   = traverseToStateVariable("obs_v" + ax);
        state_variable_observer_position[i]   = traverseToStateVariable("obs_p" + ax);
        state_variable_tonic_state[i]         = traverseToStateVariable("ton_state_" + ax);
        state_variable_tonic_vel[i]           = traverseToStateVariable("ton_vel_" + ax);
        state_variable_tonic_int[i]           = traverseToStateVariable("ton_int_" + ax);
        state_variable_canal_state[i]         = traverseToStateVariable("sc_state_" + ax);
        state_variable_canal_vel[i]           = traverseToStateVariable("sc_vel_" + ax);
        state_variable_phasic_state[i]        = traverseToStateVariable("phas_state_" + ax);
        state_variable_phasic_vel[i]          = traverseToStateVariable("phas_vel_" + ax);
        state_variable_cb_grav_adapt[i]       = traverseToStateVariable("cb_grav_adapt_" + ax);
    }
    state_variable_highg_acc_env    = traverseToStateVariable("highg_acc_env");
    state_variable_oto_reweight_env = traverseToStateVariable("oto_reweight_env");
    for (int qi = 0; qi < 4; ++qi) {
        state_variable_observer_quaternion[qi] = traverseToStateVariable("obs_q" + std::to_string(qi));
    }
    for (int r = 0; r < 4; ++r) {
        state_variable_lig_reflex[r] = traverseToStateVariable("lig_ref_" + std::to_string(r));
    }

    cached_skull = &model.getBodySet().get("skull");
    cached_trunk = model.getBodySet().contains("spine") ? &model.getBodySet().get("spine") : nullptr;

    SimTK::State work_state = state;
    model.realizePosition(work_state);
    baseline_R_head_to_world  = cached_skull->getTransformInGround(work_state).R();
    baseline_R_trunk_to_world = cached_trunk ? cached_trunk->getTransformInGround(work_state).R() : SimTK::Rotation();
    baseline_R_head_to_trunk  = (~baseline_R_trunk_to_world) * baseline_R_head_to_world;

    // --- 1. Coordinate-Free Geometric Discovery of Cervical Generalized Coordinates ---
    std::set<std::string> dependent_coord_names;
    const auto& con_set = model.getConstraintSet();
    for (int ci = 0; ci < con_set.getSize(); ++ci) {
        if (const auto* cc = dynamic_cast<const CoordinateCouplerConstraint*>(&con_set.get(ci))) {
            dependent_coord_names.insert(cc->getDependentCoordinateName());
        }
    }

    // Unlock & unprescribe all non-dependent candidate coordinates on scratch work_state to probe pure kinematics
    const auto& coord_set = model.getCoordinateSet();
    std::vector<double> saved_q(coord_set.getSize(), 0.0);
    for (int ci = 0; ci < coord_set.getSize(); ++ci) {
        const Coordinate& c = coord_set.get(ci);
        saved_q[ci] = c.getValue(work_state);
        if (dependent_coord_names.count(c.getName()) == 0) {
            Coordinate& mut_c = const_cast<Coordinate&>(c);
            mut_c.setLocked(work_state, false);
            mut_c.setIsPrescribed(work_state, false);
        }
    }
    model.realizePosition(work_state);

    const SimTK::Transform X_T0 = cached_trunk ? cached_trunk->getTransformInGround(work_state) : SimTK::Transform();
    const SimTK::Transform X_S0 = cached_skull->getTransformInGround(work_state);
    const SimTK::Mat33 R_A0_W = X_S0.R().asMat33() * (~baseline_R_head_to_trunk.asMat33());

    neck_coords.clear();
    neck_q0.clear();
    neck_qmin.clear();
    neck_qmax.clear();
    std::vector<double> joint_heights;

    const double probe_dq = 1e-4;
    for (int ci = 0; ci < coord_set.getSize(); ++ci) {
        const Coordinate& c = coord_set.get(ci);
        if (dependent_coord_names.count(c.getName()) > 0) continue;
        Coordinate& mut_c = const_cast<Coordinate&>(c);
        const double q_orig = saved_q[ci];
        try {
            mut_c.setValue(work_state, q_orig + probe_dq, true);
            model.realizePosition(work_state);
            const SimTK::Transform X_T1 = cached_trunk ? cached_trunk->getTransformInGround(work_state) : SimTK::Transform();
            const SimTK::Transform X_S1 = cached_skull->getTransformInGround(work_state);
            mut_c.setValue(work_state, q_orig, true);
            model.realizePosition(work_state);

            const double trunk_disp = (X_T1.p() - X_T0.p()).norm() + logSO3((~X_T0.R().asMat33()) * X_T1.R().asMat33()).norm();
            if (trunk_disp > 1e-7) continue; // Base/trunk-to-ground coordinate

            const SimTK::Transform X_TS0 = (~X_T0) * X_S0;
            const SimTK::Transform X_TS1 = (~X_T1) * X_S1;
            const double rel_disp = (X_TS1.p() - X_TS0.p()).norm() + logSO3((~X_TS0.R().asMat33()) * X_TS1.R().asMat33()).norm();
            if (rel_disp > 1e-7) {
                neck_coords.push_back(&c);
                neck_q0.push_back(q_orig);
                neck_qmin.push_back(c.getRangeMin() + 0.012);
                neck_qmax.push_back(c.getRangeMax() - 0.012);
                const SimTK::Vec3 jp = c.getJoint().getParentFrame().getTransformInGround(work_state).p();
                joint_heights.push_back(jp[1]);
            }
        } catch (...) {
            mut_c.setValue(work_state, q_orig, false);
        }
    }
    num_neck_dofs = static_cast<int>(neck_coords.size());
    last_exact_q.assign(num_neck_dofs, 0.0);
    estimated_neck_q.resize(num_neck_dofs);  estimated_neck_q.setToZero();
    estimated_neck_qd.resize(num_neck_dofs); estimated_neck_qd.setToZero();
    cuneate_neck_q.resize(num_neck_dofs);    cuneate_neck_q.setToZero();

    // Compute upper (+1) vs lower (-1) cervical dipole sign purely from joint elevation between trunk and skull
    neck_dipole_sign.assign(num_neck_dofs, 1.0);
    if (num_neck_dofs > 0) {
        double h_mean = 0.0;
        for (double h : joint_heights) h_mean += h;
        h_mean /= static_cast<double>(num_neck_dofs);
        for (int k = 0; k < num_neck_dofs; ++k) {
            neck_dipole_sign[k] = (joint_heights[k] >= h_mean) ? 1.0 : -1.0;
        }
    }

    // --- 2. Discover Active Cervical Muscles (Nonzero Moment Arm on Discovered Neck Manifold) ---
    const auto& muscles = model.getMuscles();
    active_muscle_indices.clear();
    muscle_names.clear();
    for (int mi = 0; mi < muscles.getSize(); ++mi) {
        const Muscle& mus = muscles.get(mi);
        double arm_sum = 0.0;
        for (int k = 0; k < num_neck_dofs; ++k) {
            arm_sum += std::abs(mus.computeMomentArm(work_state, *const_cast<Coordinate*>(neck_coords[k])));
        }
        if (arm_sum > 1e-6) {
            active_muscle_indices.push_back(mi);
            muscle_names.push_back(mus.getName());
        }
    }
    const int n_mus = static_cast<int>(active_muscle_indices.size());

    // --- 3. Initialize Muscle, Spindle, GTO, Renshaw & Ligament Caches ---
    moment_arm_q0.resize(num_neck_dofs, n_mus);
    moment_arm_q0.setToZero();
    moment_arm_d1.assign(num_neck_dofs, SimTK::Matrix(num_neck_dofs, n_mus, 0.0));
    moment_arm_d2.assign(num_neck_dofs, SimTK::Matrix(num_neck_dofs, n_mus, 0.0));

    baseline_muscle_activations.resize(n_mus, 0.03);
    baseline_primary_afferent_rates.resize(n_mus, 0.0);
    baseline_secondary_afferent_rates.resize(n_mus, 0.0);
    baseline_tendon_afferent_rates.assign(n_mus, 0.0);
    last_alpha_motor_excitations.assign(n_mus, 0.03);
    last_voluntary_alpha_excitations.assign(n_mus, 0.03);

    state_variable_cervico_collic.assign(n_mus, nullptr);
    state_variable_cervico_collic_vel.assign(n_mus, nullptr);
    state_variable_spindle_length_change.assign(n_mus, nullptr);
    state_variable_spindle_velocity.assign(n_mus, nullptr);
    state_variable_spindle_dynamic_fusimotor.assign(n_mus, nullptr);
    state_variable_spindle_static_fusimotor.assign(n_mus, nullptr);
    state_variable_gto_ib.assign(n_mus, nullptr);
    state_variable_renshaw.assign(n_mus, nullptr);

    cached_primary_afferent_outputs.assign(n_mus, nullptr);
    cached_secondary_afferent_outputs.assign(n_mus, nullptr);
    cached_tendon_afferent_outputs.assign(n_mus, nullptr);
    cached_fiber_length_scales.resize(n_mus, 0.0);
    cached_primary_afferent_velocity_scales.resize(n_mus, 0.0);
    cached_maximum_isometric_forces.resize(n_mus, 0.0);
    cached_minimum_excitations.resize(n_mus, 0.03);
    cached_optimal_fiber_lengths.resize(n_mus, 0.05);
    cached_tendon_slack_lengths.resize(n_mus, 0.05);
    cached_cosine_pennation_angles.resize(n_mus, 1.0);
    baseline_normalized_fiber_lengths.resize(n_mus, 1.0);
    baseline_reference_ia_rates.resize(n_mus, 0.0);
    baseline_reference_ii_rates.resize(n_mus, 0.0);

    const int total_lead_slots = NUM_FIXED_LEAD_SLOTS + 5 * n_mus;
    state_variable_lead_comp.assign(total_lead_slots, nullptr);
    lead_u0.assign(total_lead_slots, 0.0);
    lead_u_curr.assign(total_lead_slots, 0.0);
    lead_tau_p.assign(total_lead_slots, 0.0020);
    lead_u0_initialized = false;
    for (int s = 0; s < NUM_FIXED_LEAD_SLOTS; ++s) {
        state_variable_lead_comp[s] = traverseToStateVariable("lead_fixed_" + std::to_string(s));
    }

    for (int mi = 0; mi < n_mus; ++mi) {
        const Muscle& muscle = muscles.get(active_muscle_indices[mi]);
        const std::string& mname = muscle_names[mi];

        // 72-Muscle Static Equilibrium Co-Contraction Baseline u0
        double u_min = 0.030;
        if      (mname == "Sterno_hyoid")          u_min = 0.0139717799;
        else if (mname == "Sterno_hyoid_L")        u_min = 0.0139511131;
        else if (mname == "SternoThyroid")         u_min = 0.0140165696;
        else if (mname == "SternoThyroid_L")       u_min = 0.0139732141;
        else if (mname == "Omo_hyoid")             u_min = 0.0140871490;
        else if (mname == "Omo_hyoid_L")           u_min = 0.0140976246;
        else if (mname == "stern_mast")            u_min = 0.0138502387;
        else if (mname == "stern_mast_L")          u_min = 0.0139775579;
        else if (mname == "cleid_mast")            u_min = 0.0140710738;
        else if (mname == "cleid_mast_L")          u_min = 0.0141093965;
        else if (mname == "cleid_occ")             u_min = 0.0141636123;
        else if (mname == "cleid_occ_L")           u_min = 0.0142268903;
        else if (mname == "scalenus_ant")          u_min = 0.0145635402;
        else if (mname == "scalenus_ant_L")        u_min = 0.0144509115;
        else if (mname == "digastric_post")        u_min = 0.0257784706;
        else if (mname == "digastric_post_L")      u_min = 0.0260315629;
        else if (mname == "digastric_ant")         u_min = 0.0351035340;
        else if (mname == "digastric_ant_L")       u_min = 0.0352121622;
        else if (mname == "Geniohyoid")            u_min = 0.0358026992;
        else if (mname == "Geniohyoid_L")          u_min = 0.0357473405;
        else if (mname == "Mylohyoid_Post")        u_min = 0.0353313751;
        else if (mname == "Mylohyoid_Post_L")      u_min = 0.0354097912;
        else if (mname == "Mylohyoid_Ant")         u_min = 0.0356659213;
        else if (mname == "Mylohyoid_Ant_L")       u_min = 0.0358758631;
        else if (mname == "Stylohyoid_Lat")        u_min = 0.0326644051;
        else if (mname == "Stylohyoid_Lat_L")      u_min = 0.0327149724;
        else if (mname == "Stylohyoid_Med")        u_min = 0.0327060436;
        else if (mname == "Stylohyoid_Med_L")      u_min = 0.0327566529;
        else if (mname == "scalenus_med")          u_min = 0.0326753625;
        else if (mname == "scalenus_med_L")        u_min = 0.0325128869;
        else if (mname == "scalenus_post")         u_min = 0.0317342344;
        else if (mname == "scalenus_post_L")       u_min = 0.0316441400;
        else if (mname == "long_cap_sklc4")        u_min = 0.0353821366;
        else if (mname == "long_cap_sklc4_L")      u_min = 0.0353826554;
        else if (mname == "long_col_c1thx")        u_min = 0.0295571264;
        else if (mname == "long_col_c1thx_L")      u_min = 0.0295123366;
        else if (mname == "long_col_c1c5")         u_min = 0.0334928305;
        else if (mname == "long_col_c1c5_L")       u_min = 0.0334346502;
        else if (mname == "long_col_c5thx")        u_min = 0.0298662164;
        else if (mname == "long_col_c5thx_L")      u_min = 0.0298183092;
        else if (mname == "trap_cl")               u_min = 0.0222292352;
        else if (mname == "trap_cl_L")             u_min = 0.0221871083;
        else if (mname == "trap_acr")              u_min = 0.0391398794;
        else if (mname == "trap_acr_L")            u_min = 0.0389673090;
        else if (mname == "splen_cap_sklc6")       u_min = 0.0232569485;
        else if (mname == "splen_cap_sklc6_L")     u_min = 0.0232838387;
        else if (mname == "splen_cap_sklthx")      u_min = 0.0255911988;
        else if (mname == "splen_cap_sklthx_L")    u_min = 0.0256264685;
        else if (mname == "splen_cerv_c3thx")      u_min = 0.0486577974;
        else if (mname == "splen_cerv_c3thx_L")    u_min = 0.0485928643;
        else if (mname == "semi_cap_sklc5")        u_min = 0.0221330996;
        else if (mname == "semi_cap_sklc5_L")      u_min = 0.0222535606;
        else if (mname == "semi_cap_sklthx")       u_min = 0.0230533402;
        else if (mname == "semi_cap_sklthx_L")     u_min = 0.0231172728;
        else if (mname == "levator_scap")          u_min = 0.0493851759;
        else if (mname == "levator_scap_L")        u_min = 0.0493189848;
        else if (mname == "semi_cerv_c3thx")       u_min = 0.0694157512;
        else if (mname == "semi_cerv_c3thx_L")     u_min = 0.0693322383;
        else if (mname == "longissi_cap_sklc6")    u_min = 0.0305156418;
        else if (mname == "longissi_cap_sklc6_L")  u_min = 0.0305784910;
        else if (mname == "longissi_cerv_c4thx")   u_min = 0.0687598170;
        else if (mname == "longissi_cerv_c4thx_L") u_min = 0.0686729647;
        else if (mname == "iliocost_cerv_c5rib")   u_min = 0.0665445799;
        else if (mname == "iliocost_cerv_c5rib_L") u_min = 0.0664567190;
        else if (mname == "rectcap_post_maj")      u_min = 0.0305471736;
        else if (mname == "rectcap_post_maj_L")    u_min = 0.0305686797;
        else if (mname == "rectcap_post_min")      u_min = 0.0307442921;
        else if (mname == "rectcap_post_min_L")    u_min = 0.0308294544;
        else if (mname == "obl_cap_sup")           u_min = 0.0320102550;
        else if (mname == "obl_cap_sup_L")         u_min = 0.0321224423;
        else if (mname == "obl_cap_inf")           u_min = 0.0358471301;
        else if (mname == "obl_cap_inf_L")         u_min = 0.0358175853;

        cached_minimum_excitations[mi] = u_min;
        last_alpha_motor_excitations[mi] = u_min;
        last_voluntary_alpha_excitations[mi] = u_min;

        state_variable_cervico_collic[mi]            = traverseToStateVariable("ccr_state_" + mname);
        state_variable_cervico_collic_vel[mi]        = traverseToStateVariable("ccr_vel_" + mname);
        state_variable_spindle_length_change[mi]     = traverseToStateVariable("sp_dL_" + mname);
        state_variable_spindle_velocity[mi]          = traverseToStateVariable("sp_v_" + mname);
        state_variable_spindle_dynamic_fusimotor[mi] = traverseToStateVariable("sp_fdyn_" + mname);
        state_variable_spindle_static_fusimotor[mi]  = traverseToStateVariable("sp_fstat_" + mname);
        state_variable_gto_ib[mi]                    = traverseToStateVariable("ib_state_" + mname);
        state_variable_renshaw[mi]                   = traverseToStateVariable("ren_state_" + mname);

        const int bl = NUM_FIXED_LEAD_SLOTS + 5 * mi;
        state_variable_lead_comp[bl + 0] = traverseToStateVariable("lead_5a_"   + mname);
        state_variable_lead_comp[bl + 1] = traverseToStateVariable("lead_5bu_"  + mname);
        state_variable_lead_comp[bl + 2] = traverseToStateVariable("lead_5bL_"  + mname);
        state_variable_lead_comp[bl + 3] = traverseToStateVariable("lead_5bLp_" + mname);
        state_variable_lead_comp[bl + 4] = traverseToStateVariable("lead_1b_"   + mname);

        const double l_opt = std::max(1e-4, muscle.getOptimalFiberLength());
        const double cos_p = std::max(0.5, std::cos(muscle.getPennationAngleAtOptimalFiberLength()));
        cached_optimal_fiber_lengths[mi] = l_opt;
        cached_tendon_slack_lengths[mi] = muscle.getTendonSlackLength();
        cached_cosine_pennation_angles[mi] = cos_p;
        double init_l_norm = 1.0;
        try { init_l_norm = std::clamp(muscle.getFiberLength(state) / l_opt, 0.2, 2.5); } catch (...) {}
        baseline_normalized_fiber_lengths[mi] = init_l_norm;
        cached_fiber_length_scales[mi] = l_opt / (369.1427 * cos_p);
        cached_primary_afferent_velocity_scales[mi] = l_opt / (35000.0 * cos_p);
        cached_maximum_isometric_forces[mi] = muscle.getMaxIsometricForce();
        baseline_muscle_activations[mi] = muscle.getActivation(state);

        double ria0 = 0.0, rii0 = 0.0, rk0 = 1.0;
        evaluateReferenceMileusnicSpindle(mi, init_l_norm, 0.0, 0.0, 0.0, 0.0, ria0, rii0, rk0);
        baseline_reference_ia_rates[mi] = ria0;
        baseline_reference_ii_rates[mi] = rii0;

        const Component* sp_comp = nullptr;
        const Component* gto_comp = nullptr;
        for (const auto& comp : muscle.getComponentList()) {
            if (comp.getName() == "spindle") sp_comp = &comp;
            else if (comp.getName() == "GTO") gto_comp = &comp;
        }
        if (sp_comp) {
            try {
                cached_primary_afferent_outputs[mi]   = static_cast<const Output<double>*>(&sp_comp->getOutput("primary_Ia"));
                cached_secondary_afferent_outputs[mi] = static_cast<const Output<double>*>(&sp_comp->getOutput("secondary_II"));
                baseline_primary_afferent_rates[mi]   = cached_primary_afferent_outputs[mi]->getValue(state);
                baseline_secondary_afferent_rates[mi] = cached_secondary_afferent_outputs[mi]->getValue(state);
            } catch (...) {}
        }
        try {
            cached_tendon_afferent_outputs[mi] = static_cast<const Output<double>*>(&muscle.getOutput("tendon_Ib"));
        } catch (...) {
            if (gto_comp) {
                try { cached_tendon_afferent_outputs[mi] = static_cast<const Output<double>*>(&gto_comp->getOutput("aggregate_firing_rate")); } catch (...) {}
            }
        }
        if (cached_tendon_afferent_outputs[mi] && state.getSystemStage() >= SimTK::Stage::Velocity) {
            try { baseline_tendon_afferent_rates[mi] = cached_tendon_afferent_outputs[mi]->getValue(state); } catch (...) {}
        }

        for (int k = 0; k < num_neck_dofs; ++k) {
            moment_arm_q0(k, mi) = muscle.computeMomentArm(work_state, *const_cast<Coordinate*>(neck_coords[k]));
        }
    }

    // --- 4. Measure Exact Intrinsic Covariant Tensors { J_omega, J_c1, J_com, M_d, Kg_d, Kdisc_d, Ddisc_d, Gmu_d } ---
    J_omega_d.resize(3, num_neck_dofs); J_omega_d.setToZero();
    J_c1_d.resize(3, num_neck_dofs);    J_c1_d.setToZero();
    J_com_d.resize(3, num_neck_dofs);   J_com_d.setToZero();
    M_neck_d.resize(num_neck_dofs, num_neck_dofs);     M_neck_d.setToZero();
    Kg_neck_d.resize(num_neck_dofs, num_neck_dofs);    Kg_neck_d.setToZero();
    Kdisc_neck_d.resize(num_neck_dofs, num_neck_dofs); Kdisc_neck_d.setToZero();
    Ddisc_neck_d.resize(num_neck_dofs, num_neck_dofs); Ddisc_neck_d.setToZero();
    Gmu_neck_d.resize(num_neck_dofs, num_neck_dofs);   Gmu_neck_d.setToZero();

    // Collect BushingForces and DynamicLigamentForces for pure analytical osteoligamentous pullback
    struct BushInfo {
        const PhysicalFrame* f1 = nullptr;
        const PhysicalFrame* f2 = nullptr;
        SimTK::Vec3 k_rot{0.0}, k_trans{0.0}, c_rot{0.0}, c_trans{0.0};
    };
    std::vector<BushInfo> bushings;
    cached_ligaments.clear();
    const auto& fset = model.getForceSet();
    for (int fi = 0; fi < fset.getSize(); ++fi) {
        const Force& frc = fset.get(fi);
        if (const auto* bf = dynamic_cast<const BushingForce*>(&frc)) {
            try {
                BushInfo bi;
                bi.f1 = &bf->getSocket<PhysicalFrame>("frame1").getConnectee();
                bi.f2 = &bf->getSocket<PhysicalFrame>("frame2").getConnectee();
                bi.k_rot   = bf->get_rotational_stiffness();
                bi.k_trans = bf->get_translational_stiffness();
                bi.c_rot   = bf->get_rotational_damping();
                bi.c_trans = bf->get_translational_damping();
                bushings.push_back(bi);
            } catch (...) {}
        } else if (frc.getConcreteClassName() == "DynamicLigamentForce") {
            try {
                const std::string& lname = frc.getName();
                if (lname.rfind("AF_", 0) == 0) continue;
                LigamentCacheEntry entry;
                entry.strain_out = static_cast<const Output<double>*>(&frc.getOutput("strain"));
                entry.rate_out   = static_cast<const Output<double>*>(&frc.getOutput("strain_rate"));
                entry.toe_strain = frc.hasProperty("toe_strain")
                    ? Property<double>::getAs(frc.getPropertyByName("toe_strain")).getValue() : 1.15;
                entry.strain_grad_d.resize(num_neck_dofs);
                entry.strain_grad_d.setToZero();
                cached_ligaments.push_back(entry);
            } catch (...) {}
        }
    }

    const int n_bodies = model.getBodySet().getSize();
    std::vector<std::vector<SimTK::Vec3>> body_v_cols(num_neck_dofs, std::vector<SimTK::Vec3>(n_bodies, SimTK::Vec3(0.0)));
    std::vector<std::vector<SimTK::Vec3>> body_w_cols(num_neck_dofs, std::vector<SimTK::Vec3>(n_bodies, SimTK::Vec3(0.0)));
    std::vector<std::vector<SimTK::Vec3>> bush_w_cols(num_neck_dofs, std::vector<SimTK::Vec3>(bushings.size(), SimTK::Vec3(0.0)));
    std::vector<std::vector<SimTK::Vec3>> bush_v_cols(num_neck_dofs, std::vector<SimTK::Vec3>(bushings.size(), SimTK::Vec3(0.0)));

    const SimTK::Vec3 g_world = model.getGravity();
    auto evalGravPotential = [&](const SimTK::State& s_eval) -> double {
        double V = 0.0;
        for (int bi = 0; bi < n_bodies; ++bi) {
            const Body& b = model.getBodySet().get(bi);
            const SimTK::Vec3 p_com = b.findStationLocationInGround(s_eval, b.getMassCenter());
            V -= b.getMass() * (~g_world * p_com);
        }
        return V;
    };

    const SimTK::Vec3 skull_com_loc = cached_skull->getMassCenter();
    const Frame* shift_base_frame = cached_skull;
    try {
        const auto& jset = model.getJointSet();
        for (int ji = 0; ji < jset.getSize(); ++ji) {
            const Joint& jnt = jset.get(ji);
            if (&jnt.getChildFrame().findBaseFrame() == cached_skull) {
                const Frame& pf = jnt.getParentFrame().findBaseFrame();
                if (&pf != cached_trunk && pf.getName() != "ground") {
                    shift_base_frame = &pf;
                    break;
                }
            }
        }
    } catch (...) {}
    const double h_kin = 2e-3;
    const double h_jac = 0.05;

    for (int d = 0; d < num_neck_dofs; ++d) {
        Coordinate* mut_c = const_cast<Coordinate*>(neck_coords[d]);

        // Small step h_kin for exact constrained J_omega, J_c1, J_com, M_d, Bushing pullback, and ligament strain gradients
        mut_c->setValue(work_state, neck_q0[d] + h_kin, true);
        model.realizePosition(work_state);
        const SimTK::Transform X_Sp = cached_skull->getTransformInGround(work_state);
        const SimTK::Vec3 c1_p  = shift_base_frame->getTransformInGround(work_state).p();
        const SimTK::Vec3 com_p = cached_skull->findStationLocationInGround(work_state, skull_com_loc);
        std::vector<SimTK::Vec3> b_com_p(n_bodies);
        std::vector<SimTK::Mat33> b_R_p(n_bodies);
        for (int bi = 0; bi < n_bodies; ++bi) {
            const Body& b = model.getBodySet().get(bi);
            b_com_p[bi] = b.findStationLocationInGround(work_state, b.getMassCenter());
            b_R_p[bi]   = b.getTransformInGround(work_state).R().asMat33();
        }
        std::vector<SimTK::Transform> bush_p(bushings.size());
        for (size_t bi = 0; bi < bushings.size(); ++bi) {
            bush_p[bi] = (~bushings[bi].f1->getTransformInGround(work_state)) * bushings[bi].f2->getTransformInGround(work_state);
        }
        std::vector<double> lig_eps_p(cached_ligaments.size(), 1.0);
        for (size_t li = 0; li < cached_ligaments.size(); ++li) {
            if (cached_ligaments[li].strain_out) lig_eps_p[li] = cached_ligaments[li].strain_out->getValue(work_state);
        }

        mut_c->setValue(work_state, neck_q0[d] - h_kin, true);
        model.realizePosition(work_state);
        const SimTK::Transform X_Sm = cached_skull->getTransformInGround(work_state);
        const SimTK::Vec3 c1_m  = shift_base_frame->getTransformInGround(work_state).p();
        const SimTK::Vec3 com_m = cached_skull->findStationLocationInGround(work_state, skull_com_loc);
        for (int bi = 0; bi < n_bodies; ++bi) {
            const Body& b = model.getBodySet().get(bi);
            const SimTK::Vec3 b_com_m = b.findStationLocationInGround(work_state, b.getMassCenter());
            const SimTK::Mat33 b_R_m  = b.getTransformInGround(work_state).R().asMat33();
            body_v_cols[d][bi] = (b_com_p[bi] - b_com_m) / (2.0 * h_kin);
            // Angular velocity resolved in body bi's local frame (where b.getInertia() lives)
            body_w_cols[d][bi] = logSO3((~b_R_m) * b_R_p[bi]) / (2.0 * h_kin);
        }
        for (size_t bi = 0; bi < bushings.size(); ++bi) {
            const SimTK::Transform bush_m = (~bushings[bi].f1->getTransformInGround(work_state)) * bushings[bi].f2->getTransformInGround(work_state);
            bush_w_cols[d][bi] = logSO3((~bush_m.R().asMat33()) * bush_p[bi].R().asMat33()) / (2.0 * h_kin);
            bush_v_cols[d][bi] = (bush_p[bi].p() - bush_m.p()) / (2.0 * h_kin);
        }
        for (size_t li = 0; li < cached_ligaments.size(); ++li) {
            const double eps_m = cached_ligaments[li].strain_out ? cached_ligaments[li].strain_out->getValue(work_state) : 1.0;
            cached_ligaments[li].strain_grad_d[d] = (lig_eps_p[li] - eps_m) / (2.0 * h_kin);
        }

        // Skull angular Jacobian in anatomical frame A0: R_A = R_S * (R_H0^T)^T
        const SimTK::Mat33 R_Ap = X_Sp.R().asMat33() * (~baseline_R_head_to_trunk.asMat33());
        const SimTK::Mat33 R_Am = X_Sm.R().asMat33() * (~baseline_R_head_to_trunk.asMat33());
        const SimTK::Vec3 w_col = logSO3((~R_Am) * R_Ap) / (2.0 * h_kin);
        const SimTK::Vec3 vc1_col  = (~X_T0.R().asMat33()) * ((c1_p - c1_m) / (2.0 * h_kin));
        const SimTK::Vec3 vcom_col = (~X_T0.R().asMat33()) * ((com_p - com_m) / (2.0 * h_kin));
        for (int r = 0; r < 3; ++r) {
            J_omega_d(r, d) = w_col[r];
            J_c1_d(r, d)    = vc1_col[r];
            J_com_d(r, d)   = vcom_col[r];
        }

        // Moment-arm quadratic Taylor slopes around q0
        mut_c->setValue(work_state, neck_q0[d] + h_jac, true);
        model.realizePosition(work_state);
        SimTK::Matrix Jp(num_neck_dofs, n_mus);
        for (int m = 0; m < n_mus; ++m) {
            const Muscle& mus = muscles.get(active_muscle_indices[m]);
            for (int r = 0; r < num_neck_dofs; ++r)
                Jp(r, m) = mus.computeMomentArm(work_state, *const_cast<Coordinate*>(neck_coords[r]));
        }
        mut_c->setValue(work_state, neck_q0[d] - h_jac, true);
        model.realizePosition(work_state);
        SimTK::Matrix Jm(num_neck_dofs, n_mus);
        for (int m = 0; m < n_mus; ++m) {
            const Muscle& mus = muscles.get(active_muscle_indices[m]);
            for (int r = 0; r < num_neck_dofs; ++r)
                Jm(r, m) = mus.computeMomentArm(work_state, *const_cast<Coordinate*>(neck_coords[r]));
        }
        mut_c->setValue(work_state, neck_q0[d], true);
        model.realizePosition(work_state);

        for (int m = 0; m < n_mus; ++m) {
            for (int r = 0; r < num_neck_dofs; ++r) {
                moment_arm_d1[d](r, m) = (Jp(r, m) - Jm(r, m)) / (2.0 * h_jac);
                moment_arm_d2[d](r, m) = (Jp(r, m) - 2.0 * moment_arm_q0(r, m) + Jm(r, m)) / (h_jac * h_jac);
            }
        }
    }

    // Populate contiguous L1-friendly Taylor moment-arm tables [m*d + r] and [(m*d + r)*d + k]
    flat_ma_q0.assign(static_cast<size_t>(n_mus * num_neck_dofs), 0.0);
    flat_ma_d1.assign(static_cast<size_t>(n_mus * num_neck_dofs * num_neck_dofs), 0.0);
    flat_ma_d2.assign(static_cast<size_t>(n_mus * num_neck_dofs * num_neck_dofs), 0.0);
    flat_J_curr.assign(static_cast<size_t>(n_mus * num_neck_dofs), 0.0);
    scratch_A_qp.assign(static_cast<size_t>(n_mus * num_neck_dofs), 0.0);
    scratch_J_curr.resize(num_neck_dofs, n_mus);
    for (int m = 0; m < n_mus; ++m) {
        for (int r = 0; r < num_neck_dofs; ++r) {
            const int mr = m * num_neck_dofs + r;
            flat_ma_q0[mr]  = moment_arm_q0(r, m);
            flat_J_curr[mr] = moment_arm_q0(r, m);
            scratch_J_curr(r, m) = moment_arm_q0(r, m);
            for (int k = 0; k < num_neck_dofs; ++k) {
                flat_ma_d1[mr * num_neck_dofs + k] = moment_arm_d1[k](r, m);
                flat_ma_d2[mr * num_neck_dofs + k] = moment_arm_d2[k](r, m);
            }
        }
    }

    // Assemble M_neck_d, Kdisc_neck_d, Ddisc_neck_d, Gmu_neck_d, and Kg_neck_d
    const double Vg0 = evalGravPotential(work_state);
    std::vector<double> Vg_plus(num_neck_dofs, Vg0), Vg_minus(num_neck_dofs, Vg0);
    for (int i = 0; i < num_neck_dofs; ++i) {
        Coordinate* ci = const_cast<Coordinate*>(neck_coords[i]);
        ci->setValue(work_state, neck_q0[i] + h_kin, true); model.realizePosition(work_state); Vg_plus[i]  = evalGravPotential(work_state);
        ci->setValue(work_state, neck_q0[i] - h_kin, true); model.realizePosition(work_state); Vg_minus[i] = evalGravPotential(work_state);
        ci->setValue(work_state, neck_q0[i], true);         model.realizePosition(work_state);
    }

    for (int i = 0; i < num_neck_dofs; ++i) {
        for (int j = i; j < num_neck_dofs; ++j) {
            double m_ij = 0.0;
            for (int bi = 0; bi < n_bodies; ++bi) {
                const Body& b = model.getBodySet().get(bi);
                const SimTK::Mat33 I_com = b.getInertia().toMat33();
                m_ij += b.getMass() * (~body_v_cols[i][bi] * body_v_cols[j][bi])
                      + (~body_w_cols[i][bi] * (I_com * body_w_cols[j][bi]));
            }
            M_neck_d(i, j) = m_ij; M_neck_d(j, i) = m_ij;

            double kd_ij = 0.0, dd_ij = 0.0;
            for (size_t bi = 0; bi < bushings.size(); ++bi) {
                for (int a = 0; a < 3; ++a) {
                    kd_ij += bushings[bi].k_rot[a]   * bush_w_cols[i][bi][a] * bush_w_cols[j][bi][a]
                           + bushings[bi].k_trans[a] * bush_v_cols[i][bi][a] * bush_v_cols[j][bi][a];
                    dd_ij += bushings[bi].c_rot[a]   * bush_w_cols[i][bi][a] * bush_w_cols[j][bi][a]
                           + bushings[bi].c_trans[a] * bush_v_cols[i][bi][a] * bush_v_cols[j][bi][a];
                }
            }
            if (i == j) kd_ij = std::max(kd_ij, 0.25);
            Kdisc_neck_d(i, j) = kd_ij; Kdisc_neck_d(j, i) = kd_ij;
            Ddisc_neck_d(i, j) = dd_ij; Ddisc_neck_d(j, i) = dd_ij;

            double gm_ij = 0.0;
            for (int m = 0; m < n_mus; ++m) {
                gm_ij += moment_arm_q0(i, m) * cached_maximum_isometric_forces[m] * moment_arm_q0(j, m);
            }
            Gmu_neck_d(i, j) = gm_ij; Gmu_neck_d(j, i) = gm_ij;

            double kg_ij = 0.0;
            if (i == j) {
                kg_ij = -(Vg_plus[i] - 2.0 * Vg0 + Vg_minus[i]) / (h_kin * h_kin);
            } else {
                Coordinate* ci = const_cast<Coordinate*>(neck_coords[i]);
                Coordinate* cj = const_cast<Coordinate*>(neck_coords[j]);
                ci->setValue(work_state, neck_q0[i] + h_kin, false);
                cj->setValue(work_state, neck_q0[j] + h_kin, true); model.realizePosition(work_state);
                const double Vpp = evalGravPotential(work_state);
                cj->setValue(work_state, neck_q0[j] - h_kin, true); model.realizePosition(work_state);
                const double Vpm = evalGravPotential(work_state);
                ci->setValue(work_state, neck_q0[i] - h_kin, false);
                cj->setValue(work_state, neck_q0[j] + h_kin, true); model.realizePosition(work_state);
                const double Vmp = evalGravPotential(work_state);
                cj->setValue(work_state, neck_q0[j] - h_kin, true); model.realizePosition(work_state);
                const double Vmm = evalGravPotential(work_state);
                ci->setValue(work_state, neck_q0[i], false);
                cj->setValue(work_state, neck_q0[j], true); model.realizePosition(work_state);
                kg_ij = -(Vpp - Vpm - Vmp + Vmm) / (4.0 * h_kin * h_kin);
            }
            Kg_neck_d(i, j) = kg_ij; Kg_neck_d(j, i) = kg_ij;
        }
    }

    // --- 5. Compliant Chain Mode P_comp_d, Nullspace Projectors, and 3x3 Pushforward Tensors on so(3)* ---
    if (num_neck_dofs > 0) {
        SimTK::Matrix JwJwt = J_omega_d * (~J_omega_d);
        for (int r = 0; r < 3; ++r) JwJwt(r, r) += 1e-8;
        SimTK::FactorLU lu_jw(JwJwt);
        SimTK::Matrix inv_JwJwt(3, 3, 0.0);
        for (int c = 0; c < 3; ++c) {
            SimTK::Vector e(3, 0.0), col(3, 0.0); e[c] = 1.0;
            lu_jw.solve(e, col);
            for (int r = 0; r < 3; ++r) inv_JwJwt(r, c) = col[r];
        }
        J_omega_pinv = (~J_omega_d) * inv_JwJwt;

        // Solve Kdisc * X = J_omega^T -> P_comp = X * (J_omega * X)^-1
        SimTK::FactorLU lu_kd(Kdisc_neck_d);
        SimTK::Matrix Kinv_JwT(num_neck_dofs, 3, 0.0);
        for (int c = 0; c < 3; ++c) {
            SimTK::Vector rhs(num_neck_dofs, 0.0), sol(num_neck_dofs, 0.0);
            for (int r = 0; r < num_neck_dofs; ++r) rhs[r] = J_omega_d(c, r);
            lu_kd.solve(rhs, sol);
            for (int r = 0; r < num_neck_dofs; ++r) Kinv_JwT(r, c) = sol[r];
        }
        SimTK::Matrix Jw_Kinv_JwT = J_omega_d * Kinv_JwT;
        for (int r = 0; r < 3; ++r) Jw_Kinv_JwT(r, r) += 1e-8;
        SimTK::FactorLU lu_comp(Jw_Kinv_JwT);
        SimTK::Matrix inv_comp(3, 3, 0.0);
        for (int c = 0; c < 3; ++c) {
            SimTK::Vector e(3, 0.0), col(3, 0.0); e[c] = 1.0;
            lu_comp.solve(e, col);
            for (int r = 0; r < 3; ++r) inv_comp(r, c) = col[r];
        }
        P_comp_d = Kinv_JwT * inv_comp;

        SimTK::Matrix Id(num_neck_dofs, num_neck_dofs, 0.0);
        for (int i = 0; i < num_neck_dofs; ++i) Id(i, i) = 1.0;
        Pi_null_d        = Id - P_comp_d * J_omega_d;
        Pi_null_wrench_d = Id - (~J_omega_d) * (~J_omega_pinv);

        auto toMat33 = [](const SimTK::Matrix& M) -> SimTK::Mat33 {
            SimTK::Mat33 out(0.0);
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    out(r, c) = 0.5 * (M(r, c) + M(c, r));
            return out;
        };
        tensor_I3     = toMat33((~P_comp_d) * M_neck_d     * P_comp_d);
        tensor_Kdisc3 = toMat33((~P_comp_d) * Kdisc_neck_d * P_comp_d);
        tensor_Kg3    = projectSymmetric3x3PSD(toMat33((~P_comp_d) * Kg_neck_d * P_comp_d));
        // Z_2(g_hat)-equivariant projection of muscle capacity tensor G_mu,3 onto horizontal-tilt (+) vertical-axial subspaces:
        const SimTK::Vec3 g_hat_A = ((~R_A0_W) * g_world) / std::max(1e-6, g_world.norm());
        SimTK::Mat33 P_grav(1.0);
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                P_grav(r, c) -= 2.0 * g_hat_A[r] * g_hat_A[c];
        const SimTK::Mat33 Gmu3_raw = toMat33((~P_comp_d) * Gmu_neck_d * P_comp_d);
        tensor_Gmu3 = projectSymmetric3x3PSD(0.5 * (Gmu3_raw + P_grav * Gmu3_raw * P_grav));
        try {
            // Graviceptive pendulum squared-frequency operator Omega_g^2 = I_3^-1 * K_g,3 (0 around g_hat, ~73 s^-2 on tilt)
            tensor_Omega_sq = tensor_I3.invert() * tensor_Kg3;
        } catch (...) {
            tensor_Omega_sq = SimTK::Mat33(0.0);
        }

        // Coordinate-free ligament clustering & protective muscle recruitment matrix W_lig (M x 4):
        // Clusters 0..3 correspond to the 4 horizontal tilt hemispheres (+Z/-Z sagittal, +X/-X coronal) in so(3)
        // via g_3 = P_comp^T * grad_strain, recruiting muscles whose wrench unloads the stretched ligament.
        lig_to_muscle_weights.resize(n_mus, 4);
        lig_to_muscle_weights.setToZero();
        SimTK::Vector cluster_cnt(4, 0.0);
        SimTK::Matrix cluster_grad(num_neck_dofs, 4, 0.0);
        for (auto& lig : cached_ligaments) {
            SimTK::Vec3 g3(0.0);
            for (int r = 0; r < 3; ++r)
                for (int k = 0; k < num_neck_dofs; ++k)
                    g3[r] += P_comp_d(k, r) * lig.strain_grad_d[k];
            // Cluster 0: stretched under negative-Z rotation (flexion, g3[2] < 0); Cluster 1: g3[2] > 0; Cluster 2/3: g3[0] < 0 / > 0
            if (std::abs(g3[2]) >= std::abs(g3[0])) {
                lig.cluster = (g3[2] <= 0.0) ? 0 : 1;
            } else {
                lig.cluster = (g3[0] <= 0.0) ? 2 : 3;
            }
            cluster_cnt[lig.cluster] += 1.0;
            for (int k = 0; k < num_neck_dofs; ++k) cluster_grad(k, lig.cluster) += lig.strain_grad_d[k];
        }
        for (int cl = 0; cl < 4; ++cl) {
            if (cluster_cnt[cl] < 0.5) continue;
            SimTK::Vector g_bar(num_neck_dofs, 0.0), dir_d(num_neck_dofs, 0.0);
            for (int k = 0; k < num_neck_dofs; ++k) g_bar[k] = cluster_grad(k, cl) / cluster_cnt[cl];
            lu_kd.solve(g_bar, dir_d);
            const double d_norm = std::max(1e-8, dir_d.norm());
            for (int m = 0; m < n_mus; ++m) {
                double align = 0.0;
                double m_norm_sq = 1e-12;
                for (int k = 0; k < num_neck_dofs; ++k) {
                    const double jk = moment_arm_q0(k, m);
                    align -= jk * dir_d[k]; // Muscle torque +jk opposes strain gradient +dir_d
                    m_norm_sq += jk * jk;
                }
                lig_to_muscle_weights(m, cl) = std::max(0.0, align / (std::sqrt(m_norm_sq) * d_norm));
            }
        }

        // Coordinate-free GL(d)-invariant hemispheric capacity balance weights w_hemi[m] on T_q* Q:
        // Prevents flexor/extensor anatomical bulk asymmetry from rectifying zero-mean oscillations into DC posture drift.
        ccr_hemispheric_weights.assign(static_cast<size_t>(n_mus), 1.0);
        SimTK::Matrix A_cap(num_neck_dofs, n_mus, 0.0);
        SimTK::Matrix V_cap(num_neck_dofs, n_mus, 0.0);
        for (int m = 0; m < n_mus; ++m) {
            SimTK::Vector am(num_neck_dofs, 0.0), vm(num_neck_dofs, 0.0);
            const double fm = cached_maximum_isometric_forces[m];
            for (int k = 0; k < num_neck_dofs; ++k) {
                am[k] = fm * moment_arm_q0(k, m);
                A_cap(k, m) = am[k];
            }
            lu_kd.solve(am, vm);
            for (int k = 0; k < num_neck_dofs; ++k) V_cap(k, m) = vm[k];
        }
        for (int m = 0; m < n_mus; ++m) {
            double norm_sq = 0.0;
            for (int k = 0; k < num_neck_dofs; ++k) norm_sq += A_cap(k, m) * V_cap(k, m);
            const double norm_m = std::sqrt(std::max(1e-12, norm_sq));
            double c_pos = 0.0, c_neg = 0.0;
            for (int n = 0; n < n_mus; ++n) {
                double proj = 0.0;
                for (int k = 0; k < num_neck_dofs; ++k) proj += (V_cap(k, m) / norm_m) * A_cap(k, n);
                if (proj > 0.0) c_pos += proj;
                else            c_neg -= proj;
            }
            if (c_pos + c_neg > 1e-6) {
                ccr_hemispheric_weights[m] = std::clamp((2.0 * c_neg) / (c_pos + c_neg), 0.15, 2.50);
            }
        }
    }

    updateDecoderAndSynergies(moment_arm_q0);
    while (!history.empty()) {
        if (history_pool.size() < 64) history_pool.push_back(std::move(history.back().second));
        history.pop_back();
    }
    is_initialized = true;
    last_jacobian_update_time = state.getTime();
}

void HeadNeckNeuralController::updateDecoderAndSynergies(const SimTK::Matrix& J_curr) const {
    const int d = J_curr.nrow();
    const int M = J_curr.ncol();
    if (d == 0 || M == 0) return;

    // 1. Kinematic Decoder D_d = -(J * J^T)^-1 * J  (d x M)
    SimTK::Matrix G_kin = J_curr * (~J_curr);
    for (int r = 0; r < d; ++r) G_kin(r, r) += 1e-8;
    SimTK::FactorLU lu_kin(G_kin);
    proprioceptive_decoder_d.resize(d, M);
    for (int m = 0; m < M; ++m) {
        SimTK::Vector col_in(d, 0.0), col_out(d, 0.0);
        for (int r = 0; r < d; ++r) col_in[r] = -J_curr(r, m);
        lu_kin.solve(col_in, col_out);
        for (int r = 0; r < d; ++r) proprioceptive_decoder_d(r, m) = col_out[r];
    }

    // 2. GL(d)-Invariant Orthonormal Muscle-Synergy Basis V_syn in R^(M x r) via Modified Gram-Schmidt on A_0^T = F_max^(1/2) * J^T
    synergy_proj_V.resize(M, d);
    synergy_proj_V.setToZero();
    int r_valid = 0;
    for (int k = 0; k < d; ++k) {
        SimTK::Vector v(M, 0.0);
        for (int m = 0; m < M; ++m) {
            const double sf = std::sqrt(std::max(1.0, cached_maximum_isometric_forces[m]));
            v[m] = sf * J_curr(k, m);
        }
        for (int j = 0; j < r_valid; ++j) {
            double dot = 0.0;
            for (int m = 0; m < M; ++m) dot += synergy_proj_V(m, j) * v[m];
            for (int m = 0; m < M; ++m) v[m] -= dot * synergy_proj_V(m, j);
        }
        const double vn = v.norm();
        if (vn > 1e-6) {
            for (int m = 0; m < M; ++m) synergy_proj_V(m, r_valid) = v[m] / vn;
            ++r_valid;
        }
    }
}

void HeadNeckNeuralController::evaluateMomentArmMatrix(
    const SimTK::State& state, double& out_w_exact) const {
    const int d = num_neck_dofs;
    const int M = static_cast<int>(active_muscle_indices.size());
    if (d == 0 || M == 0) { out_w_exact = 0.0; return; }
    if (static_cast<int>(flat_J_curr.size()) != M * d) {
        flat_J_curr.assign(static_cast<size_t>(M * d), 0.0);
    }

    double dq_stk[32] = {0.0};
    double dq_hsq[32] = {0.0};
    const int d_cap = std::min(d, 32);
    double max_abs_dq = 0.0;
    for (int k = 0; k < d_cap; ++k) {
        const double dqk = neck_coords[k]->getValue(state) - neck_q0[k];
        dq_stk[k] = dqk;
        dq_hsq[k] = 0.5 * dqk * dqk;
        max_abs_dq = std::max(max_abs_dq, std::abs(dqk));
    }
    const double* q0_p = flat_ma_q0.data();
    const double* d1_p = flat_ma_d1.data();
    const double* d2_p = flat_ma_d2.data();
    double* j_p = flat_J_curr.data();
    for (int m = 0; m < M; ++m) {
        for (int r = 0; r < d; ++r) {
            const int mr = m * d + r;
            double val = q0_p[mr];
            const int base = mr * d;
            for (int k = 0; k < d_cap; ++k) {
                val += d1_p[base + k] * dq_stk[k] + d2_p[base + k] * dq_hsq[k];
            }
            j_p[mr] = val;
        }
    }
    out_w_exact = smoothstep01((max_abs_dq - 0.14) / 0.14);
    if (out_w_exact > 0.0) {
        double max_step = 0.0;
        for (int k = 0; k < d; ++k) {
            max_step = std::max(max_step, std::abs(neck_coords[k]->getValue(state) - last_exact_q[k]));
        }
        if (last_exact_jacobian_time < 0.0 || exact_moment_arm_cache.nrow() != d ||
            std::abs(state.getTime() - last_exact_jacobian_time) >= 0.015 || max_step >= 0.025) {
            if (exact_moment_arm_cache.nrow() != d || exact_moment_arm_cache.ncol() != M) {
                exact_moment_arm_cache.resize(d, M);
            }
            const auto& muscles = getModel().getMuscles();
            for (int m = 0; m < M; ++m) {
                const Muscle& mus = muscles.get(active_muscle_indices[m]);
                for (int r = 0; r < d; ++r) {
                    exact_moment_arm_cache(r, m) = mus.computeMomentArm(state, *const_cast<Coordinate*>(neck_coords[r]));
                }
            }
            for (int k = 0; k < d; ++k) last_exact_q[k] = neck_coords[k]->getValue(state);
            last_exact_jacobian_time = state.getTime();
        }
        for (int m = 0; m < M; ++m) {
            for (int r = 0; r < d; ++r) {
                const int mr = m * d + r;
                j_p[mr] = (1.0 - out_w_exact) * j_p[mr] + out_w_exact * exact_moment_arm_cache(r, m);
            }
        }
    }
}

void HeadNeckNeuralController::solveNullspaceShiftIK(
    double w_trans,
    double shift_gate,
    double station_blend,
    const SimTK::Vec3& xi_ref,
    const SimTK::Vec3& omega_ref,
    const SimTK::Vec3& alpha_ref,
    const SimTK::Vec3& p_ref_m,
    const SimTK::Vec3& v_ref_m,
    const SimTK::Vec3& a_ref_m,
    std::vector<double>& out_q_null,
    std::vector<double>& out_qd_null,
    std::vector<double>& out_qdd_null,
    double& out_gamma_ik) const {
    const int d = num_neck_dofs;
    out_q_null.clear();
    out_qd_null.clear();
    out_qdd_null.clear();
    out_gamma_ik = 0.0;
    if (d == 0) return;

    const double w_eff = std::max(0.0, w_trans) * std::clamp(shift_gate, 0.0, 1.0);
    if (w_eff <= 1e-10) return;

    const double b = std::clamp(station_blend, 0.0, 1.0);
    const SimTK::Matrix J_sta = (1.0 - b) * J_c1_d + b * J_com_d; // 3 x d
    // Project onto horizontal plane orthogonal to anatomical vertical e_2 = (0, 1, 0) -> rows 0 (X) and 2 (Z)
    SimTK::Matrix J_h(2, d, 0.0);
    for (int c = 0; c < d; ++c) {
        J_h(0, c) = J_sta(0, c);
        J_h(1, c) = J_sta(2, c);
    }
    const SimTK::Matrix J_h_null = J_h * Pi_null_d; // 2 x d
    SimTK::FactorLU lu_kd(Kdisc_neck_d);
    SimTK::Matrix Kinv_JhnT(d, 2, 0.0);
    for (int col = 0; col < 2; ++col) {
        SimTK::Vector rhs(d, 0.0), sol(d, 0.0);
        for (int r = 0; r < d; ++r) rhs[r] = J_h_null(col, r);
        lu_kd.solve(rhs, sol);
        for (int r = 0; r < d; ++r) Kinv_JhnT(r, col) = sol[r];
    }
    const SimTK::Matrix G2 = J_h_null * Kinv_JhnT; // 2 x 2 compliance in horizontal plane
    const double trG = 0.5 * (G2(0, 0) + G2(1, 1));
    out_gamma_ik = (w_eff * trG) / (w_eff * trG + 2e-5);
    if (out_gamma_ik <= 1e-10) { out_gamma_ik = 0.0; return; }

    out_q_null.assign(static_cast<size_t>(d), 0.0);
    out_qd_null.assign(static_cast<size_t>(d), 0.0);
    out_qdd_null.assign(static_cast<size_t>(d), 0.0);

    SimTK::Mat22 H2(G2(0, 0) + 2e-5 / w_eff, G2(0, 1), G2(1, 0), G2(1, 1) + 2e-5 / w_eff);
    const SimTK::Mat22 H2_inv = H2.invert();
    const SimTK::Matrix R_null = Pi_null_d * Kinv_JhnT; // d x 2

    auto map2 = [&](const SimTK::Vec3& p_in, const SimTK::Vec3& xi_in) -> SimTK::Vec2 {
        double q_att[32] = {0.0};
        for (int k = 0; k < std::min(d, 32); ++k)
            for (int a = 0; a < 3; ++a) q_att[k] += P_comp_d(k, a) * xi_in[a];
        double px_att = 0.0, pz_att = 0.0;
        for (int k = 0; k < std::min(d, 32); ++k) {
            px_att += J_h(0, k) * q_att[k];
            pz_att += J_h(1, k) * q_att[k];
        }
        return H2_inv * SimTK::Vec2(p_in[0] - px_att, p_in[2] - pz_att);
    };

    const SimTK::Vec2 lam_p = map2(p_ref_m, xi_ref);
    const SimTK::Vec2 lam_v = map2(v_ref_m, omega_ref);
    const SimTK::Vec2 lam_a = map2(a_ref_m, alpha_ref);
    for (int k = 0; k < d; ++k) {
        out_q_null[k]   = R_null(k, 0) * lam_p[0] + R_null(k, 1) * lam_p[1];
        out_qd_null[k]  = R_null(k, 0) * lam_v[0] + R_null(k, 1) * lam_v[1];
        out_qdd_null[k] = R_null(k, 0) * lam_a[0] + R_null(k, 1) * lam_a[1];
    }

    // Smoothly scale q_null* if total joint target P_comp * xi_ref + q_null* approaches joint ROM limits
    double scale_rom = 1.0;
    for (int k = 0; k < d; ++k) {
        double q_att_k = 0.0;
        for (int a = 0; a < 3; ++a) q_att_k += P_comp_d(k, a) * xi_ref[a];
        const double q_tot = q_att_k + out_q_null[k];
        if (q_tot > neck_qmax[k] && std::abs(out_q_null[k]) > 1e-6) {
            scale_rom = std::min(scale_rom, std::max(0.0, (neck_qmax[k] - q_att_k) / out_q_null[k]));
        } else if (q_tot < neck_qmin[k] && std::abs(out_q_null[k]) > 1e-6) {
            scale_rom = std::min(scale_rom, std::max(0.0, (neck_qmin[k] - q_att_k) / out_q_null[k]));
        }
    }
    for (int k = 0; k < d; ++k) out_q_null[k] *= scale_rom;
}

double HeadNeckNeuralController::evalProperLead(
    int slot, double u_val, double tau_d, const SimTK::State& state, double pole_override) const {
    const double raw_pole   = (pole_override > 0.0) ? pole_override : get_tau_lead_pole();
    const double pole_floor = std::clamp(raw_pole, 0.0020, 0.2000);
    const double tau_p      = std::max(pole_floor, 0.50 * tau_d);
    const double d_lead     = (tau_d + tau_p) / tau_p;
    if (slot >= 0 && slot < static_cast<int>(lead_u_curr.size())) {
        if (!lead_u0_initialized || state.getTime() <= 1e-9) {
            lead_u0[slot] = u_val;
            lead_u_curr[slot] = u_val;
            lead_tau_p[slot] = tau_p;
            return u_val;
        }
        lead_u_curr[slot] = u_val;
        lead_tau_p[slot]  = tau_p;
        if (!get_enable_lead_compensators()) return u_val;
        const double x_tilde = state_variable_lead_comp[slot] ? state_variable_lead_comp[slot]->getValue(state) : 0.0;
        const double du = u_val - lead_u0[slot];
        return lead_u0[slot] + d_lead * du + (1.0 - d_lead) * x_tilde;
    }
    return u_val;
}

void HeadNeckNeuralController::evaluateReferenceMileusnicSpindle(
    int /*muscle_index*/,
    double expected_norm_length,
    double expected_norm_velocity,
    double dynamic_fusimotor_activation,
    double static_fusimotor_activation,
    double static_gamma_pps,
    double& out_ref_ia,
    double& out_ref_ii,
    double& out_kappa_gain) const {
    constexpr double K_SR = 10.4649, K_PR = 0.15, L_0SR = 0.04, L_0PR = 0.76;
    constexpr double L_NSR = 0.0423, L_NPR = 0.89, L_sec = 0.04, X = 0.7, R = 0.46;
    constexpr double a_pow = 0.3, C_L = 1.0, C_S = 0.42, S_occ = 0.156;
    constexpr double beta_0_b1 = 0.0605, beta_1_b1 = 0.2592, Gamma_1_b1 = 0.0289, G_pri_b1 = 20000.0;
    constexpr double beta_0_b2 = 0.0822, beta_2_b2 = -0.046,  Gamma_2_b2 = 0.0636, G_pri_b2 = 10000.0, G_sec_b2 = 7250.0;
    constexpr double beta_0_ch = 0.0822, beta_2_ch = -0.069,  Gamma_2_ch = 0.0954, G_pri_ch = 10000.0, G_sec_ch = 7250.0;

    const double L_hat  = std::clamp(expected_norm_length, 0.2, 2.5);
    const double Lp_hat = std::clamp(expected_norm_velocity, -20.0, 20.0);
    const double f_d    = std::clamp(dynamic_fusimotor_activation, 0.0, 1.0);
    const double f_s    = std::clamp(static_fusimotor_activation, 0.0, 1.0);
    const double g_s    = std::max(0.0, static_gamma_pps);
    const double f_c    = (g_s * g_s) / (g_s * g_s + 8100.0);

    const double C_asym = (Lp_hat > 0.0) ? C_L : C_S;
    const double phi_v  = Lp_hat * std::pow(std::sqrt(Lp_hat * Lp_hat + 1e-4), a_pow - 1.0);
    const double Phi_b1 = C_asym * (beta_0_b1 + beta_1_b1 * f_d) * phi_v;
    const double Phi_b2 = C_asym * (beta_0_b2 + beta_2_b2 * f_s) * phi_v;
    const double Phi_ch = C_asym * (beta_0_ch + beta_2_ch * f_c) * phi_v;

    const double spring_disp = L_hat - L_0SR - L_0PR;
    const double damp_disp   = L_hat - L_0SR - R;
    const double T1_hat = (K_SR * (K_PR * spring_disp + Phi_b1 * damp_disp + Gamma_1_b1 * f_d)) / std::max(1.0, K_SR + K_PR + Phi_b1);
    const double T2_hat = (K_SR * (K_PR * spring_disp + Phi_b2 * damp_disp + Gamma_2_b2 * f_s)) / std::max(1.0, K_SR + K_PR + Phi_b2);
    const double Tc_hat = (K_SR * (K_PR * spring_disp + Phi_ch * damp_disp + Gamma_2_ch * f_c)) / std::max(1.0, K_SR + K_PR + Phi_ch);

    const double APbag1  = G_pri_b1 * ((T1_hat / K_SR) - (L_NSR - L_0SR));
    const double APbag2  = X * (L_sec / L_0SR) * ((T2_hat / K_SR) - (L_NSR - L_0SR))
                         + (1.0 - X) * (L_sec / L_0PR) * (L_hat - (T2_hat / K_SR) - L_0SR - L_NPR);
    const double APchain = X * (L_sec / L_0SR) * ((Tc_hat / K_SR) - (L_NSR - L_0SR))
                         + (1.0 - X) * (L_sec / L_0PR) * (L_hat - (Tc_hat / K_SR) - L_0SR - L_NPR);
    const double pri_stat = G_pri_b2 * APbag2 + G_pri_ch * APchain;
    const double diff     = APbag1 - pri_stat;
    const double s_max    = 0.5 * (APbag1 + pri_stat + std::sqrt(diff * diff + 1e-4));
    const double s_min    = 0.5 * (APbag1 + pri_stat - std::sqrt(diff * diff + 1e-4));

    out_ref_ia = s_max + S_occ * s_min;
    out_ref_ii = G_sec_b2 * APbag2 + G_sec_ch * APchain;
    out_kappa_gain = 1.0 + (beta_1_b1 * f_d) / std::max(0.015, beta_0_b1 + beta_2_b2 * f_s);
}

bool HeadNeckNeuralController::sampleDelayedVector(
    double current_time,
    double delay_duration,
    const std::vector<double> HistoryRecord::* field,
    double* out_buf,
    int expected_size) const {
    if (history.empty() || expected_size <= 0) return false;
    const size_t n = static_cast<size_t>(expected_size);
    const double target_time = current_time - delay_duration;
    auto cmp = [](const std::pair<double, HistoryRecord>& entry, double ts) { return entry.first < ts; };
    auto it_up = std::lower_bound(history.begin(), history.end(), target_time, cmp);

    if (it_up == history.end()) {
        const HistoryRecord& src = history.back().second;
        const std::vector<double>& v = src.*field;
        if (!src.valid || v.size() != n) return false;
        for (size_t i = 0; i < n; ++i) out_buf[i] = v[i];
        return true;
    }
    if (it_up == history.begin()) {
        const HistoryRecord& src = it_up->second;
        const std::vector<double>& v = src.*field;
        if (!src.valid || v.size() != n) return false;
        for (size_t i = 0; i < n; ++i) out_buf[i] = v[i];
        return true;
    }
    auto it_lo = it_up; --it_lo;
    const HistoryRecord& A = it_lo->second;
    const HistoryRecord& B = it_up->second;
    const std::vector<double>& va = A.*field;
    const std::vector<double>& vb = B.*field;
    if (!(A.valid || B.valid) || va.size() != n || vb.size() != n) return false;
    const double t0 = it_lo->first, t1 = it_up->first;
    const double w = (t1 > t0) ? (target_time - t0) / (t1 - t0) : 0.0;
    const double omw = 1.0 - w;
    for (size_t i = 0; i < n; ++i) out_buf[i] = omw * va[i] + w * vb[i];
    return true;
}

HeadNeckNeuralController::DelayedData HeadNeckNeuralController::getDelayedData(
    double current_time, double delay_duration, bool interpolate_vectors, bool need_lin_acc) const {
    DelayedData out;
    if (history.empty()) return out;
    const double target_time = current_time - delay_duration;
    auto cmp = [](const std::pair<double, HistoryRecord>& entry, double ts) { return entry.first < ts; };
    auto it_up = std::lower_bound(history.begin(), history.end(), target_time, cmp);

    auto copyFixed = [](const HistoryRecord& src, HistoryRecord& dst, bool copy_vecs) {
        dst.valid                            = src.valid;
        dst.est_head_rotvec_4b               = src.est_head_rotvec_4b;
        dst.est_head_rotvec_4a               = src.est_head_rotvec_4a;
        dst.canal_irr_omega_2a               = src.canal_irr_omega_2a;
        dst.canal_luen_omega_2b              = src.canal_luen_omega_2b;
        dst.oto_irr_force_2a                 = src.oto_irr_force_2a;
        dst.oto_luen_force_2b                = src.oto_luen_force_2b;
        dst.estimated_rotation_head_to_world = src.estimated_rotation_head_to_world;
        dst.trunk_rotation_world             = src.trunk_rotation_world;
        dst.trunk_omega_world                = src.trunk_omega_world;
        dst.head_linear_velocity             = src.head_linear_velocity;
        dst.est_neck_rotvec_1a3              = src.est_neck_rotvec_1a3;
        dst.est_neck_omega_1a3               = src.est_neck_omega_1a3;
        dst.mahony_omega_4b                  = src.mahony_omega_4b;
        dst.est_lin_vel                      = src.est_lin_vel;
        dst.est_lin_pos                      = src.est_lin_pos;
        dst.cuneate_shift_3b                 = src.cuneate_shift_3b;
        dst.cuneate_shift_vel_3b             = src.cuneate_shift_vel_3b;
        dst.cortical_vol_torque              = src.cortical_vol_torque;
        dst.traj_rot_1a                      = src.traj_rot_1a;
        dst.traj_vel_1b                      = src.traj_vel_1b;
        dst.traj_acc_1b                      = src.traj_acc_1b;
        dst.traj_trans_acc_1b                = src.traj_trans_acc_1b;
        dst.eff_tonic_tilt_3a                = src.eff_tonic_tilt_3a;
        dst.eff_canal_vel_3a                 = src.eff_canal_vel_3a;
        dst.eff_phasic_eta_3a                = src.eff_phasic_eta_3a;
        dst.traj_rot_1c                      = src.traj_rot_1c;
        dst.traj_vel_1c                      = src.traj_vel_1c;
        dst.brainstem_vcr_torque             = src.brainstem_vcr_torque;
        dst.raw_lig_pac                      = src.raw_lig_pac;
        dst.raw_lig_golgi                    = src.raw_lig_golgi;
        dst.voluntary_activity_gate          = src.voluntary_activity_gate;
        if (!src.cortical_vol_null_wrench.empty()) dst.cortical_vol_null_wrench = src.cortical_vol_null_wrench;
        if (!src.traj_q_null_1c.empty())           dst.traj_q_null_1c           = src.traj_q_null_1c;
        if (!src.traj_qd_null_1c.empty())          dst.traj_qd_null_1c          = src.traj_qd_null_1c;
        if (copy_vecs) {
            dst.raw_spindle_ia_rate                  = src.raw_spindle_ia_rate;
            dst.raw_spindle_ii_rate                  = src.raw_spindle_ii_rate;
            dst.spinal_ccr_drive                     = src.spinal_ccr_drive;
            dst.spinal_motor_excitations             = src.spinal_motor_excitations;
            dst.spinal_gamma_dyn_commands            = src.spinal_gamma_dyn_commands;
            dst.spinal_gamma_stat_commands           = src.spinal_gamma_stat_commands;
            dst.expected_normalized_fiber_lengths    = src.expected_normalized_fiber_lengths;
            dst.expected_normalized_fiber_velocities = src.expected_normalized_fiber_velocities;
            dst.raw_gto_ib_rate                      = src.raw_gto_ib_rate;
            dst.planned_voluntary_alpha              = src.planned_voluntary_alpha;
        }
    };

    if (it_up == history.end()) {
        copyFixed(history.back().second, out.record, interpolate_vectors);
    } else if (it_up == history.begin()) {
        copyFixed(it_up->second, out.record, interpolate_vectors);
    } else {
        auto it_lo = it_up; --it_lo;
        const double t0 = it_lo->first, t1 = it_up->first;
        const double w = (t1 > t0) ? (target_time - t0) / (t1 - t0) : 0.0;
        const double omw = 1.0 - w;
        const HistoryRecord& A = it_lo->second;
        const HistoryRecord& B = it_up->second;
        HistoryRecord& R = out.record;
        R.valid                            = A.valid || B.valid;
        R.est_head_rotvec_4b               = omw * A.est_head_rotvec_4b               + w * B.est_head_rotvec_4b;
        R.est_head_rotvec_4a               = omw * A.est_head_rotvec_4a               + w * B.est_head_rotvec_4a;
        R.canal_irr_omega_2a               = omw * A.canal_irr_omega_2a               + w * B.canal_irr_omega_2a;
        R.canal_luen_omega_2b              = omw * A.canal_luen_omega_2b              + w * B.canal_luen_omega_2b;
        R.oto_irr_force_2a                 = omw * A.oto_irr_force_2a                 + w * B.oto_irr_force_2a;
        R.oto_luen_force_2b                = omw * A.oto_luen_force_2b                + w * B.oto_luen_force_2b;
        R.estimated_rotation_head_to_world = omw * A.estimated_rotation_head_to_world + w * B.estimated_rotation_head_to_world;
        R.trunk_rotation_world             = omw * A.trunk_rotation_world             + w * B.trunk_rotation_world;
        R.trunk_omega_world                = omw * A.trunk_omega_world                + w * B.trunk_omega_world;
        R.head_linear_velocity             = omw * A.head_linear_velocity             + w * B.head_linear_velocity;
        R.est_neck_rotvec_1a3              = omw * A.est_neck_rotvec_1a3              + w * B.est_neck_rotvec_1a3;
        R.est_neck_omega_1a3               = omw * A.est_neck_omega_1a3               + w * B.est_neck_omega_1a3;
        R.mahony_omega_4b                  = omw * A.mahony_omega_4b                  + w * B.mahony_omega_4b;
        R.est_lin_vel                      = omw * A.est_lin_vel                      + w * B.est_lin_vel;
        R.est_lin_pos                      = omw * A.est_lin_pos                      + w * B.est_lin_pos;
        R.cuneate_shift_3b                 = omw * A.cuneate_shift_3b                 + w * B.cuneate_shift_3b;
        R.cuneate_shift_vel_3b             = omw * A.cuneate_shift_vel_3b             + w * B.cuneate_shift_vel_3b;
        R.cortical_vol_torque              = omw * A.cortical_vol_torque              + w * B.cortical_vol_torque;
        R.traj_rot_1a                      = omw * A.traj_rot_1a                      + w * B.traj_rot_1a;
        R.traj_vel_1b                      = omw * A.traj_vel_1b                      + w * B.traj_vel_1b;
        R.traj_acc_1b                      = omw * A.traj_acc_1b                      + w * B.traj_acc_1b;
        R.traj_trans_acc_1b                = omw * A.traj_trans_acc_1b                + w * B.traj_trans_acc_1b;
        R.eff_tonic_tilt_3a                = omw * A.eff_tonic_tilt_3a                + w * B.eff_tonic_tilt_3a;
        R.eff_canal_vel_3a                 = omw * A.eff_canal_vel_3a                 + w * B.eff_canal_vel_3a;
        R.eff_phasic_eta_3a                = omw * A.eff_phasic_eta_3a                + w * B.eff_phasic_eta_3a;
        R.traj_rot_1c                      = omw * A.traj_rot_1c                      + w * B.traj_rot_1c;
        R.traj_vel_1c                      = omw * A.traj_vel_1c                      + w * B.traj_vel_1c;
        R.brainstem_vcr_torque             = omw * A.brainstem_vcr_torque             + w * B.brainstem_vcr_torque;
        R.raw_lig_pac                      = omw * A.raw_lig_pac                      + w * B.raw_lig_pac;
        R.raw_lig_golgi                    = omw * A.raw_lig_golgi                    + w * B.raw_lig_golgi;
        R.voluntary_activity_gate          = omw * A.voluntary_activity_gate          + w * B.voluntary_activity_gate;

        auto lerpV = [&](const std::vector<double>& va, const std::vector<double>& vb, std::vector<double>& vo) {
            if (!va.empty() && va.size() == vb.size()) {
                vo.resize(va.size());
                for (size_t i = 0; i < vo.size(); ++i) vo[i] = omw * va[i] + w * vb[i];
            }
        };
        lerpV(A.cortical_vol_null_wrench, B.cortical_vol_null_wrench, R.cortical_vol_null_wrench);
        lerpV(A.traj_q_null_1c,           B.traj_q_null_1c,           R.traj_q_null_1c);
        lerpV(A.traj_qd_null_1c,          B.traj_qd_null_1c,          R.traj_qd_null_1c);
        if (interpolate_vectors) {
            lerpV(A.raw_spindle_ia_rate,                  B.raw_spindle_ia_rate,                  R.raw_spindle_ia_rate);
            lerpV(A.raw_spindle_ii_rate,                  B.raw_spindle_ii_rate,                  R.raw_spindle_ii_rate);
            lerpV(A.spinal_ccr_drive,                     B.spinal_ccr_drive,                     R.spinal_ccr_drive);
            lerpV(A.spinal_motor_excitations,             B.spinal_motor_excitations,             R.spinal_motor_excitations);
            lerpV(A.spinal_gamma_dyn_commands,            B.spinal_gamma_dyn_commands,            R.spinal_gamma_dyn_commands);
            lerpV(A.spinal_gamma_stat_commands,           B.spinal_gamma_stat_commands,           R.spinal_gamma_stat_commands);
            lerpV(A.expected_normalized_fiber_lengths,    B.expected_normalized_fiber_lengths,    R.expected_normalized_fiber_lengths);
            lerpV(A.expected_normalized_fiber_velocities, B.expected_normalized_fiber_velocities, R.expected_normalized_fiber_velocities);
            lerpV(A.raw_gto_ib_rate,                      B.raw_gto_ib_rate,                      R.raw_gto_ib_rate);
            lerpV(A.planned_voluntary_alpha,              B.planned_voluntary_alpha,              R.planned_voluntary_alpha);
        }
    }

    if (need_lin_acc) {
        const double fd_win = 0.001;
        const double prev_t = target_time - fd_win;
        auto it_prev = std::lower_bound(history.begin(), history.end(), prev_t, cmp);
        if (it_prev != history.end() && it_prev != history.begin()) {
            auto it_p0 = it_prev; --it_p0;
            const double wp = (it_prev->first > it_p0->first) ? (prev_t - it_p0->first) / (it_prev->first - it_p0->first) : 0.0;
            const SimTK::Vec3 v_prev = (1.0 - wp) * it_p0->second.head_linear_velocity + wp * it_prev->second.head_linear_velocity;
            out.linear_acceleration = (out.record.head_linear_velocity - v_prev) / fd_win;
        }
    }
    return out;
}

void HeadNeckNeuralController::computeStateVariableDerivatives(const SimTK::State& state) const {
    if (!is_initialized) {
        initializeGeometryAndBaseline(state);
    }
    const double urgency_now = get_urgency();
    const double tau3 = computeEffectiveTrajectoryTau3(get_trajectory_tau(), urgency_now);
    const double w3 = 1.0 / tau3;
    const double w3_2 = w3 * w3;
    const double w3_3 = w3_2 * w3;

    // 1. Isotropic 3rd-order jerk-limited trajectory planner on SE(3) = so(3) x R^3
    const SimTK::Vec3 xi_des = get_desired_rotation_vec();
    const SimTK::Vec3 p_des  = get_desired_translation_vec();
    SimTK::Vec3 traj_xi(0.0), traj_om(0.0), traj_al(0.0);
    SimTK::Vec3 traj_p(0.0),  traj_v(0.0),  traj_a(0.0);
    for (int i = 0; i < 3; ++i) {
        traj_xi[i] = state_variable_traj_rot[i]->getValue(state);
        traj_om[i] = state_variable_traj_vel[i]->getValue(state);
        traj_al[i] = state_variable_traj_acc[i]->getValue(state);
        state_variable_traj_rot[i]->setDerivative(state, traj_om[i]);
        state_variable_traj_vel[i]->setDerivative(state, traj_al[i]);
        state_variable_traj_acc[i]->setDerivative(state, w3_3 * (xi_des[i] - traj_xi[i]) - 3.0 * w3_2 * traj_om[i] - 3.0 * w3 * traj_al[i]);

        traj_p[i] = state_variable_traj_pos[i]->getValue(state);
        traj_v[i] = state_variable_traj_tvel[i]->getValue(state);
        traj_a[i] = state_variable_traj_tacc[i]->getValue(state);
        state_variable_traj_pos[i]->setDerivative(state, traj_v[i]);
        state_variable_traj_tvel[i]->setDerivative(state, traj_a[i]);
        state_variable_traj_tacc[i]->setDerivative(state, w3_3 * (p_des[i] - traj_p[i]) - 3.0 * w3_2 * traj_v[i] - 3.0 * w3 * traj_a[i]);
    }

    const double noise_time = state.getTime();
    const double global_noise_scale = std::max(0.0, get_noise_scale());
    const bool use_snoise = get_enable_sensor_noise() && (global_noise_scale > 1e-9);
    const double s_scale  = use_snoise ? global_noise_scale : 0.0;
    const int n_mus = static_cast<int>(active_muscle_indices.size());
    if (use_snoise) {
        noise_bank.ensureCapacity(noise_time, get_noise_seed(), std::max(1, n_mus), get_enable_gto_reflex());
    }

    // 2. Semicircular Canals (6 anatomical planes) & Luenberger Inverse Observer + Velocity Storage in Skull Frame
    const SimTK::Vec3 omega_skull_H = computeSkullBodyAngularVelocity(*cached_skull, state);
    const SimTK::Vec3 r_proj = right_canal_projection_matrix * omega_skull_H;
    const SimTK::Vec3 l_proj = left_canal_projection_matrix  * omega_skull_H;
    const double canal_in[6] = { r_proj[0], r_proj[1], r_proj[2], l_proj[0], l_proj[1], l_proj[2] };
    double reg_out[6] = {0.0}, irr_out[6] = {0.0};

    constexpr double reg_canal_t1 = 0.0175, reg_canal_t2 = 0.0027;
    constexpr double reg_passband_gain = 2.83 * (reg_canal_t2 / reg_canal_t1);  // 0.4366285714
    constexpr double irr_passband_gain = 27.09 * (0.0006 / 0.0300);             // 0.5418000000
    constexpr double luen_c_ft = reg_canal_t2 / reg_canal_t1;
    constexpr double luen_c_sw = 1.0 - luen_c_ft;

    for (int ci = 0; ci < 6; ++ci) {
        for (int fi = 0; fi < 2; ++fi) {
            const auto& cp = canal_filter_parameters[fi];
            const double x0 = state_variable_semicircular_canal[ci][fi][0]->getValue(state);
            const double x1 = state_variable_semicircular_canal[ci][fi][1]->getValue(state);
            state_variable_semicircular_canal[ci][fi][0]->setDerivative(state, x1);
            state_variable_semicircular_canal[ci][fi][1]->setDerivative(state, canal_in[ci] - cp.denominator_linear_a1 * x1 - cp.denominator_constant_a0 * x0);
            double y = cp.state_space_output_c0 * x0 + cp.state_space_output_c1 * x1 + cp.direct_feedthrough_d * canal_in[ci];
            if (fi == 0) {
                if (use_snoise) applyVestibularCanalNoise(y, reg_passband_gain, ci, 0.010, 0.025, noise_time, s_scale);
                reg_out[ci] = y;
            } else {
                if (use_snoise) applyVestibularCanalNoise(y, irr_passband_gain, 6 + ci, 0.020, 0.050, noise_time, s_scale);
                irr_out[ci] = y;
            }
        }
    }
    const SimTK::Vec3 reg_omega_raw = (0.5 / reg_passband_gain) * (
        right_canal_inverse_matrix * SimTK::Vec3(reg_out[0], reg_out[1], reg_out[2]) +
        left_canal_inverse_matrix  * SimTK::Vec3(reg_out[3], reg_out[4], reg_out[5]));
    const SimTK::Vec3 irr_omega_raw = (0.5 / irr_passband_gain) * (
        right_canal_inverse_matrix * SimTK::Vec3(irr_out[0], irr_out[1], irr_out[2]) +
        left_canal_inverse_matrix  * SimTK::Vec3(irr_out[3], irr_out[4], irr_out[5]));

    SimTK::Vec3 luen_omega_H(0.0), vs_omega_H(0.0), fused_canal_H(0.0);
    for (int i = 0; i < 3; ++i) {
        const double ls = state_variable_luen_omega[i]->getValue(state);
        vs_omega_H[i]   = state_variable_vs_omega[i]->getValue(state);
        state_variable_luen_omega[i]->setDerivative(state, (reg_omega_raw[i] - ls) / reg_canal_t1);
        luen_omega_H[i]  = luen_c_sw * ls + luen_c_ft * reg_omega_raw[i];
        fused_canal_H[i] = luen_omega_H[i] + vs_omega_H[i];
    }

    // 3. 3D Otolith Maculae & Luenberger Specific-Force Observer in Skull Frame
    const SimTK::Rotation R_HW_now = cached_skull->getTransformInGround(state).R();
    const SimTK::Vec3 g_world(0.0, -9.81, 0.0);
    DelayedData acc_data = getDelayedData(state.getTime(), 0.0005, false, true);
    const SimTK::Vec3 f_gif_H = (~R_HW_now) * (acc_data.linear_acceleration - g_world);

    constexpr double reg_adp_r = 37.4 / 66.0, irr_adp_r = 37.4 / 245.0;
    constexpr double reg_adp_tau = 1.0, irr_adp_tau = 0.25;
    constexpr double reg_t1 = 0.0276, reg_pb = 0.0175;
    constexpr double irr_t1 = 0.0272, irr_pb = 0.0070;
    constexpr double reg_pg = 66.0 / 9.81, irr_pg = 245.0 / 9.81;
    constexpr double reg_ft = reg_t1 / reg_pb, reg_sw = 1.0 - reg_ft;
    constexpr double irr_ft = irr_t1 / irr_pb, irr_sw = 1.0 - irr_ft;
    constexpr double luen_ft = reg_pb / reg_t1, luen_sw = 1.0 - luen_ft;
    constexpr double luen_adp_tau = reg_adp_tau / reg_adp_r;
    constexpr double luen_adp_w   = (1.0 / reg_adp_r) - 1.0;

    // Coordinate-free hair-bundle radial tanh saturation in the macular shear plane orthogonal to resting gravity
    SimTK::Vec3 irr_stage1_vec(0.0);
    for (int i = 0; i < 3; ++i) {
        const double ia = state_variable_irregular_otolith_adapt[i]->getValue(state);
        state_variable_irregular_otolith_adapt[i]->setDerivative(state, (f_gif_H[i] - ia) / irr_adp_tau);
        irr_stage1_vec[i] = (irr_adp_r - 1.0) * ia + f_gif_H[i];
    }
    const double hair_tanh = std::max(0.0, get_phasic_oto_tanh_mps2());
    if (hair_tanh > 1e-6) {
        const SimTK::Vec3 u_up0_H = (~baseline_R_head_to_world.asMat33()) * SimTK::Vec3(0.0, 1.0, 0.0);
        const SimTK::Vec3 axial_part = (~u_up0_H * irr_stage1_vec) * u_up0_H;
        const SimTK::Vec3 shear_part = irr_stage1_vec - axial_part;
        irr_stage1_vec = axial_part + applyRadialTanhSat(shear_part, hair_tanh);
    }

    SimTK::Vec3 curr_irr_oto_H(0.0), curr_luen_oto_H(0.0);
    for (int i = 0; i < 3; ++i) {
        const double ra = state_variable_regular_otolith_adapt[i]->getValue(state);
        const double rm = state_variable_regular_otolith[i]->getValue(state);
        const double im = state_variable_irregular_otolith[i]->getValue(state);
        const double lm = state_variable_luenberger_otolith[i]->getValue(state);
        const double la = state_variable_luenberger_otolith_adapt[i]->getValue(state);

        state_variable_regular_otolith_adapt[i]->setDerivative(state, (f_gif_H[i] - ra) / reg_adp_tau);
        const double reg_s1 = (reg_adp_r - 1.0) * ra + f_gif_H[i];
        state_variable_regular_otolith[i]->setDerivative(state, (reg_s1 - rm) / reg_pb);
        double reg_rate = reg_pg * (reg_sw * rm + reg_ft * reg_s1);
        if (use_snoise) applyRegularOtolithNoise(reg_rate, reg_pg, i, noise_time, s_scale);
        const double reg_force = reg_rate / reg_pg;

        state_variable_luenberger_otolith[i]->setDerivative(state, (reg_force - lm) / reg_t1);
        const double luen_s1 = luen_sw * lm + luen_ft * reg_force;
        state_variable_luenberger_otolith_adapt[i]->setDerivative(state, (luen_s1 - la) / luen_adp_tau);
        curr_luen_oto_H[i] = luen_adp_w * la + luen_s1;

        state_variable_irregular_otolith[i]->setDerivative(state, (irr_stage1_vec[i] - im) / irr_pb);
        curr_irr_oto_H[i] = irr_sw * im + irr_ft * irr_stage1_vec[i];
        if (use_snoise) applyIrregularOtolithForceNoise(curr_irr_oto_H[i], i, noise_time, s_scale);
    }

    // 4. Spindle Fusimotor, Parallel Reference Cancellation, and Coordinate-Free d-DOF Decoder
    const double tau_spindle_ia_delay = 0.0010;
    const double tau_spindle_ii_delay = 0.0025;
    const double tau_agamma_delay     = 0.0025;

    double q_obs = state_variable_observer_quaternion[0]->getValue(state);
    double q_obx = state_variable_observer_quaternion[1]->getValue(state);
    double q_oby = state_variable_observer_quaternion[2]->getValue(state);
    double q_obz = state_variable_observer_quaternion[3]->getValue(state);
    const double q_obn = std::sqrt(q_obs * q_obs + q_obx * q_obx + q_oby * q_oby + q_obz * q_obz);
    if (q_obn > 1e-6) { q_obs /= q_obn; q_obx /= q_obn; q_oby /= q_obn; q_obz /= q_obn; }
    const SimTK::Rotation R_est_HW(SimTK::Quaternion(q_obs, q_obx, q_oby, q_obz));
    const SimTK::Mat33 R_est_AW = R_est_HW.asMat33() * (~baseline_R_head_to_trunk.asMat33());

    const SimTK::Vec3 trunk_omega_W = cached_trunk
        ? cached_trunk->getMobilizedBody().getBodyAngularVelocity(state)
        : SimTK::Vec3(0.0);
    const SimTK::Mat33 R_TW_now = cached_trunk
        ? cached_trunk->getTransformInGround(state).R().asMat33()
        : SimTK::Mat33(1.0);

    // Relative neck angular velocity estimate in frame A
    const SimTK::Vec3 omega_est_A = baseline_R_head_to_trunk.asMat33() * fused_canal_H - (~R_est_AW) * trunk_omega_W;
    double qd_est_pinv[32] = {0.0};
    for (int k = 0; k < std::min(num_neck_dofs, 32); ++k) {
        qd_est_pinv[k] = J_omega_pinv(k, 0) * omega_est_A[0]
                       + J_omega_pinv(k, 1) * omega_est_A[1]
                       + J_omega_pinv(k, 2) * omega_est_A[2];
    }

    double clean_ccr_ia_rates[160] = {0.0};
    double est_dl[160] = {0.0}, est_vl[160] = {0.0}, cun_dl[160] = {0.0};
    const int n_mus_cap = std::min(n_mus, 160);
    if (n_mus_cap > 0) {
        const auto& muscles_ref = getModel().getMuscles();
        double ia_buf[160] = {0.0}, ii_buf[160] = {0.0};
        double gd_buf[160] = {0.0}, gs_buf[160] = {0.0}, el_buf[160] = {0.0}, elp_buf[160] = {0.0};
        const bool has_ia = sampleDelayedVector(state.getTime(), tau_spindle_ia_delay, &HistoryRecord::raw_spindle_ia_rate, ia_buf, n_mus_cap);
        const bool has_ii = sampleDelayedVector(state.getTime(), tau_spindle_ii_delay, &HistoryRecord::raw_spindle_ii_rate, ii_buf, n_mus_cap);
        const bool has_gm =
            sampleDelayedVector(state.getTime(), tau_agamma_delay, &HistoryRecord::spinal_gamma_dyn_commands, gd_buf, n_mus_cap) &&
            sampleDelayedVector(state.getTime(), tau_agamma_delay, &HistoryRecord::spinal_gamma_stat_commands, gs_buf, n_mus_cap) &&
            sampleDelayedVector(state.getTime(), tau_agamma_delay, &HistoryRecord::expected_normalized_fiber_lengths, el_buf, n_mus_cap) &&
            sampleDelayedVector(state.getTime(), tau_agamma_delay, &HistoryRecord::expected_normalized_fiber_velocities, elp_buf, n_mus_cap);

        const double k_gd = get_enable_fusimotor() ? std::clamp(get_K_gamma_dyn(),  0.0, 1.0) : 0.0;
        const double k_gs = get_enable_fusimotor() ? std::clamp(get_K_gamma_stat(), 0.0, 1.0) : 0.0;
        const double th_ia = std::max(0.0, get_thresh_spindle_ia());
        const double th_ii = std::max(0.0, get_thresh_spindle_ii());
        const double* q0_ma = flat_ma_q0.data();

        for (int mi = 0; mi < n_mus_cap; ++mi) {
            const double u0_a = cached_minimum_excitations[mi];
            const double L0_n = baseline_normalized_fiber_lengths[mi];
            double arr_gd = has_gm ? gd_buf[mi]  : (100.0 * k_gd * (0.25 + 0.75 * u0_a));
            double arr_gs = has_gm ? gs_buf[mi]  : (100.0 * k_gs * u0_a);
            double exp_L  = has_gm ? el_buf[mi]  : L0_n;
            double exp_Lp = has_gm ? elp_buf[mi] : 0.0;

            const double fd_st = state_variable_spindle_dynamic_fusimotor[mi]->getValue(state);
            const double fs_st = state_variable_spindle_static_fusimotor[mi]->getValue(state);
            const double fd_tgt = (arr_gd * arr_gd) / (arr_gd * arr_gd + 3600.0);
            const double fs_tgt = (arr_gs * arr_gs) / (arr_gs * arr_gs + 3600.0);
            state_variable_spindle_dynamic_fusimotor[mi]->setDerivative(state, (fd_tgt - fd_st) / 0.070);
            state_variable_spindle_static_fusimotor[mi]->setDerivative(state,  (fs_tgt - fs_st) / 0.205);

            double ia_val = has_ia
                ? ia_buf[mi]
                : (cached_primary_afferent_outputs[mi] ? cached_primary_afferent_outputs[mi]->getValue(state) : baseline_primary_afferent_rates[mi]);
            double ii_val = has_ii
                ? ii_buf[mi]
                : (cached_secondary_afferent_outputs[mi] ? cached_secondary_afferent_outputs[mi]->getValue(state) : baseline_secondary_afferent_rates[mi]);
            if (use_snoise && !has_ia) applySpindleNoise(ia_val, 24,         0.35, 0.025, mi, noise_time, s_scale);
            if (use_snoise && !has_ii) applySpindleNoise(ii_val, 24 + n_mus, 0.20, 0.015, mi, noise_time, s_scale);

            double ref_ia = 0.0, ref_ii = 0.0, kap = 1.0;
            evaluateReferenceMileusnicSpindle(mi, exp_L, exp_Lp, fd_st, fs_st, arr_gs, ref_ia, ref_ii, kap);
            double exp_ia = baseline_primary_afferent_rates[mi]   + (ref_ia - baseline_reference_ia_rates[mi]);
            double exp_ii = baseline_secondary_afferent_rates[mi] + (ref_ii - baseline_reference_ii_rates[mi]);
            const double gd0 = 100.0 * k_gd * (0.25 + 0.75 * u0_a);
            const double gs0 = 100.0 * k_gs * u0_a;
            const double fd0 = (gd0 * gd0) / (gd0 * gd0 + 3600.0);
            const double fs0 = (gs0 * gs0) / (gs0 * gs0 + 3600.0);
            const double kap0 = 1.0 + (0.2592 * fd0) / std::max(0.015, 0.0605 - 0.046 * fs0);
            if (!get_enable_efference_cancellation()) {
                exp_ia = baseline_primary_afferent_rates[mi];
                exp_ii = baseline_secondary_afferent_rates[mi];
                kap = kap0;
            }
            const double raw_d_ia = (ia_val - exp_ia) * (std::max(1.0, kap0) / std::max(1.0, kap));
            const double raw_d_ii = (ii_val - exp_ii);
            clean_ccr_ia_rates[mi] = std::clamp(std::max(0.0, applyBiologicalDeadband(raw_d_ia, th_ia)), 0.0, 500.0);

            const double net_ia = applyBiologicalDeadband(raw_d_ia, th_ia);
            const double net_ii = applyBiologicalDeadband(raw_d_ii, th_ii);
            const double act_now = muscles_ref.get(active_muscle_indices[mi]).getActivation(state);
            const double u_plan_ref = std::max(baseline_muscle_activations[mi], last_voluntary_alpha_excitations[mi]);
            const double recon_len = computeTendonCompensatedMtuLength(
                cached_fiber_length_scales[mi] * net_ii,
                act_now,
                u_plan_ref,
                cached_tendon_slack_lengths[mi],
                get_enable_efference_cancellation());

            const double u_est = last_alpha_motor_excitations[mi];
            const double w_tau = 0.5 * (1.0 + std::tanh((u_est - act_now) / 0.005));
            const double tau_a = 0.025 + (0.010 - 0.025) * w_tau;
            const double da_dt = (u_est - act_now) / tau_a;

            double r_dot_qd = 0.0;
            const int m_base = mi * num_neck_dofs;
            for (int k = 0; k < std::min(num_neck_dofs, 32); ++k) r_dot_qd += q0_ma[m_base + k] * qd_est_pinv[k];
            const double eff_sc = std::max(1e-4, cached_optimal_fiber_lengths[mi] * cached_cosine_pennation_angles[mi]);
            const double Lp_cun = get_enable_efference_cancellation()
                ? (-(r_dot_qd + 0.052 * cached_tendon_slack_lengths[mi] * da_dt) / eff_sc)
                : 0.0;
            const double ref_ii_c = evaluateCuneateNeutralReferenceII(L0_n, Lp_cun, fs_st, arr_gs);
            const double exp_ii_c = get_enable_efference_cancellation()
                ? (baseline_secondary_afferent_rates[mi] + (ref_ii_c - baseline_reference_ii_rates[mi]))
                : baseline_secondary_afferent_rates[mi];
            const double dl_cun = cached_fiber_length_scales[mi] * applyBiologicalDeadband(ii_val - exp_ii_c, th_ii);
            cun_dl[mi] = computeTendonCompensatedMtuLength(dl_cun, act_now, baseline_muscle_activations[mi], cached_tendon_slack_lengths[mi], get_enable_efference_cancellation());

            const double sp_dl = state_variable_spindle_length_change[mi]->getValue(state);
            const double sp_vl = state_variable_spindle_velocity[mi]->getValue(state);
            const double dyn_ia = net_ia - 1.37931034 * net_ii;
            const double vel_err = (recon_len - sp_dl) / 0.020;
            const double vel_ia  = cached_primary_afferent_velocity_scales[mi] * dyn_ia;
            state_variable_spindle_length_change[mi]->setDerivative(state, sp_vl + (recon_len - sp_dl) / 0.008);
            state_variable_spindle_velocity[mi]->setDerivative(state, (vel_err + vel_ia - sp_vl) / 0.004);
            est_dl[mi] = sp_dl;
            est_vl[mi] = sp_vl;
        }

        if (get_enable_decoder() && num_neck_dofs > 0) {
            for (int r = 0; r < num_neck_dofs; ++r) {
                double sq = 0.0, sv = 0.0, sc = 0.0;
                for (int m = 0; m < n_mus_cap; ++m) {
                    const double dw = proprioceptive_decoder_d(r, m);
                    sq += dw * est_dl[m];
                    sv += dw * est_vl[m];
                    sc += dw * cun_dl[m];
                }
                estimated_neck_q[r]  = sq;
                estimated_neck_qd[r] = sv;
                cuneate_neck_q[r]    = sc;
            }
            estimated_neck_rotvec  = mulMatVec3(J_omega_d, estimated_neck_q);
            estimated_neck_omega   = mulMatVec3(J_omega_d, estimated_neck_qd);
            cuneate_posture_rotvec = mulMatVec3(J_omega_d, cuneate_neck_q);
            const double b_sh = std::clamp(get_shift_station_blend(), 0.0, 1.0);
            cuneate_posture_shift_m = 0.945 * mulBlendedJstaVec(b_sh, J_c1_d, J_com_d, cuneate_neck_q);
        } else {
            estimated_neck_q.setToZero();
            estimated_neck_qd.setToZero();
            cuneate_neck_q.setToZero();
            estimated_neck_rotvec = SimTK::Vec3(0.0);
            estimated_neck_omega  = SimTK::Vec3(0.0);
            cuneate_posture_rotvec = SimTK::Vec3(0.0);
            cuneate_posture_shift_m = SimTK::Vec3(0.0);
        }
    }
    for (int i = 0; i < 3; ++i) {
        const double xc = state_variable_cuneate_rot[i]->getValue(state);
        const double tc = get_enable_decoder() ? cuneate_posture_rotvec[i] : 0.0;
        state_variable_cuneate_rot[i]->setDerivative(state, (tc - xc) / 0.05);
    }

    // 5. Cerebellar SE2(3) Mahony Observer on SO(3) x R^6
    const double tau_2b_cb   = 0.0030;
    const double tau_3b_cun  = 0.0030;
    const double tau_4b_thal = 0.0130;
    const double tau_1a_tilt = 0.0090;
    const double tau_1b_can  = 0.0100;
    const double tau_2a_vn   = 0.0020;
    const double tau_4a_unc  = 0.0015;

    DelayedData del_2b = getDelayedData(state.getTime(), tau_2b_cb);
    DelayedData del_3b = getDelayedData(state.getTime(), tau_3b_cun);
    DelayedData del_4b = getDelayedData(state.getTime(), tau_4b_thal);

    const SimTK::Vec3 omega_H_cb = (!history.empty() && del_2b.record.valid) ? del_2b.record.canal_luen_omega_2b : fused_canal_H;
    const SimTK::Vec3 f_oto_H_cb = (!history.empty() && del_2b.record.valid) ? del_2b.record.oto_luen_force_2b   : curr_luen_oto_H;
    mahony_omega_cached = baseline_R_head_to_trunk.asMat33() * omega_H_cb;

    const SimTK::Vec3 est_lin_v(
        state_variable_observer_velocity[0]->getValue(state),
        state_variable_observer_velocity[1]->getValue(state),
        state_variable_observer_velocity[2]->getValue(state));
    const SimTK::Vec3 est_lin_p(
        state_variable_observer_position[0]->getValue(state),
        state_variable_observer_position[1]->getValue(state),
        state_variable_observer_position[2]->getValue(state));

    const SimTK::Vec3 xi_head_now = logSO3(R_est_HW.asMat33() * (~baseline_R_head_to_world.asMat33()));
    const SimTK::Vec3 del_xi_head = (!get_enable_mahony()) ? SimTK::Vec3(0.0)
        : ((!history.empty() && del_4b.record.valid) ? del_4b.record.est_head_rotvec_4b : xi_head_now);
    const SimTK::Vec3 del_om_head = (!get_enable_mahony()) ? SimTK::Vec3(0.0)
        : ((!history.empty() && del_4b.record.valid) ? del_4b.record.mahony_omega_4b    : mahony_omega_cached);
    const SimTK::Vec3 del_obs_p   = (!get_enable_mahony()) ? SimTK::Vec3(0.0)
        : ((!history.empty() && del_4b.record.valid) ? del_4b.record.est_lin_pos        : est_lin_p);
    const SimTK::Vec3 del_obs_v   = (!get_enable_mahony()) ? SimTK::Vec3(0.0)
        : ((!history.empty() && del_4b.record.valid) ? del_4b.record.est_lin_vel        : est_lin_v);

    // Washout & Nullspace Horizontal Translation IK
    const double e_eng = computeVoluntaryEngagement(urgency_now);
    const double w_tr  = std::clamp(get_w_translation(), 0.0, 1.0);
    const double k_op  = get_k_shift_obs_pos();
    const double k_ov  = get_k_shift_obs_vel();
    const double g_obs_w = get_enable_mahony() ? (e_eng * w_tr * smoothstep01((std::abs(k_op) + std::abs(k_ov)) / 1e-3)) : 0.0;
    const double tau_wash = get_tau_obs_washout();
    const double tau_vwash = std::max(0.2, 0.50 * tau_wash);

    SimTK::Vec3 obs_p_hp(0.0), obs_v_hp(0.0);
    for (int i = 0; i < 3; ++i) {
        const double xw  = state_variable_obs_pos_wash[i]->getValue(state);
        const double xvw = state_variable_obs_vel_wash[i]->getValue(state);
        if (tau_wash > 1e-3) {
            state_variable_obs_pos_wash[i]->setDerivative(state, (g_obs_w * del_obs_p[i] - xw) / tau_wash);
            state_variable_obs_vel_wash[i]->setDerivative(state, (g_obs_w * del_obs_v[i] - xvw) / tau_vwash);
            obs_p_hp[i] = del_obs_p[i] - g_obs_w * xw;
            obs_v_hp[i] = del_obs_v[i] - g_obs_w * xvw;
        } else {
            state_variable_obs_pos_wash[i]->setDerivative(state, -xw / 0.2);
            state_variable_obs_vel_wash[i]->setDerivative(state, -xvw / 0.2);
            obs_p_hp[i] = del_obs_p[i];
            obs_v_hp[i] = del_obs_v[i];
        }
    }
    obs_p_hp[1] = 0.0; obs_v_hp[1] = 0.0; // Horizontal tangent plane

    const double shift_cmd_mag = std::max(p_des.norm(), std::max(traj_p.norm(), traj_v.norm()));
    const double shift_gate = w_tr * std::max(g_obs_w, smoothstep01(shift_cmd_mag / 1e-5));
    const double vol_shift_gate = w_tr * e_eng;
    const SimTK::Vec3 p_ref_sh = traj_p + k_op * obs_p_hp + k_ov * obs_v_hp;

    std::vector<double> q_null_ref, qd_null_ref, qdd_null_ref;
    double gamma_ik = 0.0;
    solveNullspaceShiftIK(w_tr, shift_gate, get_shift_station_blend(), traj_xi, traj_om, traj_al, p_ref_sh, traj_v, traj_a, q_null_ref, qd_null_ref, qdd_null_ref, gamma_ik);

    for (int i = 0; i < 3; ++i) {
        const double xs = state_variable_cuneate_shift[i]->getValue(state);
        const double gs = get_enable_decoder() ? shift_gate : 0.0;
        const double tau_cs = 0.05 + 0.09 * vol_shift_gate;
        state_variable_cuneate_shift[i]->setDerivative(state, gs * ((cuneate_posture_shift_m[i] - xs) / tau_cs) + (1.0 - gs) * (-xs / 0.05));
    }

    // 6. Cortical SE(3) Impedance Law (Strictly 0 when urgency == 0!)
    const bool act_task = get_enable_task_controller();
    const bool act_tint = act_task && get_enable_task_integral();
    const double u_pos = std::clamp(urgency_now, 0.0, 1.0);
    const double urg_scale_p = 1.0 + 0.75 * u_pos;
    const double urg_scale_d = std::sqrt(urg_scale_p);

    // Geodesic SO(3) tracking error e_R in frame A
    const SimTK::Mat33 R_est_del = expSO3(del_xi_head);
    const SimTK::Mat33 R_traj    = expSO3(traj_xi);
    const SimTK::Vec3 e_R = R_est_del * logSO3((~R_est_del) * R_traj);
    const SimTK::Vec3 e_w = traj_om - del_om_head;

    const double w_tk  = std::max(0.0, get_omega_task());
    const double z_tk  = std::max(0.0, get_zeta_task());
    const SimTK::Mat33 K_task_base = (w_tk * w_tk) * tensor_I3
                                   + std::max(0.0, get_c_grav_task()) * tensor_Kg3
                                   + std::max(0.0, get_c_musc_task()) * tensor_Gmu3
                                   + std::max(0.0, get_c_disc_task()) * tensor_Kdisc3;
    const SimTK::Mat33 K_task_eff = urg_scale_p * K_task_base;
    // Exact coordinate-free Riemannian critical damping tensor: D_eff = 2 * zeta * I_3 * (I_3^-1 * K_eff)^(1/2)
    // computed via quadratically convergent Denman-Beavers iteration on gl(3, R) when task is engaged.
    SimTK::Mat33 D_task_eff = (2.0 * z_tk * w_tk * urg_scale_d) * tensor_I3;
    if (act_task && e_eng > 1e-12) {
        try {
            SimTK::Mat33 Y_db = tensor_I3.invert() * (K_task_eff + SimTK::Mat33(1e-6));
            SimTK::Mat33 Z_db(1.0);
            for (int db_it = 0; db_it < 6; ++db_it) {
                const SimTK::Mat33 Y_inv = Y_db.invert();
                const SimTK::Mat33 Z_inv = Z_db.invert();
                Y_db = 0.5 * (Y_db + Z_inv);
                Z_db = 0.5 * (Z_db + Y_inv);
            }
            const SimTK::Mat33 D_raw = (2.0 * z_tk) * (tensor_I3 * Y_db);
            D_task_eff = 0.5 * (D_raw + ~D_raw);
        } catch (...) {}
    }
    const double ki_rate = act_tint ? std::max(0.0, get_ki_task()) : 0.0;

    SimTK::Vec3 e_int_vec(0.0);
    for (int i = 0; i < 3; ++i) {
        const double ei = state_variable_rot_err_int[i]->getValue(state);
        e_int_vec[i] = ei;
        const double k_diag = std::max(0.5, K_task_base(i, i) * std::max(0.1, ki_rate));
        const double lim = std::max(0.6, 45.0 / k_diag);
        const double same_dir = smoothstep01((ei * e_R[i]) / 1e-4);
        const double lim_gate = 1.0 - smoothstep01((std::abs(ei) - 0.90 * lim) / std::max(1e-4, 0.10 * lim)) * same_dir;
        const double bleed = -10.0 * computeSmoothIntegratorExcess(ei, lim);
        const double def = last_qp_torque_deficit[i];
        const double s_def = smoothstep01((std::abs(def) - 4.0) / 2.0) * smoothstep01((def * e_R[i]) / 1e-3);
        const double a_sat = e_eng * s_def;
        const double idot = e_eng * ((1.0 - a_sat) * (lim_gate * e_R[i]) - a_sat * (0.10 * ei) + bleed) + (1.0 - e_eng) * (-20.0 * ei);
        state_variable_rot_err_int[i]->setDerivative(state, act_tint ? idot : 0.0);
    }

    const SimTK::Vec3 u_up_A(0.0, 1.0, 0.0);
    const SimTK::Vec3 trans_coup = get_k_trans_obs_pos() * (u_up_A % del_obs_p) + get_k_trans_obs_vel() * (u_up_A % del_obs_v);
    const SimTK::Vec3 tau_vol_unfilt = (act_task && e_eng > 1e-12)
        ? (e_eng * (K_task_eff  * e_R
                  + D_task_eff  * e_w
                  + ki_rate     * (K_task_base * e_int_vec)
                  + tensor_I3   * traj_al
                  + K_task_base * trans_coup))
        : SimTK::Vec3(0.0);

    // Horizontal shift PID in R^3 (filtered by 2nd-order cortical LPF)
    const SimTK::Vec3 del_cun_sh = (!get_enable_decoder()) ? SimTK::Vec3(0.0)
        : ((!history.empty() && del_4b.record.valid) ? del_4b.record.cuneate_shift_3b
            : SimTK::Vec3(state_variable_cuneate_shift[0]->getValue(state), state_variable_cuneate_shift[1]->getValue(state), state_variable_cuneate_shift[2]->getValue(state)));
    const SimTK::Vec3 del_cun_sh_v = (!get_enable_decoder()) ? SimTK::Vec3(0.0)
        : ((!history.empty() && del_4b.record.valid) ? del_4b.record.cuneate_shift_vel_3b : SimTK::Vec3(0.0));

    SimTK::Vec3 p_ref_clamped(0.0);
    if (gamma_ik > 1e-12 && num_neck_dofs > 0) {
        const double b_sh = std::clamp(get_shift_station_blend(), 0.0, 1.0);
        double q_tot[32] = {0.0};
        const int d_cap = std::min(num_neck_dofs, 32);
        for (int k = 0; k < d_cap; ++k) {
            q_tot[k] = P_comp_d(k, 0) * traj_xi[0] + P_comp_d(k, 1) * traj_xi[1] + P_comp_d(k, 2) * traj_xi[2]
                     + (k < static_cast<int>(q_null_ref.size()) ? q_null_ref[k] : 0.0);
        }
        p_ref_clamped = mulBlendedJsta(b_sh, J_c1_d, J_com_d, q_tot, d_cap);
        p_ref_clamped[1] = 0.0;
    }
    const SimTK::Vec3 e_sh_p = gamma_ik * (p_ref_clamped - SimTK::Vec3(del_cun_sh[0], 0.0, del_cun_sh[2]));
    const SimTK::Vec3 e_sh_v = gamma_ik * (SimTK::Vec3(traj_v[0], 0.0, traj_v[2]) - SimTK::Vec3(del_cun_sh_v[0], 0.0, del_cun_sh_v[2]));
    SimTK::Vec3 sh_int_vec(0.0);
    for (int i = 0; i < 3; ++i) {
        const double si = state_variable_shift_err_int[i]->getValue(state);
        sh_int_vec[i] = si;
        const double g_si = (act_tint && i != 1) ? shift_gate : 0.0;
        const double lim = 0.06;
        const double bleed = -10.0 * computeSmoothIntegratorExcess(si, lim);
        const double sdot = g_si * (e_sh_p[i] + bleed) + (1.0 - g_si) * (-2.0 * si);
        state_variable_shift_err_int[i]->setDerivative(state, sdot);
    }
    // Convert station position error [m] into characteristic force-like command filtered at fc_voluntary
    const SimTK::Vec3 sh_cmd_unfilt = act_task
        ? (shift_gate * (urg_scale_p * std::max(0.0, get_omega_shift()) * 120.0 * e_sh_p
                       + urg_scale_d * std::max(0.0, get_zeta_shift())  * 120.0 * e_sh_v
                       + (act_tint ? std::max(0.0, get_ki_shift()) * 120.0 : 0.0) * sh_int_vec))
        : SimTK::Vec3(0.0);

    const double fc_vol = std::max(0.1, get_fc_voluntary() * (1.0 + 1.5 * u_pos));
    const double wc = 2.0 * SimTK::Pi * fc_vol;
    const double wc2 = wc * wc;
    for (int i = 0; i < 3; ++i) {
        const double vp = state_variable_vol_filt_pos[i]->getValue(state);
        const double vv = state_variable_vol_filt_vel[i]->getValue(state);
        state_variable_vol_filt_pos[i]->setDerivative(state, vv);
        state_variable_vol_filt_vel[i]->setDerivative(state, wc2 * (tau_vol_unfilt[i] - vp) - 2.0 * wc * vv);

        const double sp = state_variable_vol_shift_filt_pos[i]->getValue(state);
        const double sv = state_variable_vol_shift_filt_vel[i]->getValue(state);
        state_variable_vol_shift_filt_pos[i]->setDerivative(state, sv);
        state_variable_vol_shift_filt_vel[i]->setDerivative(state, wc2 * (sh_cmd_unfilt[i] - sp) - 2.0 * wc * sv);
    }

    // 7. Mahony Attitude & Translation Observer Derivatives (Pure Lie Group FK: R_T^W * exp([xi_neck x]) * R_H0^T)
    const SimTK::Vec3 pred_up_H = (f_oto_H_cb.norm() > 1e-4) ? SimTK::Vec3(f_oto_H_cb.normalize()) : ((~R_est_HW) * SimTK::Vec3(0.0, 1.0, 0.0));
    const SimTK::Vec3 exp_up_H  = (~R_est_HW) * SimTK::Vec3(0.0, 1.0, 0.0);
    const SimTK::Vec3 oto_att_err_H = pred_up_H % exp_up_H;

    const double trans_acc_sq = (f_oto_H_cb - 9.81 * exp_up_H).normSqr();
    const double sig_a = use_snoise ? (0.030 * 0.70710678 * s_scale) : 0.0;
    const double trans_sq_eff = std::max(0.0, trans_acc_sq - 3.0 * sig_a * sig_a);
    const double oto_acc_gate = 1.0 / (1.0 + 16.0 * trans_sq_eff);

    const SimTK::Vec3 arr_xi_neck = (!history.empty() && del_3b.record.valid)
        ? del_3b.record.est_neck_rotvec_1a3
        : SimTK::Vec3(state_variable_cuneate_rot[0]->getValue(state), state_variable_cuneate_rot[1]->getValue(state), state_variable_cuneate_rot[2]->getValue(state));
    const SimTK::Mat33 arr_R_TW = (!history.empty() && del_3b.record.valid) ? del_3b.record.trunk_rotation_world : R_TW_now;
    const SimTK::Mat33 R_prop_HW = arr_R_TW * expSO3(arr_xi_neck) * baseline_R_head_to_trunk.asMat33();
    const SimTK::Mat33 E_prop = (~R_est_HW.asMat33()) * R_prop_HW;
    const SimTK::Vec3 prop_att_err_H(
        0.5 * (E_prop(2, 1) - E_prop(1, 2)),
        0.5 * (E_prop(0, 2) - E_prop(2, 0)),
        0.5 * (E_prop(1, 0) - E_prop(0, 1)));

    const double vol_att_mag = (traj_xi.norm() + 0.25 * traj_om.norm()) / 0.08;
    const double vol_posture_gate = e_eng * smoothstep01(vol_att_mag);
    const double prop_gate = 1.0 - 0.92 * std::max(e_eng, vol_shift_gate) * oto_acc_gate;
    const double w_oto_m = std::max(0.0, get_w_oto_mahony()) * (1.0 + 3.0 * vol_posture_gate * oto_acc_gate);
    const SimTK::Vec3 tot_att_err_H = (w_oto_m * oto_acc_gate) * oto_att_err_H + prop_gate * prop_att_err_H;

    const double Kp_mah = std::max(0.0, get_Kp_mahony());
    const double Ki_mah = std::max(0.0, get_Ki_mahony());
    const double vs_lk  = std::max(0.0, get_vs_leak_rate()) * (1.0 - 0.85 * vol_posture_gate);
    const SimTK::Vec3 w_fused_H = omega_H_cb + Kp_mah * tot_att_err_H;

    const double q0_dot = 0.5 * (-q_obx * w_fused_H[0] - q_oby * w_fused_H[1] - q_obz * w_fused_H[2]);
    const double q1_dot = 0.5 * ( q_obs * w_fused_H[0] + q_oby * w_fused_H[2] - q_obz * w_fused_H[1]);
    const double q2_dot = 0.5 * ( q_obs * w_fused_H[1] - q_obx * w_fused_H[2] + q_obz * w_fused_H[0]);
    const double q3_dot = 0.5 * ( q_obs * w_fused_H[2] + q_obx * w_fused_H[1] - q_oby * w_fused_H[0]);

    const SimTK::Mat33 arr_R_est_HW = (!history.empty() && del_2b.record.valid) ? del_2b.record.estimated_rotation_head_to_world : R_est_HW.asMat33();
    const SimTK::Vec3 a_lin_W = (arr_R_est_HW * f_oto_H_cb) + g_world;
    const SimTK::Vec3 v_lin_dot = a_lin_W   - (1.0 / std::max(0.2, get_tau_vel_leak())) * est_lin_v;
    const SimTK::Vec3 p_lin_dot = est_lin_v - (1.0 / std::max(0.2, get_tau_pos_leak())) * est_lin_p;

    const bool mah_on = get_enable_mahony();
    const double w_cup_reg = 1.0 / 5.7;
    for (int i = 0; i < 3; ++i) {
        state_variable_vs_omega[i]->setDerivative(
            state, w_cup_reg * luen_omega_H[i] + (mah_on ? (Ki_mah * tot_att_err_H[i] - vs_lk * vs_omega_H[i]) : (-0.05 * vs_omega_H[i])));
        state_variable_observer_velocity[i]->setDerivative(state, mah_on ? v_lin_dot[i] : 0.0);
        state_variable_observer_position[i]->setDerivative(state, mah_on ? p_lin_dot[i] : 0.0);
    }
    state_variable_observer_quaternion[0]->setDerivative(state, mah_on ? q0_dot : 0.0);
    state_variable_observer_quaternion[1]->setDerivative(state, mah_on ? q1_dot : 0.0);
    state_variable_observer_quaternion[2]->setDerivative(state, mah_on ? q2_dot : 0.0);
    state_variable_observer_quaternion[3]->setDerivative(state, mah_on ? q3_dot : 0.0);

    // 8. Cerebellar Gravity Adaptation & Brainstem VCR 2nd-Order Filter ODEs on so(3)
    const SimTK::Vec3 cb_grav_H = (~R_est_HW.asMat33()) * g_world;
    SimTK::Vec3 cb_g_adp(0.0);
    for (int i = 0; i < 3; ++i) {
        cb_g_adp[i] = state_variable_cb_grav_adapt[i]->getValue(state);
        state_variable_cb_grav_adapt[i]->setDerivative(state, (cb_grav_H[i] - cb_g_adp[i]) / irr_adp_tau);
    }
    const SimTK::Vec3 cb_adapted_grav_H = (irr_adp_r - 1.0) * cb_g_adp + cb_grav_H;

    DelayedData del_1a = getDelayedData(state.getTime(), tau_1a_tilt);
    DelayedData del_1b = getDelayedData(state.getTime(), tau_1b_can);
    DelayedData del_2a = getDelayedData(state.getTime(), tau_2a_vn);
    DelayedData del_4a = getDelayedData(state.getTime(), tau_4a_unc);

    SimTK::Vec3 fb_rot = (!history.empty() && del_1a.record.valid) ? del_1a.record.traj_rot_1a : traj_xi;
    SimTK::Vec3 fb_vel = (!history.empty() && del_1b.record.valid) ? del_1b.record.traj_vel_1b : traj_om;
    SimTK::Vec3 fb_acc = (!history.empty() && del_1b.record.valid) ? del_1b.record.traj_acc_1b : traj_al;
    SimTK::Vec3 fb_tac = (!history.empty() && del_1b.record.valid) ? del_1b.record.traj_trans_acc_1b : SimTK::Vec3(0.0);
    if (!get_enable_efference_cancellation()) {
        fb_rot = SimTK::Vec3(0.0); fb_vel = SimTK::Vec3(0.0); fb_acc = SimTK::Vec3(0.0); fb_tac = SimTK::Vec3(0.0);
    }
    const SimTK::Vec3 fb_acc_H = (~R_est_HW.asMat33()) * fb_tac;
    const SimTK::Vec3 arr_eff_ton = (!history.empty() && del_4a.record.valid) ? del_4a.record.eff_tonic_tilt_3a : (xi_head_now - fb_rot);
    const SimTK::Vec3 arr_eff_sc  = (!history.empty() && del_4a.record.valid) ? del_4a.record.eff_canal_vel_3a  : fb_vel;
    const SimTK::Vec3 arr_eff_ph  = (!history.empty() && del_4a.record.valid) ? del_4a.record.eff_phasic_eta_3a : (-cb_adapted_grav_H + fb_acc_H);

    const SimTK::Vec3 irr_omega_A_now = baseline_R_head_to_trunk.asMat33() * irr_omega_raw;
    const SimTK::Vec3 arr_irr_om_A  = (!history.empty() && del_2a.record.valid) ? del_2a.record.canal_irr_omega_2a : irr_omega_A_now;
    const SimTK::Vec3 arr_irr_oto_H = (!history.empty() && del_2a.record.valid) ? del_2a.record.oto_irr_force_2a   : curr_irr_oto_H;

    // Coordinate-free apparent inertial whiplash vector in frame A: eta_whip = df_A x u_up_A
    // (+X accel -> +5*e_z extension whiplash; +Z accel -> -5*e_x leftward whiplash; +Y accel -> 0!)
    const SimTK::Vec3 df_phas_H = arr_irr_oto_H - arr_eff_ph;
    const SimTK::Vec3 df_phas_A = baseline_R_head_to_trunk.asMat33() * df_phas_H;
    const SimTK::Vec3 u_up_est_A = (~R_est_AW) * SimTK::Vec3(0.0, 1.0, 0.0);
    SimTK::Vec3 u_ton_in  = applyDeadbandVec3(arr_eff_ton, std::max(0.0, get_thresh_vest_tilt()));
    SimTK::Vec3 u_sc_in   = applyDeadbandVec3(arr_irr_om_A - arr_eff_sc, std::max(0.0, get_thresh_vest_scc()));
    SimTK::Vec3 u_phas_in = applyDeadbandVec3(df_phas_A % u_up_est_A, std::max(0.0, get_thresh_vest_oto()));
    const double ph_in_sat = std::max(0.0, get_phasic_oto_sat_mps2());
    u_phas_in = applyRadialLinearKneeSat(u_phas_in, 0.85 * ph_in_sat, ph_in_sat);

    // Coordinate-free somatogravic Hill-4 horizontal acceleration reweighting & high-G envelope
    const SimTK::Vec3 a_tr_H = f_oto_H_cb - 9.81 * exp_up_H;
    const SimTK::Vec3 a_tr_horiz = a_tr_H - (~exp_up_H * a_tr_H) * exp_up_H;
    const double a_horiz_mag = a_tr_horiz.norm();
    const double rw_env = state_variable_oto_reweight_env->getValue(state);
    const double rw_rec = std::max(0.020, get_oto_reweight_tau_s());
    const double w_att  = 0.5 * (1.0 + std::tanh((a_horiz_mag - rw_env) / 0.15));
    const double tau_rw = rw_rec + (0.005 - rw_rec) * w_att;
    state_variable_oto_reweight_env->setDerivative(state, (a_horiz_mag - rw_env) / tau_rw);
    const double rw_a0 = std::max(0.0, get_oto_reweight_a0_mps2());
    const double rw_eff = std::max(rw_env, a_horiz_mag);
    const double rw_r2  = (rw_a0 > 1e-6) ? ((rw_eff * rw_eff) / (rw_a0 * rw_a0)) : 0.0;
    const double w_oto_rw = (rw_a0 > 1e-6) ? (1.0 / (1.0 + rw_r2 * rw_r2)) : 1.0;
    const double sqrt_w_oto = std::sqrt(w_oto_rw);

    const double hg_knee = std::max(0.0, get_highg_oto_knee_mps2());
    const double hg_env  = std::max(0.0, state_variable_highg_acc_env->getValue(state));
    const double ex_reg  = (hg_knee > 1e-6) ? std::max(0.0, std::sqrt(trans_sq_eff) - hg_knee) : 0.0;
    const double ex_irr  = (hg_knee > 1e-6) ? std::max(0.0, 0.5 * (u_phas_in.norm() - std::max(0.1, get_highg_irr_knee_ratio()) * hg_knee)) : 0.0;
    const double hg_ex   = std::max(ex_reg, ex_irr);
    state_variable_highg_acc_env->setDerivative(state, (hg_ex - hg_env) / ((hg_ex > hg_env) ? 0.002 : 0.250));
    const double g_highg = computeHighGAttenuation(hg_knee, std::max(hg_env, hg_ex), get_highg_transition_mps2());

    const bool vcr_on = get_enable_vcr();
    const bool ton_on = vcr_on && get_enable_vcr_tonic();
    const bool sc_on  = vcr_on && get_enable_vcr_canal();
    const bool ph_on  = vcr_on && get_enable_vcr_phasic();

    const double vol_move_mag = std::max(
        std::max(xi_des.norm(), std::max(traj_xi.norm(), 0.25 * traj_om.norm())) / 0.0087,
        std::max(p_des.norm(),  std::max(traj_p.norm(),  0.25 * traj_v.norm()))  / 0.0020);
    const double vol_phas_sup   = e_eng * smoothstep01(vol_move_mag);
    const double brace_phas_sup = std::clamp(get_highg_brace_phas_suppress(), 0.0, 0.90) * smoothstep01((urgency_now - 0.50) / 0.50);
    const double phas_gate      = std::max(0.90 * vol_phas_sup, brace_phas_sup);

    const double p_fast = 250.0;
    const double p_ton_b  = std::max(0.5, get_p_ton());
    const double p_sc_b   = std::max(0.5, get_p_sc());
    const double p_sc_fl  = std::max(0.5, get_highg_p_sc_floor());
    const double p_sc_eff_base = std::min(p_sc_b, p_sc_fl) + g_highg * std::max(0.0, p_sc_b - p_sc_fl);
    const double p_phas_b = std::max(0.2, get_p_phas()) + (1.0 - g_highg) * std::max(0.0, get_highg_phas_pole_boost());
    const double c_om_psc = std::max(0.0, get_c_omega_p_sc());
    const double tau_ti   = std::max(0.25, get_tau_ton_int());

    const SimTK::Vec3 x_sc_vec(
        state_variable_canal_state[0]->getValue(state),
        state_variable_canal_state[1]->getValue(state),
        state_variable_canal_state[2]->getValue(state));
    const SimTK::Vec3 v_sc_vec(
        state_variable_canal_vel[0]->getValue(state),
        state_variable_canal_vel[1]->getValue(state),
        state_variable_canal_vel[2]->getValue(state));
    // Coordinate-free Angelaki/Raphan-Cohen graviceptive tilt-dumping canal pole operator P_sc = p_sc * I_3 + c_omega * (I_3^-1 * K_g,3)
    const SimTK::Mat33 P_sc_op = SimTK::Mat33(p_sc_eff_base) + c_om_psc * tensor_Omega_sq;
    const SimTK::Vec3 u_sc_eff = sc_on ? u_sc_in : SimTK::Vec3(0.0);
    const SimTK::Vec3 p_sc_err = P_sc_op * (u_sc_eff - x_sc_vec);
    const SimTK::Vec3 p_sc_vel = P_sc_op * v_sc_vec;

    for (int i = 0; i < 3; ++i) {
        const double ut = ton_on ? u_ton_in[i] : 0.0;
        const double xt = state_variable_tonic_state[i]->getValue(state);
        const double vt = state_variable_tonic_vel[i]->getValue(state);
        state_variable_tonic_state[i]->setDerivative(state, vt);
        state_variable_tonic_vel[i]->setDerivative(state, (p_ton_b * p_fast) * (ut - xt) - (p_ton_b + p_fast) * vt);
        const double xti = state_variable_tonic_int[i]->getValue(state);
        state_variable_tonic_int[i]->setDerivative(state, xt - xti / tau_ti);

        const double vs = v_sc_vec[i];
        state_variable_canal_state[i]->setDerivative(state, vs);
        state_variable_canal_vel[i]->setDerivative(state, p_fast * p_sc_err[i] - (p_sc_vel[i] + p_fast * vs));

        const double up = ph_on ? (sqrt_w_oto * g_highg * (1.0 - phas_gate) * u_phas_in[i]) : 0.0;
        const double xp = state_variable_phasic_state[i]->getValue(state);
        const double vp = state_variable_phasic_vel[i]->getValue(state);
        state_variable_phasic_state[i]->setDerivative(state, vp);
        state_variable_phasic_vel[i]->setDerivative(state, (p_phas_b * p_fast) * (up - xp) - (p_phas_b + p_fast) * vp);
    }

    // 9. Spinal Fiber CCR & Tier B (GTO Ib, Renshaw, Articular Ligament) ODEs
    const double p_ccr_val = std::max(0.5, get_p_ccr());
    const bool ccr_sp_on = get_enable_ccr() && get_enable_ccr_spinal();
    for (int mi = 0; mi < n_mus; ++mi) {
        const double xc = state_variable_cervico_collic[mi]->getValue(state);
        const double vc = state_variable_cervico_collic_vel[mi]->getValue(state);
        const double u_ia = ccr_sp_on ? clean_ccr_ia_rates[mi] : 0.0;
        state_variable_cervico_collic[mi]->setDerivative(state, vc);
        state_variable_cervico_collic_vel[mi]->setDerivative(state, (p_ccr_val * p_fast) * (u_ia - xc) - (p_ccr_val + p_fast) * vc);
    }

    if (get_enable_gto_reflex() && n_mus_cap > 0) {
        double ib_buf[160] = {0.0};
        const bool has_ib = sampleDelayedVector(state.getTime(), 0.0025, &HistoryRecord::raw_gto_ib_rate, ib_buf, n_mus_cap);
        const double p_ib = std::max(5.0, get_p_gto_ib());
        const double th_ib = std::max(0.0, get_thresh_gto_ib());
        for (int m = 0; m < n_mus_cap; ++m) {
            double ib_m = has_ib ? ib_buf[m]
                : (cached_tendon_afferent_outputs[m] ? cached_tendon_afferent_outputs[m]->getValue(state) : baseline_tendon_afferent_rates[m]);
            if (use_snoise && !has_ib) applySpindleNoise(ib_m, 24 + 3 * n_mus, 0.25, 0.015, m, noise_time, s_scale);
            const double ib_st = state_variable_gto_ib[m]->getValue(state);
            const double ib_net = std::max(0.0, applyBiologicalDeadband(ib_m - baseline_tendon_afferent_rates[m], th_ib));
            state_variable_gto_ib[m]->setDerivative(state, p_ib * (ib_net - ib_st));
        }
    } else {
        for (int m = 0; m < n_mus; ++m) state_variable_gto_ib[m]->setDerivative(state, -10.0 * state_variable_gto_ib[m]->getValue(state));
    }

    if (get_enable_renshaw() && n_mus_cap > 0) {
        double rn_buf[160] = {0.0};
        const bool has_rn = sampleDelayedVector(state.getTime(), 0.0020, &HistoryRecord::spinal_motor_excitations, rn_buf, n_mus_cap);
        const double tau_rn = std::max(0.004, get_tau_renshaw());
        for (int m = 0; m < n_mus_cap; ++m) {
            const double u_d = has_rn ? rn_buf[m] : last_alpha_motor_excitations[m];
            const double du_r = std::max(0.0, u_d - cached_minimum_excitations[m]);
            const double rv = state_variable_renshaw[m]->getValue(state);
            state_variable_renshaw[m]->setDerivative(state, (du_r - rv) / tau_rn);
        }
    } else {
        for (int m = 0; m < n_mus; ++m) state_variable_renshaw[m]->setDerivative(state, -20.0 * state_variable_renshaw[m]->getValue(state));
    }

    if (get_enable_ligament_reflex()) {
        DelayedData del_lg = getDelayedData(state.getTime(), 0.0150, false);
        const bool has_lg = (!history.empty() && del_lg.record.valid);
        const double vg = has_lg ? std::clamp(del_lg.record.voluntary_activity_gate, 0.0, 1.0) : 0.0;
        const double p_lg = std::max(5.0, get_p_lig_reflex());
        for (int r = 0; r < 4; ++r) {
            const double pac = has_lg ? del_lg.record.raw_lig_pac[r]   : 0.0;
            const double gol = has_lg ? del_lg.record.raw_lig_golgi[r] : 0.0;
            const double u_l = std::clamp(0.45 * pac + (1.0 - 0.85 * vg) * gol, 0.0, 1.0);
            const double lv  = state_variable_lig_reflex[r]->getValue(state);
            state_variable_lig_reflex[r]->setDerivative(state, p_lg * (u_l - lv));
        }
    } else {
        for (int r = 0; r < 4; ++r) state_variable_lig_reflex[r]->setDerivative(state, -10.0 * state_variable_lig_reflex[r]->getValue(state));
    }

    // 10. Proper Lead Compensator States
    const bool int_leads = (state.getTime() > 1e-9 && lead_u0_initialized);
    for (size_t k = 0; k < state_variable_lead_comp.size(); ++k) {
        if (!state_variable_lead_comp[k]) continue;
        const double xt = state_variable_lead_comp[k]->getValue(state);
        const double du = int_leads ? (lead_u_curr[k] - lead_u0[k]) : 0.0;
        const double tp = (k < lead_tau_p.size() && lead_tau_p[k] > 1e-4) ? lead_tau_p[k] : 0.0020;
        state_variable_lead_comp[k]->setDerivative(state, (du - xt) / tp);
    }
}

void HeadNeckNeuralController::computeControls(const SimTK::State& state, SimTK::Vector& controls) const {
    if (!is_initialized) {
        initializeGeometryAndBaseline(state);
    }
    const int d = num_neck_dofs;
    const int M = static_cast<int>(active_muscle_indices.size());
    const auto& muscles = getModel().getMuscles();

    const SimTK::Vec3 traj_xi(
        state_variable_traj_rot[0]->getValue(state),
        state_variable_traj_rot[1]->getValue(state),
        state_variable_traj_rot[2]->getValue(state));
    const SimTK::Vec3 traj_om(
        state_variable_traj_vel[0]->getValue(state),
        state_variable_traj_vel[1]->getValue(state),
        state_variable_traj_vel[2]->getValue(state));
    const SimTK::Vec3 traj_al(
        state_variable_traj_acc[0]->getValue(state),
        state_variable_traj_acc[1]->getValue(state),
        state_variable_traj_acc[2]->getValue(state));
    const SimTK::Vec3 traj_p(
        state_variable_traj_pos[0]->getValue(state),
        state_variable_traj_pos[1]->getValue(state),
        state_variable_traj_pos[2]->getValue(state));
    const SimTK::Vec3 traj_v(
        state_variable_traj_tvel[0]->getValue(state),
        state_variable_traj_tvel[1]->getValue(state),
        state_variable_traj_tvel[2]->getValue(state));
    const SimTK::Vec3 traj_a(
        state_variable_traj_tacc[0]->getValue(state),
        state_variable_traj_tacc[1]->getValue(state),
        state_variable_traj_tacc[2]->getValue(state));

    double w_exact = 0.0;
    evaluateMomentArmMatrix(state, w_exact);
    if (state.getTime() - last_jacobian_update_time > 0.02) {
        if (scratch_J_curr.nrow() != d || scratch_J_curr.ncol() != M) scratch_J_curr.resize(d, M);
        for (int m = 0; m < M; ++m)
            for (int r = 0; r < d; ++r)
                scratch_J_curr(r, m) = flat_J_curr[m * d + r];
        updateDecoderAndSynergies(scratch_J_curr);
        last_jacobian_update_time = state.getTime();
    }

    const double noise_time = state.getTime();
    const double g_nscale   = std::max(0.0, get_noise_scale());
    const bool use_snoise   = get_enable_sensor_noise() && (g_nscale > 1e-9);
    const bool use_mnoise   = get_enable_motor_noise()  && (g_nscale > 1e-9);
    const double s_scale    = use_snoise ? g_nscale : 0.0;
    const double m_scale    = use_mnoise ? g_nscale : 0.0;
    if (use_snoise || use_mnoise) {
        noise_bank.ensureCapacity(noise_time, get_noise_seed(), std::max(1, M), get_enable_gto_reflex());
    }

    // 1. Reconstruct Peripheral Vestibular Signals & Mahony Attitude
    const SimTK::Vec3 omega_skull_H = computeSkullBodyAngularVelocity(*cached_skull, state);
    const SimTK::Vec3 r_proj = right_canal_projection_matrix * omega_skull_H;
    const SimTK::Vec3 l_proj = left_canal_projection_matrix  * omega_skull_H;
    const double canal_in[6] = { r_proj[0], r_proj[1], r_proj[2], l_proj[0], l_proj[1], l_proj[2] };
    constexpr double reg_canal_t1_c = 0.0175, reg_canal_t2_c = 0.0027;
    constexpr double reg_passband_c = 2.83 * (reg_canal_t2_c / reg_canal_t1_c);
    constexpr double irr_passband_c = 27.09 * (0.0006 / 0.0300);
    constexpr double luen_ft_c = reg_canal_t2_c / reg_canal_t1_c;
    constexpr double luen_sw_c = 1.0 - luen_ft_c;

    double reg_out_c[6] = {0.0}, irr_out[6] = {0.0};
    for (int ci = 0; ci < 6; ++ci) {
        {
            const auto& cp0 = canal_filter_parameters[0];
            const double x0 = state_variable_semicircular_canal[ci][0][0]->getValue(state);
            const double x1 = state_variable_semicircular_canal[ci][0][1]->getValue(state);
            double y0 = cp0.state_space_output_c0 * x0 + cp0.state_space_output_c1 * x1 + cp0.direct_feedthrough_d * canal_in[ci];
            if (use_snoise) applyVestibularCanalNoise(y0, reg_passband_c, ci, 0.010, 0.025, noise_time, s_scale);
            reg_out_c[ci] = y0;
        }
        const auto& cp = canal_filter_parameters[1];
        const double x0 = state_variable_semicircular_canal[ci][1][0]->getValue(state);
        const double x1 = state_variable_semicircular_canal[ci][1][1]->getValue(state);
        double y = cp.state_space_output_c0 * x0 + cp.state_space_output_c1 * x1 + cp.direct_feedthrough_d * canal_in[ci];
        if (use_snoise) applyVestibularCanalNoise(y, irr_passband_c, 6 + ci, 0.020, 0.050, noise_time, s_scale);
        irr_out[ci] = y;
    }
    const SimTK::Vec3 reg_omega_Hc = (0.5 / reg_passband_c) * (
        right_canal_inverse_matrix * SimTK::Vec3(reg_out_c[0], reg_out_c[1], reg_out_c[2]) +
        left_canal_inverse_matrix  * SimTK::Vec3(reg_out_c[3], reg_out_c[4], reg_out_c[5]));
    const SimTK::Vec3 irr_omega_H = (0.5 / irr_passband_c) * (
        right_canal_inverse_matrix * SimTK::Vec3(irr_out[0], irr_out[1], irr_out[2]) +
        left_canal_inverse_matrix  * SimTK::Vec3(irr_out[3], irr_out[4], irr_out[5]));
    const SimTK::Vec3 irr_omega_A = baseline_R_head_to_trunk.asMat33() * irr_omega_H;
    dbg_last_canal_irr_head = irr_omega_H;

    SimTK::Vec3 fused_canal_H(0.0);
    for (int i = 0; i < 3; ++i) {
        const double ls = state_variable_luen_omega[i]->getValue(state);
        const double luen_i = luen_sw_c * ls + luen_ft_c * reg_omega_Hc[i];
        fused_canal_H[i] = luen_i + state_variable_vs_omega[i]->getValue(state);
    }
    mahony_omega_cached = get_enable_mahony() ? (baseline_R_head_to_trunk.asMat33() * fused_canal_H) : SimTK::Vec3(0.0);

    double q0 = state_variable_observer_quaternion[0]->getValue(state);
    double q1 = state_variable_observer_quaternion[1]->getValue(state);
    double q2 = state_variable_observer_quaternion[2]->getValue(state);
    double q3 = state_variable_observer_quaternion[3]->getValue(state);
    const double qn = std::sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (qn > 1e-6) { q0 /= qn; q1 /= qn; q2 /= qn; q3 /= qn; }
    const SimTK::Rotation R_est_HW = get_enable_mahony()
        ? SimTK::Rotation(SimTK::Quaternion(q0, q1, q2, q3))
        : baseline_R_head_to_world;
    const SimTK::Mat33 R_est_AW = R_est_HW.asMat33() * (~baseline_R_head_to_trunk.asMat33());
    const SimTK::Vec3 xi_head_now = get_enable_mahony()
        ? logSO3(R_est_HW.asMat33() * (~baseline_R_head_to_world.asMat33()))
        : SimTK::Vec3(0.0);

    // Otolith specific force readout
    const SimTK::Rotation R_HW_true = cached_skull->getTransformInGround(state).R();
    const SimTK::Vec3 g_world(0.0, -9.81, 0.0);
    DelayedData acc_data = getDelayedData(state.getTime(), 0.0005, false, true);
    const SimTK::Vec3 f_gif_H = (~R_HW_true) * (acc_data.linear_acceleration - g_world);

    constexpr double reg_adp_r = 37.4 / 66.0, irr_adp_r = 37.4 / 245.0;
    constexpr double reg_t1 = 0.0276, reg_pb = 0.0175, irr_t1 = 0.0272, irr_pb = 0.0070;
    constexpr double reg_pg = 66.0 / 9.81;
    constexpr double reg_ft = reg_t1 / reg_pb, reg_sw = 1.0 - reg_ft;
    constexpr double irr_ft = irr_t1 / irr_pb, irr_sw = 1.0 - irr_ft;
    constexpr double luen_ft = reg_pb / reg_t1, luen_sw = 1.0 - luen_ft;
    constexpr double luen_adp_w = (1.0 / reg_adp_r) - 1.0;

    SimTK::Vec3 irr_s1(0.0);
    for (int i = 0; i < 3; ++i) {
        irr_s1[i] = (irr_adp_r - 1.0) * state_variable_irregular_otolith_adapt[i]->getValue(state) + f_gif_H[i];
    }
    const double hair_tanh = std::max(0.0, get_phasic_oto_tanh_mps2());
    if (hair_tanh > 1e-6) {
        const SimTK::Vec3 u_up0_H = (~baseline_R_head_to_world.asMat33()) * SimTK::Vec3(0.0, 1.0, 0.0);
        const SimTK::Vec3 ax_p = (~u_up0_H * irr_s1) * u_up0_H;
        irr_s1 = ax_p + applyRadialTanhSat(irr_s1 - ax_p, hair_tanh);
    }
    SimTK::Vec3 oto_irr_H(0.0), oto_luen_H(0.0);
    for (int i = 0; i < 3; ++i) {
        const double ra = state_variable_regular_otolith_adapt[i]->getValue(state);
        const double rm = state_variable_regular_otolith[i]->getValue(state);
        const double im = state_variable_irregular_otolith[i]->getValue(state);
        const double lm = state_variable_luenberger_otolith[i]->getValue(state);
        const double la = state_variable_luenberger_otolith_adapt[i]->getValue(state);
        const double reg_s1 = (reg_adp_r - 1.0) * ra + f_gif_H[i];
        double reg_rate = reg_pg * (reg_sw * rm + reg_ft * reg_s1);
        if (use_snoise) applyRegularOtolithNoise(reg_rate, reg_pg, i, noise_time, s_scale);
        const double reg_f = reg_rate / reg_pg;
        const double luen_s1 = luen_sw * lm + luen_ft * reg_f;
        oto_luen_H[i] = luen_adp_w * la + luen_s1;
        oto_irr_H[i]  = irr_sw * im + irr_ft * irr_s1[i];
        if (use_snoise) applyIrregularOtolithForceNoise(oto_irr_H[i], i, noise_time, s_scale);
    }
    dbg_last_oto_luen_head = oto_luen_H;
    dbg_last_oto_irr_head  = oto_irr_H;

    // 2. Anatomical Inter-Block Delays & Lead Compensators
    const double tau_1a_tilt   = 0.0090;
    const double tau_1b_can    = 0.0100;
    const double tau_1c_cst    = 0.0145;
    const double tau_2a_vn     = 0.0020;
    const double tau_2b_cb     = 0.0030;
    const double tau_ia_del    = 0.0010;
    const double tau_ii_del    = 0.0025;
    const double tau_3b_cun    = 0.0030;
    const double tau_4a_unc    = 0.0015;
    const double tau_4b_thal   = 0.0130;
    const double tau_5_mvst    = 0.0025;
    const double tau_7a_alpha  = 0.0025;
    const double tau_7b_gamma  = 0.0025;

    const double urgency_ctrl  = get_urgency();
    const double e_eng_ctrl    = computeVoluntaryEngagement(urgency_ctrl);
    const double s_supra_ctrl  = computeSupraspinalAlertness(urgency_ctrl);
    const double s_spinal_ctrl = computeSpinalAlertness(urgency_ctrl);
    const double hg_env_ctrl   = std::max(0.0, state_variable_highg_acc_env->getValue(state));
    const double g_highg_ctrl  = computeHighGAttenuation(std::max(0.0, get_highg_oto_knee_mps2()), hg_env_ctrl, get_highg_transition_mps2());
    const double tau_lead_eff  = std::clamp(get_tau_lead_pole(), 0.0020, 0.2000) + (1.0 - g_highg_ctrl) * std::max(0.0, get_highg_tau_lead_add_s());

    const double t_sim = state.getTime();
    DelayedData del_4b_ctrl = getDelayedData(t_sim, tau_4b_thal);
    const SimTK::Vec3 del_obs_p_c = (!get_enable_mahony()) ? SimTK::Vec3(0.0)
        : ((!history.empty() && del_4b_ctrl.record.valid) ? del_4b_ctrl.record.est_lin_pos : getObserverPositionVector(state));
    const SimTK::Vec3 del_obs_v_c = (!get_enable_mahony()) ? SimTK::Vec3(0.0)
        : ((!history.empty() && del_4b_ctrl.record.valid) ? del_4b_ctrl.record.est_lin_vel : getObserverVelocityVector(state));

    const double w_tr_c = std::clamp(get_w_translation(), 0.0, 1.0);
    const double k_op_c = get_k_shift_obs_pos(), k_ov_c = get_k_shift_obs_vel();
    const double g_obs_wc = get_enable_mahony() ? (e_eng_ctrl * w_tr_c * smoothstep01((std::abs(k_op_c) + std::abs(k_ov_c)) / 1e-3)) : 0.0;
    const double tau_w_c  = get_tau_obs_washout();
    SimTK::Vec3 obs_p_hpc(0.0), obs_v_hpc(0.0);
    for (int i = 0; i < 3; ++i) {
        obs_p_hpc[i] = (tau_w_c > 1e-3) ? (del_obs_p_c[i] - g_obs_wc * state_variable_obs_pos_wash[i]->getValue(state)) : del_obs_p_c[i];
        obs_v_hpc[i] = (tau_w_c > 1e-3) ? (del_obs_v_c[i] - g_obs_wc * state_variable_obs_vel_wash[i]->getValue(state)) : del_obs_v_c[i];
    }
    obs_p_hpc[1] = 0.0; obs_v_hpc[1] = 0.0;

    const SimTK::Vec3 p_des_c = get_desired_translation_vec();
    const double sh_mag_c = std::max(p_des_c.norm(), std::max(traj_p.norm(), traj_v.norm()));
    const double sh_gate_c = w_tr_c * std::max(g_obs_wc, smoothstep01(sh_mag_c / 1e-5));
    const SimTK::Vec3 p_ref_c = traj_p + k_op_c * obs_p_hpc + k_ov_c * obs_v_hpc;

    std::vector<double> q_null_c, qd_null_c, qdd_null_c;
    double gamma_ik_c = 0.0;
    solveNullspaceShiftIK(w_tr_c, sh_gate_c, get_shift_station_blend(), traj_xi, traj_om, traj_al, p_ref_c, traj_v, traj_a, q_null_c, qd_null_c, qdd_null_c, gamma_ik_c);

    // Map filtered station force command + feedforward into d-DOF nullspace wrench
    const int d_cap = std::min(d, 32);
    const int M_cap = std::min(M, 160);
    const double b_sh_c = std::clamp(get_shift_station_blend(), 0.0, 1.0);
    const SimTK::Vec3 f_sh_filt(
        get_enable_task_controller() ? (sh_gate_c * state_variable_vol_shift_filt_pos[0]->getValue(state)) : 0.0,
        0.0,
        get_enable_task_controller() ? (sh_gate_c * state_variable_vol_shift_filt_pos[2]->getValue(state)) : 0.0);
    std::vector<double> w_null_vol_raw;
    if (get_enable_task_controller() && sh_gate_c > 1e-10 && d_cap > 0) {
        w_null_vol_raw.assign(static_cast<size_t>(d), 0.0);
        double raw_sh_d[32] = {0.0};
        const double ff_gain = w_tr_c * e_eng_ctrl * std::max(0.0, get_c_ff_shift());
        for (int r = 0; r < d_cap; ++r) {
            double jtf = 0.0;
            for (int a = 0; a < 3; ++a) {
                const double j_ar = (1.0 - b_sh_c) * J_c1_d(a, r) + b_sh_c * J_com_d(a, r);
                jtf += j_ar * f_sh_filt[a];
            }
            double kd_qn = 0.0, md_qddn = 0.0;
            if (static_cast<int>(q_null_c.size()) == d && static_cast<int>(qdd_null_c.size()) == d) {
                for (int c = 0; c < d_cap; ++c) {
                    kd_qn   += Kdisc_neck_d(r, c) * q_null_c[c];
                    md_qddn += M_neck_d(r, c)     * qdd_null_c[c];
                }
            }
            raw_sh_d[r] = jtf + ff_gain * kd_qn + 0.02 * md_qddn;
        }
        for (int r = 0; r < d_cap; ++r) {
            double acc = 0.0;
            for (int c = 0; c < d_cap; ++c) acc += Pi_null_wrench_d(r, c) * raw_sh_d[c];
            w_null_vol_raw[r] = acc;
        }
    }

    const SimTK::Vec3 vol_y(
        get_enable_task_controller() ? state_variable_vol_filt_pos[0]->getValue(state) : 0.0,
        get_enable_task_controller() ? state_variable_vol_filt_pos[1]->getValue(state) : 0.0,
        get_enable_task_controller() ? state_variable_vol_filt_pos[2]->getValue(state) : 0.0);

    // Sender-side proper lead compensators
    SimTK::Vec3 canal_irr_2a, oto_irr_2a, canal_luen_2b, oto_luen_2b;
    SimTK::Vec3 est_ori_4a, est_ori_4b, mah_om_4b;
    SimTK::Vec3 traj_rot_1a, traj_vel_1b, traj_acc_1b, traj_tacc_1b, vol_tau_1c, traj_rot_1c, traj_vel_1c;
    double qdd_plan_tot[32] = {0.0};
    for (int k = 0; k < d_cap; ++k) {
        qdd_plan_tot[k] = P_comp_d(k, 0) * traj_al[0] + P_comp_d(k, 1) * traj_al[1] + P_comp_d(k, 2) * traj_al[2]
                        + (k < static_cast<int>(qdd_null_c.size()) ? qdd_null_c[k] : 0.0);
    }
    const SimTK::Vec3 trans_acc_plan = mulBlendedJsta(b_sh_c, J_c1_d, J_com_d, qdd_plan_tot, d_cap);
    for (int i = 0; i < 3; ++i) {
        canal_irr_2a[i] = evalProperLead(0  + i, irr_omega_A[i],         tau_2a_vn,   state, tau_lead_eff);
        oto_irr_2a[i]   = evalProperLead(3  + i, oto_irr_H[i],           tau_2a_vn,   state, tau_lead_eff);
        canal_luen_2b[i]= evalProperLead(6  + i, fused_canal_H[i],       tau_2b_cb,   state, tau_lead_eff);
        oto_luen_2b[i]  = evalProperLead(12 + i, oto_luen_H[i],          tau_2b_cb,   state, tau_lead_eff);
        est_ori_4a[i]   = evalProperLead(36 + i, xi_head_now[i],         tau_4a_unc,  state, tau_lead_eff);
        est_ori_4b[i]   = evalProperLead(39 + i, xi_head_now[i],         tau_4b_thal, state, tau_lead_eff);
        mah_om_4b[i]    = evalProperLead(42 + i, mahony_omega_cached[i], tau_4b_thal, state, tau_lead_eff);
        traj_rot_1a[i]  = evalProperLead(45 + i, traj_xi[i],             tau_1a_tilt, state, tau_lead_eff);
        traj_vel_1b[i]  = evalProperLead(48 + i, traj_om[i],             tau_1b_can,  state, tau_lead_eff);
        traj_acc_1b[i]  = evalProperLead(51 + i, traj_al[i],             tau_1b_can,  state, tau_lead_eff);
        vol_tau_1c[i]   = evalProperLead(54 + i, vol_y[i],               tau_1c_cst,  state, tau_lead_eff);
        traj_rot_1c[i]  = evalProperLead(57 + i, traj_xi[i],             tau_1c_cst,  state, tau_lead_eff);
        traj_vel_1c[i]  = evalProperLead(60 + i, traj_om[i],             tau_1c_cst,  state, tau_lead_eff);
        traj_tacc_1b[i] = evalProperLead(71 + i, trans_acc_plan[i],      tau_1b_can,  state, tau_lead_eff);
    }

    DelayedData del_1a_c = getDelayedData(t_sim, tau_1a_tilt);
    DelayedData del_1b_c = getDelayedData(t_sim, tau_1b_can);
    DelayedData del_1c_c = getDelayedData(t_sim, tau_1c_cst);
    DelayedData del_2a_c = getDelayedData(t_sim, tau_2a_vn);
    DelayedData del_4a_c = getDelayedData(t_sim, tau_4a_unc);

    SimTK::Vec3 cb_tr_1a  = (!history.empty() && del_1a_c.record.valid) ? del_1a_c.record.traj_rot_1a       : traj_xi;
    SimTK::Vec3 cb_tv_1b  = (!history.empty() && del_1b_c.record.valid) ? del_1b_c.record.traj_vel_1b       : traj_om;
    SimTK::Vec3 cb_tta_1b = (!history.empty() && del_1b_c.record.valid) ? del_1b_c.record.traj_trans_acc_1b : trans_acc_plan;
    if (!get_enable_efference_cancellation()) {
        cb_tr_1a = SimTK::Vec3(0.0); cb_tv_1b = SimTK::Vec3(0.0); cb_tta_1b = SimTK::Vec3(0.0);
    }
    const SimTK::Vec3 cb_grav_H = (~R_est_HW.asMat33()) * g_world;
    const SimTK::Vec3 cb_g_adp(
        state_variable_cb_grav_adapt[0]->getValue(state),
        state_variable_cb_grav_adapt[1]->getValue(state),
        state_variable_cb_grav_adapt[2]->getValue(state));
    const SimTK::Vec3 cb_adapted_grav_H = (irr_adp_r - 1.0) * cb_g_adp + cb_grav_H;
    const SimTK::Vec3 c_ton_raw  = xi_head_now - cb_tr_1a;
    const SimTK::Vec3 c_phas_raw = -cb_adapted_grav_H + ((~R_est_HW.asMat33()) * cb_tta_1b);

    SimTK::Vec3 eff_ton_3a, eff_sc_3a, eff_ph_3a;
    for (int i = 0; i < 3; ++i) {
        eff_ton_3a[i] = evalProperLead(24 + i, c_ton_raw[i],  tau_4a_unc, state, tau_lead_eff);
        eff_sc_3a[i]  = evalProperLead(27 + i, cb_tv_1b[i],   tau_4a_unc, state, tau_lead_eff);
        eff_ph_3a[i]  = evalProperLead(30 + i, c_phas_raw[i], tau_4a_unc, state, tau_lead_eff);
    }

    // Diagnostic readout of phasic_oto_input in frame A (apparent whiplash df_A x u_up_A)
    {
        const SimTK::Vec3 arr_irr_oto = (!history.empty() && del_2a_c.record.valid) ? del_2a_c.record.oto_irr_force_2a   : oto_irr_H;
        const SimTK::Vec3 arr_eff_ph  = (!history.empty() && del_4a_c.record.valid) ? del_4a_c.record.eff_phasic_eta_3a  : c_phas_raw;
        const SimTK::Vec3 df_A = baseline_R_head_to_trunk.asMat33() * (arr_irr_oto - arr_eff_ph);
        const SimTK::Vec3 u_up_A = (~R_est_AW) * SimTK::Vec3(0.0, 1.0, 0.0);
        dbg_last_phasic_oto_input = df_A % u_up_A;
    }

    // 3. Populate HistoryRecord (recycled from history_pool for zero steady-state heap allocation) & Decode Spindle States
    HistoryRecord rec;
    if (!history_pool.empty()) {
        rec = std::move(history_pool.back());
        history_pool.pop_back();
    }
    rec.valid                            = true;
    rec.est_head_rotvec_4b               = est_ori_4b;
    rec.est_head_rotvec_4a               = est_ori_4a;
    rec.canal_irr_omega_2a               = canal_irr_2a;
    rec.canal_luen_omega_2b              = canal_luen_2b;
    rec.oto_irr_force_2a                 = oto_irr_2a;
    rec.oto_luen_force_2b                = oto_luen_2b;
    rec.estimated_rotation_head_to_world = R_est_HW.asMat33();
    rec.trunk_rotation_world             = cached_trunk ? cached_trunk->getTransformInGround(state).R().asMat33() : SimTK::Mat33(1.0);
    rec.trunk_omega_world                = cached_trunk ? cached_trunk->getMobilizedBody().getBodyAngularVelocity(state) : SimTK::Vec3(0.0);
    rec.head_linear_velocity             = cached_skull->getVelocityInGround(state)[1];
    rec.mahony_omega_4b                  = mah_om_4b;
    rec.est_lin_vel                      = getObserverVelocityVector(state);
    rec.est_lin_pos                      = getObserverPositionVector(state);
    rec.cortical_vol_torque              = vol_tau_1c;
    rec.cortical_vol_null_wrench.assign(w_null_vol_raw.begin(), w_null_vol_raw.end());
    rec.traj_rot_1a                      = traj_rot_1a;
    rec.traj_vel_1b                      = traj_vel_1b;
    rec.traj_acc_1b                      = traj_acc_1b;
    rec.traj_trans_acc_1b                = traj_tacc_1b;
    rec.eff_tonic_tilt_3a                = eff_ton_3a;
    rec.eff_canal_vel_3a                 = eff_sc_3a;
    rec.eff_phasic_eta_3a                = eff_ph_3a;
    rec.traj_rot_1c                      = traj_rot_1c;
    rec.traj_vel_1c                      = traj_vel_1c;
    rec.traj_q_null_1c.assign(q_null_c.begin(), q_null_c.end());
    rec.traj_qd_null_1c.assign(qd_null_c.begin(), qd_null_c.end());

    if (history.empty()) {
        const double k_gs_init = get_enable_fusimotor() ? std::clamp(get_K_gamma_stat(), 0.0, 1.0) : 0.0;
        for (int mi = 0; mi < M; ++mi) {
            const Muscle& mus_i = muscles.get(active_muscle_indices[mi]);
            const double l_opt_i = std::max(1e-4, cached_optimal_fiber_lengths[mi]);
            double L0_i = baseline_normalized_fiber_lengths[mi];
            try { L0_i = std::clamp(mus_i.getFiberLength(state) / l_opt_i, 0.2, 2.5); } catch (...) {}
            baseline_normalized_fiber_lengths[mi] = L0_i;
            baseline_primary_afferent_rates[mi]   = cached_primary_afferent_outputs[mi]   ? cached_primary_afferent_outputs[mi]->getValue(state)   : 0.0;
            baseline_secondary_afferent_rates[mi] = cached_secondary_afferent_outputs[mi] ? cached_secondary_afferent_outputs[mi]->getValue(state) : 0.0;
            const double fd0 = state_variable_spindle_dynamic_fusimotor[mi]->getValue(state);
            const double fs0 = state_variable_spindle_static_fusimotor[mi]->getValue(state);
            const double gs0 = 100.0 * k_gs_init * cached_minimum_excitations[mi];
            double ria0 = 0.0, rii0 = 0.0, kp0 = 1.0;
            evaluateReferenceMileusnicSpindle(mi, L0_i, 0.0, fd0, fs0, gs0, ria0, rii0, kp0);
            baseline_reference_ia_rates[mi] = ria0;
            baseline_reference_ii_rates[mi] = rii0;
            if (cached_tendon_afferent_outputs[mi]) {
                try { baseline_tendon_afferent_rates[mi] = cached_tendon_afferent_outputs[mi]->getValue(state); } catch (...) {}
            }
        }
    }

    rec.raw_spindle_ia_rate.resize(static_cast<size_t>(M));
    rec.raw_spindle_ii_rate.resize(static_cast<size_t>(M));
    rec.raw_gto_ib_rate.resize(static_cast<size_t>(M));
    rec.raw_lig_pac   = SimTK::Vec4(0.0);
    rec.raw_lig_golgi = SimTK::Vec4(0.0);
    if (get_enable_ligament_reflex() && !cached_ligaments.empty()) {
        double sum_p[4] = {0.0}, max_p[4] = {0.0}, sum_g[4] = {0.0}, max_g[4] = {0.0};
        int cnt[4] = {0};
        for (const auto& entry : cached_ligaments) {
            const int r = entry.cluster;
            ++cnt[r];
            const double lam  = entry.strain_out ? entry.strain_out->getValue(state) : 1.0;
            const double edot = entry.rate_out   ? entry.rate_out->getValue(state)   : 0.0;
            const double eps  = std::max(0.0, lam - 1.0);
            if (eps > 0.03 && edot > 1.5) {
                const double dv = edot - 1.5;
                const double sp = smoothstep01((eps - 0.03) / 0.04) * (dv / (dv + 4.0));
                sum_p[r] += sp; if (sp > max_p[r]) max_p[r] = sp;
            }
            const double th_g = std::max(0.15, entry.toe_strain - 1.0);
            if (eps > th_g) {
                const double de = eps - th_g;
                const double sg = (de * de) / (de * de + 0.175 * 0.175);
                sum_g[r] += sg; if (sg > max_g[r]) max_g[r] = sg;
            }
        }
        for (int r = 0; r < 4; ++r) {
            if (cnt[r] > 0) {
                const double inv_n = 1.0 / static_cast<double>(cnt[r]);
                rec.raw_lig_pac[r]   = 0.5 * max_p[r] + 0.5 * (sum_p[r] * inv_n);
                rec.raw_lig_golgi[r] = 0.5 * max_g[r] + 0.5 * (sum_g[r] * inv_n);
            }
        }
    }

    const SimTK::Vec3 trunk_om_W = rec.trunk_omega_world;
    const SimTK::Vec3 om_neck_est = mahony_omega_cached - (~R_est_AW) * trunk_om_W;
    double qd_pinv_c[32] = {0.0};
    for (int k = 0; k < d_cap; ++k) {
        qd_pinv_c[k] = J_omega_pinv(k, 0) * om_neck_est[0]
                     + J_omega_pinv(k, 1) * om_neck_est[1]
                     + J_omega_pinv(k, 2) * om_neck_est[2];
    }
    const double k_gs_c = get_enable_fusimotor() ? std::clamp(get_K_gamma_stat(), 0.0, 1.0) : 0.0;
    const double th_ii_c = std::max(0.0, get_thresh_spindle_ii());
    double ii_del_buf[160] = {0.0}, gs_del_buf[160] = {0.0};
    const bool has_ii_c = sampleDelayedVector(t_sim, tau_ii_del,   &HistoryRecord::raw_spindle_ii_rate,        ii_del_buf, M_cap);
    const bool has_gm_c = sampleDelayedVector(t_sim, tau_7b_gamma, &HistoryRecord::spinal_gamma_stat_commands, gs_del_buf, M_cap);

    double est_dl_c[160] = {0.0}, est_vl_c[160] = {0.0}, cun_dl_c[160] = {0.0};
    const double* j_curr_p = flat_J_curr.data();
    for (int mi = 0; mi < M_cap; ++mi) {
        est_dl_c[mi] = state_variable_spindle_length_change[mi]->getValue(state);
        est_vl_c[mi] = state_variable_spindle_velocity[mi]->getValue(state);
        double raw_ia = cached_primary_afferent_outputs[mi]   ? cached_primary_afferent_outputs[mi]->getValue(state)   : baseline_primary_afferent_rates[mi];
        double raw_ii = cached_secondary_afferent_outputs[mi] ? cached_secondary_afferent_outputs[mi]->getValue(state) : baseline_secondary_afferent_rates[mi];
        if (use_snoise) {
            applySpindleNoise(raw_ia, 24,     0.35, 0.025, mi, noise_time, s_scale);
            applySpindleNoise(raw_ii, 24 + M, 0.20, 0.015, mi, noise_time, s_scale);
        }
        rec.raw_spindle_ia_rate[mi] = raw_ia;
        rec.raw_spindle_ii_rate[mi] = raw_ii;
        if (cached_tendon_afferent_outputs[mi]) {
            double raw_ib = cached_tendon_afferent_outputs[mi]->getValue(state);
            if (use_snoise) applySpindleNoise(raw_ib, 24 + 3 * M, 0.25, 0.015, mi, noise_time, s_scale);
            rec.raw_gto_ib_rate[mi] = raw_ib;
        } else {
            rec.raw_gto_ib_rate[mi] = 0.0;
        }

        const double ii_del_v = has_ii_c ? ii_del_buf[mi] : raw_ii;
        const double fs_now = state_variable_spindle_static_fusimotor[mi]->getValue(state);
        const double gs_arr = has_gm_c ? gs_del_buf[mi] : (100.0 * k_gs_c * cached_minimum_excitations[mi]);
        const double act_m  = muscles.get(active_muscle_indices[mi]).getActivation(state);
        const double u_est  = last_alpha_motor_excitations[mi];
        const double w_t    = 0.5 * (1.0 + std::tanh((u_est - act_m) / 0.005));
        const double da_dt  = (u_est - act_m) / (0.025 + (0.010 - 0.025) * w_t);
        double r_qd = 0.0;
        const int m_base = mi * d;
        for (int k = 0; k < d_cap; ++k) r_qd += j_curr_p[m_base + k] * qd_pinv_c[k];
        const double eff_sc = std::max(1e-4, cached_optimal_fiber_lengths[mi] * cached_cosine_pennation_angles[mi]);
        const double Lp_c = get_enable_efference_cancellation() ? (-(r_qd + 0.052 * cached_tendon_slack_lengths[mi] * da_dt) / eff_sc) : 0.0;
        const double rii_c = evaluateCuneateNeutralReferenceII(baseline_normalized_fiber_lengths[mi], Lp_c, fs_now, gs_arr);
        const double eii_c = get_enable_efference_cancellation()
            ? (baseline_secondary_afferent_rates[mi] + (rii_c - baseline_reference_ii_rates[mi]))
            : baseline_secondary_afferent_rates[mi];
        const double dl_c = cached_fiber_length_scales[mi] * applyBiologicalDeadband(ii_del_v - eii_c, th_ii_c);
        cun_dl_c[mi] = computeTendonCompensatedMtuLength(dl_c, act_m, baseline_muscle_activations[mi], cached_tendon_slack_lengths[mi], get_enable_efference_cancellation());
    }

    if (get_enable_decoder() && d > 0) {
        for (int r = 0; r < d; ++r) {
            double sq = 0.0, sv = 0.0, sc = 0.0;
            for (int m = 0; m < M_cap; ++m) {
                const double dw = proprioceptive_decoder_d(r, m);
                sq += dw * est_dl_c[m];
                sv += dw * est_vl_c[m];
                sc += dw * cun_dl_c[m];
            }
            estimated_neck_q[r]  = sq;
            estimated_neck_qd[r] = sv;
            cuneate_neck_q[r]    = sc;
        }
        estimated_neck_rotvec   = mulMatVec3(J_omega_d, estimated_neck_q);
        estimated_neck_omega    = mulMatVec3(J_omega_d, estimated_neck_qd);
        cuneate_posture_rotvec  = mulMatVec3(J_omega_d, cuneate_neck_q);
        cuneate_posture_shift_m = 0.945 * mulBlendedJstaVec(b_sh_c, J_c1_d, J_com_d, cuneate_neck_q);
    } else {
        estimated_neck_q.setToZero(); estimated_neck_qd.setToZero(); cuneate_neck_q.setToZero();
        estimated_neck_rotvec = SimTK::Vec3(0.0); estimated_neck_omega = SimTK::Vec3(0.0);
        cuneate_posture_rotvec = SimTK::Vec3(0.0); cuneate_posture_shift_m = SimTK::Vec3(0.0);
    }

    const SimTK::Vec3 sh_vel_vec = (get_enable_decoder() && sh_gate_c > 1e-12)
        ? (sh_gate_c * mulBlendedJstaVec(b_sh_c, J_c1_d, J_com_d, estimated_neck_qd))
        : SimTK::Vec3(0.0);
    for (int i = 0; i < 3; ++i) {
        const double cun_ri = get_enable_decoder() ? state_variable_cuneate_rot[i]->getValue(state) : 0.0;
        rec.est_neck_rotvec_1a3[i] = evalProperLead(15 + i, cun_ri,                  tau_3b_cun, state, tau_lead_eff);
        rec.est_neck_omega_1a3[i]  = evalProperLead(18 + i, estimated_neck_omega[i], tau_3b_cun, state, tau_lead_eff);
        const double sh_p = get_enable_decoder() ? (sh_gate_c * state_variable_cuneate_shift[i]->getValue(state)) : 0.0;
        rec.cuneate_shift_3b[i]     = evalProperLead(64 + i, sh_p,          tau_4b_thal, state, tau_lead_eff);
        rec.cuneate_shift_vel_3b[i] = evalProperLead(67 + i, sh_vel_vec[i], tau_4b_thal, state, tau_lead_eff);
    }

    // 4. Brainstem VCR Covariant Tensor Pencil Wrench on so(3)*
    const double rw_env_c = state_variable_oto_reweight_env->getValue(state);
    const SimTK::Vec3 exp_up_Hc = (~R_est_HW) * SimTK::Vec3(0.0, 1.0, 0.0);
    const SimTK::Vec3 a_tr_Hc   = oto_luen_H - 9.81 * exp_up_Hc;
    const double a_hor_c = (a_tr_Hc - (~exp_up_Hc * a_tr_Hc) * exp_up_Hc).norm();
    const double rw_a0_c = std::max(0.0, get_oto_reweight_a0_mps2());
    const double rw_ef_c = std::max(rw_env_c, a_hor_c);
    const double rw_r2_c = (rw_a0_c > 1e-6) ? ((rw_ef_c * rw_ef_c) / (rw_a0_c * rw_a0_c)) : 0.0;
    const double w_oto_c = (rw_a0_c > 1e-6) ? (1.0 / (1.0 + rw_r2_c * rw_r2_c)) : 1.0;
    const double sqrt_wo_c = std::sqrt(w_oto_c);
    dbg_last_w_oto_rw = w_oto_c;

    SimTK::Vec3 tau_vcr_now(0.0);
    if (get_enable_vcr()) {
        const SimTK::Vec3 x_ton(state_variable_tonic_state[0]->getValue(state),  state_variable_tonic_state[1]->getValue(state),  state_variable_tonic_state[2]->getValue(state));
        const SimTK::Vec3 x_ti( state_variable_tonic_int[0]->getValue(state),    state_variable_tonic_int[1]->getValue(state),    state_variable_tonic_int[2]->getValue(state));
        const SimTK::Vec3 x_sc( state_variable_canal_state[0]->getValue(state),  state_variable_canal_state[1]->getValue(state),  state_variable_canal_state[2]->getValue(state));
        const SimTK::Vec3 v_sc( state_variable_canal_vel[0]->getValue(state),    state_variable_canal_vel[1]->getValue(state),    state_variable_canal_vel[2]->getValue(state));
        const SimTK::Vec3 x_ph( state_variable_phasic_state[0]->getValue(state), state_variable_phasic_state[1]->getValue(state), state_variable_phasic_state[2]->getValue(state));
        const SimTK::Vec3 v_ph( state_variable_phasic_vel[0]->getValue(state),   state_variable_phasic_vel[1]->getValue(state),   state_variable_phasic_vel[2]->getValue(state));

        SimTK::Vec3 tau_ton_part(0.0);
        if (get_enable_vcr_tonic()) {
            const double w_t = std::max(0.0, get_omega_ton());
            const SimTK::Mat33 K_ton_mat = s_supra_ctrl * projectSymmetric3x3PSD(
                get_alpha_grav_ton() * tensor_Kg3 +
                (w_t * w_t)          * tensor_I3 +
                get_gamma_musc_ton() * tensor_Gmu3 +
                get_c_disc_ton()     * tensor_Kdisc3);
            const double ki_g = s_supra_ctrl * std::max(0.0, get_ki_grav_ton());
            const SimTK::Vec3 tau_ti_raw = (ki_g > 0.0) ? (ki_g * (tensor_Kg3 * x_ti)) : SimTK::Vec3(0.0);
            tau_ton_part = -(K_ton_mat * x_ton + applyRadialTanhSat(tau_ti_raw, 3.5));
        }

        SimTK::Vec3 tau_sc_part(0.0);
        if (get_enable_vcr_canal()) {
            const double sc_boost = 1.0 + std::max(0.0, get_highg_sc_gain_boost()) * (1.0 - g_highg_ctrl);
            const SimTK::Mat33 D_sc_mat = (s_supra_ctrl * sc_boost) * projectSymmetric3x3PSD(
                std::max(0.0, get_omega_sc()) * tensor_I3 +
                get_beta_musc_sc()            * tensor_Gmu3 +
                get_c_disc_sc()               * tensor_Kdisc3 +
                get_c_grav_sc()               * tensor_Kg3);
            const double z_s = std::clamp(get_z_sc(), 0.0, 0.500);
            tau_sc_part = -(D_sc_mat * (x_sc + z_s * v_sc));
        }

        SimTK::Vec3 tau_ph_part(0.0);
        if (get_enable_vcr_phasic()) {
            constexpr double inv_g = 1.0 / 9.81;
            const SimTK::Mat33 K_phas_mat = (s_supra_ctrl * inv_g) * projectSymmetric3x3PSD(
                get_alpha_inert_phas() * tensor_Kg3 +
                get_gamma_musc_phas()  * tensor_Gmu3 +
                get_c_disc_phas()      * tensor_Kdisc3);
            const double z_p = std::clamp(get_z_phas(), 0.0, 0.500);
            const double s_fl = std::clamp(get_highg_phas_sust_floor(), 0.0, 1.0);
            const double s_mu = s_fl + (1.0 - s_fl) * std::sqrt(g_highg_ctrl);
            SimTK::Vec3 t_sust = -(s_mu * (K_phas_mat * x_ph));
            SimTK::Vec3 t_lead = -(g_highg_ctrl * z_p * (K_phas_mat * v_ph));
            const double l_sat = std::max(0.0, get_phasic_lead_sat_nm());
            if (l_sat > 1e-6) t_lead = applyRadialLinearKneeSat(t_lead, 0.90 * l_sat, l_sat);
            const double p_sat = std::max(0.0, get_phasic_torque_sat_nm());
            if (p_sat > 1e-6) t_sust = applyRadialLinearKneeSat(t_sust, 0.90 * p_sat, p_sat);
            tau_ph_part = sqrt_wo_c * (t_sust + t_lead);
            tau_ph_part = applyRadialTanhSat(tau_ph_part, std::max(0.0, get_phasic_torque_tanh_nm()));
        }
        tau_vcr_now = tau_ton_part + tau_sc_part + tau_ph_part;
    }

    for (int i = 0; i < 3; ++i) {
        rec.brainstem_vcr_torque[i] = evalProperLead(33 + i, tau_vcr_now[i], tau_5_mvst, state, tau_lead_eff);
    }

    // 5. Spinal Fiber + Tucker/SVD Synergy Ia Reflex Drive (GL(d)-Invariant Tensor Contraction, normalized per 100 pps)
    rec.spinal_ccr_drive.assign(static_cast<size_t>(M), 0.0);
    if (get_enable_ccr() && get_enable_ccr_spinal() && M_cap > 0) {
        const double k_loc = s_spinal_ctrl * std::max(0.0, get_kappa_ia_local());
        const double k_syn = s_spinal_ctrl * std::max(0.0, get_kappa_ia_synergy());
        const double z_c   = std::clamp(get_z_ccr(), 0.0, 5.0);
        const bool has_wh  = (static_cast<int>(ccr_hemispheric_weights.size()) == M);
        double ia_sig[160] = {0.0};
        double ia_bal[160] = {0.0};
        double syn_drive[160] = {0.0};
        for (int m = 0; m < M_cap; ++m) {
            const double xc = state_variable_cervico_collic[m]->getValue(state);
            const double vc = state_variable_cervico_collic_vel[m]->getValue(state);
            const double wh = has_wh ? ccr_hemispheric_weights[m] : 1.0;
            ia_sig[m] = 0.01 * std::max(0.0, xc + z_c * vc);
            ia_bal[m] = wh * ia_sig[m];
        }
        // Rank-r synergy projection on hemispherically balanced Ia drive: P_syn * ia_bal = V_syn * (V_syn^T * ia_bal)
        if (k_syn > 1e-10 && synergy_proj_V.nrow() == M) {
            const int r_cols = std::min(synergy_proj_V.ncol(), 32);
            double coeff[32] = {0.0};
            for (int c = 0; c < r_cols; ++c)
                for (int m = 0; m < M_cap; ++m) coeff[c] += synergy_proj_V(m, c) * ia_bal[m];
            for (int m = 0; m < M_cap; ++m) {
                double s = 0.0;
                for (int c = 0; c < r_cols; ++c) s += synergy_proj_V(m, c) * coeff[c];
                syn_drive[m] = std::max(0.0, s);
            }
        }
        for (int m = 0; m < M_cap; ++m) {
            rec.spinal_ccr_drive[m] = std::clamp(k_loc * ia_bal[m] + k_syn * syn_drive[m], 0.0, 1.0);
        }
    }

    // 6. Spinal Lamina IX Summing Junction: Descending Voluntary + MVST VCR + Covariant CCR + Nullspace Anti-Buckling
    DelayedData del_mvst = getDelayedData(t_sim, tau_5_mvst);
    const SimTK::Vec3 vcr_arr = (!get_enable_vcr()) ? SimTK::Vec3(0.0)
        : ((!history.empty() && del_mvst.record.valid) ? del_mvst.record.brainstem_vcr_torque : rec.brainstem_vcr_torque);
    const SimTK::Vec3 vol_arr = (!get_enable_task_controller()) ? SimTK::Vec3(0.0)
        : ((!history.empty() && del_1c_c.record.valid) ? del_1c_c.record.cortical_vol_torque : vol_tau_1c);
    static const std::vector<double> kEmptyStdVec;
    const std::vector<double>& vol_null_arr = (!get_enable_task_controller()) ? kEmptyStdVec
        : ((!history.empty() && del_1c_c.record.valid && static_cast<int>(del_1c_c.record.cortical_vol_null_wrench.size()) == d)
            ? del_1c_c.record.cortical_vol_null_wrench : w_null_vol_raw);
    const SimTK::Vec3 plan_xi_1c = (!history.empty() && del_1c_c.record.valid) ? del_1c_c.record.traj_rot_1c : traj_xi;
    const SimTK::Vec3 plan_om_1c = (!history.empty() && del_1c_c.record.valid) ? del_1c_c.record.traj_vel_1c : traj_om;
    const std::vector<double>& plan_qn_1c = (!history.empty() && del_1c_c.record.valid && static_cast<int>(del_1c_c.record.traj_q_null_1c.size()) == d)
        ? del_1c_c.record.traj_q_null_1c : q_null_c;
    const std::vector<double>& plan_qdn_1c = (!history.empty() && del_1c_c.record.valid && static_cast<int>(del_1c_c.record.traj_qd_null_1c.size()) == d)
        ? del_1c_c.record.traj_qd_null_1c : qd_null_c;

    const double vol_pos_n = (plan_xi_1c.norm() + 0.5 * stdVecNorm(plan_qn_1c)) / 0.08;
    const double vol_vel_n = (plan_om_1c.norm() + 0.5 * stdVecNorm(plan_qdn_1c)) / 0.10;
    const double vol_act_gate = e_eng_ctrl * smoothstep01(std::max(vol_pos_n, vol_vel_n));

    const bool sp_cancels_plan = get_enable_fusimotor() && get_enable_efference_cancellation();
    SimTK::Vec3 tau_ccr_3(0.0);
    if (get_enable_ccr() && get_enable_ccr_task()) {
        const SimTK::Mat33 K_ccr_mat = s_spinal_ctrl * projectSymmetric3x3PSD(
            get_c_disc_ccr()     * tensor_Kdisc3 +
            get_gamma_musc_ccr() * tensor_Gmu3 +
            get_c_grav_ccr()     * tensor_Kg3);
        const SimTK::Vec3 ref_xi = sp_cancels_plan ? SimTK::Vec3(0.0) : plan_xi_1c;
        const SimTK::Vec3 ref_om = sp_cancels_plan ? SimTK::Vec3(0.0) : plan_om_1c;
        const double pos_gate = 1.0 - 0.85 * vol_act_gate;
        const double z_c = std::clamp(get_z_ccr(), 0.0, 5.0);
        tau_ccr_3 = -(K_ccr_mat * (pos_gate * (estimated_neck_rotvec - ref_xi) + z_c * (estimated_neck_omega - ref_om)));
    }

    SimTK::Vec3 total_tau_3 = vol_arr + vcr_arr + tau_ccr_3;
    total_tau_3 = applyRadialLinearKneeSat(total_tau_3, 120.0, 150.0);

    // Chain Nullspace Curvature Anti-Buckling Wrench in ker(J_omega):
    // Penalizes deviation from the compliant chain shape P_comp * xi via K_disc,d
    double w_buckle_d[32] = {0.0};
    SimTK::Vec3 dbg_bkl_dipole(0.0);
    if (get_enable_antibuckling() && d_cap > 0) {
        const double kap_b = s_spinal_ctrl * std::max(0.0, get_kappa_buckle());
        const double bet_b = std::max(s_spinal_ctrl * std::max(0.0, get_beta_buckle()), vol_act_gate * 0.05);
        double comb_null[32] = {0.0};
        for (int r = 0; r < d_cap; ++r) {
            double qn = 0.0, qdn = 0.0;
            for (int c = 0; c < d_cap; ++c) {
                qn  += Pi_null_d(r, c) * estimated_neck_q[c];
                qdn += Pi_null_d(r, c) * estimated_neck_qd[c];
            }
            if (!sp_cancels_plan) {
                if (r < static_cast<int>(plan_qn_1c.size()))  qn  -= plan_qn_1c[r];
                if (r < static_cast<int>(plan_qdn_1c.size())) qdn -= plan_qdn_1c[r];
            }
            comb_null[r] = kap_b * qn + bet_b * qdn;
        }
        double raw_bkl[32] = {0.0};
        for (int r = 0; r < d_cap; ++r) {
            double acc = 0.0;
            for (int c = 0; c < d_cap; ++c) acc -= Kdisc_neck_d(r, c) * comb_null[c];
            raw_bkl[r] = acc;
        }
        for (int k = 0; k < d_cap; ++k) {
            double wb = 0.0;
            for (int c = 0; c < d_cap; ++c) wb += Pi_null_wrench_d(k, c) * raw_bkl[c];
            w_buckle_d[k] = wb;
            const SimTK::Vec3 jk_col(J_omega_d(0, k), J_omega_d(1, k), J_omega_d(2, k));
            dbg_bkl_dipole += (0.5 * neck_dipole_sign[k] * (-wb)) * jk_col;
        }
    }

    // Distribute 3D wrench + nullspace wrenches onto d generalized coordinates for Bounded QP:
    double target_torque_qp[32] = {0.0};
    if (d_cap > 0) {
        const double comp_sh = std::clamp(get_vcr_compliant_share(), 0.0, 1.0);
        const SimTK::Vec3 vc_sum = vol_arr + tau_ccr_3;
        double vcr_d[32] = {0.0};
        for (int r = 0; r < d_cap; ++r) {
            vcr_d[r] = J_omega_d(0, r) * vcr_arr[0] + J_omega_d(1, r) * vcr_arr[1] + J_omega_d(2, r) * vcr_arr[2];
        }
        if (comp_sh > 1e-6) {
            try {
                const SimTK::Vec3 a_vcr = tensor_I3.invert() * vcr_arr;
                double qdd_vcr[32] = {0.0}, w_in[32] = {0.0};
                for (int r = 0; r < d_cap; ++r)
                    qdd_vcr[r] = P_comp_d(r, 0) * a_vcr[0] + P_comp_d(r, 1) * a_vcr[1] + P_comp_d(r, 2) * a_vcr[2];
                for (int r = 0; r < d_cap; ++r)
                    for (int c = 0; c < d_cap; ++c) w_in[r] += M_neck_d(r, c) * qdd_vcr[c];
                for (int r = 0; r < d_cap; ++r) {
                    double nw = 0.0;
                    for (int c = 0; c < d_cap; ++c) nw += Pi_null_wrench_d(r, c) * w_in[c];
                    vcr_d[r] += comp_sh * nw;
                }
            } catch (...) {}
        }
        for (int r = 0; r < d_cap; ++r) {
            const double vc_r = J_omega_d(0, r) * vc_sum[0] + J_omega_d(1, r) * vc_sum[1] + J_omega_d(2, r) * vc_sum[2];
            const double vn_r = (r < static_cast<int>(vol_null_arr.size())) ? vol_null_arr[r] : 0.0;
            target_torque_qp[r] = vc_r + vcr_d[r] + vn_r + w_buckle_d[r];
        }
    }

    // 7. Exact d-DOF Semismooth-Newton Bounded QP Synergy Solver (Zero-Heap Stack Cholesky)
    double alpha_exc[160] = {0.0};
    for (int m = 0; m < M_cap; ++m) alpha_exc[m] = cached_minimum_excitations[m];

    double deliv_qp_d[32] = {0.0};
    if (d_cap > 0 && M_cap > 0) {
        if (static_cast<int>(scratch_A_qp.size()) < M_cap * d_cap) {
            scratch_A_qp.assign(static_cast<size_t>(M_cap * d_cap), 0.0);
        }
        double* A_qp = scratch_A_qp.data();
        double u_max_qp[160] = {0.0};
        for (int m = 0; m < M_cap; ++m) {
            double f_eff = cached_maximum_isometric_forces[m];
            if (w_exact > 0.0) {
                const Muscle& mus = muscles.get(active_muscle_indices[m]);
                const double fl = std::clamp(mus.getActiveForceLengthMultiplier(state), 0.10, 1.25);
                const double ca = std::clamp(mus.getCosPennationAngle(state), 0.50, 1.0);
                f_eff *= ((1.0 - w_exact) + w_exact * (fl * ca));
            }
            u_max_qp[m] = std::max(0.0, 1.0 - cached_minimum_excitations[m]);
            for (int r = 0; r < d_cap; ++r) A_qp[m * d_cap + r] = j_curr_p[m * d + r] * f_eff;
        }

        const double rho = 1e-3 * (1.0 + 2.0 * w_exact);
        auto evalDual = [&](const double* lam, double* du_o, double* z_o, double* g_o) -> double {
            double psi = 0.0;
            for (int r = 0; r < d_cap; ++r) {
                psi += 0.5 * rho * lam[r] * lam[r] - target_torque_qp[r] * lam[r];
                g_o[r] = rho * lam[r] - target_torque_qp[r];
            }
            for (int m = 0; m < M_cap; ++m) {
                const double* a_m = A_qp + m * d_cap;
                double z = 0.0;
                for (int r = 0; r < d_cap; ++r) z += a_m[r] * lam[r];
                z_o[m] = z;
                double du = 0.0;
                if (z >= u_max_qp[m]) {
                    du = u_max_qp[m];
                    psi += u_max_qp[m] * z - 0.5 * u_max_qp[m] * u_max_qp[m];
                } else if (z > 0.0) {
                    du = z;
                    psi += 0.5 * z * z;
                }
                du_o[m] = du;
                if (du > 0.0) {
                    for (int r = 0; r < d_cap; ++r) g_o[r] += a_m[r] * du;
                }
            }
            return psi;
        };

        double G_full[32 * 32] = {0.0};
        double H_qp[32 * 32]   = {0.0};
        for (int m = 0; m < M_cap; ++m) {
            const double* a_m = A_qp + m * d_cap;
            for (int r = 0; r < d_cap; ++r) {
                const double ar = a_m[r];
                for (int c = 0; c < d_cap; ++c) G_full[r * d_cap + c] += ar * a_m[c];
            }
        }
        for (int i = 0; i < d_cap * d_cap; ++i) H_qp[i] = G_full[i];
        for (int r = 0; r < d_cap; ++r) H_qp[r * d_cap + r] += rho;

        double lam_qp[32] = {0.0};
        if (!solveSPDCholesky(d_cap, H_qp, target_torque_qp, lam_qp)) {
            for (int r = 0; r < d_cap; ++r) lam_qp[r] = target_torque_qp[r] / std::max(1e-6, H_qp[r * d_cap + r]);
        }

        double du_qp[160] = {0.0}, z_qp[160] = {0.0}, grad_qp[32] = {0.0};
        double psi_curr = evalDual(lam_qp, du_qp, z_qp, grad_qp);
        double neg_g[32] = {0.0}, dlam[32] = {0.0}, lam_try[32] = {0.0}, du_try[160] = {0.0}, z_try[160] = {0.0}, g_try[32] = {0.0};

        for (int it = 0; it < 20; ++it) {
            double g_inf = 0.0;
            for (int r = 0; r < d_cap; ++r) g_inf = std::max(g_inf, std::abs(grad_qp[r]));
            if (g_inf < 1e-6) break;

            for (int i = 0; i < d_cap * d_cap; ++i) H_qp[i] = 0.0;
            for (int r = 0; r < d_cap; ++r) H_qp[r * d_cap + r] = rho;
            int n_free = 0;
            for (int m = 0; m < M_cap; ++m) {
                if (z_qp[m] > 0.0 && z_qp[m] < u_max_qp[m]) {
                    ++n_free;
                    const double* a_m = A_qp + m * d_cap;
                    for (int r = 0; r < d_cap; ++r) {
                        const double arm = a_m[r];
                        for (int c = 0; c < d_cap; ++c) H_qp[r * d_cap + c] += arm * a_m[c];
                    }
                }
            }
            if (n_free < d_cap) {
                for (int i = 0; i < d_cap * d_cap; ++i) H_qp[i] += 0.05 * G_full[i];
            }

            for (int r = 0; r < d_cap; ++r) neg_g[r] = -grad_qp[r];
            if (!solveSPDCholesky(d_cap, H_qp, neg_g, dlam)) {
                for (int r = 0; r < d_cap; ++r) dlam[r] = neg_g[r] / std::max(1e-6, H_qp[r * d_cap + r]);
            }

            double gd = 0.0;
            for (int r = 0; r < d_cap; ++r) gd += grad_qp[r] * dlam[r];
            if (gd >= 0.0) {
                gd = 0.0;
                for (int r = 0; r < d_cap; ++r) { dlam[r] = neg_g[r]; gd += grad_qp[r] * dlam[r]; }
            }

            bool acc = false;
            double alpha = 1.0;
            for (int ls = 0; ls < 12; ++ls) {
                for (int r = 0; r < d_cap; ++r) lam_try[r] = lam_qp[r] + alpha * dlam[r];
                const double psi_try = evalDual(lam_try, du_try, z_try, g_try);
                if (psi_try <= psi_curr + 1e-4 * alpha * gd || psi_try < psi_curr) {
                    for (int r = 0; r < d_cap; ++r) { lam_qp[r] = lam_try[r]; grad_qp[r] = g_try[r]; }
                    for (int m = 0; m < M_cap; ++m) { du_qp[m] = du_try[m]; z_qp[m] = z_try[m]; }
                    psi_curr = psi_try;
                    acc = true; break;
                }
                alpha *= 0.5;
            }
            if (!acc) break;
        }

        double max_u = 0.0;
        for (int m = 0; m < M_cap; ++m) {
            alpha_exc[m] = std::clamp(cached_minimum_excitations[m] + du_qp[m], cached_minimum_excitations[m], 1.0);
            max_u = std::max(max_u, alpha_exc[m]);
            if (du_qp[m] > 0.0) {
                const double* a_m = A_qp + m * d_cap;
                for (int r = 0; r < d_cap; ++r) deliv_qp_d[r] += a_m[r] * du_qp[m];
            }
        }
        last_qp_max_u = max_u;
    }
    if (static_cast<int>(last_alpha_motor_excitations.size()) != M) last_alpha_motor_excitations.resize(static_cast<size_t>(M));
    for (int m = 0; m < M_cap; ++m) last_alpha_motor_excitations[m] = alpha_exc[m];

    // Project delivered d-DOF wrench onto 3D skull wrench via J_omega_pinv^T (exact dual of P_comp / J_omega)
    SimTK::Vec3 deliv_3dof(0.0), tgt_3dof(0.0);
    for (int a = 0; a < 3; ++a) {
        double sd = 0.0, st = 0.0;
        for (int r = 0; r < d_cap; ++r) {
            const double jpr = J_omega_pinv(r, a);
            sd += jpr * deliv_qp_d[r];
            st += jpr * target_torque_qp[r];
        }
        deliv_3dof[a] = sd;
        tgt_3dof[a]   = st;
    }
    last_qp_torque_deficit      = tgt_3dof - deliv_3dof;
    dbg_last_qp_delivered_3dof  = deliv_3dof;
    dbg_last_vcr_torque         = vcr_arr;
    dbg_last_vol_torque         = vol_arr;
    dbg_last_ccr_torque         = tau_ccr_3;
    dbg_last_buckle_torque      = dbg_bkl_dipole;
    dbg_last_vol_shift_wrench   = f_sh_filt;
    dbg_last_vol_gate           = vol_act_gate;
    rec.voluntary_activity_gate = vol_act_gate;

    // 8. Sherrington Reciprocal Inhibition, Tier B Pool Sum, & Two-Stage Coordinate-Free Fusimotor Law
    const double recip_gain = get_enable_reciprocal_inhibition()
        ? (1.0 - vol_act_gate * (1.0 - get_K_recip_inhib()))
        : 1.0;

    const bool fus_en  = get_enable_fusimotor();
    // Planned voluntary recruitment lambda_vol for smooth fusimotor inversion (only nonzero when vol_act_gate > 0)
    double lambda_vol[32] = {0.0};
    if (fus_en && vol_act_gate > 1e-12 && d_cap > 0 && M_cap > 0) {
        double G_vol[32 * 32] = {0.0};
        for (int m = 0; m < M_cap; ++m) {
            const double fm = cached_maximum_isometric_forces[m];
            const int mb = m * d;
            for (int r = 0; r < d_cap; ++r) {
                const double jr = j_curr_p[mb + r] * fm;
                for (int c = 0; c < d_cap; ++c) G_vol[r * d_cap + c] += jr * (j_curr_p[mb + c] * fm);
            }
        }
        for (int r = 0; r < d_cap; ++r) G_vol[r * d_cap + r] += 1e-3;
        double vol_d[32] = {0.0};
        for (int r = 0; r < d_cap; ++r) {
            vol_d[r] = J_omega_d(0, r) * vol_arr[0] + J_omega_d(1, r) * vol_arr[1] + J_omega_d(2, r) * vol_arr[2]
                     + (r < static_cast<int>(vol_null_arr.size()) ? vol_null_arr[r] : 0.0);
        }
        if (!solveSPDCholesky(d_cap, G_vol, vol_d, lambda_vol)) {
            for (int r = 0; r < d_cap; ++r) lambda_vol[r] = vol_d[r] / std::max(1e-3, G_vol[r * d_cap + r]);
        }
    }

    double q_plan_d[32] = {0.0}, qd_plan_d[32] = {0.0};
    for (int k = 0; k < d_cap; ++k) {
        q_plan_d[k]  = P_comp_d(k, 0) * plan_xi_1c[0] + P_comp_d(k, 1) * plan_xi_1c[1] + P_comp_d(k, 2) * plan_xi_1c[2]
                     + (k < static_cast<int>(plan_qn_1c.size()) ? plan_qn_1c[k] : 0.0);
        qd_plan_d[k] = P_comp_d(k, 0) * plan_om_1c[0] + P_comp_d(k, 1) * plan_om_1c[1] + P_comp_d(k, 2) * plan_om_1c[2]
                     + (k < static_cast<int>(plan_qdn_1c.size()) ? plan_qdn_1c[k] : 0.0);
    }

    const double k_gd  = fus_en ? std::clamp(get_K_gamma_dyn(),  0.0, 1.0) : 0.0;
    const double k_gs  = fus_en ? std::clamp(get_K_gamma_stat(), 0.0, 1.0) : 0.0;
    const bool gto_on  = get_enable_gto_reflex();
    const bool ren_on  = get_enable_renshaw();
    const bool lig_on  = get_enable_ligament_reflex();
    const double k_gto = gto_on ? (s_spinal_ctrl * std::max(0.0, get_K_gto_ib()) * (1.0 - std::clamp(get_K_gto_vol_suppress(), 0.0, 1.0) * vol_act_gate)) : 0.0;
    const double k_ren = ren_on ? (s_spinal_ctrl * std::max(0.0, get_K_renshaw())) : 0.0;
    const double k_lig = lig_on ? (s_spinal_ctrl * std::max(0.0, get_K_lig_protective())) : 0.0;

    double lig_st[4] = {0.0};
    if (lig_on) {
        for (int r = 0; r < 4; ++r) lig_st[r] = std::max(0.0, state_variable_lig_reflex[r]->getValue(state));
    }

    constexpr double K_SR_sp = 10.4649, K_PR_sp = 0.15, L_0SR_sp = 0.04, L_0PR_sp = 0.76, R_sp = 0.46;
    constexpr double a_pow_sp = 0.3, C_L_sp = 1.0, C_S_sp = 0.42;
    constexpr double beta_0_b2_sp = 0.0822, beta_2_b2_sp = -0.046, Gamma_2_b2_sp = 0.0636;
    constexpr double tau_s_sp = 0.205, freq_b2_sp = 60.0, freq_b1_sp = 60.0, sigma_v_sp = 0.50;

    rec.spinal_motor_excitations.resize(static_cast<size_t>(M));
    rec.spinal_alpha_excitations.resize(static_cast<size_t>(M));
    rec.spinal_gamma_dyn_commands.resize(static_cast<size_t>(M));
    rec.spinal_gamma_stat_commands.resize(static_cast<size_t>(M));
    rec.expected_normalized_fiber_lengths.resize(static_cast<size_t>(M));
    rec.expected_normalized_fiber_velocities.resize(static_cast<size_t>(M));
    rec.planned_voluntary_alpha.resize(static_cast<size_t>(M));

    const double tau_5a_comp = tau_7a_alpha + 0.0100;
    const double tau_5b_comp = tau_7b_gamma;
    const bool ccr_sp_active = get_enable_ccr() && get_enable_ccr_spinal();

    for (int mi = 0; mi < M_cap; ++mi) {
        const double u_min = cached_minimum_excitations[mi];
        const double u_qp  = alpha_exc[mi];
        const int bl = NUM_FIXED_LEAD_SLOTS + 5 * mi;

        double ccr_inh = 0.0;
        if (ccr_sp_active) {
            double inh_f = 1.0;
            const double u_tr = u_min + 0.05;
            if (u_qp <= u_min) inh_f = recip_gain;
            else if (u_qp < u_tr) {
                const double tf = (u_qp - u_min) / (u_tr - u_min);
                inh_f = recip_gain + (tf * tf * (3.0 - 2.0 * tf)) * (1.0 - recip_gain);
            }
            ccr_inh = std::max(0.0, inh_f * rec.spinal_ccr_drive[mi]);
        }

        const double u_ib  = gto_on ? (k_gto * std::max(0.0, state_variable_gto_ib[mi]->getValue(state)))  : 0.0;
        const double u_ren = ren_on ? (k_ren * std::max(0.0, state_variable_renshaw[mi]->getValue(state))) : 0.0;
        double u_lig = 0.0;
        if (lig_on && lig_to_muscle_weights.nrow() == M) {
            for (int r = 0; r < 4; ++r) u_lig += lig_to_muscle_weights(mi, r) * lig_st[r];
            u_lig *= k_lig;
        }

        const double exc_raw = std::clamp(u_qp + ccr_inh + u_lig - u_ib - u_ren, u_min, 1.0);
        const double exc_7a  = std::clamp(evalProperLead(bl + 0, exc_raw, tau_5a_comp, state, tau_lead_eff), u_min, 1.0);
        rec.spinal_motor_excitations[mi] = exc_7a;
        rec.spinal_alpha_excitations[mi] = u_qp;

        double du_vol = 0.0;
        const int mb = mi * d;
        if (fus_en && vol_act_gate > 1e-12) {
            const double fm = cached_maximum_isometric_forces[mi];
            for (int k = 0; k < d_cap; ++k) du_vol += (j_curr_p[mb + k] * fm) * lambda_vol[k];
        }
        const double u_plan = fus_en ? std::clamp(u_min + vol_act_gate * std::max(0.0, du_vol), u_min, 1.0) : u_min;
        last_voluntary_alpha_excitations[mi] = u_plan;
        rec.planned_voluntary_alpha[mi] = u_plan;

        const double u_plan_7b = fus_en ? std::clamp(evalProperLead(bl + 1, u_plan, tau_5b_comp, state, tau_lead_eff), u_min, 1.0) : u_min;
        const double L0_eq = baseline_normalized_fiber_lengths[mi];
        const double eff_sc = std::max(1e-4, cached_optimal_fiber_lengths[mi] * cached_cosine_pennation_angles[mi]);

        double r_dot_q = 0.0, r_dot_qd = 0.0;
        for (int k = 0; k < d_cap; ++k) {
            const double jkm = j_curr_p[mb + k];
            r_dot_q  += jkm * q_plan_d[k];
            r_dot_qd += jkm * qd_plan_d[k];
        }
        const double dL_kin = -r_dot_q / eff_sc;
        const double dL_ten = -(0.049 * cached_tendon_slack_lengths[mi] * std::max(0.0, u_plan - u_min)) / eff_sc;
        const double L_hat  = fus_en ? std::clamp(L0_eq + dL_kin + dL_ten, 0.2, 2.5) : L0_eq;
        const double Lp_hat = fus_en ? std::clamp(-r_dot_qd / eff_sc, -20.0, 20.0) : 0.0;

        const double L_hat_7b  = fus_en ? std::clamp(evalProperLead(bl + 2, L_hat,  tau_5b_comp, state, tau_lead_eff), 0.2, 2.5)   : L0_eq;
        const double Lp_hat_7b = fus_en ? std::clamp(evalProperLead(bl + 3, Lp_hat, tau_5b_comp, state, tau_lead_eff), -20.0, 20.0) : 0.0;

        const double gs_base = 100.0 * k_gs * u_plan_7b;
        const double fs_base = (gs_base * gs_base) / (gs_base * gs_base + 3600.0);
        const double T_base  = (K_SR_sp / (K_SR_sp + K_PR_sp)) * (K_PR_sp * (L0_eq - L_0SR_sp - L_0PR_sp) + Gamma_2_b2_sp * fs_base);

        const double C_2   = (Lp_hat_7b >= 0.0) ? C_L_sp : C_S_sp;
        const double phi_v = Lp_hat_7b * std::pow(std::sqrt(Lp_hat_7b * Lp_hat_7b + 1e-4), a_pow_sp - 1.0);
        const double len_t = std::max(0.0, L_hat_7b - L_0SR_sp - (T_base / K_SR_sp) - R_sp);
        const double v_b2  = C_2 * len_t * phi_v;
        const double num_s = T_base - K_PR_sp * (L_hat_7b - L_0SR_sp - (T_base / K_SR_sp) - L_0PR_sp) - beta_0_b2_sp * v_b2;
        const double den_s = std::max(0.015, Gamma_2_b2_sp + beta_2_b2_sp * v_b2);
        const double fs_st = std::clamp(num_s / den_s, 0.0, 0.99);
        const double dfs   = std::clamp((-K_PR_sp * Lp_hat_7b) / den_s, -2.0, 2.0);
        const double u_s   = std::clamp(fs_st + tau_s_sp * dfs, 0.0, 0.99);
        const double gs_cmd = std::clamp(freq_b2_sp * std::sqrt(u_s / std::max(1e-6, 1.0 - u_s)), 0.0, 100.0);

        const double gd_tsk = 100.0 * k_gd * (0.25 + 0.75 * u_plan_7b);
        const double fd_tsk = (gd_tsk * gd_tsk) / (gd_tsk * gd_tsk + 3600.0);
        const double v_str  = std::max(0.0, Lp_hat_7b);
        const double fd_st  = std::clamp(fd_tsk * std::exp(-(v_str * v_str) / (2.0 * sigma_v_sp * sigma_v_sp)), 0.0, 0.99);
        const double gd_cmd = std::clamp(freq_b1_sp * std::sqrt(fd_st / std::max(1e-6, 1.0 - fd_st)), 0.0, 100.0);

        rec.spinal_gamma_dyn_commands[mi]            = gd_cmd;
        rec.spinal_gamma_stat_commands[mi]           = gs_cmd;
        rec.expected_normalized_fiber_lengths[mi]    = L_hat_7b;
        rec.expected_normalized_fiber_velocities[mi] = Lp_hat_7b;
    }

    if (!lead_u0_initialized) {
        lead_u0_initialized = true;
    }

    while (!history.empty() && history.back().first >= t_sim) {
        if (history_pool.size() < 64) history_pool.push_back(std::move(history.back().second));
        history.pop_back();
    }
    history.emplace_back(t_sim, std::move(rec));
    while (!history.empty() && history.front().first < t_sim - 0.025) {
        if (history_pool.size() < 64) history_pool.push_back(std::move(history.front().second));
        history.pop_front();
    }

    // 9. Ventral Root Efferent Delivery (A-alpha & A-gamma)
    double del_mot_buf[160] = {0.0}, del_gd_buf[160] = {0.0}, del_gs_buf[160] = {0.0};
    const bool has_7a = sampleDelayedVector(t_sim, tau_7a_alpha, &HistoryRecord::spinal_motor_excitations, del_mot_buf, M_cap);
    const bool has_7b =
        sampleDelayedVector(t_sim, tau_7b_gamma, &HistoryRecord::spinal_gamma_dyn_commands,  del_gd_buf, M_cap) &&
        sampleDelayedVector(t_sim, tau_7b_gamma, &HistoryRecord::spinal_gamma_stat_commands, del_gs_buf, M_cap);

    for (int mi = 0; mi < M_cap; ++mi) {
        const Muscle& mus = muscles.get(active_muscle_indices[mi]);
        double exc = has_7a ? del_mot_buf[mi] : alpha_exc[mi];
        exc = std::clamp(exc, cached_minimum_excitations[mi], 1.0);
        if (use_mnoise) applyMotorNoise(exc, mi, M, t_sim, m_scale);

        const int nc = mus.numControls();
        if (scratch_m_ctrl.size() != nc) scratch_m_ctrl.resize(nc);
        scratch_m_ctrl[0] = exc;
        if (nc >= 3) {
            scratch_m_ctrl[1] = has_7b ? del_gd_buf[mi] : (100.0 * k_gd * (0.25 + 0.75 * last_voluntary_alpha_excitations[mi]));
            scratch_m_ctrl[2] = has_7b ? del_gs_buf[mi] : (100.0 * k_gs * last_voluntary_alpha_excitations[mi]);
        }
        mus.addInControls(scratch_m_ctrl, controls);
    }
}
