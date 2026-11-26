#ifndef OPENSIM_HEAD_NECK_NEURAL_CONTROLLER_H_
#define OPENSIM_HEAD_NECK_NEURAL_CONTROLLER_H_

#include <OpenSim/OpenSim.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace OpenSim {

/**
 * Coordinate-Free Geometric & Tensor-Network Head-Neck Neuromuscular Controller.
 *
 * All sensory observers, cortical commands, brainstem reflexes (VCR), spinal reflexes (CCR/Ia/Ib/Renshaw/Ligament),
 * intersegmental chain anti-buckling, and bounded muscle synergy allocation operate intrinsically on:
 *   - SO(3) / Sp(1) & Lie algebra so(3) (resolved in the anatomical skull/trunk spatial triad [e1=X, e2=Y, e3=Z]),
 *   - Covariant (0,2) physical plant tensors { I_3(q), K_g,3(q), K_disc,3(q), G_mu,3(q) } pulled back from Simbody,
 *   - GL(d)-invariant muscle-fiber Tucker/SVD synergy projectors P_syn = V_syn * V_syn^T on R^M,
 *   - Disc-metric-orthogonal kinematic chain nullspace Pi_null = I_d - P_comp * J_omega on T_q Q.
 *
 * Zero Euler/Cardan coordinate names exist anywhere in controller properties, states, outputs, or control laws.
 */
class HeadNeckNeuralController : public Controller {
OpenSim_DECLARE_CONCRETE_OBJECT(HeadNeckNeuralController, Controller);

public:
    // --- 1. Voluntary SE(3) Command & Cortical Impedance Invariants ---
    OpenSim_DECLARE_PROPERTY(desired_rotation_vec, SimTK::Vec3,
        "Desired skull attitude rotation vector xi_d in so(3) [rad] relative to neutral (R_d = exp([xi_d x]) * R_0; default: (0,0,0))");
    OpenSim_DECLARE_PROPERTY(desired_translation_vec, SimTK::Vec3,
        "Desired horizontal skull/C1-on-trunk translation p_d in R^3 [m] (default: (0,0,0))");
    OpenSim_DECLARE_PROPERTY(w_translation, double,
        "Scalar Riemannian task-priority weight in [0, 1] for horizontal translation vs attitude (0=attitude-only; default: 0.0)");
    OpenSim_DECLARE_PROPERTY(urgency, double,
        "Unified CNS alertness & vigor dial u in [-1, 1]: -1=passive, 0=quiet/reflex baseline, (0,1]=voluntary/braced (default: 0.0)");
    OpenSim_DECLARE_PROPERTY(trajectory_tau, double,
        "Base time constant tau_0 [s] for 3rd-order critically-damped SE(3) trajectory planner (default: 0.28)");
    OpenSim_DECLARE_PROPERTY(fc_voluntary, double,
        "Cortical motor command 2nd-order low-pass cutoff frequency [Hz] (default: 2.15)");

    OpenSim_DECLARE_PROPERTY(omega_task, double,
        "Cortical SO(3) attitude impedance natural frequency [rad/s] multiplying I_3 (default: 12.0)");
    OpenSim_DECLARE_PROPERTY(zeta_task, double,
        "Cortical SO(3) attitude impedance damping ratio [-] (default: 1.05)");
    OpenSim_DECLARE_PROPERTY(c_grav_task, double,
        "Cortical SO(3) attitude stiffness multiple on gravitational Hessian K_g,3 [-] (default: 1.40)");
    OpenSim_DECLARE_PROPERTY(c_musc_task, double,
        "Cortical SO(3) attitude stiffness weight on muscle capacity tensor G_mu,3 [-] (default: 2.20)");
    OpenSim_DECLARE_PROPERTY(c_disc_task, double,
        "Cortical SO(3) attitude stiffness multiple on osteoligamentous tensor K_disc,3 [-] (default: 1.20)");
    OpenSim_DECLARE_PROPERTY(ki_task, double,
        "Cortical SO(3) attitude integral rate [1/s] multiplying task stiffness tensor (default: 2.20)");

    OpenSim_DECLARE_PROPERTY(omega_shift, double,
        "Cortical nullspace translation stiffness scale [-] multiplying K_disc,d (default: 1.20)");
    OpenSim_DECLARE_PROPERTY(zeta_shift, double,
        "Cortical nullspace translation derivative time constant [s] multiplying K_disc,d (default: 0.06)");
    OpenSim_DECLARE_PROPERTY(ki_shift, double,
        "Cortical nullspace translation integral rate [1/s] multiplying K_disc,d (default: 3.50)");
    OpenSim_DECLARE_PROPERTY(c_ff_shift, double,
        "Voluntary nullspace S-bend feedforward stiffness multiple on K_disc,d [-] (default: 1.00)");
    OpenSim_DECLARE_PROPERTY(k_trans_obs_pos, double,
        "Cortical attitude stabilization coupling from SE2(3) horizontal position [1/m] via u_up x p_obs (default: 0.0)");
    OpenSim_DECLARE_PROPERTY(k_trans_obs_vel, double,
        "Cortical attitude stabilization coupling from SE2(3) horizontal velocity [s/m] via u_up x v_obs (default: 0.0)");
    OpenSim_DECLARE_PROPERTY(k_shift_obs_pos, double,
        "SE2(3) observer horizontal position-to-shift counter-translation gain [m/m] (default: 0.0)");
    OpenSim_DECLARE_PROPERTY(k_shift_obs_vel, double,
        "SE2(3) observer horizontal velocity-to-shift counter-translation gain [s] (default: 0.0)");
    OpenSim_DECLARE_PROPERTY(shift_station_blend, double,
        "Horizontal shift station blend [0=atlanto-occipital skull origin, 1=skull COM] (default: 0.0)");
    OpenSim_DECLARE_PROPERTY(tau_obs_washout, double,
        "High-pass washout time constant [s] on SE2(3) observer position/velocity feeding shift loop (default: 4.0)");

    // --- 2. Brainstem Vestibulo-Collic Reflex (VCR) Covariant Tensor Pencil Invariants ---
    // K_ton = alpha_grav_ton * K_g,3 + omega_ton^2 * I_3 + gamma_musc_ton * G_mu,3 + c_disc_ton * K_disc,3
    OpenSim_DECLARE_PROPERTY(alpha_grav_ton, double,
        "Tonic VCR dimensionless multiple on gravitational Hessian K_g,3 (>=1 guarantees static tilt stability; default: 0.65)");
    OpenSim_DECLARE_PROPERTY(omega_ton, double,
        "Tonic VCR natural frequency [rad/s] on kinetic inertia tensor I_3 (default: 5.0)");
    OpenSim_DECLARE_PROPERTY(gamma_musc_ton, double,
        "Tonic VCR dimensionless weight on muscle capacity tensor G_mu,3 (default: 0.08)");
    OpenSim_DECLARE_PROPERTY(c_disc_ton, double,
        "Tonic VCR dimensionless multiple on osteoligamentous stiffness tensor K_disc,3 (default: 0.15)");
    OpenSim_DECLARE_PROPERTY(p_ton, double,
        "Tonic VCR 2nd-order filter primary pole [rad/s] (default: 12.0)");
    OpenSim_DECLARE_PROPERTY(ki_grav_ton, double,
        "Brainstem INC/MVN leaky verticality integrator rate [1/s] multiplying K_g,3 (identically 0 around gravity; default: 0.35)");
    OpenSim_DECLARE_PROPERTY(tau_ton_int, double,
        "Brainstem tonic verticality integrator leak time constant [s] (default: 4.0)");

    // D_sc = omega_sc * I_3 + beta_musc_sc * G_mu,3 + c_disc_sc * K_disc,3 + c_grav_sc * K_g,3
    OpenSim_DECLARE_PROPERTY(omega_sc, double,
        "Semicircular canal VCR damping rate [rad/s] multiplying kinetic inertia tensor I_3 (default: 16.0)");
    OpenSim_DECLARE_PROPERTY(beta_musc_sc, double,
        "Semicircular canal VCR damping time scale [s] multiplying muscle capacity tensor G_mu,3 (default: 0.05)");
    OpenSim_DECLARE_PROPERTY(c_disc_sc, double,
        "Semicircular canal VCR damping time scale [s] multiplying osteoligamentous tensor K_disc,3 (default: 0.05)");
    OpenSim_DECLARE_PROPERTY(c_grav_sc, double,
        "Semicircular canal VCR gravito-rotational damping time scale [s] multiplying K_g,3 (default: 0.0)");
    OpenSim_DECLARE_PROPERTY(z_sc, double,
        "Semicircular canal VCR state-space lead zero time constant [s] (default: 0.010)");
    OpenSim_DECLARE_PROPERTY(p_sc, double,
        "Semicircular canal VCR 2nd-order filter primary pole [rad/s] (default: 18.0)");
    OpenSim_DECLARE_PROPERTY(c_omega_p_sc, double,
        "Covariant natural-frequency pole boost [-] scaling p_sc by I_3^-1 * K_disc,3 (default: 0.0)");

    // Phasic otolith acts on coordinate-free cross product eta_a = u_up_head x delta_f_head in so(3):
    // K_phas = alpha_inert_phas * (K_g,3 / g) + gamma_musc_phas * (G_mu,3 / g) + c_disc_phas * (K_disc,3 / g)
    OpenSim_DECLARE_PROPERTY(alpha_inert_phas, double,
        "Phasic otolith dimensionless inertial-whiplash cancellation gain on (K_g,3 / g) * (u_up x df) (default: 0.35)");
    OpenSim_DECLARE_PROPERTY(gamma_musc_phas, double,
        "Phasic otolith dimensionless muscle-capacity gain on (G_mu,3 / g) * (u_up x df) (default: 0.05)");
    OpenSim_DECLARE_PROPERTY(c_disc_phas, double,
        "Phasic otolith dimensionless osteoligamentous gain on (K_disc,3 / g) * (u_up x df) (default: 0.0)");
    OpenSim_DECLARE_PROPERTY(z_phas, double,
        "Phasic otolith VCR state-space lead zero time constant [s] (default: 0.010)");
    OpenSim_DECLARE_PROPERTY(p_phas, double,
        "Phasic otolith VCR 2nd-order filter primary pole [rad/s] (default: 5.0)");

    // Smooth C^infty vestibular peripheral & high-G sensory reweighting invariants
    OpenSim_DECLARE_PROPERTY(oto_reweight_a0_mps2, double,
        "Somatogravic Hill-4 horizontal linear acceleration half-power scale a0 [m/s^2] (0=off; default: 10.0)");
    OpenSim_DECLARE_PROPERTY(oto_reweight_tau_s, double,
        "Somatogravic horizontal acceleration envelope recovery time constant [s] (default: 0.18)");
    OpenSim_DECLARE_PROPERTY(phasic_oto_tanh_mps2, double,
        "C^infty utricular hair-bundle tangential shear tanh ceiling [m/s^2] before macular lead (0=off; default: 16.0)");
    OpenSim_DECLARE_PROPERTY(phasic_oto_sat_mps2, double,
        "Optional linear-below-knee C^1 phasic otolith input ceiling [m/s^2] (0=off; default: 0.0)");
    OpenSim_DECLARE_PROPERTY(phasic_torque_sat_nm, double,
        "Optional linear-below-knee C^1 sustained phasic VCR torque ceiling [Nm] (0=off; default: 0.0)");
    OpenSim_DECLARE_PROPERTY(phasic_lead_sat_nm, double,
        "Optional linear-below-knee C^1 phasic VCR lead torque ceiling [Nm] (0=off; default: 0.0)");
    OpenSim_DECLARE_PROPERTY(phasic_torque_tanh_nm, double,
        "Optional C^infty post-filter phasic VCR torque tanh ceiling [Nm] (0=off; default: 0.0)");
    OpenSim_DECLARE_PROPERTY(highg_oto_knee_mps2, double,
        "High-G translational acceleration threshold [m/s^2] (0=off; default: 0.0)");
    OpenSim_DECLARE_PROPERTY(highg_transition_mps2, double,
        "High-G quadratic attenuation half-width [m/s^2] (default: 0.65)");
    OpenSim_DECLARE_PROPERTY(highg_irr_knee_ratio, double,
        "Irregular otolith excess ratio relative to highg_oto_knee_mps2 (default: 1.90)");
    OpenSim_DECLARE_PROPERTY(highg_phas_pole_boost, double,
        "High-G phasic pole boost [rad/s] scaled by (1 - g_highg) (default: 0.0)");
    OpenSim_DECLARE_PROPERTY(highg_phas_sust_floor, double,
        "High-G floor in [0, 1] on sustained phasic gain (default: 0.0)");
    OpenSim_DECLARE_PROPERTY(highg_p_sc_floor, double,
        "High-G canal pole floor [rad/s] as g_highg -> 0 (default: 60.0)");
    OpenSim_DECLARE_PROPERTY(highg_tau_lead_add_s, double,
        "High-G additive synaptic lead pole relaxation [s] scaled by (1 - g_highg) (default: 0.0)");
    OpenSim_DECLARE_PROPERTY(highg_sc_gain_boost, double,
        "High-G otolith-to-canal sensory reweighting boost [-] scaled by (1 - g_highg) (default: 0.0)");
    OpenSim_DECLARE_PROPERTY(highg_brace_phas_suppress, double,
        "Ballistic high-urgency phasic suppression in [0, 0.90] for u > 0.5 (default: 0.0)");
    OpenSim_DECLARE_PROPERTY(vcr_compliant_share, double,
        "Chain distribution blend in [0, 1] from equal-wrench J_omega^T (0) to inertial wrench M_d*P_comp (1) (default: 0.0)");

    // --- 3. Spinal Proprioceptive CCR & Muscle-Fiber Synergy Tensor Network ---
    OpenSim_DECLARE_PROPERTY(kappa_ia_local, double,
        "Homonymous 1D muscle-fiber Ia stretch reflex gain [excitation / (imp/s)] (default: 0.003)");
    OpenSim_DECLARE_PROPERTY(kappa_ia_synergy, double,
        "GL(d)-invariant Tucker/SVD muscle-synergy Ia reflex gain on P_syn = V_syn * V_syn^T (default: 0.002)");
    OpenSim_DECLARE_PROPERTY(c_disc_ccr, double,
        "Covariant spinal CCR stiffness multiple on osteoligamentous tensor K_disc,3 [-] (default: 0.10)");
    OpenSim_DECLARE_PROPERTY(gamma_musc_ccr, double,
        "Covariant spinal CCR stiffness weight on muscle capacity tensor G_mu,3 [-] (default: 0.04)");
    OpenSim_DECLARE_PROPERTY(c_grav_ccr, double,
        "Covariant spinal CCR stiffness multiple on gravitational Hessian K_g,3 [-] (default: 0.0)");
    OpenSim_DECLARE_PROPERTY(z_ccr, double,
        "Spinal CCR velocity lead time constant [s] (default: 0.008)");
    OpenSim_DECLARE_PROPERTY(p_ccr, double,
        "Spinal CCR 2nd-order interneuron filter pole [rad/s] (default: 85.0)");
    OpenSim_DECLARE_PROPERTY(K_recip_inhib, double,
        "Voluntary-gated Sherrington antagonist reciprocal inhibition floor in [0, 1] (0=full block; default: 0.0)");
    OpenSim_DECLARE_PROPERTY(K_gamma_dyn, double,
        "Dynamic fusimotor alpha-gamma coactivation gain in [0, 1] (default: 0.90)");
    OpenSim_DECLARE_PROPERTY(K_gamma_stat, double,
        "Static fusimotor alpha-gamma coactivation gain in [0, 1] (default: 0.70)");

    // --- 4. Chain Nullspace Curvature Anti-Buckling & Tier B Spinal Loops ---
    OpenSim_DECLARE_PROPERTY(kappa_buckle, double,
        "Dimensionless chain nullspace curvature stiffness multiple on K_disc,d (default: 0.15)");
    OpenSim_DECLARE_PROPERTY(beta_buckle, double,
        "Chain nullspace curvature damping time scale [s] multiplying K_disc,d (default: 0.006)");

    OpenSim_DECLARE_PROPERTY(tau_lead_pole, double,
        "Universal causal biproper 1st-order synaptic lead compensator pole [s] (default: 0.012)");

    OpenSim_DECLARE_PROPERTY(K_gto_ib, double,
        "Golgi Tendon Organ (Ib) autogenic inhibition gain [excitation / (imp/s)] (default: 0.00045)");
    OpenSim_DECLARE_PROPERTY(p_gto_ib, double,
        "Disynaptic Group Ib interneuron low-pass pole [rad/s] (default: 150.0)");
    OpenSim_DECLARE_PROPERTY(thresh_gto_ib, double,
        "C1-smooth Group Ib recruitment threshold above baseline [imp/s] (default: 0.50)");
    OpenSim_DECLARE_PROPERTY(K_gto_vol_suppress, double,
        "Voluntary-gated presynaptic suppression of Group Ib inhibition in [0, 1] (default: 0.85)");
    OpenSim_DECLARE_PROPERTY(K_renshaw, double,
        "Lamina IX Renshaw cell recurrent inhibition dimensionless gain (default: 0.16)");
    OpenSim_DECLARE_PROPERTY(tau_renshaw, double,
        "Renshaw cell membrane decay time constant [s] (default: 0.022)");
    OpenSim_DECLARE_PROPERTY(K_lig_protective, double,
        "Articular ligamento-muscular protective reflex gain (default: 0.09)");
    OpenSim_DECLARE_PROPERTY(p_lig_reflex, double,
        "Polysynaptic articular ligament reflex pole [rad/s] (default: 40.0)");

    // --- 5. Sensory Recruitment Thresholds, SE2(3) Observer & Noise Invariants ---
    OpenSim_DECLARE_PROPERTY(thresh_spindle_ia, double, "Primary (Ia) spindle modulation threshold [imp/s] (default: 0.50)");
    OpenSim_DECLARE_PROPERTY(thresh_spindle_ii, double, "Secondary (II) spindle modulation threshold [imp/s] (default: 0.25)");
    OpenSim_DECLARE_PROPERTY(thresh_vest_tilt, double, "Vestibular graviceptive tilt threshold [rad] (default: 0.002)");
    OpenSim_DECLARE_PROPERTY(thresh_vest_scc, double, "Vestibular semicircular canal velocity threshold [rad/s] (default: 0.005)");
    OpenSim_DECLARE_PROPERTY(thresh_vest_oto, double, "Vestibular otolith specific-force threshold [m/s^2] (default: 0.005)");

    OpenSim_DECLARE_PROPERTY(Kp_mahony, double, "Mahony SO(3) proportional attitude fusion gain [rad/s] (default: 0.25)");
    OpenSim_DECLARE_PROPERTY(Ki_mahony, double, "Mahony SO(3) integral velocity-storage bias lock gain [rad/s^2] (default: 0.008)");
    OpenSim_DECLARE_PROPERTY(w_oto_mahony, double, "Mahony otolith gravity weight relative to proprioceptive FK (default: 0.50)");
    OpenSim_DECLARE_PROPERTY(vs_leak_rate, double, "Velocity-storage leak rate [1/s] (default: 0.05)");
    OpenSim_DECLARE_PROPERTY(tau_vel_leak, double, "Cerebellar SE2(3) linear velocity-storage leak time constant [s] (default: 15.0)");
    OpenSim_DECLARE_PROPERTY(tau_pos_leak, double, "Cerebellar SE2(3) linear position-tether leak time constant [s] (default: 5.0)");

    // --- 6. Subsystem Enable Switches (Zero Axis-Specific Booleans; All Subsystems ON in Full Model) ---
    OpenSim_DECLARE_PROPERTY(enable_ccr, bool, "Master switch enabling Cervico-Collic Reflex (default: true)");
    OpenSim_DECLARE_PROPERTY(enable_ccr_spinal, bool, "Switch enabling fiber/synergy Ia CCR reflex drive (default: true)");
    OpenSim_DECLARE_PROPERTY(enable_ccr_task, bool, "Switch enabling covariant tensor CCR wrench feedback (default: true)");
    OpenSim_DECLARE_PROPERTY(enable_vcr, bool, "Master switch enabling Brainstem Vestibulo-Collic Reflex (default: true)");
    OpenSim_DECLARE_PROPERTY(enable_vcr_tonic, bool, "Switch enabling tonic graviceptive VCR tensor pathway (default: true)");
    OpenSim_DECLARE_PROPERTY(enable_vcr_canal, bool, "Switch enabling semicircular canal VCR tensor pathway (default: true)");
    OpenSim_DECLARE_PROPERTY(enable_vcr_phasic, bool, "Switch enabling phasic otolith VCR tensor pathway (default: true)");
    OpenSim_DECLARE_PROPERTY(enable_task_controller, bool, "Master switch enabling Cortical SE(3) task controller (default: true)");
    OpenSim_DECLARE_PROPERTY(enable_task_integral, bool, "Switch enabling Cortical SE(3) integral action (default: true)");
    OpenSim_DECLARE_PROPERTY(enable_decoder, bool, "Switch enabling proprioceptive Jacobian state decoder (default: true)");
    OpenSim_DECLARE_PROPERTY(enable_mahony, bool, "Switch enabling cerebellar SE2(3) Mahony observer (default: true)");
    OpenSim_DECLARE_PROPERTY(enable_efference_cancellation, bool, "Switch enabling spindle & vestibular efference cancellation (default: true)");
    OpenSim_DECLARE_PROPERTY(enable_fusimotor, bool, "Switch enabling two-stage dynamic/static fusimotor drive (default: true)");
    OpenSim_DECLARE_PROPERTY(enable_reciprocal_inhibition, bool, "Switch enabling Sherrington reciprocal inhibition (default: true)");
    OpenSim_DECLARE_PROPERTY(enable_antibuckling, bool, "Switch enabling chain nullspace curvature anti-buckling (default: true)");
    OpenSim_DECLARE_PROPERTY(enable_lead_compensators, bool, "Switch enabling causal biproper 1st-order branch lead compensators (default: true)");
    OpenSim_DECLARE_PROPERTY(enable_eye_afferents, bool, "Include extraocular muscles in controller state allocation (default: false)");
    OpenSim_DECLARE_PROPERTY(enable_gto_reflex, bool, "Switch enabling Group Ib Golgi Tendon Organ reflex (default: true)");
    OpenSim_DECLARE_PROPERTY(enable_renshaw, bool, "Switch enabling Lamina IX Renshaw recurrent inhibition (default: true)");
    OpenSim_DECLARE_PROPERTY(enable_ligament_reflex, bool, "Switch enabling articular ligamento-muscular reflex (default: true)");
    OpenSim_DECLARE_PROPERTY(enable_sensor_noise, bool, "Toggle Tier-1 physiological afferent noise (default: false)");
    OpenSim_DECLARE_PROPERTY(enable_motor_noise, bool, "Toggle Tier-1 physiological motor noise (default: false)");
    OpenSim_DECLARE_PROPERTY(noise_seed, int, "Deterministic PRNG seed for noise bank (default: 42)");
    OpenSim_DECLARE_PROPERTY(noise_scale, double, "Global amplitude scale for physiological noise (default: 1.0)");

    HeadNeckNeuralController();

    // --- Coordinate-Free 3D Geometric & Diagnostic Outputs (All Vec3s in Unified Spatial/Anatomical Basis [X, Y, Z]) ---
    OpenSim_DECLARE_OUTPUT(vestibular_omega, SimTK::Vec3, getVestibularAngularVelocity, SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(mahony_rotvec, SimTK::Vec3, getMahonyRotationVector, SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(mahony_omega, SimTK::Vec3, getMahonyAngularVelocity, SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(est_neck_rotvec, SimTK::Vec3, getEstimatedNeckRotationVector, SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(est_neck_omega, SimTK::Vec3, getEstimatedNeckAngularVelocity, SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(cuneate_rotvec, SimTK::Vec3, getCuneateRotationVector, SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(cuneate_shift_vec, SimTK::Vec3, getCuneateShiftVector, SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(obs_pos_vec, SimTK::Vec3, getObserverPositionVector, SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(obs_vel_vec, SimTK::Vec3, getObserverVelocityVector, SimTK::Stage::Velocity);

    OpenSim_DECLARE_OUTPUT(dbg_vcr_torque, SimTK::Vec3, getDbgVcrTorque, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(dbg_vol_torque, SimTK::Vec3, getDbgVolTorque, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(dbg_ccr_torque, SimTK::Vec3, getDbgCcrTorque, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(dbg_buckle_torque, SimTK::Vec3, getDbgBuckleTorque, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(dbg_qp_delivered_3dof, SimTK::Vec3, getDbgQpDelivered3Dof, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(dbg_oto_luen_head, SimTK::Vec3, getDbgOtoLuenHead, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(dbg_oto_irr_head, SimTK::Vec3, getDbgOtoIrrHead, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(dbg_phasic_oto_input, SimTK::Vec3, getDbgPhasicOtoInput, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(dbg_canal_irr_head, SimTK::Vec3, getDbgCanalIrrHead, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(dbg_vol_shift_wrench, SimTK::Vec3, getDbgVolShiftWrench, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(dbg_w_oto_rw, double, getDbgWOtoRw, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(dbg_vol_gate, double, getDbgVolGate, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(dbg_tensor_inertia_diag, SimTK::Vec3, getDbgInertiaDiag, SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(dbg_tensor_kgrav_diag, SimTK::Vec3, getDbgKgravDiag, SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(dbg_tensor_kdisc_diag, SimTK::Vec3, getDbgKdiscDiag, SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(dbg_tensor_gmusc_diag, SimTK::Vec3, getDbgGmuscDiag, SimTK::Stage::Velocity);

    SimTK::Vec3 getVestibularAngularVelocity(const SimTK::State& state) const;
    SimTK::Vec3 getMahonyRotationVector(const SimTK::State& state) const;
    SimTK::Vec3 getMahonyAngularVelocity(const SimTK::State& state) const;
    SimTK::Vec3 getEstimatedNeckRotationVector(const SimTK::State&) const { return estimated_neck_rotvec; }
    SimTK::Vec3 getEstimatedNeckAngularVelocity(const SimTK::State&) const { return estimated_neck_omega; }
    SimTK::Vec3 getCuneateRotationVector(const SimTK::State& state) const;
    SimTK::Vec3 getCuneateShiftVector(const SimTK::State& state) const;
    SimTK::Vec3 getObserverPositionVector(const SimTK::State& state) const;
    SimTK::Vec3 getObserverVelocityVector(const SimTK::State& state) const;

    SimTK::Vec3 getDbgVcrTorque(const SimTK::State&) const { return dbg_last_vcr_torque; }
    SimTK::Vec3 getDbgVolTorque(const SimTK::State&) const { return dbg_last_vol_torque; }
    SimTK::Vec3 getDbgCcrTorque(const SimTK::State&) const { return dbg_last_ccr_torque; }
    SimTK::Vec3 getDbgBuckleTorque(const SimTK::State&) const { return dbg_last_buckle_torque; }
    SimTK::Vec3 getDbgQpDelivered3Dof(const SimTK::State&) const { return dbg_last_qp_delivered_3dof; }
    SimTK::Vec3 getDbgOtoLuenHead(const SimTK::State&) const { return dbg_last_oto_luen_head; }
    SimTK::Vec3 getDbgOtoIrrHead(const SimTK::State&) const { return dbg_last_oto_irr_head; }
    SimTK::Vec3 getDbgPhasicOtoInput(const SimTK::State&) const { return dbg_last_phasic_oto_input; }
    SimTK::Vec3 getDbgCanalIrrHead(const SimTK::State&) const { return dbg_last_canal_irr_head; }
    SimTK::Vec3 getDbgVolShiftWrench(const SimTK::State&) const { return dbg_last_vol_shift_wrench; }
    double getDbgWOtoRw(const SimTK::State&) const { return dbg_last_w_oto_rw; }
    double getDbgVolGate(const SimTK::State&) const { return dbg_last_vol_gate; }
    SimTK::Vec3 getDbgInertiaDiag(const SimTK::State&) const { return SimTK::Vec3(tensor_I3(0,0), tensor_I3(1,1), tensor_I3(2,2)); }
    SimTK::Vec3 getDbgKgravDiag(const SimTK::State&) const { return SimTK::Vec3(tensor_Kg3(0,0), tensor_Kg3(1,1), tensor_Kg3(2,2)); }
    SimTK::Vec3 getDbgKdiscDiag(const SimTK::State&) const { return SimTK::Vec3(tensor_Kdisc3(0,0), tensor_Kdisc3(1,1), tensor_Kdisc3(2,2)); }
    SimTK::Vec3 getDbgGmuscDiag(const SimTK::State&) const { return SimTK::Vec3(tensor_Gmu3(0,0), tensor_Gmu3(1,1), tensor_Gmu3(2,2)); }

    void computeControls(const SimTK::State& state, SimTK::Vector& controls) const override;
    void computeStateVariableDerivatives(const SimTK::State& state) const override;

protected:
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    void extendConnectToModel(Model& model) override;
    void extendInitStateFromProperties(SimTK::State& state) const override;

private:
    struct CanalFilterParameters {
        double denominator_constant_a0;
        double denominator_linear_a1;
        double numerator_zero_b1;
        double state_space_output_c0;
        double state_space_output_c1;
        double direct_feedthrough_d;
    };
    CanalFilterParameters canal_filter_parameters[2];

    struct HistoryRecord {
        bool valid = false;
        SimTK::Vec3 est_head_rotvec_4b{0.0};                  // Branch 3->4 (13.0 ms): lead-compensated Mahony log(R_A^W)
        SimTK::Vec3 est_head_rotvec_4a{0.0};                  // Branch 3->2B (1.5 ms): lead-compensated Mahony log(R_A^W)
        SimTK::Vec3 canal_irr_omega_2a{0.0};                  // Branch 2A (2.0 ms): irregular canal omega in frame A
        SimTK::Vec3 canal_luen_omega_2b{0.0};                 // Branch 2B (3.0 ms): regular canal Luenberger omega in skull H
        SimTK::Vec3 oto_irr_force_2a{0.0};                    // Branch 2A (2.0 ms): irregular otolith specific force in skull H
        SimTK::Vec3 oto_luen_force_2b{0.0};                   // Branch 2B (3.0 ms): regular Luenberger specific force in skull H
        SimTK::Mat33 estimated_rotation_head_to_world{1.0};   // Observer R_hat_H^W
        SimTK::Mat33 trunk_rotation_world{1.0};               // Trunk R_T^W
        SimTK::Vec3 trunk_omega_world{0.0};                   // Trunk spatial angular velocity
        SimTK::Vec3 head_linear_velocity{0.0};                // Skull spatial linear velocity in ground
        SimTK::Vec3 est_neck_rotvec_1a3{0.0};                 // Branch 1A->3 (3.0 ms): decoded neck rotation vector J_omega * q_hat
        SimTK::Vec3 est_neck_omega_1a3{0.0};                  // Branch 1A->3 (3.0 ms): decoded neck angular velocity
        SimTK::Vec3 mahony_omega_4b{0.0};                     // Branch 3->4 (13.0 ms): fused angular velocity in frame A
        SimTK::Vec3 est_lin_vel{0.0};                         // SE2(3) linear velocity in ground
        SimTK::Vec3 est_lin_pos{0.0};                         // SE2(3) linear position in ground
        SimTK::Vec3 cuneate_shift_3b{0.0};                    // Branch 3->4 (13.0 ms): decoded horizontal station shift [m]
        SimTK::Vec3 cuneate_shift_vel_3b{0.0};                // Branch 3->4 (13.0 ms): decoded horizontal station shift speed [m/s]
        SimTK::Vec3 cortical_vol_torque{0.0};                 // Branch 1C (14.5 ms): cortical 3D wrench in frame A
        std::vector<double> cortical_vol_null_wrench;         // Branch 1C (14.5 ms): d-DOF nullspace translation wrench
        SimTK::Vec3 traj_rot_1a{0.0};                         // Branch 1A (9.0 ms): planned rotation vector in so(3)
        SimTK::Vec3 traj_vel_1b{0.0};                         // Branch 1B (10.0 ms): planned angular velocity in so(3)
        SimTK::Vec3 traj_acc_1b{0.0};                         // Branch 1B (10.0 ms): planned angular acceleration in so(3)
        SimTK::Vec3 traj_trans_acc_1b{0.0};                   // Branch 1B (10.0 ms): planned station linear accel [m/s^2]
        SimTK::Vec3 eff_tonic_tilt_3a{0.0};                   // Branch 3->2B (1.5 ms): cerebellar tonic tilt efference in so(3)
        SimTK::Vec3 eff_canal_vel_3a{0.0};                    // Branch 3->2B (1.5 ms): cerebellar canal vel efference in so(3)
        SimTK::Vec3 eff_phasic_eta_3a{0.0};                   // Branch 3->2B (1.5 ms): cerebellar phasic specific-force efference in H
        SimTK::Vec3 traj_rot_1c{0.0};                         // Branch 1C (14.5 ms): planned rotation vector at spinal cord
        SimTK::Vec3 traj_vel_1c{0.0};                         // Branch 1C (14.5 ms): planned angular velocity at spinal cord
        std::vector<double> traj_q_null_1c;                   // Branch 1C (14.5 ms): planned d-DOF nullspace S-bend q_null*
        std::vector<double> traj_qd_null_1c;                  // Branch 1C (14.5 ms): planned d-DOF nullspace S-bend qd_null*
        SimTK::Vec3 brainstem_vcr_torque{0.0};                // Path 5 (2.5 ms MVST): brainstem 3D VCR wrench in frame A
        std::vector<double> raw_spindle_ia_rate;
        std::vector<double> raw_spindle_ii_rate;
        std::vector<double> spinal_ccr_drive;
        std::vector<double> spinal_motor_excitations;
        std::vector<double> spinal_alpha_excitations;
        std::vector<double> spinal_gamma_dyn_commands;
        std::vector<double> spinal_gamma_stat_commands;
        std::vector<double> expected_normalized_fiber_lengths;
        std::vector<double> expected_normalized_fiber_velocities;
        std::vector<double> raw_gto_ib_rate;
        std::vector<double> planned_voluntary_alpha;
        SimTK::Vec4 raw_lig_pac{0.0};
        SimTK::Vec4 raw_lig_golgi{0.0};
        double voluntary_activity_gate = 0.0;
    };

    struct DelayedData {
        HistoryRecord record;
        SimTK::Vec3 linear_acceleration{0.0};
    };

    mutable std::deque<std::pair<double, HistoryRecord>> history;
    mutable std::vector<HistoryRecord> history_pool;
    DelayedData getDelayedData(double current_time, double delay_duration, bool interpolate_vectors = false, bool need_lin_acc = false) const;
    bool sampleDelayedVector(
        double current_time,
        double delay_duration,
        const std::vector<double> HistoryRecord::* field,
        double* out_buf,
        int expected_size) const;

    // --- Intrinsic Discovered Neck Manifold (d generalized coordinates, M active muscles) & Physical (0,2) Tensors ---
    mutable int num_neck_dofs = 0;
    mutable std::vector<const Coordinate*> neck_coords;
    mutable std::vector<double> neck_q0;
    mutable std::vector<double> neck_qmin;
    mutable std::vector<double> neck_qmax;
    mutable std::vector<double> neck_dipole_sign;             // +1 upper cervical, -1 lower cervical (purely from joint station height)

    mutable SimTK::Matrix J_omega_d;                          // 3 x d angular Jacobian mapping qdot -> omega_neck in frame A
    mutable SimTK::Matrix J_omega_pinv;                       // d x 3 kinematic right pseudoinverse J_w^T * (J_w * J_w^T)^-1
    mutable SimTK::Matrix J_c1_d;                             // 3 x d linear Jacobian of skull origin relative to trunk
    mutable SimTK::Matrix J_com_d;                            // 3 x d linear Jacobian of skull COM relative to trunk
    mutable SimTK::Matrix P_comp_d;                           // d x 3 compliant-equilibrium chain distribution (J_w * P_comp = I_3)
    mutable SimTK::Matrix Pi_null_d;                          // d x d compliant nullspace projector I_d - P_comp * J_w
    mutable SimTK::Matrix Pi_null_wrench_d;                   // d x d zero-net-attitude wrench projector I_d - J_w^T * J_w_pinv^T

    mutable SimTK::Matrix M_neck_d;                           // d x d constrained Riemannian mass-inertia tensor
    mutable SimTK::Matrix Kg_neck_d;                          // d x d gravitational destabilizing Hessian -nabla^2 V_g
    mutable SimTK::Matrix Kdisc_neck_d;                       // d x d passive osteoligamentous stiffness tensor
    mutable SimTK::Matrix Ddisc_neck_d;                       // d x d passive osteoligamentous damping tensor
    mutable SimTK::Matrix Gmu_neck_d;                         // d x d muscle structural capacity tensor J_l * F_max * J_l^T

    mutable SimTK::Mat33 tensor_I3{1.0};                      // 3 x 3 kinetic metric I_3 on so(3)*
    mutable SimTK::Mat33 tensor_Kg3{0.0};                     // 3 x 3 PSD gravitational Hessian K_g,3 on so(3)*
    mutable SimTK::Mat33 tensor_Kdisc3{1.0};                  // 3 x 3 osteoligamentous stiffness K_disc,3 on so(3)*
    mutable SimTK::Mat33 tensor_Gmu3{1.0};                    // 3 x 3 muscle capacity tensor G_mu,3 on so(3)*
    mutable SimTK::Mat33 tensor_Omega_sq{1.0};                // 3 x 3 intrinsic squared natural frequency I_3^-1 * K_disc,3

    mutable SimTK::Matrix moment_arm_q0;                      // d x M baseline muscle moment arms
    mutable std::vector<SimTK::Matrix> moment_arm_d1;         // d elements of (d x M) 1st-order Taylor slopes
    mutable std::vector<SimTK::Matrix> moment_arm_d2;         // d elements of (d x M) 2nd-order Taylor curvatures
    mutable std::vector<double> flat_ma_q0;                   // Contiguous M*d baseline moment arms [m*d + r]
    mutable std::vector<double> flat_ma_d1;                   // Contiguous M*d*d Taylor 1st-order slopes [(m*d + r)*d + k]
    mutable std::vector<double> flat_ma_d2;                   // Contiguous M*d*d Taylor 2nd-order curvatures [(m*d + r)*d + k]
    mutable std::vector<double> flat_J_curr;                  // Contiguous M*d evaluated moment arms [m*d + r]
    mutable SimTK::Matrix scratch_J_curr;                     // Preallocated d x M matrix for periodic SVD/decoder refresh
    mutable std::vector<double> scratch_A_qp;                 // Preallocated M*d capacity-weighted moment arms for QP
    mutable SimTK::Vector scratch_m_ctrl;                     // Preallocated muscle control vector for addInControls
    mutable SimTK::Matrix exact_moment_arm_cache;             // d x M exact OpenSim moment arms cached at large angles
    mutable double last_exact_jacobian_time = -1.0;
    mutable std::vector<double> last_exact_q;

    mutable SimTK::Matrix proprioceptive_decoder_d;           // d x M kinematic pseudoinverse -(J_l * J_l^T)^-1 * J_l
    mutable SimTK::Matrix synergy_proj_V;                     // M x r orthonormal Tucker/SVD fiber synergy basis (P_syn = V * V^T)
    mutable std::vector<double> ccr_hemispheric_weights;      // M GL(d)-invariant agonist/antagonist ray capacity weights

    mutable SimTK::Vector estimated_neck_q;                   // Decoded d-DOF neck displacement q - q0
    mutable SimTK::Vector estimated_neck_qd;                  // Decoded d-DOF neck speed qdot
    mutable SimTK::Vector cuneate_neck_q;                     // Tendon-compensated static II d-DOF neck posture
    mutable SimTK::Vec3 estimated_neck_rotvec{0.0};           // J_omega_d * estimated_neck_q in so(3)
    mutable SimTK::Vec3 estimated_neck_omega{0.0};            // J_omega_d * estimated_neck_qd in so(3)
    mutable SimTK::Vec3 cuneate_posture_rotvec{0.0};          // J_omega_d * cuneate_neck_q in so(3)
    mutable SimTK::Vec3 cuneate_posture_shift_m{0.0};         // J_sta * cuneate_neck_q in R^3 [m]
    mutable SimTK::Vec3 mahony_omega_cached{0.0};             // Cached fused angular velocity in frame A

    mutable const Body* cached_skull = nullptr;
    mutable const Body* cached_trunk = nullptr;
    mutable SimTK::Rotation baseline_R_head_to_world;
    mutable SimTK::Rotation baseline_R_trunk_to_world;
    mutable SimTK::Rotation baseline_R_head_to_trunk;         // Constant anatomical resting offset R_H0^T

    void updateDecoderAndSynergies(const SimTK::Matrix& J_curr) const;
    void evaluateMomentArmMatrix(const SimTK::State& state, double& out_w_exact) const;
    void solveNullspaceShiftIK(
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
        double& out_gamma_ik) const;

    // --- Muscle, Spindle, GTO, Renshaw, Ligament & State Caches ---
    mutable std::vector<int> active_muscle_indices;
    mutable std::vector<std::string> muscle_names;
    mutable std::vector<const StateVariable*> state_variable_cervico_collic;
    mutable std::vector<const StateVariable*> state_variable_cervico_collic_vel;
    mutable std::vector<const StateVariable*> state_variable_spindle_length_change;
    mutable std::vector<const StateVariable*> state_variable_spindle_velocity;
    mutable std::vector<const StateVariable*> state_variable_spindle_dynamic_fusimotor;
    mutable std::vector<const StateVariable*> state_variable_spindle_static_fusimotor;

    mutable const StateVariable* state_variable_semicircular_canal[6][2][2] = {{{nullptr}}};
    mutable const StateVariable* state_variable_traj_rot[3] = {nullptr};
    mutable const StateVariable* state_variable_traj_vel[3] = {nullptr};
    mutable const StateVariable* state_variable_traj_acc[3] = {nullptr};
    mutable const StateVariable* state_variable_traj_pos[3] = {nullptr};
    mutable const StateVariable* state_variable_traj_tvel[3] = {nullptr};
    mutable const StateVariable* state_variable_traj_tacc[3] = {nullptr};
    mutable const StateVariable* state_variable_rot_err_int[3] = {nullptr};
    mutable const StateVariable* state_variable_shift_err_int[3] = {nullptr};
    mutable const StateVariable* state_variable_vol_filt_pos[3] = {nullptr};
    mutable const StateVariable* state_variable_vol_filt_vel[3] = {nullptr};
    mutable const StateVariable* state_variable_vol_shift_filt_pos[3] = {nullptr};
    mutable const StateVariable* state_variable_vol_shift_filt_vel[3] = {nullptr};

    mutable const StateVariable* state_variable_luen_omega[3] = {nullptr};
    mutable const StateVariable* state_variable_vs_omega[3] = {nullptr};
    mutable const StateVariable* state_variable_highg_acc_env = nullptr;
    mutable const StateVariable* state_variable_oto_reweight_env = nullptr;
    mutable const StateVariable* state_variable_cuneate_rot[3] = {nullptr};
    mutable const StateVariable* state_variable_cuneate_shift[3] = {nullptr};
    mutable const StateVariable* state_variable_obs_pos_wash[3] = {nullptr};
    mutable const StateVariable* state_variable_obs_vel_wash[3] = {nullptr};

    mutable const StateVariable* state_variable_regular_otolith_adapt[3] = {nullptr};
    mutable const StateVariable* state_variable_regular_otolith[3] = {nullptr};
    mutable const StateVariable* state_variable_irregular_otolith_adapt[3] = {nullptr};
    mutable const StateVariable* state_variable_irregular_otolith[3] = {nullptr};
    mutable const StateVariable* state_variable_luenberger_otolith[3] = {nullptr};
    mutable const StateVariable* state_variable_luenberger_otolith_adapt[3] = {nullptr};

    mutable const StateVariable* state_variable_observer_quaternion[4] = {nullptr};
    mutable const StateVariable* state_variable_observer_velocity[3] = {nullptr};
    mutable const StateVariable* state_variable_observer_position[3] = {nullptr};

    mutable const StateVariable* state_variable_tonic_state[3] = {nullptr};
    mutable const StateVariable* state_variable_tonic_vel[3] = {nullptr};
    mutable const StateVariable* state_variable_tonic_int[3] = {nullptr};
    mutable const StateVariable* state_variable_canal_state[3] = {nullptr};
    mutable const StateVariable* state_variable_canal_vel[3] = {nullptr};
    mutable const StateVariable* state_variable_phasic_state[3] = {nullptr};
    mutable const StateVariable* state_variable_phasic_vel[3] = {nullptr};
    mutable const StateVariable* state_variable_cb_grav_adapt[3] = {nullptr};

    SimTK::Mat33 right_canal_projection_matrix;
    SimTK::Mat33 left_canal_projection_matrix;
    SimTK::Mat33 right_canal_inverse_matrix;
    SimTK::Mat33 left_canal_inverse_matrix;

    mutable std::vector<const OpenSim::Output<double>*> cached_primary_afferent_outputs;
    mutable std::vector<const OpenSim::Output<double>*> cached_secondary_afferent_outputs;
    mutable std::vector<const OpenSim::Output<double>*> cached_tendon_afferent_outputs;
    mutable std::vector<double> cached_fiber_length_scales;
    mutable std::vector<double> cached_primary_afferent_velocity_scales;
    mutable std::vector<double> cached_maximum_isometric_forces;
    mutable std::vector<double> cached_minimum_excitations;
    mutable std::vector<double> cached_optimal_fiber_lengths;
    mutable std::vector<double> cached_tendon_slack_lengths;
    mutable std::vector<double> cached_cosine_pennation_angles;
    mutable std::vector<double> baseline_normalized_fiber_lengths;
    mutable std::vector<double> baseline_reference_ia_rates;
    mutable std::vector<double> baseline_reference_ii_rates;
    mutable std::vector<double> baseline_muscle_activations;
    mutable std::vector<double> baseline_primary_afferent_rates;
    mutable std::vector<double> baseline_secondary_afferent_rates;
    mutable std::vector<double> baseline_tendon_afferent_rates;
    mutable std::vector<const StateVariable*> state_variable_gto_ib;
    mutable std::vector<const StateVariable*> state_variable_renshaw;
    mutable const StateVariable* state_variable_lig_reflex[4] = {nullptr};

    struct LigamentCacheEntry {
        const OpenSim::Output<double>* strain_out = nullptr;
        const OpenSim::Output<double>* rate_out = nullptr;
        double toe_strain = 1.15;
        int cluster = 0;
        SimTK::Vector strain_grad_d;                          // d-DOF geometric strain gradient d(strain)/dq at q0
    };
    mutable std::vector<LigamentCacheEntry> cached_ligaments;
    mutable SimTK::Matrix lig_to_muscle_weights;              // M x 4 coordinate-free protective recruitment matrix

    mutable std::vector<double> last_alpha_motor_excitations;
    mutable std::vector<double> last_voluntary_alpha_excitations;
    mutable SimTK::Vec3 last_qp_torque_deficit{0.0};
    mutable double last_qp_max_u = 0.0;

    mutable SimTK::Vec3 dbg_last_vcr_torque{0.0};
    mutable SimTK::Vec3 dbg_last_vol_torque{0.0};
    mutable SimTK::Vec3 dbg_last_ccr_torque{0.0};
    mutable SimTK::Vec3 dbg_last_buckle_torque{0.0};
    mutable SimTK::Vec3 dbg_last_qp_delivered_3dof{0.0};
    mutable SimTK::Vec3 dbg_last_oto_luen_head{0.0};
    mutable SimTK::Vec3 dbg_last_oto_irr_head{0.0};
    mutable SimTK::Vec3 dbg_last_phasic_oto_input{0.0};
    mutable SimTK::Vec3 dbg_last_canal_irr_head{0.0};
    mutable SimTK::Vec3 dbg_last_vol_shift_wrench{0.0};
    mutable double dbg_last_w_oto_rw = 1.0;
    mutable double dbg_last_vol_gate = 0.0;

    mutable bool is_initialized = false;
    mutable double last_jacobian_update_time = -1.0;

    static constexpr int NUM_FIXED_LEAD_SLOTS = 80;
    mutable std::vector<const StateVariable*> state_variable_lead_comp;
    mutable std::vector<double> lead_u0;
    mutable std::vector<double> lead_u_curr;
    mutable std::vector<double> lead_tau_p;
    mutable bool lead_u0_initialized = false;
    double evalProperLead(int slot, double u_val, double tau_d, const SimTK::State& state, double pole_override = -1.0) const;

    void initializeGeometryAndBaseline(const SimTK::State& state) const;
    void evaluateReferenceMileusnicSpindle(
        int muscle_index,
        double expected_norm_length,
        double expected_norm_velocity,
        double dynamic_fusimotor_activation,
        double static_fusimotor_activation,
        double static_gamma_pps,
        double& out_ref_ia,
        double& out_ref_ii,
        double& out_kappa_gain) const;

    // Lazy 200 Hz Delta_t-invariant OU noise bank evaluated via variance-normalized C^2 cubic B-spline
    struct BiologicalNoiseBank {
        static constexpr double kDt = 0.005;
        static constexpr int kChunkKnots = 200;
        int cached_seed = -1;
        int n_muscles = 0;
        bool include_gto = false;
        int n_channels = 0;
        int num_knots = 0;
        std::mt19937_64 rng;
        std::vector<double> alphas;
        std::vector<double> sigmas;
        std::vector<double> bspline_norms;
        std::vector<double> state_vals;
        std::vector<double> knots;

        static double tauForChannel(int ch, int n_mus) {
            if (ch < 6) return 0.015;
            if (ch < 12) return 0.012;
            if (ch < 18) return 0.020;
            if (ch < 24) return 0.015;
            if (ch < 24 + n_mus) return 0.020;
            if (ch < 24 + 2 * n_mus) return 0.030;
            if (ch < 24 + 3 * n_mus) return 0.060;
            return 0.020;
        }

        void reset(int seed, int n_mus, bool with_gto) {
            cached_seed = seed;
            n_muscles = std::max(0, n_mus);
            include_gto = with_gto;
            n_channels = 24 + (include_gto ? 4 : 3) * n_muscles;
            num_knots = 0;
            rng.seed(static_cast<std::uint64_t>(static_cast<std::uint32_t>(seed)) + 0x9e3779b97f4a7c15ULL);
            alphas.resize(n_channels);
            sigmas.resize(n_channels);
            bspline_norms.resize(n_channels);
            state_vals.resize(n_channels);
            knots.clear();
            std::normal_distribution<double> std_norm(0.0, 1.0);
            for (int ch = 0; ch < n_channels; ++ch) {
                const double tau = tauForChannel(ch, n_muscles);
                const double a = std::exp(-kDt / std::max(1e-4, tau));
                alphas[ch] = a;
                sigmas[ch] = std::sqrt(std::max(0.0, 1.0 - a * a));
                bspline_norms[ch] = 1.0 / std::sqrt(std::max(0.1, (18.0 + 16.0 * a + 2.0 * a * a) / 36.0));
                state_vals[ch] = std_norm(rng);
            }
            knots.insert(knots.end(), state_vals.begin(), state_vals.end());
            num_knots = 1;
        }

        void ensureCapacity(double t_sec, int seed, int n_mus, bool with_gto = false) {
            const int target_n_mus = std::max(0, n_mus);
            if (cached_seed != seed || n_muscles != target_n_mus || include_gto != with_gto || n_channels == 0) {
                reset(seed, target_n_mus, with_gto);
            }
            const int needed = static_cast<int>(std::floor(std::max(0.0, t_sec) / kDt)) + 4;
            if (num_knots >= needed) return;
            std::normal_distribution<double> std_norm(0.0, 1.0);
            while (num_knots < needed) {
                const int add = std::max(kChunkKnots, needed - num_knots);
                const int old_size = static_cast<int>(knots.size());
                knots.resize(static_cast<size_t>(old_size + add * n_channels));
                for (int k = 0; k < add; ++k) {
                    double* dst = knots.data() + static_cast<size_t>(old_size + k * n_channels);
                    for (int ch = 0; ch < n_channels; ++ch) {
                        state_vals[ch] = alphas[ch] * state_vals[ch] + sigmas[ch] * std_norm(rng);
                        dst[ch] = state_vals[ch];
                    }
                }
                num_knots += add;
            }
        }

        double eval(double t_sec, int ch) const {
            if (ch < 0 || ch >= n_channels || num_knots < 4) return 0.0;
            const double u = std::max(0.0, t_sec) / kDt;
            const int k = std::clamp(static_cast<int>(std::floor(u)), 0, num_knots - 3);
            const double f = u - static_cast<double>(k);
            const int k0 = std::max(0, k - 1);
            const double p0 = knots[static_cast<size_t>(k0) * n_channels + ch];
            const double p1 = knots[static_cast<size_t>(k) * n_channels + ch];
            const double p2 = knots[static_cast<size_t>(k + 1) * n_channels + ch];
            const double p3 = knots[static_cast<size_t>(k + 2) * n_channels + ch];
            const double f2 = f * f, f3 = f2 * f, omf = 1.0 - f;
            const double w0 = (omf * omf * omf) / 6.0;
            const double w1 = (3.0 * f3 - 6.0 * f2 + 4.0) / 6.0;
            const double w2 = (-3.0 * f3 + 3.0 * f2 + 3.0 * f + 1.0) / 6.0;
            const double w3 = f3 / 6.0;
            return bspline_norms[ch] * (w0 * p0 + w1 * p1 + w2 * p2 + w3 * p3);
        }
    };
    mutable BiologicalNoiseBank noise_bank;

    void applyVestibularCanalNoise(double& val, double pass_gain, int ch_idx, double base_sd, double rel_sd, double t_sec, double s_scale) const {
        if (s_scale <= 0.0 || std::abs(pass_gain) < 1e-12) return;
        const double r_phys = val / pass_gain;
        val += pass_gain * s_scale * std::hypot(base_sd, rel_sd * r_phys) * noise_bank.eval(t_sec, ch_idx);
    }
    void applyRegularOtolithNoise(double& rate, double pass_gain, int axis_idx, double t_sec, double s_scale) const {
        if (s_scale <= 0.0 || std::abs(pass_gain) < 1e-12) return;
        const double eta_reg = 0.5 * (noise_bank.eval(t_sec, 12 + axis_idx) + noise_bank.eval(t_sec, 15 + axis_idx));
        const double a_phys = rate / pass_gain;
        rate += pass_gain * s_scale * std::hypot(0.030, 0.004 * a_phys) * eta_reg;
    }
    void applyIrregularOtolithForceNoise(double& force_phys, int axis_idx, double t_sec, double s_scale) const {
        if (s_scale <= 0.0) return;
        const double eta_irr = 0.5 * (noise_bank.eval(t_sec, 18 + axis_idx) + noise_bank.eval(t_sec, 21 + axis_idx));
        force_phys += s_scale * std::hypot(0.055, 0.015 * force_phys) * eta_irr;
    }
    void applySpindleNoise(double& rate, int ch_offset, double base_sd, double rel_sd, int mus_idx, double t_sec, double s_scale) const {
        if (s_scale <= 0.0 || rate <= 1e-9) return;
        const double active_env = rate / (rate + 1.5);
        const double sig = s_scale * std::hypot(base_sd * active_env, rel_sd * rate);
        rate = std::max(0.0, rate + sig * noise_bank.eval(t_sec, ch_offset + mus_idx));
    }
    void applyMotorNoise(double& exc, int mus_idx, int n_mus, double t_sec, double m_scale) const {
        if (m_scale <= 0.0) return;
        const double fmax_m = (mus_idx >= 0 && mus_idx < static_cast<int>(cached_maximum_isometric_forces.size()))
            ? cached_maximum_isometric_forces[mus_idx] : 60.0;
        const double mot_sz = std::clamp(std::sqrt(60.0 / std::max(5.0, fmax_m)), 0.80, 2.00);
        const double mot_sig = m_scale * mot_sz * std::hypot(0.0015, 0.035 * exc);
        exc = std::clamp(exc + mot_sig * noise_bank.eval(t_sec, 24 + 2 * n_mus + mus_idx), 0.0, 1.0);
    }
};

} // namespace OpenSim

#endif // OPENSIM_HEAD_NECK_NEURAL_CONTROLLER_H_
