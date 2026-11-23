#ifndef OPENSIM_HEAD_NECK_NEURAL_CONTROLLER_H_
#define OPENSIM_HEAD_NECK_NEURAL_CONTROLLER_H_

#include <OpenSim/OpenSim.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <map>
#include <random>
#include <utility>
#include <vector>

namespace OpenSim {

class HeadNeckNeuralController : public Controller {
OpenSim_DECLARE_CONCRETE_OBJECT(HeadNeckNeuralController, Controller);

public:

    OpenSim_DECLARE_PROPERTY(Kp_proprioception, double, "Gain for 72-muscle spinal CCR (default: 0.050)");
    OpenSim_DECLARE_PROPERTY(Kp_proprioception_roll, double, "Roll Gain for 72-muscle spinal CCR (default: 0.050)");
    OpenSim_DECLARE_PROPERTY(Kp_proprioception_yaw, double, "Yaw Gain for 72-muscle spinal CCR (default: 0.050)");
    OpenSim_DECLARE_PROPERTY(z_ccr, double, "Pitch Cervico-Collic Reflex TF Zero in s (default: 0.008)");
    OpenSim_DECLARE_PROPERTY(p_ccr, double, "Pitch Cervico-Collic Reflex TF Pole in rad/s (default: 99.919586)");
    OpenSim_DECLARE_PROPERTY(p_ccr_roll, double, "Roll Cervico-Collic Reflex TF Pole in rad/s (default: 99.919586)");
    OpenSim_DECLARE_PROPERTY(p_ccr_yaw, double, "Yaw Cervico-Collic Reflex TF Pole in rad/s (default: 99.919586)");
    OpenSim_DECLARE_PROPERTY(K_recip_inhib, double, "Reciprocal inhibition fraction for antagonists (0.0=full block, 1.0=no block)");

    // Voluntary command & cortical 2 Hz low-pass planner
    OpenSim_DECLARE_PROPERTY(desired_yaw, double, "Voluntary desired yaw angle for the head (rad)");
    OpenSim_DECLARE_PROPERTY(desired_pitch, double, "Voluntary desired pitch angle for the head (rad)");
    OpenSim_DECLARE_PROPERTY(desired_roll, double, "Voluntary desired roll angle for the head (rad)");
    OpenSim_DECLARE_PROPERTY(desired_ap_shift, double, "Voluntary desired AP head-on-trunk shift in m (+ anterior protraction, - retraction; default: 0.0)");
    OpenSim_DECLARE_PROPERTY(desired_lat_shift, double, "Voluntary desired lateral head-on-trunk shift in m (+ rightward glide, - leftward glide; default: 0.0)");
    OpenSim_DECLARE_PROPERTY(trajectory_tau, double, "Time constant for the internal 2nd-order trajectory planner");
    OpenSim_DECLARE_PROPERTY(fc_voluntary, double, "Voluntary controller low-pass cutoff frequency in Hz (default: 2.0)");
    OpenSim_DECLARE_PROPERTY(tau_lead_pole, double, "Pitch synaptic pole time constant in s for causal biproper 1st-order lead compensators (default: 0.008)");
    OpenSim_DECLARE_PROPERTY(tau_lead_pole_roll, double, "Roll synaptic pole time constant in s for causal biproper 1st-order lead compensators (default: 0.008)");
    OpenSim_DECLARE_PROPERTY(tau_lead_pole_yaw, double, "Yaw synaptic pole time constant in s for causal biproper 1st-order lead compensators (default: 0.008)");

    // Per-Axis Brainstem VCR & Task-Space Decoder CCR Properties (Pitch, Roll, Yaw — explicit non-negative parameters)
    OpenSim_DECLARE_PROPERTY(K_ton, double, "Pitch Tonic otolith TF Gain (default: 4.5)");
    OpenSim_DECLARE_PROPERTY(p_ton, double, "Pitch Tonic otolith TF Pole in rad/s (default: 16.227225)");
    OpenSim_DECLARE_PROPERTY(K_sc, double, "Pitch Semicircular canal TF Gain (default: 5.20)");
    OpenSim_DECLARE_PROPERTY(z_sc, double, "Pitch Semicircular canal TF Zero in s (default: 0.013)");
    OpenSim_DECLARE_PROPERTY(p_sc, double, "Pitch Semicircular canal TF Pole in rad/s (default: 55.0)");
    OpenSim_DECLARE_PROPERTY(K_phas, double, "Pitch Phasic otolith TF Gain (default: 11.2)");
    OpenSim_DECLARE_PROPERTY(z_phas, double, "Pitch Phasic otolith TF Zero in s (default: 0.008)");
    OpenSim_DECLARE_PROPERTY(p_phas, double, "Pitch Phasic otolith TF Pole in rad/s (default: 4.711937)");
    OpenSim_DECLARE_PROPERTY(K_ton_roll, double, "Roll Tonic otolith TF Gain (default: 1.15)");
    OpenSim_DECLARE_PROPERTY(K_ton_yaw, double, "Yaw Tonic attitude TF Gain (default: 6.2)");
    OpenSim_DECLARE_PROPERTY(p_ton_roll, double, "Roll Tonic otolith TF Pole in rad/s (default: 3.2)");
    OpenSim_DECLARE_PROPERTY(p_ton_yaw, double, "Yaw Tonic attitude TF Pole in rad/s (default: 16.227225)");
    OpenSim_DECLARE_PROPERTY(K_sc_roll, double, "Roll Semicircular canal TF Gain (default: 3.5)");
    OpenSim_DECLARE_PROPERTY(K_sc_yaw, double, "Yaw Semicircular canal TF Gain (default: 1.32)");
    OpenSim_DECLARE_PROPERTY(z_sc_roll, double, "Roll Semicircular canal TF Zero in s (default: 0.018)");
    OpenSim_DECLARE_PROPERTY(z_sc_yaw, double, "Yaw Semicircular canal TF Zero in s (default: 0.016)");
    OpenSim_DECLARE_PROPERTY(p_sc_roll, double, "Roll Semicircular canal TF Pole in rad/s (default: 60.0)");
    OpenSim_DECLARE_PROPERTY(p_sc_yaw, double, "Yaw Semicircular canal TF Pole in rad/s (default: 3.5)");
    OpenSim_DECLARE_PROPERTY(K_phas_roll, double, "Roll Phasic otolith TF Gain (default: 11.0)");
    OpenSim_DECLARE_PROPERTY(z_phas_roll, double, "Roll Phasic otolith TF Zero in s (default: 0.012)");
    OpenSim_DECLARE_PROPERTY(p_phas_roll, double, "Roll Phasic otolith TF Pole in rad/s (default: 5.747)");
    OpenSim_DECLARE_PROPERTY(phasic_oto_sat_mps2, double, "Optional irregular otolith afferent linear-below-knee C^1 saturation in m/s^2 (0=off, default: 0.0)");
    OpenSim_DECLARE_PROPERTY(phasic_torque_sat_nm, double, "Sustained vestibulospinal phasic VCR torque linear-below-knee C^1 ceiling in Nm (0=off, default: 8.5)");
    OpenSim_DECLARE_PROPERTY(Kp_prop_pitch, double, "Pitch Spindle-Decoded CCR Proportional Gain in Nm/rad (default: 1.10)");
    OpenSim_DECLARE_PROPERTY(Kp_prop_roll, double, "Roll Spindle-Decoded CCR Proportional Gain in Nm/rad (default: 2.45)");
    OpenSim_DECLARE_PROPERTY(Kp_prop_yaw, double, "Yaw Spindle-Decoded CCR Proportional Gain in Nm/rad (default: 14.0)");
    OpenSim_DECLARE_PROPERTY(z_ccr_roll, double, "Roll Spindle-Decoded CCR & Muscle Lead Zero in s (default: 0.005)");
    OpenSim_DECLARE_PROPERTY(z_ccr_yaw, double, "Yaw Spindle-Decoded CCR & Muscle Lead Zero in s (default: 0.050)");

    // Gamma motor neuron drive (Alpha-Gamma Coactivation)
    OpenSim_DECLARE_PROPERTY(K_gamma_dyn, double, "Gain for Vestibular-driven Gamma_dyn");
    OpenSim_DECLARE_PROPERTY(K_gamma_stat, double, "Gain for Alpha-Gamma Coactivation");

    // Explicit Cortical Task-Space PID Gains (Pitch, Roll, Yaw)
    OpenSim_DECLARE_PROPERTY(Kp_task_pitch, double, "Explicit Task Kp for pitch in Nm/rad (default: 3.2)");
    OpenSim_DECLARE_PROPERTY(Kd_task_pitch, double, "Explicit Task Kd for pitch in Nms/rad (default: 1.45)");
    OpenSim_DECLARE_PROPERTY(Kp_task_roll, double, "Explicit Task Kp for roll in Nm/rad (default: 1.55)");
    OpenSim_DECLARE_PROPERTY(Kd_task_roll, double, "Explicit Task Kd for roll in Nms/rad (default: 1.35)");
    OpenSim_DECLARE_PROPERTY(Kp_task_yaw, double, "Explicit Task Kp for yaw in Nm/rad (default: 0.55)");
    OpenSim_DECLARE_PROPERTY(Kd_task_yaw, double, "Explicit Task Kd for yaw in Nms/rad (default: 0.05)");
    OpenSim_DECLARE_PROPERTY(Ki_task_pitch, double, "Explicit Task Ki for pitch in Nm/(rad*s) (default: 0.45)");
    OpenSim_DECLARE_PROPERTY(Ki_task_roll, double, "Explicit Task Ki for roll in Nm/(rad*s) (default: 1.15)");
    OpenSim_DECLARE_PROPERTY(Ki_task_yaw, double, "Explicit Task Ki for yaw in Nm/(rad*s) (default: 0.05)");

    // Cortical voluntary roll & pitch stabilization driven by cerebellar SE2(3) velocity/position (obs_v*, obs_p*)
    OpenSim_DECLARE_PROPERTY(Kv_trans_roll, double, "Voluntary roll torque gain on SE2(3) lateral velocity obs_vz in Nm/(m/s) (default: 0.0)");
    OpenSim_DECLARE_PROPERTY(Kp_trans_roll, double, "Voluntary roll torque gain on SE2(3) lateral position obs_pz in Nm/m (default: 0.0)");
    OpenSim_DECLARE_PROPERTY(Kv_trans_pitch, double, "Voluntary pitch torque gain on SE2(3) AP velocity obs_vx in Nm/(m/s) (default: 0.0)");
    OpenSim_DECLARE_PROPERTY(Kp_trans_pitch, double, "Voluntary pitch torque gain on SE2(3) AP position obs_px in Nm/m (default: 0.0)");
    OpenSim_DECLARE_PROPERTY(tau_vel_leak, double, "Cerebellar SE2(3) linear velocity-storage leak time constant in s (default: 15.0)");
    OpenSim_DECLARE_PROPERTY(tau_pos_leak, double, "Cerebellar SE2(3) egocentric linear position-tether leak time constant in s (default: 5.0)");

    // Cortical horizontal head-on-trunk shift PID gains, 2x2 IK weights & SE2(3) counter-translation couplings
    OpenSim_DECLARE_PROPERTY(Kp_shift_ap, double, "Cortical AP shift differential proportional gain Kp_shift_ap in Nm/rad (default: 4.5)");
    OpenSim_DECLARE_PROPERTY(Kd_shift_ap, double, "Cortical AP shift differential derivative gain Kd_shift_ap in Nms/rad (default: 0.45)");
    OpenSim_DECLARE_PROPERTY(Ki_shift_ap, double, "Cortical AP shift differential integral gain Ki_shift_ap in Nm/(rad*s) (default: 6.0)");
    OpenSim_DECLARE_PROPERTY(Kp_shift_lat, double, "Cortical lateral shift differential proportional gain Kp_shift_lat in Nm/rad (default: 4.5)");
    OpenSim_DECLARE_PROPERTY(Kd_shift_lat, double, "Cortical lateral shift differential derivative gain Kd_shift_lat in Nms/rad (default: 0.45)");
    OpenSim_DECLARE_PROPERTY(Ki_shift_lat, double, "Cortical lateral shift differential integral gain Ki_shift_lat in Nm/(rad*s) (default: 6.0)");
    OpenSim_DECLARE_PROPERTY(Kp_shift_obs_px, double, "SE2(3) AP position-to-shift counter-translation gain (obs_px -> desired AP shift) in m/m (default: 0.0)");
    OpenSim_DECLARE_PROPERTY(Kv_shift_obs_vx, double, "SE2(3) AP velocity-to-shift counter-translation gain (obs_vx -> desired AP shift) in m/(m/s) (default: 0.0)");
    OpenSim_DECLARE_PROPERTY(Kp_shift_obs_pz, double, "SE2(3) lateral position-to-shift counter-translation gain (obs_pz -> desired Lat shift) in m/m (default: 0.0)");
    OpenSim_DECLARE_PROPERTY(Kv_shift_obs_vz, double, "SE2(3) lateral velocity-to-shift counter-translation gain (obs_vz -> desired Lat shift) in m/(m/s) (default: 0.0)");
    OpenSim_DECLARE_PROPERTY(w_pos_ap, double, "2x2 weighted IK weight for AP head-on-trunk shift vs attitude-only control (0=attitude-only; default: 1.0)");
    OpenSim_DECLARE_PROPERTY(w_pos_lat, double, "2x2 weighted IK weight for lateral head-on-trunk shift vs attitude-only control (0=attitude-only; default: 1.0)");
    OpenSim_DECLARE_PROPERTY(shift_station_blend, double, "Horizontal shift station blend [0=C1/C0 atlanto-occipital pivot, 1=skull COM] (default: 0.0)");
    OpenSim_DECLARE_PROPERTY(tau_obs_washout, double, "First-order high-pass washout time constant on obs_px/obs_pz feeding C1 shift loop in s (default: 4.0)");
    OpenSim_DECLARE_PROPERTY(vol_kp_shift_ap, double, "Voluntary-gated AP differential proportional gain floor in Nm/rad (default: 6.5)");
    OpenSim_DECLARE_PROPERTY(vol_kp_shift_lat, double, "Voluntary-gated lateral differential proportional gain floor in Nm/rad (default: 6.5)");
    OpenSim_DECLARE_PROPERTY(vol_ki_shift_ap, double, "Voluntary-gated AP differential integral gain floor in Nm/(rad*s) (default: 40.0)");
    OpenSim_DECLARE_PROPERTY(vol_ki_shift_lat, double, "Voluntary-gated lateral differential integral gain floor in Nm/(rad*s) (default: 16.0)");
    OpenSim_DECLARE_PROPERTY(vol_diff_ff_stiff_ap, double, "Voluntary-gated AP differential stiffness feedforward in Nm/rad (default: 5.5)");
    OpenSim_DECLARE_PROPERTY(vol_diff_ff_stiff_lat, double, "Voluntary-gated lateral differential stiffness feedforward in Nm/rad (default: 7.0)");
    OpenSim_DECLARE_PROPERTY(vol_diff_ff_vel_ap, double, "Voluntary-gated AP differential velocity feedforward in Nms/rad (default: 0.0)");
    OpenSim_DECLARE_PROPERTY(vol_diff_ff_vel_lat, double, "Voluntary-gated lateral differential velocity feedforward in Nms/rad (default: 0.0)");
    OpenSim_DECLARE_PROPERTY(urgency, double, "Unified CNS alertness & movement vigor dial u in [-1.0, 1.0]: -1.0=passive neck, (-1,0)=hypo-alert/relaxed, 0.0=alert APEO baseline, (0,1]=braced/ballistic fast tracking (default: 0.0)");
    OpenSim_DECLARE_PROPERTY(highg_oto_knee_mps2, double, "Linear-below-knee translational acceleration threshold [m/s^2] above which high-G otolith reweighting & shock damping engage (default: 7.80, strictly above APEO).");

    // Intersegmental 6-DOF QP anti-buckling gains (acting in nullspace of q_1 + q_2)
    OpenSim_DECLARE_PROPERTY(Kp_buckle, double, "6-DOF QP intersegmental anti-buckling proportional gain in Nm/rad (default: 1.5)");
    OpenSim_DECLARE_PROPERTY(Kp_buckle_roll, double, "Roll 6-DOF QP intersegmental anti-buckling proportional gain in Nm/rad (default: 1.5)");
    OpenSim_DECLARE_PROPERTY(Kp_buckle_yaw, double, "Yaw 6-DOF QP intersegmental anti-buckling proportional gain in Nm/rad (default: 1.5)");
    OpenSim_DECLARE_PROPERTY(Kd_buckle, double, "Pitch 6-DOF QP intersegmental anti-buckling derivative gain in Nms/rad (default: 0.15)");
    OpenSim_DECLARE_PROPERTY(Kd_buckle_roll, double, "Roll 6-DOF QP intersegmental anti-buckling derivative gain in Nms/rad (default: 0.15)");
    OpenSim_DECLARE_PROPERTY(Kd_buckle_yaw, double, "Yaw 6-DOF QP intersegmental anti-buckling derivative gain in Nms/rad (default: 0.15)");

    // Biological sensory receptor recruitment thresholds (C1-smooth deadbands; literature defaults)
    OpenSim_DECLARE_PROPERTY(thresh_spindle_ia, double, "Primary (Ia) spindle discharge modulation threshold in imp/s (Matthews & Stein 1969; default: 0.50)");
    OpenSim_DECLARE_PROPERTY(thresh_spindle_ii, double, "Secondary (II) spindle discharge modulation threshold in imp/s (Matthews & Stein 1969; default: 0.25)");
    OpenSim_DECLARE_PROPERTY(thresh_vest_tilt, double, "Vestibular nuclei graviceptive tilt threshold in rad (Bringoux et al. 2002; default: 0.002)");
    OpenSim_DECLARE_PROPERTY(thresh_vest_scc, double, "Vestibular nuclei semicircular canal velocity threshold in rad/s (Massot et al. 2011; default: 0.005)");
    OpenSim_DECLARE_PROPERTY(thresh_vest_oto, double, "Vestibular nuclei otolith specific-force threshold in m/s^2 (default: 0.005)");

    // Explicit branch/feature diagnostic switches (all default true so full system always runs; false only when explicitly ablated and reported)
    OpenSim_DECLARE_PROPERTY(enable_axis_pitch, bool, "Master switch enabling pitch axis control/reflex pathways (default: true)");
    OpenSim_DECLARE_PROPERTY(enable_axis_roll, bool, "Master switch enabling roll axis control/reflex pathways (default: true)");
    OpenSim_DECLARE_PROPERTY(enable_axis_yaw, bool, "Master switch enabling yaw axis control/reflex pathways (default: true)");

    OpenSim_DECLARE_PROPERTY(enable_ccr, bool, "Master switch enabling Cervico-Collic Reflex (spinal Ia + task-decoded CCR) (default: true)");
    OpenSim_DECLARE_PROPERTY(enable_ccr_spinal, bool, "Switch enabling 72-muscle spinal Ia CCR reflex drive (default: true)");
    OpenSim_DECLARE_PROPERTY(enable_ccr_task, bool, "Switch enabling task-space decoded CCR torque feedback (default: true)");
    OpenSim_DECLARE_PROPERTY(enable_ccr_pitch, bool, "Switch enabling pitch CCR pathways (default: true)");
    OpenSim_DECLARE_PROPERTY(enable_ccr_roll, bool, "Switch enabling roll CCR pathways (default: true)");
    OpenSim_DECLARE_PROPERTY(enable_ccr_yaw, bool, "Switch enabling yaw CCR pathways (default: true)");

    OpenSim_DECLARE_PROPERTY(enable_vcr, bool, "Master switch enabling Brainstem Vestibulo-Collic Reflex (VCR) (default: true)");
    OpenSim_DECLARE_PROPERTY(enable_vcr_tonic, bool, "Switch enabling tonic graviceptive tilt VCR filter (default: true)");
    OpenSim_DECLARE_PROPERTY(enable_vcr_canal, bool, "Switch enabling semicircular canal angular-velocity VCR filter (default: true)");
    OpenSim_DECLARE_PROPERTY(enable_vcr_phasic, bool, "Switch enabling irregular otolith phasic acceleration VCR filter (default: true)");
    OpenSim_DECLARE_PROPERTY(enable_vcr_pitch, bool, "Switch enabling pitch VCR pathway (default: true)");
    OpenSim_DECLARE_PROPERTY(enable_vcr_roll, bool, "Switch enabling roll VCR pathway (default: true)");
    OpenSim_DECLARE_PROPERTY(enable_vcr_yaw, bool, "Switch enabling yaw VCR pathway (default: true)");

    OpenSim_DECLARE_PROPERTY(enable_task_controller, bool, "Master switch enabling Cortical Task-Space PID + feedforward controller (default: true)");
    OpenSim_DECLARE_PROPERTY(enable_task_integral, bool, "Switch enabling cortical task-space integral action Ki_task_* (default: true)");
    OpenSim_DECLARE_PROPERTY(enable_task_pitch, bool, "Switch enabling pitch cortical task controller (default: true)");
    OpenSim_DECLARE_PROPERTY(enable_task_roll, bool, "Switch enabling roll cortical task controller (default: true)");
    OpenSim_DECLARE_PROPERTY(enable_task_yaw, bool, "Switch enabling yaw cortical task controller (default: true)");

    OpenSim_DECLARE_PROPERTY(enable_decoder, bool, "Switch enabling 72-muscle 6-DOF proprioceptive state decoder (default: true)");
    OpenSim_DECLARE_PROPERTY(enable_mahony, bool, "Switch enabling cerebellar Mahony spatial attitude/velocity observer (default: true)");
    OpenSim_DECLARE_PROPERTY(enable_efference_cancellation, bool, "Switch enabling internal spindle & vestibular efference-copy cancellation (default: true)");
    OpenSim_DECLARE_PROPERTY(enable_fusimotor, bool, "Switch enabling dynamic gamma_d and static gamma_s fusimotor drive (default: true)");
    OpenSim_DECLARE_PROPERTY(enable_reciprocal_inhibition, bool, "Switch enabling voluntary-gated Sherrington reciprocal inhibition on antagonists (default: true)");
    OpenSim_DECLARE_PROPERTY(enable_antibuckling, bool, "Switch enabling 6-DOF QP nullspace intersegmental anti-buckling torque (default: true)");
    OpenSim_DECLARE_PROPERTY(enable_lead_compensators, bool, "Switch enabling causal biproper 1st-order branch lead compensators evalProperLead (default: true)");

    OpenSim_DECLARE_PROPERTY(enable_eye_afferents, bool, "If true, includes extraocular eye muscles (r_*, l_*) in controller state allocation (default: false)");

    // Physiological sensory & motor noise controls (default: false = exact deterministic baseline)
    OpenSim_DECLARE_PROPERTY(enable_sensor_noise, bool, "Toggle Tier-1 physiological vestibular & spindle afferent noise (default: false)");
    OpenSim_DECLARE_PROPERTY(enable_motor_noise, bool, "Toggle Tier-1 physiological 72-muscle alpha-motoneuron SDN (default: false)");
    OpenSim_DECLARE_PROPERTY(noise_seed, int, "Deterministic PRNG seed for physiological noise bank (default: 42)");
    OpenSim_DECLARE_PROPERTY(noise_scale, double, "Global amplitude scale for physiological noise (1.0 = healthy Tier 1, default: 1.0)");
    OpenSim_DECLARE_PROPERTY(Kp_mahony, double, "Mahony SO(3) proportional attitude fusion gain [rad/s] (default: 0.25).");
    OpenSim_DECLARE_PROPERTY(Ki_mahony, double, "Mahony SO(3) integral velocity-storage bias lock gain [rad/s^2] (default: 0.008).");
    OpenSim_DECLARE_PROPERTY(w_oto_mahony, double, "Inverse-variance Mahony weight on otolith gravity relative to pooled cervical proprioception (default: 0.50).");
    OpenSim_DECLARE_PROPERTY(vs_leak_rate, double, "Physiological velocity-storage leak rate [1/s] (tau_vs = 20 s -> 0.05, default: 0.05).");

    HeadNeckNeuralController();
    
    OpenSim_DECLARE_OUTPUT(omega_recon_pitch, double, getReconstructedCanalAngularVelocityPitch, SimTK::Stage::Dynamics);
    double getReconstructedCanalAngularVelocityPitch(const SimTK::State& state) const {
        if (history.empty()) return 0.0;
        auto iterator = std::lower_bound(
            history.begin(), history.end(), state.getTime(),
            [](const std::pair<double, HistoryRecord>& entry, double timestamp) { return entry.first < timestamp; });
        if (iterator == history.end()) {
            if (history.empty()) return 0.0;
            --iterator;
        }
        if (!iterator->second.valid) return 0.0;
        return iterator->second.reconstructed_canal_angular_velocity[2];
    }

    // Shared 3D Vestibular Angular Velocity Output [roll, yaw, pitch] (rad/s) for VOR FixationController
    OpenSim_DECLARE_OUTPUT(vestibular_omega, SimTK::Vec3, getVestibularAngularVelocity, SimTK::Stage::Velocity);
    SimTK::Vec3 getVestibularAngularVelocity(const SimTK::State& state) const {
        double angular_velocity_roll  = state_variable_luenberger_angular_velocity[0] ? state_variable_luenberger_angular_velocity[0]->getValue(state) : getStateVariableValue(state, "luen_omega_roll");
        double angular_velocity_yaw   = state_variable_luenberger_angular_velocity[1] ? state_variable_luenberger_angular_velocity[1]->getValue(state) : getStateVariableValue(state, "luen_omega_yaw");
        double angular_velocity_pitch = state_variable_luenberger_angular_velocity[2] ? state_variable_luenberger_angular_velocity[2]->getValue(state) : getStateVariableValue(state, "luen_omega_pitch");
        return SimTK::Vec3(angular_velocity_roll, angular_velocity_yaw, angular_velocity_pitch);
    }

    // Decoded proprioceptive relative state OpenSim Outputs (accessible by reporters and python scripts)
    OpenSim_DECLARE_OUTPUT(est_neck_pitch, double, getOutputNeckPitch, SimTK::Stage::Velocity);
    double getOutputNeckPitch(const SimTK::State& state) const { return estimated_neck_relative_angle[0]; }

    OpenSim_DECLARE_OUTPUT(est_neck_roll, double, getOutputNeckRoll, SimTK::Stage::Velocity);
    double getOutputNeckRoll(const SimTK::State& state) const { return estimated_neck_relative_angle[1]; }

    OpenSim_DECLARE_OUTPUT(est_neck_yaw, double, getOutputNeckYaw, SimTK::Stage::Velocity);
    double getOutputNeckYaw(const SimTK::State& state) const { return estimated_neck_relative_angle[2]; }

    OpenSim_DECLARE_OUTPUT(est_neck_pitch_speed, double, getOutputNeckPitchSpeed, SimTK::Stage::Velocity);
    double getOutputNeckPitchSpeed(const SimTK::State& state) const { return estimated_neck_relative_speed[0]; }

    OpenSim_DECLARE_OUTPUT(est_neck_roll_speed, double, getOutputNeckRollSpeed, SimTK::Stage::Velocity);
    double getOutputNeckRollSpeed(const SimTK::State& state) const { return estimated_neck_relative_speed[1]; }

    OpenSim_DECLARE_OUTPUT(est_neck_yaw_speed, double, getOutputNeckYawSpeed, SimTK::Stage::Velocity);
    double getOutputNeckYawSpeed(const SimTK::State& state) const { return estimated_neck_relative_speed[2]; }

    // Tendon-compliance-compensated Cuneate static secondary-spindle neck posture [pitch, roll, yaw] (rad)
    OpenSim_DECLARE_OUTPUT(cuneate_neck_pitch, double, getCuneateNeckPitch, SimTK::Stage::Velocity);
    double getCuneateNeckPitch(const SimTK::State& state) const {
        return state_variable_cuneate_posture[0] ? state_variable_cuneate_posture[0]->getValue(state) : cuneate_posture_neck_angle[0];
    }

    OpenSim_DECLARE_OUTPUT(cuneate_neck_roll, double, getCuneateNeckRoll, SimTK::Stage::Velocity);
    double getCuneateNeckRoll(const SimTK::State& state) const {
        return state_variable_cuneate_posture[1] ? state_variable_cuneate_posture[1]->getValue(state) : cuneate_posture_neck_angle[1];
    }

    OpenSim_DECLARE_OUTPUT(cuneate_neck_yaw, double, getCuneateNeckYaw, SimTK::Stage::Velocity);
    double getCuneateNeckYaw(const SimTK::State& state) const {
        return state_variable_cuneate_posture[2] ? state_variable_cuneate_posture[2]->getValue(state) : cuneate_posture_neck_angle[2];
    }

    // Mahony Spatial Observer OpenSim Outputs (accessible by reporters and python scripts)
    OpenSim_DECLARE_OUTPUT(mahony_pitch, double, getMahonyPitch, SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(mahony_roll, double, getMahonyRoll, SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(mahony_yaw, double, getMahonyYaw, SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(mahony_pitch_speed, double, getMahonyPitchSpeed, SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(mahony_roll_speed, double, getMahonyRollSpeed, SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(mahony_yaw_speed, double, getMahonyYawSpeed, SimTK::Stage::Velocity);

    double getMahonyPitch(const SimTK::State& state) const;
    double getMahonyRoll(const SimTK::State& state) const;
    double getMahonyYaw(const SimTK::State& state) const;
    double getMahonyPitchSpeed(const SimTK::State& state) const;
    double getMahonyRollSpeed(const SimTK::State& state) const;
    double getMahonyYawSpeed(const SimTK::State& state) const;

    // 6-DOF Cuneate head-on-trunk horizontal shift outputs (m) and SE2(3) translational observer outputs
    OpenSim_DECLARE_OUTPUT(cuneate_ap_shift, double, getCuneateApShift, SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(cuneate_lat_shift, double, getCuneateLatShift, SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(obs_pos_x, double, getObserverPosX, SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(obs_pos_z, double, getObserverPosZ, SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(obs_vel_x, double, getObserverVelX, SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(obs_vel_z, double, getObserverVelZ, SimTK::Stage::Velocity);

    double getCuneateApShift(const SimTK::State& state) const {
        const double x_filt = state_variable_cuneate_diff[0] ? state_variable_cuneate_diff[0]->getValue(state) : 0.0;
        return (std::abs(x_filt) > 1e-12) ? x_filt : cuneate_posture_shift_m[0];
    }
    double getCuneateLatShift(const SimTK::State& state) const {
        const double x_filt = state_variable_cuneate_diff[1] ? state_variable_cuneate_diff[1]->getValue(state) : 0.0;
        return (std::abs(x_filt) > 1e-12) ? x_filt : cuneate_posture_shift_m[1];
    }
    double getObserverPosX(const SimTK::State& state) const {
        if (!get_enable_mahony()) return 0.0;
        return state_variable_observer_position[0] ? state_variable_observer_position[0]->getValue(state) : getStateVariableValue(state, "obs_px");
    }
    double getObserverPosZ(const SimTK::State& state) const {
        if (!get_enable_mahony()) return 0.0;
        return state_variable_observer_position[2] ? state_variable_observer_position[2]->getValue(state) : getStateVariableValue(state, "obs_pz");
    }
    double getObserverVelX(const SimTK::State& state) const {
        if (!get_enable_mahony()) return 0.0;
        return state_variable_observer_velocity[0] ? state_variable_observer_velocity[0]->getValue(state) : getStateVariableValue(state, "obs_vx");
    }
    double getObserverVelZ(const SimTK::State& state) const {
        if (!get_enable_mahony()) return 0.0;
        return state_variable_observer_velocity[2] ? state_variable_observer_velocity[2]->getValue(state) : getStateVariableValue(state, "obs_vz");
    }

    // Decoded proprioceptive relative state (passive observer / multisensory fusion)
    const SimTK::Vec3& getEstimatedNeckRelativeAngle() const { return estimated_neck_relative_angle; }
    const SimTK::Vec3& getEstimatedNeckRelativeSpeed() const { return estimated_neck_relative_speed; }
    double getEstimatedNeckPitch() const { return estimated_neck_relative_angle[0]; }
    double getEstimatedNeckRoll() const { return estimated_neck_relative_angle[1]; }
    double getEstimatedNeckYaw() const { return estimated_neck_relative_angle[2]; }
    double getEstimatedNeckPitchSpeed() const { return estimated_neck_relative_speed[0]; }
    double getEstimatedNeckRollSpeed() const { return estimated_neck_relative_speed[1]; }
    double getEstimatedNeckYawSpeed() const { return estimated_neck_relative_speed[2]; }

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
        SimTK::Vec3 estimated_head_orientation{0.0};               // Branch 3B (13.0ms -> Sub 4 Cortex): (roll, yaw, pitch) proper lead compensated
        SimTK::Vec3 estimated_head_orientation_4a{0.0};            // Branch 3A (1.5ms -> Sub 2B): (roll, yaw, pitch) proper lead compensated
        SimTK::Vec3 reconstructed_canal_angular_velocity{0.0};     // Branch 2A (2.0ms -> Sub 2B): irregular canal velocity proper lead compensated
        SimTK::Vec3 luenberger_angular_velocity{0.0};              // Branch 2B (3.0ms -> Sub 3): regular canal Luenberger velocity proper lead compensated
        SimTK::Vec3 luenberger_angular_acceleration{0.0};          // Branch 2B (3.0ms -> Sub 3): regular canal Luenberger acceleration proper lead compensated
        SimTK::Vec3 reconstructed_otolith_specific_force{0.0};     // Branch 2A (2.0ms -> Sub 2B): irregular otolith GIF proper lead compensated
        SimTK::Vec3 luenberger_otolith_specific_force{0.0};        // Branch 2B (3.0ms -> Sub 3): regular otolith Luenberger GIF proper lead compensated
        SimTK::Mat33 rotation_world_to_head{1.0};                  // True skull rotation from world to head at record timestamp
        SimTK::Mat33 estimated_rotation_head_to_world{1.0};        // Branch 2B/3A: Observer estimated rotation R_hat_H^W
        SimTK::Vec3 trunk_orientation{0.0};                        // Branch 1A->3 (3.0ms -> Sub 3): (roll, yaw, pitch) trunk orientation proper lead compensated
        SimTK::Vec3 trunk_angular_velocity{0.0};                   // Branch 1A->3 (3.0ms -> Sub 3): (roll, yaw, pitch) trunk angular velocity
        SimTK::Vec3 head_linear_velocity{0.0};                     // Linear velocity of the head in ground frame
        SimTK::Vector raw_spindle_ia_rate;                         // Raw Mileusnic (2006) spindle Ia firing rate (above baseline) before 1.0 ms peripheral axon delay
        SimTK::Vector raw_spindle_ii_rate;                         // Raw Mileusnic (2006) spindle II firing rate (above baseline) before 2.5 ms peripheral axon delay
        SimTK::Vector primary_afferent_firing_rate;                // Branch 1A->1B (1.0 ms Ia axon -> C1-C8 Sub 1B): Size N delayed spindle Ia rate (uncompensated; Mileusnic intrinsic lead-lag)
        SimTK::Vec3 estimated_neck_relative_angle{0.0};            // Branch 1A->3 (3.0ms -> Sub 3): Size 3 (pitch, roll, yaw) proper lead compensated
        SimTK::Vec3 estimated_neck_relative_speed{0.0};            // Branch 1A->3 (3.0ms -> Sub 3): Size 3 (pitch, roll, yaw) proper lead compensated
        SimTK::Vec3 raw_neck_relative_angle{0.0};                  // Clean Luenberger-filtered neck relative angle (pitch, roll, yaw) before Branch 1A->3 compensator
        SimTK::Vec3 raw_neck_relative_speed{0.0};                  // Clean Luenberger-filtered neck relative speed (pitch, roll, yaw) before Branch 1A->3 compensator
        SimTK::Vec3 raw_neck_relative_accel{0.0};                  // Clean Luenberger-filtered neck relative accel (pitch, roll, yaw) before Branch 1A->3 compensator
        SimTK::Vec<6> raw_neck_6dof_angle{0.0};                    // Clean Luenberger-filtered 6-DOF neck angles [pitch1, pitch2, roll1, roll2, yaw1, yaw2]
        SimTK::Vec<6> raw_neck_6dof_speed{0.0};                    // Clean Luenberger-filtered 6-DOF neck speeds [pitch1, pitch2, roll1, roll2, yaw1, yaw2]
        SimTK::Vec<6> raw_neck_6dof_accel{0.0};                    // Clean Luenberger-filtered 6-DOF neck accels [pitch1, pitch2, roll1, roll2, yaw1, yaw2]
        SimTK::Vec3 mahony_estimated_angular_velocity{0.0};        // Branch 3B (13.0ms -> Sub 4 Cortex): Size 3 (pitch, roll, yaw) proper lead compensated
        SimTK::Vec3 estimated_linear_velocity{0.0};                // Branch 3B (13.0ms -> Sub 4 Cortex): SE2(3) observer linear velocity [x, y, z]
        SimTK::Vec3 estimated_linear_position{0.0};                // Branch 3B (13.0ms -> Sub 4 Cortex): SE2(3) observer egocentric position [x, y, z]
        SimTK::Vec2 cuneate_diff_angle_1a3{0.0};                   // Branch 1A->3 (3.0ms -> Sub 3): Size 2 [pitch1-pitch2, roll1-roll2] Cuneate diff angle
        SimTK::Vec2 cuneate_diff_speed_1a3{0.0};                   // Branch 1A->3 (3.0ms -> Sub 3): Size 2 [dpitch, droll] Cuneate diff speed
        SimTK::Vec2 cuneate_diff_angle_3b{0.0};                    // Branch 3B (13.0ms -> Sub 4 Cortex): Size 2 [pitch1-pitch2, roll1-roll2] Cuneate diff angle
        SimTK::Vec2 cuneate_diff_speed_3b{0.0};                    // Branch 3B (13.0ms -> Sub 4 Cortex): Size 2 [dpitch, droll] Cuneate diff speed
        SimTK::Vec3 cortical_voluntary_torque{0.0};                // Branch 4C (14.5ms -> Sub 5): Size 3 (pitch, roll, yaw) proper lead compensated
        SimTK::Vec2 cortical_voluntary_diff_torque{0.0};           // Branch 4C (14.5ms -> Sub 5): Size 2 [AP pitch1-pitch2, Lat roll1-roll2] shift torque
        SimTK::Vec3 trajectory_position_1a{0.0};                   // Branch 4A (9.0ms -> Sub 3C Tonic): Size 3 (pitch, roll, yaw) proper lead compensated
        SimTK::Vec3 trajectory_velocity_1b{0.0};                   // Branch 4B (10.0ms -> Sub 3C Canal): Size 3 (pitch, roll, yaw) proper lead compensated
        SimTK::Vec3 trajectory_acceleration_1b{0.0};               // Branch 4B (10.0ms -> Sub 3C Phasic): Size 3 (pitch, roll, yaw) proper lead compensated
        SimTK::Vec2 trajectory_diff_acc_1b{0.0};                   // Branch 4B (10.0ms -> Sub 3C Phasic): Size 2 [AP, Lat] planned shift accel (rad/s^2)
        SimTK::Vec3 eff_tonic_tilt_3a{0.0};                        // Branch 3A (1.5ms -> Sub 2B Tonic): Size 3 (pitch, roll, yaw) proper lead compensated
        SimTK::Vec3 eff_canal_vel_3a{0.0};                         // Branch 3A (1.5ms -> Sub 2B Canal): Size 3 (pitch, roll, yaw) proper lead compensated
        SimTK::Vec3 eff_phasic_grav_acc_3a{0.0};                   // Branch 3A (1.5ms -> Sub 2B Phasic): Size 3 (pitch, roll, yaw) proper lead compensated
        SimTK::Vec3 trajectory_position{0.0};                      // Branch 4C (14.5ms -> Sub 5): Size 3 (pitch, roll, yaw) proper lead compensated
        SimTK::Vec3 trajectory_velocity{0.0};                      // Branch 4C (14.5ms -> Sub 5): Size 3 (pitch, roll, yaw) proper lead compensated
        SimTK::Vec2 trajectory_diff_position{0.0};                 // Branch 4C (14.5ms -> Sub 5): Size 2 [AP, Lat] planned diff angle (rad)
        SimTK::Vec2 trajectory_diff_velocity{0.0};                 // Branch 4C (14.5ms -> Sub 5): Size 2 [AP, Lat] planned diff speed (rad/s)
        SimTK::Vec3 brainstem_vcr_torque{0.0};                     // Path 5 (2.5ms MVST -> Sub 5): Size 3 (pitch, roll, yaw) proper lead compensated
        SimTK::Vector spinal_ccr_drive;                            // Path 6 (2.5ms C1-C8 compute -> Sub 5): Size N uncompensated CCR excitation
        SimTK::Vector spinal_motor_excitations;                    // Branch 5A (2.5ms axonal -> 72 Muscles): Size N post-pool-sum alpha excitations proper lead compensated
        SimTK::Vector spinal_alpha_excitations;                    // Branch 7A (2.5ms A-alpha -> 72 Muscles): Size N Active-Set QP descending alpha excitations
        SimTK::Vector spinal_gamma_dyn_commands;                   // Branch 5B/7B (2.5ms A-gamma -> Sub 1A/3 Spindles): Size N Stage 1B dynamic fusimotor commands proper lead compensated
        SimTK::Vector spinal_gamma_stat_commands;                  // Branch 5B/7B (2.5ms A-gamma -> Sub 1A/3 Spindles): Size N Stage 1A static fusimotor commands proper lead compensated
        SimTK::Vector expected_normalized_fiber_lengths;           // Branch 5B/7B (2.5ms A-gamma -> Sub 1A/3 Spindles): Size N expected L_hat(t) proper lead compensated
        SimTK::Vector expected_normalized_fiber_velocities;        // Branch 5B/7B (2.5ms A-gamma -> Sub 1A/3 Spindles): Size N expected dL_hat/dt(t) proper lead compensated
    };
    
    struct DelayedData {
        HistoryRecord record;
        SimTK::Vec3 linear_acceleration{0.0};                      // Extracted linear acceleration
        SimTK::Vec3 reconstructed_angular_acceleration{0.0};       // Extracted angular acceleration
    };
    
    mutable std::deque<std::pair<double, HistoryRecord>> history;
    
    DelayedData getDelayedData(double current_time, double delay_duration, bool interpolate_spindle_afferents = false) const;
    
    mutable SimTK::Matrix moment_arm_jacobian_3dof;                // 3 x N (pitch, roll, yaw)
    mutable SimTK::Matrix moment_arm_jacobian_6dof;                // 6 x N (pitch1, pitch2, roll1, roll2, yaw1, yaw2)
    mutable SimTK::Matrix jacobian_6dof_q0;                        // 6 x N baseline moment arms at q0
    mutable SimTK::Matrix jacobian_6dof_d1[6];                     // 6 x N 1st-order Taylor slopes dJ/dq_d
    mutable SimTK::Matrix jacobian_6dof_d2[6];                     // 6 x N 2nd-order Taylor curvatures d2J/dq_d^2
    mutable SimTK::Matrix exact_jacobian_6dof;                     // 6 x N exact OpenSim moment arms cached at large angles (> 8 deg)
    mutable double last_exact_jacobian_time = -1.0;                // Timestamp of last exact_jacobian_6dof refresh
    mutable double q0_6dof[6] = {0.0};                             // Baseline 6-DOF coordinates [pitch1, pitch2, roll1, roll2, yaw1, yaw2]
    mutable double J_c1_pos[2][2] = {{-0.00506, -0.08327}, {0.00570, 0.07605}};  // 2x2 positional Jacobian of C1/C0 (skull origin) [AP(p1,p2); Lat(r1,r2)] in m/rad
    mutable double J_com_pos[2][2] = {{-0.06806, -0.14632}, {0.06870, 0.13905}}; // 2x2 positional Jacobian of skull COM [AP(p1,p2); Lat(r1,r2)] in m/rad
    mutable SimTK::Matrix proprioceptive_decoder_matrix;           // 3 x N summed reconstruction matrix: (pitch1+pitch2, roll1+roll2, yaw1+yaw2)
    mutable SimTK::Matrix proprioceptive_decoder_matrix_6dof;      // 6 x N full 6-DOF reconstruction matrix: -(J6 * J6^T)^{-1} * J6
    mutable SimTK::Vec3 estimated_neck_relative_angle;             // Decoded relative neck angle [pitch, roll, yaw] (rad)
    mutable SimTK::Vec3 estimated_neck_relative_speed;             // Decoded relative neck speed [pitch, roll, yaw] (rad/s)
    mutable SimTK::Vec3 cuneate_posture_neck_angle{0.0};           // Tendon-compensated static secondary-spindle neck posture [pitch, roll, yaw] (rad)
    mutable SimTK::Vec2 cuneate_posture_shift_m{0.0};              // Tendon-compensated 6-DOF Cuneate station shift [AP x, Lat z] (m)
    mutable SimTK::Vec<6> estimated_neck_6dof_angle{0.0};          // Decoded individual 6-DOF neck angles [pitch1, pitch2, roll1, roll2, yaw1, yaw2] (rad)
    mutable SimTK::Vec<6> estimated_neck_6dof_speed{0.0};          // Decoded individual 6-DOF neck speeds [pitch1, pitch2, roll1, roll2, yaw1, yaw2] (rad/s)
    mutable SimTK::Vec3 mahony_estimated_angle;                    // Mahony estimated head attitude [pitch, roll, yaw] (rad)
    mutable SimTK::Vec3 mahony_estimated_angular_velocity;         // Mahony estimated angular velocity [pitch, roll, yaw] (rad/s)
    
    void updateDecoderMatrix() const;
    void decodeProprioceptive6DofState(
        const std::vector<double>& fiber_length_changes,
        const std::vector<double>& fiber_velocities,
        const std::vector<double>* fiber_accelerations,
        SimTK::Vec<6>* out_accel_6dof,
        SimTK::Vec3* out_accel_3dof) const;
    
    mutable SimTK::Vector last_desired_torque;                     // Store commanded torque for Efference Copy
    
    mutable std::vector<int> active_muscle_indices;
    mutable std::vector<std::string> muscle_names;
    mutable std::vector<std::string> cervico_collic_state_names;
    mutable std::vector<std::string> cervico_collic_vel_state_names;
    mutable std::vector<std::string> spindle_length_change_state_names;
    mutable std::vector<std::string> spindle_velocity_state_names;
    mutable std::vector<std::string> spindle_dynamic_fusimotor_state_names;
    mutable std::vector<std::string> spindle_static_fusimotor_state_names;
    mutable std::vector<const StateVariable*> state_variable_cervico_collic;
    mutable std::vector<const StateVariable*> state_variable_cervico_collic_vel;
    mutable std::vector<const StateVariable*> state_variable_spindle_length_change;
    mutable std::vector<const StateVariable*> state_variable_spindle_velocity;
    mutable std::vector<const StateVariable*> state_variable_spindle_dynamic_fusimotor;
    mutable std::vector<const StateVariable*> state_variable_spindle_static_fusimotor;
    mutable const StateVariable* state_variable_semicircular_canal[6][2][2] = {{{nullptr}}};
    mutable const StateVariable* state_variable_trajectory_position[3] = {nullptr};
    mutable const StateVariable* state_variable_trajectory_velocity[3] = {nullptr};
    mutable const StateVariable* state_variable_trajectory_acceleration[3] = {nullptr};
    mutable const StateVariable* state_variable_traj_diff_pos[2] = {nullptr};
    mutable const StateVariable* state_variable_traj_diff_vel[2] = {nullptr};
    mutable const StateVariable* state_variable_traj_diff_acc[2] = {nullptr};
    mutable const StateVariable* state_variable_error_integral[3] = {nullptr};
    mutable const StateVariable* state_variable_shift_error_integral[2] = {nullptr};
    mutable const StateVariable* state_variable_voluntary_filter_stage1[3] = {nullptr};
    mutable const StateVariable* state_variable_voluntary_filter_stage2[3] = {nullptr};
    mutable const StateVariable* state_variable_vol_diff_filter_stage1[2] = {nullptr};
    mutable const StateVariable* state_variable_vol_diff_filter_stage2[2] = {nullptr};
    mutable const StateVariable* state_variable_luenberger_angular_velocity[3] = {nullptr};
    mutable const StateVariable* state_variable_velocity_storage[3] = {nullptr};
    mutable const StateVariable* state_variable_highg_acc_env = nullptr;
    mutable const StateVariable* state_variable_cuneate_posture[3] = {nullptr};
    mutable const StateVariable* state_variable_cuneate_diff[2] = {nullptr};
    mutable const StateVariable* state_variable_obs_wash_lpf[2] = {nullptr};
    mutable const StateVariable* state_variable_obs_vel_wash_lpf[2] = {nullptr};
    mutable const StateVariable* state_variable_regular_otolith_adapt[3] = {nullptr};
    mutable const StateVariable* state_variable_regular_otolith[3] = {nullptr};
    mutable const StateVariable* state_variable_irregular_otolith_adapt[3] = {nullptr};
    mutable const StateVariable* state_variable_irregular_otolith[3] = {nullptr};
    mutable const StateVariable* state_variable_luenberger_otolith[3] = {nullptr};
    mutable const StateVariable* state_variable_luenberger_otolith_adapt[3] = {nullptr};
    mutable const StateVariable* state_variable_observer_quaternion[4] = {nullptr};
    mutable const StateVariable* state_variable_observer_velocity[3] = {nullptr};
    mutable const StateVariable* state_variable_observer_position[3] = {nullptr};
    mutable const StateVariable* state_variable_tonic_otolith[3] = {nullptr};
    mutable const StateVariable* state_variable_tonic_otolith_vel[3] = {nullptr};
    mutable const StateVariable* state_variable_semicircular_canal_reflex[3] = {nullptr};
    mutable const StateVariable* state_variable_semicircular_canal_reflex_vel[3] = {nullptr};
    mutable const StateVariable* state_variable_phasic_otolith[3] = {nullptr};
    mutable const StateVariable* state_variable_phasic_otolith_vel[3] = {nullptr};
    mutable const StateVariable* state_variable_phasic_otolith_stage2[3] = {nullptr};
    mutable const StateVariable* state_variable_cb_canal_roll_dev = nullptr;
    mutable const Coordinate* cached_coordinate_pitch1 = nullptr;
    mutable const Coordinate* cached_coordinate_pitch2 = nullptr;
    mutable const Coordinate* cached_coordinate_roll1 = nullptr;
    mutable const Coordinate* cached_coordinate_roll2 = nullptr;
    mutable const Coordinate* cached_coordinate_yaw1 = nullptr;
    mutable const Coordinate* cached_coordinate_yaw2 = nullptr;
    mutable const Coordinate* cached_coordinate_ground_pitch = nullptr;
    mutable const Coordinate* cached_coordinate_ground_roll = nullptr;
    mutable const Coordinate* cached_coordinate_ground_yaw = nullptr;
    mutable const Coordinate* cached_coordinate_spine_tx = nullptr;
    mutable const Coordinate* cached_coordinate_spine_tz = nullptr;
    mutable const Body* cached_skull = nullptr;
    SimTK::Mat33 right_canal_projection_matrix;
    SimTK::Mat33 left_canal_projection_matrix;
    SimTK::Mat33 right_canal_inverse_matrix;
    SimTK::Mat33 left_canal_inverse_matrix;
    mutable std::vector<const OpenSim::Output<double>*> cached_primary_afferent_outputs;
    mutable std::vector<const OpenSim::Output<double>*> cached_secondary_afferent_outputs;
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
    mutable std::vector<double> last_alpha_motor_excitations;
    mutable std::vector<double> last_voluntary_alpha_excitations;
    mutable SimTK::Vec3 last_qp_torque_deficit = SimTK::Vec3(0.0);
    mutable double last_qp_max_u = 0.0;
    mutable bool is_initialized;
    mutable double last_jacobian_update_time;
    mutable SimTK::Vec3 baseline_spatial_orientation;              // (roll, yaw, pitch)
    mutable SimTK::Rotation baseline_rotation_head_to_world;

    // Proper biproper 1st-order lead compensator state C_lead(s) = (1 + tau_z*s)/(1 + tau_p*s)
    // Slots 0..35:  2A (0..5), 2B (6..14), 1A->3 (15..23), 3A efference (24..32), Path 5 MVST (33..35);
    // Slots 36..62: 3A ori (36..38), 3B ori/vel (39..44), 4A pos (45..47), 4B vel/acc (48..53), 4C vol/pos/vel (54..62);
    // Slots 63..79: 1A->3 diff ang/spd (63..66), 3B diff ang/spd (67..70), 4B diff acc (71..72),
    //               4C vol_diff/diff_pos/diff_vel (73..78), spare (79);
    // Slots 80..80+5N-1: per-muscle Sub5 Branch 5A (12.5 ms: +0), Branch 5B (2.5 ms: +1..+3), and Sub1B C1-C8 CCR (2.5 ms: +4).
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

    // Lazy 200 Hz Delta_t-invariant OU noise bank evaluated via variance-normalized C^2 cubic B-spline.
    // Zero allocation/RNG work when both enable_sensor_noise and enable_motor_noise are false.
    struct BiologicalNoiseBank {
        static constexpr double kDt = 0.005;     // 200 Hz knot spacing
        static constexpr int kChunkKnots = 200;  // 1.0 s growth chunks
        int cached_seed = -1;
        int n_muscles = 0;
        int n_channels = 0;
        int num_knots = 0;
        std::mt19937_64 rng;
        std::vector<double> alphas;
        std::vector<double> sigmas;
        std::vector<double> bspline_norms;
        std::vector<double> state_vals;
        std::vector<double> knots;

        static double tauForChannel(int ch, int n_mus) {
            if (ch < 6) return 0.015;                  // 0..5:   Regular canals
            if (ch < 12) return 0.012;                 // 6..11:  Irregular canals
            if (ch < 18) return 0.020;                 // 12..17: Regular otoliths (L/R 3D)
            if (ch < 24) return 0.015;                 // 18..23: Irregular otoliths (L/R 3D)
            if (ch < 24 + n_mus) return 0.020;         // 24..24+N-1: Spindle Ia
            if (ch < 24 + 2 * n_mus) return 0.030;     // 24+N..24+2N-1: Spindle II
            return 0.060;                              // 24+2N..24+3N-1: Motor alpha common drive
        }

        void reset(int seed, int n_mus) {
            cached_seed = seed;
            n_muscles = std::max(0, n_mus);
            n_channels = 24 + 3 * n_muscles;
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

        void ensureCapacity(double t_sec, int seed, int n_mus) {
            const int target_n_mus = std::max(0, n_mus);
            if (cached_seed != seed || n_muscles != target_n_mus || n_channels == 0) {
                reset(seed, target_n_mus);
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
            const int k1 = k;
            const int k2 = k + 1;
            const int k3 = k + 2;
            const double p0 = knots[static_cast<size_t>(k0) * n_channels + ch];
            const double p1 = knots[static_cast<size_t>(k1) * n_channels + ch];
            const double p2 = knots[static_cast<size_t>(k2) * n_channels + ch];
            const double p3 = knots[static_cast<size_t>(k3) * n_channels + ch];
            const double f2 = f * f;
            const double f3 = f2 * f;
            const double omf = 1.0 - f;
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
