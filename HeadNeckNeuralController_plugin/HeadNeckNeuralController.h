#ifndef OPENSIM_HEAD_NECK_NEURAL_CONTROLLER_H_
#define OPENSIM_HEAD_NECK_NEURAL_CONTROLLER_H_

#include <OpenSim/OpenSim.h>
#include <algorithm>
#include <deque>
#include <utility>
#include <vector>
#include <map>

namespace OpenSim {

class HeadNeckNeuralController : public Controller {
OpenSim_DECLARE_CONCRETE_OBJECT(HeadNeckNeuralController, Controller);

public:

    OpenSim_DECLARE_PROPERTY(Kp_proprioception, double, "Gain for 72-muscle spinal CCR (default: 0.050)");
    OpenSim_DECLARE_PROPERTY(z_ccr, double, "Pitch Cervico-Collic Reflex TF Zero in s (default: 0.008)");
    OpenSim_DECLARE_PROPERTY(p_ccr, double, "Cervico-Collic Reflex TF Pole in rad/s (default: 99.919586)");
    OpenSim_DECLARE_PROPERTY(K_recip_inhib, double, "Reciprocal inhibition fraction for antagonists (0.0=full block, 1.0=no block)");

    // Voluntary command & cortical 2 Hz low-pass planner
    OpenSim_DECLARE_PROPERTY(desired_yaw, double, "Voluntary desired yaw angle for the head (rad)");
    OpenSim_DECLARE_PROPERTY(desired_pitch, double, "Voluntary desired pitch angle for the head (rad)");
    OpenSim_DECLARE_PROPERTY(desired_roll, double, "Voluntary desired roll angle for the head (rad)");
    OpenSim_DECLARE_PROPERTY(trajectory_tau, double, "Time constant for the internal 2nd-order trajectory planner");
    OpenSim_DECLARE_PROPERTY(fc_voluntary, double, "Voluntary controller low-pass cutoff frequency in Hz (default: 2.0)");
    OpenSim_DECLARE_PROPERTY(tau_lead_pole, double, "Biophysical synaptic pole time constant in s for causal biproper 1st-order lead compensators (default: 0.030)");

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

    // Intersegmental 6-DOF QP anti-buckling gains (acting in nullspace of q_1 + q_2)
    OpenSim_DECLARE_PROPERTY(Kp_buckle, double, "6-DOF QP intersegmental anti-buckling proportional gain in Nm/rad (default: 1.5)");
    OpenSim_DECLARE_PROPERTY(Kd_buckle, double, "6-DOF QP intersegmental anti-buckling derivative gain in Nms/rad (default: 0.15)");

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
        SimTK::Vec3 cortical_voluntary_torque{0.0};                // Branch 4C (14.5ms -> Sub 5): Size 3 (pitch, roll, yaw) proper lead compensated
        SimTK::Vec3 trajectory_position_1a{0.0};                   // Branch 4A (9.0ms -> Sub 3C Tonic): Size 3 (pitch, roll, yaw) proper lead compensated
        SimTK::Vec3 trajectory_velocity_1b{0.0};                   // Branch 4B (10.0ms -> Sub 3C Canal): Size 3 (pitch, roll, yaw) proper lead compensated
        SimTK::Vec3 trajectory_acceleration_1b{0.0};               // Branch 4B (10.0ms -> Sub 3C Phasic): Size 3 (pitch, roll, yaw) proper lead compensated
        SimTK::Vec3 eff_tonic_tilt_3a{0.0};                        // Branch 3A (1.5ms -> Sub 2B Tonic): Size 3 (pitch, roll, yaw) proper lead compensated
        SimTK::Vec3 eff_canal_vel_3a{0.0};                         // Branch 3A (1.5ms -> Sub 2B Canal): Size 3 (pitch, roll, yaw) proper lead compensated
        SimTK::Vec3 eff_phasic_grav_acc_3a{0.0};                   // Branch 3A (1.5ms -> Sub 2B Phasic): Size 3 (pitch, roll, yaw) proper lead compensated
        SimTK::Vec3 trajectory_position{0.0};                      // Branch 4C (14.5ms -> Sub 5): Size 3 (pitch, roll, yaw) proper lead compensated
        SimTK::Vec3 trajectory_velocity{0.0};                      // Branch 4C (14.5ms -> Sub 5): Size 3 (pitch, roll, yaw) proper lead compensated
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
    mutable double q0_6dof[6] = {0.0};                             // Baseline 6-DOF coordinates [pitch1, pitch2, roll1, roll2, yaw1, yaw2]
    mutable SimTK::Matrix proprioceptive_decoder_matrix;           // 3 x N summed reconstruction matrix: (pitch1+pitch2, roll1+roll2, yaw1+yaw2)
    mutable SimTK::Matrix proprioceptive_decoder_matrix_6dof;      // 6 x N full 6-DOF reconstruction matrix: -(J6 * J6^T)^{-1} * J6
    mutable SimTK::Vec3 estimated_neck_relative_angle;             // Decoded relative neck angle [pitch, roll, yaw] (rad)
    mutable SimTK::Vec3 estimated_neck_relative_speed;             // Decoded relative neck speed [pitch, roll, yaw] (rad/s)
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
    mutable const StateVariable* state_variable_error_integral[3] = {nullptr};
    mutable const StateVariable* state_variable_voluntary_filter_stage1[3] = {nullptr};
    mutable const StateVariable* state_variable_voluntary_filter_stage2[3] = {nullptr};
    mutable const StateVariable* state_variable_luenberger_angular_velocity[3] = {nullptr};
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
    mutable bool is_initialized;
    mutable double last_jacobian_update_time;
    mutable SimTK::Vec3 baseline_spatial_orientation;              // (roll, yaw, pitch)

    // Proper biproper 1st-order lead compensator state C_lead(s) = (1 + tau_z*s)/(1 + tau_p*s)
    // Slots 0..35:  2A (0..5), 2B (6..14), 1A->3 (15..23), 3A efference (24..32), Path 5 MVST (33..35);
    // Slots 36..65: 3A ori (36..38), 3B ori/vel (39..44), 4A pos (45..47), 4B vel/acc (48..53), 4C vol/pos/vel (54..62), Pre-QP tau_act 10ms (63..65);
    // Slots 66..66+5N-1: per-muscle Sub5 Branch 5A (2.5 ms: +0), Branch 5B (12.5 ms: +1..+3), and Sub1B C1-C8 CCR (2.5 ms: +4).
    static constexpr int NUM_FIXED_LEAD_SLOTS = 66;
    mutable std::vector<const StateVariable*> state_variable_lead_comp;
    mutable std::vector<double> lead_u0;
    mutable std::vector<double> lead_u_curr;
    mutable std::vector<double> lead_tau_p;
    mutable bool lead_u0_initialized = false;
    double evalProperLead(int slot, double u_val, double tau_d, const SimTK::State& state) const;
    
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
};

} // namespace OpenSim

#endif // OPENSIM_HEAD_NECK_NEURAL_CONTROLLER_H_
