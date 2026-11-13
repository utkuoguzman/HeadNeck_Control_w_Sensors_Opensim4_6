#ifndef OPENSIM_HEAD_NECK_NEURAL_CONTROLLER_ADAPTIVE_H_
#define OPENSIM_HEAD_NECK_NEURAL_CONTROLLER_ADAPTIVE_H_

#include <OpenSim/OpenSim.h>
#include <algorithm>
#include <deque>
#include <utility>
#include <vector>
#include <map>

namespace OpenSim {

class HeadNeckNeuralControllerAdaptive : public Controller {
OpenSim_DECLARE_CONCRETE_OBJECT(HeadNeckNeuralControllerAdaptive, Controller);

public:

    OpenSim_DECLARE_PROPERTY(delay_vcr, double, "Vestibulo-Collic Reflex delay in seconds (default: 0.010)");
    OpenSim_DECLARE_PROPERTY(delay_ccr, double, "Cervico-Collic Reflex delay in seconds (default: 0.015)");
    OpenSim_DECLARE_PROPERTY(Kp_proprioception, double, "Gain for spindle Ia");
    OpenSim_DECLARE_PROPERTY(z_ccr, double, "Cervico-Collic Reflex TF Zero");
    OpenSim_DECLARE_PROPERTY(p_ccr, double, "Cervico-Collic Reflex TF Pole");
    OpenSim_DECLARE_PROPERTY(K_recip_inhib, double, "Reciprocal inhibition fraction for antagonists (0.0=full block, 1.0=no block)");
    OpenSim_DECLARE_PROPERTY(Kg_proprioception, double, "Gain for GTO Ib");
    OpenSim_DECLARE_PROPERTY(K_vestibular, double, "Gain for Vestibular descending drive");
    OpenSim_DECLARE_PROPERTY(K_otolith, double, "Gain for Otolith descending drive (Static posture)");

    // Voluntary command
    OpenSim_DECLARE_PROPERTY(desired_yaw, double, "Voluntary desired yaw angle for the head (rad)");
    OpenSim_DECLARE_PROPERTY(desired_pitch, double, "Voluntary desired pitch angle for the head (rad)");
    OpenSim_DECLARE_PROPERTY(desired_roll, double, "Voluntary desired roll angle for the head (rad)");
    OpenSim_DECLARE_PROPERTY(trajectory_tau, double, "Time constant for the internal 2nd-order trajectory planner");
    OpenSim_DECLARE_PROPERTY(delay_voluntary, double, "Cortical/voluntary delay in seconds (default: 0.040)");
    OpenSim_DECLARE_PROPERTY(fc_voluntary, double, "Voluntary controller low-pass cutoff frequency in Hz (default: 2.0)");

    // Vestibular & Reflex properties
    OpenSim_DECLARE_PROPERTY(K_ton, double, "Tonic otolith TF Gain");
    OpenSim_DECLARE_PROPERTY(p_ton, double, "Tonic otolith TF Pole");
    OpenSim_DECLARE_PROPERTY(K_sc, double, "Semicircular canal TF Gain");
    OpenSim_DECLARE_PROPERTY(z_sc, double, "Semicircular canal TF Zero");
    OpenSim_DECLARE_PROPERTY(p_sc, double, "Semicircular canal TF Pole");
    OpenSim_DECLARE_PROPERTY(K_phas, double, "Phasic otolith TF Gain");
    OpenSim_DECLARE_PROPERTY(z_phas, double, "Phasic otolith TF Zero in rad/s (default: -1.0)");
    OpenSim_DECLARE_PROPERTY(p_phas, double, "Phasic otolith TF Pole");
    OpenSim_DECLARE_PROPERTY(p_phas2, double, "Second-order phasic otolith TF Pole in rad/s (default: -1.0)");
    OpenSim_DECLARE_PROPERTY(k_p, double, "Muscle-level CCR proportional gain");

    // Gamma motor neuron drive (Alpha-Gamma Coactivation)
    OpenSim_DECLARE_PROPERTY(K_gamma_dyn, double, "Gain for Vestibular-driven Gamma_dyn");
    OpenSim_DECLARE_PROPERTY(K_gamma_stat, double, "Gain for Alpha-Gamma Coactivation");
    OpenSim_DECLARE_PROPERTY(K_gamma, double, "Gain for Gamma motor neuron dynamic setpoint shift (default: 1.0)");
    OpenSim_DECLARE_PROPERTY(Wp_base_pitch, double, "Base LQR position cost for pitch");
    OpenSim_DECLARE_PROPERTY(Wv_base_pitch, double, "Base LQR velocity cost for pitch");
    OpenSim_DECLARE_PROPERTY(Wp_base_roll, double, "Base LQR position cost for roll");
    OpenSim_DECLARE_PROPERTY(Wv_base_roll, double, "Base LQR velocity cost for roll");
    OpenSim_DECLARE_PROPERTY(Wp_base_yaw, double, "Base LQR position cost for yaw");
    OpenSim_DECLARE_PROPERTY(Wv_base_yaw, double, "Base LQR velocity cost for yaw");

    // Direct Task Space Stiffness and Damping (Disregards Wp/Wv when set >= 0.0)
    OpenSim_DECLARE_PROPERTY(Kp_task_pitch, double, "Explicit Task Kp for pitch in Nm/rad (overrides Wp if >= 0.0, default: -1.0)");
    OpenSim_DECLARE_PROPERTY(Kd_task_pitch, double, "Explicit Task Kd for pitch in Nms/rad (overrides Wv if >= 0.0, default: -1.0)");
    OpenSim_DECLARE_PROPERTY(Kp_task_roll, double, "Explicit Task Kp for roll in Nm/rad (overrides Wp if >= 0.0, default: -1.0)");
    OpenSim_DECLARE_PROPERTY(Kd_task_roll, double, "Explicit Task Kd for roll in Nms/rad (overrides Wv if >= 0.0, default: -1.0)");
    OpenSim_DECLARE_PROPERTY(Kp_task_yaw, double, "Explicit Task Kp for yaw in Nm/rad (overrides Wp if >= 0.0, default: -1.0)");
    OpenSim_DECLARE_PROPERTY(Kd_task_yaw, double, "Explicit Task Kd for yaw in Nms/rad (overrides Wv if >= 0.0, default: -1.0)");

    // Continuous Vigor logic
    OpenSim_DECLARE_PROPERTY(k_vigor, double, "Continuous Vigor multiplier for velocity gating");
    OpenSim_DECLARE_PROPERTY(Ki_task, double, "Integral gain for gravity compensation (default: 6.54)");
    OpenSim_DECLARE_PROPERTY(Ki_task_pitch, double, "Explicit Task Ki for pitch in Nm/(rad*s) (default: -1.0)");
    OpenSim_DECLARE_PROPERTY(Ki_task_roll, double, "Explicit Task Ki for roll in Nm/(rad*s) (default: 0.0)");
    OpenSim_DECLARE_PROPERTY(Ki_task_yaw, double, "Explicit Task Ki for yaw in Nm/(rad*s) (default: 0.0)");
    // Cerebellar dynamic approximation toggle
    OpenSim_DECLARE_PROPERTY(use_dynamic_jacobian, bool, "If true, recalculates the TNT Metric Tensor at every step");
    OpenSim_DECLARE_PROPERTY(K_eso, double, "Gain for Extended State Observer disturbance rejection (default: 0.0)");
    OpenSim_DECLARE_PROPERTY(enable_eye_afferents, bool, "If true, includes extraocular eye muscles (r_*, l_*) in controller state allocation (default: false)");
    OpenSim_DECLARE_PROPERTY(use_smith_predictor, bool, "If true, utilizes Smith Predictor delay compensation for Mahony proprioceptive fusion (default: true)");
    OpenSim_DECLARE_PROPERTY(delay_smith_proprio, double, "Proprioceptive-to-vestibular differential delay in seconds for Mahony history alignment (default: 0.005)");
    OpenSim_DECLARE_PROPERTY(delay_afferent_proprio, double, "Proprioceptive afferent transport delay in seconds arriving at the central observer (default: 0.007)");
    OpenSim_DECLARE_PROPERTY(delay_afferent_vestibular, double, "Vestibular afferent transport delay arriving at central observer (default: 0.002)");

    // --- CONTINUOUS ADAPTATION PROPERTIES (PATH 1 - 12 PARAMETERS) ---
    OpenSim_DECLARE_PROPERTY(enable_adaptation, bool, "If true, continuously adapts all 12 CMA-ES parameters via Simbody state variables (default: true)");
    OpenSim_DECLARE_PROPERTY(adaptation_rate_multiplier, double, "Global multiplier for all continuous adaptation rates (default: 1.0)");
    OpenSim_DECLARE_PROPERTY(sigma_leakage, double, "Continuous sigma-leakage coefficient toward nominal gains (default: 0.02)");
    OpenSim_DECLARE_PROPERTY(adaptation_start_time, double, "Simulation time in seconds before adaptation activates (default: 1.8)");
    OpenSim_DECLARE_PROPERTY(multisine_scale, double, "Scale factor for reference multisine excitation (default: 0.142887)");

    HeadNeckNeuralControllerAdaptive();

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

    // Decoded proprioceptive relative state OpenSim Outputs
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

    // Mahony Spatial Observer OpenSim Outputs
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

    // Continuous Adaptive Outputs (12 Parameters)
    OpenSim_DECLARE_OUTPUT(adapt_K_ton, double, getAdaptKton, SimTK::Stage::Velocity);
    double getAdaptKton(const SimTK::State& s) const;

    OpenSim_DECLARE_OUTPUT(adapt_p_ton, double, getAdaptPton, SimTK::Stage::Velocity);
    double getAdaptPton(const SimTK::State& s) const;

    OpenSim_DECLARE_OUTPUT(adapt_K_sc, double, getAdaptKsc, SimTK::Stage::Velocity);
    double getAdaptKsc(const SimTK::State& s) const;

    OpenSim_DECLARE_OUTPUT(adapt_z_sc, double, getAdaptZsc, SimTK::Stage::Velocity);
    double getAdaptZsc(const SimTK::State& s) const;

    OpenSim_DECLARE_OUTPUT(adapt_p_sc, double, getAdaptPsc, SimTK::Stage::Velocity);
    double getAdaptPsc(const SimTK::State& s) const;

    OpenSim_DECLARE_OUTPUT(adapt_K_phas, double, getAdaptKphas, SimTK::Stage::Velocity);
    double getAdaptKphas(const SimTK::State& s) const;

    OpenSim_DECLARE_OUTPUT(adapt_p_phas, double, getAdaptPphas, SimTK::Stage::Velocity);
    double getAdaptPphas(const SimTK::State& s) const;

    OpenSim_DECLARE_OUTPUT(adapt_kp_prop, double, getAdaptKpprop, SimTK::Stage::Velocity);
    double getAdaptKpprop(const SimTK::State& s) const;

    OpenSim_DECLARE_OUTPUT(adapt_z_ccr, double, getAdaptZccr, SimTK::Stage::Velocity);
    double getAdaptZccr(const SimTK::State& s) const;

    OpenSim_DECLARE_OUTPUT(adapt_p_ccr, double, getAdaptPccr, SimTK::Stage::Velocity);
    double getAdaptPccr(const SimTK::State& s) const;

    OpenSim_DECLARE_OUTPUT(adapt_K_gamma_dyn, double, getAdaptKgammaDyn, SimTK::Stage::Velocity);
    double getAdaptKgammaDyn(const SimTK::State& s) const;

    OpenSim_DECLARE_OUTPUT(adapt_K_gamma_stat, double, getAdaptKgammaStat, SimTK::Stage::Velocity);
    double getAdaptKgammaStat(const SimTK::State& s) const;

    OpenSim_DECLARE_OUTPUT(adapt_Kp_task_pitch, double, getAdaptKpTaskPitch, SimTK::Stage::Velocity);
    double getAdaptKpTaskPitch(const SimTK::State& s) const;

    OpenSim_DECLARE_OUTPUT(adapt_Kd_task_pitch, double, getAdaptKdTaskPitch, SimTK::Stage::Velocity);
    double getAdaptKdTaskPitch(const SimTK::State& s) const;

    OpenSim_DECLARE_OUTPUT(adapt_Ki_task_pitch, double, getAdaptKiTaskPitch, SimTK::Stage::Velocity);
    double getAdaptKiTaskPitch(const SimTK::State& s) const;

    OpenSim_DECLARE_OUTPUT(adaptation_error_deg, double, getAdaptErrorDeg, SimTK::Stage::Velocity);
    double getAdaptErrorDeg(const SimTK::State& s) const;

    OpenSim_DECLARE_OUTPUT(reference_pitch_deg, double, getRefPitchDeg, SimTK::Stage::Velocity);
    double getRefPitchDeg(const SimTK::State& s) const;

    OpenSim_DECLARE_OUTPUT(actual_pitch_deg, double, getActualPitchDeg, SimTK::Stage::Velocity);
    double getActualPitchDeg(const SimTK::State& s) const;

    // Decoded proprioceptive relative state (passive observer / multisensory fusion)
    const SimTK::Vec3& getEstimatedNeckRelativeAngle() const { return estimated_neck_relative_angle; }
    const SimTK::Vec3& getEstimatedNeckRelativeSpeed() const { return estimated_neck_relative_speed; }

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
        SimTK::Vec3 estimated_head_orientation{0.0};               // (roll, yaw, pitch) - Branch 4B compensated head orientation (Sub 4 -> Sub 1, 13.0 ms)
        SimTK::Vec3 estimated_head_orientation_4a{0.0};            // (roll, yaw, pitch) - Branch 4A compensated head orientation (Sub 4 -> Sub 5, 1.5 ms)
        SimTK::Vec3 reconstructed_canal_angular_velocity{0.0};     // (roll, yaw, pitch) - Branch 2A compensated irregular canal firing (Sub 2 -> Sub 5, 2.0 ms)
        SimTK::Vec3 luenberger_angular_velocity{0.0};              // (roll, yaw, pitch) - Branch 2B compensated regular canal velocity (Sub 2 -> Sub 4, 3.0 ms)
        SimTK::Vec3 luenberger_angular_acceleration{0.0};          // (roll, yaw, pitch) - Clean Luenberger-estimated angular acceleration d(x_Luen,scc)/dt
        SimTK::Vec3 reconstructed_otolith_specific_force{0.0};     // (x, y, z) - Branch 2A compensated irregular otolith specific force GIF (Sub 2 -> Sub 5, 2.0 ms)
        SimTK::Vec3 luenberger_otolith_specific_force{0.0};        // (x, y, z) - Branch 2B compensated regular otolith specific force GIF (Sub 2 -> Sub 4, 3.0 ms)
        SimTK::Mat33 rotation_world_to_head{1.0};                  // True skull rotation from world to head at record timestamp
        SimTK::Mat33 estimated_rotation_head_to_world{1.0};        // Observer estimated rotation from head to world at record timestamp
        SimTK::Vec3 trunk_orientation{0.0};                        // (roll, yaw, pitch) - Trunk global orientation
        SimTK::Vec3 trunk_angular_velocity{0.0};                   // (roll, yaw, pitch) - Trunk global angular velocity
        SimTK::Vec3 head_linear_velocity{0.0};                     // Linear velocity of the head in ground frame
        SimTK::Vector primary_afferent_firing_rate;                // Size N - Branch 3A compensated spindle Ia firing rate (Sub 3 -> Sub 6, 5.5 ms)
        SimTK::Vec3 estimated_neck_relative_angle{0.0};            // Size 3 (pitch, roll, yaw) - Branch 3B compensated Cuneate angle (Sub 3 -> Sub 4, 7.0 ms)
        SimTK::Vec3 estimated_neck_relative_speed{0.0};            // Size 3 (pitch, roll, yaw) - Branch 3B compensated Cuneate speed (Sub 3 -> Sub 4, 7.0 ms)
        SimTK::Vec3 mahony_estimated_angular_velocity{0.0};        // Size 3 (pitch, roll, yaw) - Branch 4B compensated Mahony head-frame angular velocity (Sub 4 -> Sub 1, 13.0 ms)
        SimTK::Vec3 mahony_estimated_angular_velocity_4a{0.0};     // Size 3 (pitch, roll, yaw) - Branch 4A compensated Mahony head-frame angular velocity (Sub 4 -> Sub 5, 1.5 ms)
        SimTK::Vec3 cortical_voluntary_torque{0.0};                // Size 3 (pitch, roll, yaw) - Branch 1C compensated voluntary torque (Sub 1 -> Sub 7, 14.5 ms)
        SimTK::Vec3 cortical_voluntary_torque_dot{0.0};            // Size 3 (pitch, roll, yaw) - First derivative of Branch 1C voluntary torque
        SimTK::Vec3 cortical_voluntary_torque_ddot{0.0};           // Size 3 (pitch, roll, yaw) - Second derivative of Branch 1C voluntary torque
        SimTK::Vec3 trajectory_position{0.0};                      // Size 3 (pitch, roll, yaw) - Branch 1C compensated reference trajectory position (Sub 1 -> Sub 7, 14.5 ms)
        SimTK::Vec3 trajectory_velocity{0.0};                      // Size 3 (pitch, roll, yaw) - Branch 1C compensated reference trajectory velocity (Sub 1 -> Sub 7, 14.5 ms)
        SimTK::Vec3 trajectory_acceleration{0.0};                  // Size 3 (pitch, roll, yaw) - Branch 1C compensated reference trajectory acceleration (Sub 1 -> Sub 7, 14.5 ms)
        SimTK::Vec3 trajectory_jerk{0.0};                          // Size 3 (pitch, roll, yaw) - Branch 1C compensated reference trajectory jerk (Sub 1 -> Sub 7, 14.5 ms)
        SimTK::Vec3 trajectory_position_1a{0.0};                   // Size 3 (pitch, roll, yaw) - Branch 1A compensated trajectory position for tonic tilt efference copy (Sub 1 -> Sub 5, 9.0 ms)
        SimTK::Vec3 trajectory_velocity_1a{0.0};                   // Size 3 (pitch, roll, yaw) - Branch 1A compensated trajectory velocity
        SimTK::Vec3 trajectory_acceleration_1a{0.0};               // Size 3 (pitch, roll, yaw) - Branch 1A compensated trajectory acceleration
        SimTK::Vec3 trajectory_velocity_1b{0.0};                   // Size 3 (pitch, roll, yaw) - Branch 1B compensated trajectory velocity for canal efference copy (Sub 1 -> Sub 5, 10.0 ms)
        SimTK::Vec3 trajectory_acceleration_1b{0.0};               // Size 3 (pitch, roll, yaw) - Branch 1B compensated trajectory acceleration for phasic efference copy (Sub 1 -> Sub 5, 10.0 ms)
        SimTK::Vec3 trajectory_jerk_1b{0.0};                       // Size 3 (pitch, roll, yaw) - Branch 1B compensated trajectory jerk
        SimTK::Vec3 trajectory_snap_1b{0.0};                       // Size 3 (pitch, roll, yaw) - Branch 1B compensated trajectory snap
        SimTK::Vec3 eff_tonic_tilt_3a{0.0};                        // Size 3 (pitch, roll, yaw) - Branch 3A compensated tonic tilt efference cancellation (Sub 3C -> Sub 2B, 1.5 ms)
        SimTK::Vec3 eff_canal_vel_3a{0.0};                         // Size 3 (pitch, roll, yaw) - Branch 3A compensated canal velocity efference cancellation (Sub 3C -> Sub 2B, 1.5 ms)
        SimTK::Vec3 eff_phasic_grav_acc_3a{0.0};                   // Size 3 (pitch, roll, yaw) - Branch 3A compensated phasic gravity/pivot cancellation (Sub 3C -> Sub 2B, 1.5 ms)
        SimTK::Vec3 brainstem_vcr_torque{0.0};                     // Size 3 (pitch, roll, yaw) - Path 5 compensated Brainstem VCR torque (Sub 5 -> Sub 7, 2.5 ms)
        SimTK::Vec3 brainstem_vcr_torque_dot{0.0};                 // Size 3 (pitch, roll, yaw) - First derivative of Path 5 Brainstem VCR torque
        SimTK::Vec3 brainstem_vcr_torque_ddot{0.0};                // Size 3 (pitch, roll, yaw) - Second derivative of Path 5 Brainstem VCR torque
        SimTK::Vector spinal_ccr_drive;                            // Size N - Path 6 compensated Spinal CCR excitation drive (Sub 6 -> Sub 7, 4.0 ms)
        SimTK::Vector spinal_ccr_drive_dot;                        // Size N - First derivative of Path 6 Spinal CCR excitation drive
        SimTK::Vector spinal_ccr_drive_ddot;                       // Size N - Second derivative of Path 6 Spinal CCR excitation drive
        SimTK::Vector spinal_motor_excitations;                    // Size N - Branch 7A compensated extrafusal alpha motor command (Sub 7 -> Muscles, 5.5 ms)
        SimTK::Vector spinal_alpha_excitations;                    // Size N - Active-Set QP descending alpha excitations
        SimTK::Vector spinal_gamma_dyn_commands;                   // Size N - Branch 7B compensated Stage 1B dynamic fusimotor command gamma_d(t) (Sub 7 -> Spindles, 8.0 ms)
        SimTK::Vector spinal_gamma_stat_commands;                  // Size N - Branch 7B compensated Stage 1A static fusimotor command gamma_s(t) (Sub 7 -> Spindles, 8.0 ms)
        SimTK::Vector expected_normalized_fiber_lengths;           // Size N - Branch 7B compensated expected normalized fiber length L_hat(t) (Sub 7 -> Spindles, 8.0 ms)
        SimTK::Vector expected_normalized_fiber_velocities;        // Size N - Branch 7B compensated expected normalized fiber velocity Ldot_hat(t) (Sub 7 -> Spindles, 8.0 ms)
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
    mutable SimTK::Matrix proprioceptive_decoder_matrix;           // 3 x N reconstruction matrix: -(J * J^T)^{-1} * J
    mutable SimTK::Vec3 estimated_neck_relative_angle;             // Decoded relative neck angle [pitch, roll, yaw] (rad)
    mutable SimTK::Vec3 estimated_neck_relative_speed;             // Decoded relative neck speed [pitch, roll, yaw] (rad/s)
    mutable SimTK::Vec3 mahony_estimated_angle;                    // Mahony estimated head attitude [pitch, roll, yaw] (rad)
    mutable SimTK::Vec3 mahony_estimated_angular_velocity;         // Mahony estimated angular velocity [pitch, roll, yaw] (rad/s)

    void updateDecoderMatrix() const;

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

    // --- CONTINUOUS ADAPTATION STATE VARIABLES (PATH 1 - 15 PARAMETERS) ---
    // 0: K_ton, 1: p_ton, 2: K_sc, 3: z_sc, 4: p_sc, 5: K_phas, 6: p_phas,
    // 7: kp_prop, 8: z_ccr, 9: p_ccr, 10: K_gamma_dyn, 11: K_gamma_stat,
    // 12: Kp_task_pitch, 13: Kd_task_pitch, 14: Ki_task_pitch
    mutable const StateVariable* sv_adapt_theta[15] = {nullptr};
    mutable const StateVariable* sv_psif_x1[12] = {nullptr};
    mutable const StateVariable* sv_psif_x2[12] = {nullptr};

    mutable double last_error_deg = 0.0;
    mutable double last_ref_pitch_deg = 0.0;
    mutable double last_actual_pitch_deg = 0.0;

    mutable const Coordinate* cached_coordinate_pitch1 = nullptr;
    mutable const Coordinate* cached_coordinate_pitch2 = nullptr;
    mutable const Coordinate* cached_coordinate_roll1 = nullptr;
    mutable const Coordinate* cached_coordinate_roll2 = nullptr;
    mutable const Coordinate* cached_coordinate_yaw1 = nullptr;
    mutable const Coordinate* cached_coordinate_yaw2 = nullptr;
    mutable const Coordinate* cached_coordinate_ground_pitch = nullptr;
    mutable const Coordinate* cached_coordinate_ground_roll = nullptr;
    mutable const Coordinate* cached_coordinate_ground_yaw = nullptr;
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

    void evaluateReferenceMileusnicSpindle(
        int muscle_index,
        double L_hat,
        double Ldot_hat,
        double f_dyn_hat,
        double f_stat_hat,
        double f_chain_hat,
        double& r_ia_hat,
        double& r_ii_hat,
        double& kappa_fd) const;

    void initializeGeometryAndBaseline(const SimTK::State& state) const;
};

} // namespace OpenSim

#endif // OPENSIM_HEAD_NECK_NEURAL_CONTROLLER_ADAPTIVE_H_
