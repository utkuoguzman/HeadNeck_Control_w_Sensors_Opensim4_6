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
    OpenSim_DECLARE_PROPERTY(z_phas, double, "Phasic otolith TF Zero in rad/s (if > 0.0, enables Fernandez & Goldberg 1976 irregular otolith phase-lead filter; default: -1.0)");
    OpenSim_DECLARE_PROPERTY(p_phas, double, "Phasic otolith TF Pole");
    OpenSim_DECLARE_PROPERTY(p_phas2, double, "Second-order phasic otolith TF Pole in rad/s (if <= 0.0, bypasses second pole; default: -1.0)");
    OpenSim_DECLARE_PROPERTY(k_p, double, "Muscle-level CCR proportional gain");

    // Per-Axis Vestibular (VCR) & Proprioceptive (CCR) Reflex Properties (Roll & Yaw independent paths; default <= -999 or < 0 falls back to base)
    OpenSim_DECLARE_PROPERTY(K_ton_roll, double, "Roll Tonic otolith TF Gain (default: -1000.0 -> uses K_ton)");
    OpenSim_DECLARE_PROPERTY(K_ton_yaw, double, "Yaw Tonic attitude TF Gain (default: -1000.0 -> uses K_ton)");
    OpenSim_DECLARE_PROPERTY(p_ton_roll, double, "Roll Tonic otolith TF Pole in rad/s (default: -1.0 -> uses p_ton)");
    OpenSim_DECLARE_PROPERTY(p_ton_yaw, double, "Yaw Tonic attitude TF Pole in rad/s (default: -1.0 -> uses p_ton)");
    OpenSim_DECLARE_PROPERTY(K_sc_roll, double, "Roll Semicircular canal TF Gain (default: -1000.0 -> uses K_sc)");
    OpenSim_DECLARE_PROPERTY(K_sc_yaw, double, "Yaw Semicircular canal TF Gain (default: -1000.0 -> uses K_sc)");
    OpenSim_DECLARE_PROPERTY(z_sc_roll, double, "Roll Semicircular canal TF Zero in s (default: -1.0 -> 0.0)");
    OpenSim_DECLARE_PROPERTY(z_sc_yaw, double, "Yaw Semicircular canal TF Zero in s (default: -1.0 -> 0.0)");
    OpenSim_DECLARE_PROPERTY(p_sc_roll, double, "Roll Semicircular canal TF Pole in rad/s (default: -1.0 -> uses p_sc)");
    OpenSim_DECLARE_PROPERTY(p_sc_yaw, double, "Yaw Semicircular canal TF Pole in rad/s (default: -1.0 -> uses p_sc)");
    OpenSim_DECLARE_PROPERTY(K_phas_roll, double, "Roll Phasic otolith TF Gain (signed; default: -1000.0 -> uses K_phas)");
    OpenSim_DECLARE_PROPERTY(z_phas_roll, double, "Roll Phasic otolith TF Zero in s (default: -1.0 -> 0.0)");
    OpenSim_DECLARE_PROPERTY(p_phas_roll, double, "Roll Phasic otolith TF Pole in rad/s (default: -1.0 -> uses p_phas)");
    OpenSim_DECLARE_PROPERTY(p_ap_roll, double, "Roll low-frequency phasic otolith all-pass/lag pole in rad/s (default: -1.0 -> disabled)");
    OpenSim_DECLARE_PROPERTY(c_ap_roll, double, "Roll low-frequency phasic otolith all-pass/lag weight (default: 0.0 -> disabled)");
    OpenSim_DECLARE_PROPERTY(Kp_prop_roll, double, "Roll Spindle-Decoded CCR Proportional Gain in Nm/rad (default: -1000.0 -> 0.0)");
    OpenSim_DECLARE_PROPERTY(Kp_prop_yaw, double, "Yaw Spindle-Decoded CCR Proportional Gain in Nm/rad (default: -1000.0 -> 0.0)");
    OpenSim_DECLARE_PROPERTY(z_ccr_roll, double, "Roll Spindle-Decoded CCR Lead Zero in s (default: -1.0 -> 0.0)");
    OpenSim_DECLARE_PROPERTY(z_ccr_yaw, double, "Yaw Spindle-Decoded CCR Lead Zero in s (default: -1.0 -> 0.0)");
    OpenSim_DECLARE_PROPERTY(p_ccr_roll, double, "Roll CCR TF Pole in rad/s (default: -1.0 -> uses p_ccr)");
    OpenSim_DECLARE_PROPERTY(p_ccr_yaw, double, "Yaw CCR TF Pole in rad/s (default: -1.0 -> uses p_ccr)");
    OpenSim_DECLARE_PROPERTY(deriv_comp_roll, double, "Roll high-frequency Taylor derivative compensator scale (default: 1.0)");
    OpenSim_DECLARE_PROPERTY(deriv_comp_yaw, double, "Yaw high-frequency Taylor derivative compensator scale (default: 1.0)");

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
        SimTK::Vec3 estimated_head_orientation{0.0};               // Branch 4B (13.0ms -> Sub 1): (roll, yaw, pitch) compensated by 1 + tau_4B*s + 0.5*tau_4B^2*s^2
        SimTK::Vec3 estimated_head_orientation_4a{0.0};            // Branch 4A (1.5ms -> Sub 5): (roll, yaw, pitch) compensated by 1 + tau_4A*s + 0.5*tau_4A^2*s^2
        SimTK::Vec3 reconstructed_canal_angular_velocity{0.0};     // Branch 2A (2.0ms -> Sub 5): irregular canal velocity compensated by 1 + tau_2A*s + 0.5*tau_2A^2*s^2
        SimTK::Vec3 luenberger_angular_velocity{0.0};              // Branch 2B (3.0ms -> Sub 4): regular canal Luenberger velocity compensated by 1 + tau_2B*s + 0.5*tau_2B^2*s^2
        SimTK::Vec3 luenberger_angular_acceleration{0.0};          // Branch 2B (3.0ms -> Sub 4): regular canal Luenberger acceleration compensated by 1 + tau_2B*s
        SimTK::Vec3 reconstructed_otolith_specific_force{0.0};     // Branch 2A (2.0ms -> Sub 5): irregular otolith GIF compensated by 1 + tau_2A*s + 0.5*tau_2A^2*s^2
        SimTK::Vec3 luenberger_otolith_specific_force{0.0};        // Branch 2B (3.0ms -> Sub 4): regular otolith Luenberger GIF compensated by 1 + tau_2B*s + 0.5*tau_2B^2*s^2
        SimTK::Mat33 rotation_world_to_head{1.0};                  // True skull rotation from world to head at record timestamp
        SimTK::Mat33 estimated_rotation_head_to_world{1.0};        // Branch 4A (1.5ms -> Sub 5) / 2B: Observer estimated rotation R_hat_H^W compensated by 1 + tau_4A*s + 0.5*tau_4A^2*s^2
        SimTK::Vec3 trunk_orientation{0.0};                        // Branch 3B (7.0ms -> Sub 4): (roll, yaw, pitch) trunk orientation compensated by 1 + tau_3B*s + 0.5*tau_3B^2*s^2
        SimTK::Vec3 trunk_angular_velocity{0.0};                   // Branch 3B (7.0ms -> Sub 4): (roll, yaw, pitch) trunk angular velocity
        SimTK::Vec3 head_linear_velocity{0.0};                     // Linear velocity of the head in ground frame
        SimTK::Vector primary_afferent_firing_rate;                // Branch 3A (5.5ms -> Sub 6): Size N clean spindle Ia rate compensated by 1 + tau_3A*s + 0.5*tau_3A^2*s^2
        SimTK::Vec3 estimated_neck_relative_angle{0.0};            // Branch 3B (7.0ms -> Sub 4): Size 3 (pitch, roll, yaw) compensated by 1 + tau_3B*s + 0.5*tau_3B^2*s^2
        SimTK::Vec3 estimated_neck_relative_speed{0.0};            // Branch 3B (7.0ms -> Sub 4): Size 3 (pitch, roll, yaw) compensated by 1 + tau_3B*s + 0.5*tau_3B^2*s^2
        SimTK::Vec3 raw_neck_relative_angle{0.0};                  // Clean Luenberger-filtered neck relative angle (pitch, roll, yaw) before Branch 3B jerk compensator
        SimTK::Vec3 raw_neck_relative_speed{0.0};                  // Clean Luenberger-filtered neck relative speed (pitch, roll, yaw) before Branch 3B jerk compensator
        SimTK::Vec3 mahony_estimated_angular_velocity{0.0};        // Branch 4B (13.0ms -> Sub 1): Size 3 (pitch, roll, yaw) compensated by 1 + tau_4B*s + 0.5*tau_4B^2*s^2
        SimTK::Vec3 cortical_voluntary_torque{0.0};                // Branch 1C (14.5ms -> Sub 7): Size 3 (pitch, roll, yaw) compensated by 1 + tau_1C*s + 0.5*tau_1C^2*s^2
        SimTK::Vec3 cortical_voluntary_torque_dot{0.0};            // Branch 1C (14.5ms -> Sub 7): Size 3 1st derivative of cortical voluntary torque
        SimTK::Vec3 cortical_voluntary_torque_ddot{0.0};           // Branch 1C (14.5ms -> Sub 7): Size 3 2nd derivative of cortical voluntary torque
        SimTK::Vec3 trajectory_position_1a{0.0};                   // Branch 4A (9.0ms -> Sub 3C Tonic): Size 3 (pitch, roll, yaw) compensated by 1 + tau_4A*s + 0.5*tau_4A^2*s^2
        SimTK::Vec3 trajectory_velocity_1a{0.0};                   // Branch 4A (9.0ms -> Sub 3C Tonic): Size 3 1st derivative of trajectory_position_1a
        SimTK::Vec3 trajectory_acceleration_1a{0.0};               // Branch 4A (9.0ms -> Sub 3C Tonic): Size 3 2nd derivative of trajectory_position_1a
        SimTK::Vec3 trajectory_velocity_1b{0.0};                   // Branch 4B (10.0ms -> Sub 3C Canal): Size 3 (pitch, roll, yaw) compensated by 1 + tau_4B*s + 0.5*tau_4B^2*s^2
        SimTK::Vec3 trajectory_acceleration_1b{0.0};               // Branch 4B (10.0ms -> Sub 3C Phasic): Size 3 (pitch, roll, yaw) compensated by 1 + tau_4B*s + 0.5*tau_4B^2*s^2
        SimTK::Vec3 trajectory_jerk_1b{0.0};                       // Branch 4B (10.0ms -> Sub 3C): Size 3 1st derivative of trajectory_acceleration_1b
        SimTK::Vec3 trajectory_snap_1b{0.0};                       // Branch 4B (10.0ms -> Sub 3C): Size 3 2nd derivative of trajectory_acceleration_1b
        SimTK::Vec3 eff_tonic_tilt_3a{0.0};                        // Branch 3A (1.5ms -> Sub 2B Tonic): Size 3 (pitch, roll, yaw) compensated by 1 + tau_3A*s + 0.5*tau_3A^2*s^2
        SimTK::Vec3 eff_canal_vel_3a{0.0};                         // Branch 3A (1.5ms -> Sub 2B Canal): Size 3 (pitch, roll, yaw) compensated by 1 + tau_3A*s + 0.5*tau_3A^2*s^2
        SimTK::Vec3 eff_phasic_grav_acc_3a{0.0};                   // Branch 3A (1.5ms -> Sub 2B Phasic): Size 3 (pitch, roll, yaw) compensated by 1 + tau_3A*s + 0.5*tau_3A^2*s^2
        SimTK::Vec3 trajectory_position{0.0};                      // Branch 1C (14.5ms -> Sub 7): Size 3 (pitch, roll, yaw) compensated by 1 + tau_1C*s + 0.5*tau_1C^2*s^2
        SimTK::Vec3 trajectory_velocity{0.0};                      // Branch 1C (14.5ms -> Sub 7): Size 3 (pitch, roll, yaw) compensated by 1 + tau_1C*s + 0.5*tau_1C^2*s^2
        SimTK::Vec3 trajectory_acceleration{0.0};                  // Branch 1C (14.5ms -> Sub 7): Size 3 (pitch, roll, yaw) compensated by 1 + tau_1C*s + 0.5*tau_1C^2*s^2
        SimTK::Vec3 trajectory_jerk{0.0};                          // Branch 1C (14.5ms -> Sub 7): Size 3 (pitch, roll, yaw) compensated by 1 + tau_1C*s + 0.5*tau_1C^2*s^2
        SimTK::Vec3 brainstem_vcr_torque{0.0};                     // Path 5 (2.5ms MVST -> Sub 7): Size 3 (pitch, roll, yaw) compensated by 1 + tau_5*s + 0.5*tau_5^2*s^2
        SimTK::Vec3 brainstem_vcr_torque_dot{0.0};                 // Path 5 (2.5ms MVST -> Sub 7): Size 3 1st derivative of VCR torque
        SimTK::Vec3 brainstem_vcr_torque_ddot{0.0};                // Path 5 (2.5ms MVST -> Sub 7): Size 3 2nd derivative of VCR torque
        SimTK::Vector spinal_ccr_drive;                            // Path 6 (4.0ms Propriospinal -> Sub 7): Size N CCR excitation compensated by 1 + tau_6*s + 0.5*tau_6^2*s^2
        SimTK::Vector spinal_ccr_drive_dot;                        // Path 6 (4.0ms Propriospinal -> Sub 7): Size N 1st derivative of CCR excitation
        SimTK::Vector spinal_ccr_drive_ddot;                       // Path 6 (4.0ms Propriospinal -> Sub 7): Size N 2nd derivative of CCR excitation
        SimTK::Vector spinal_motor_excitations;                    // Branch 7A (5.5ms A-alpha -> 72 Muscles): Size N total pool alpha excitations compensated by 1 + tau_7A*s + 0.5*tau_7A^2*s^2
        SimTK::Vector spinal_alpha_excitations;                    // Branch 7A (5.5ms A-alpha -> 72 Muscles): Size N Active-Set QP descending alpha excitations
        SimTK::Vector spinal_gamma_dyn_commands;                   // Branch 7B (8.0ms A-gamma -> Sub 3 Spindles): Size N Stage 1B dynamic fusimotor commands compensated by 1 + tau_7B*s + 0.5*tau_7B^2*s^2
        SimTK::Vector spinal_gamma_stat_commands;                  // Branch 7B (8.0ms A-gamma -> Sub 3 Spindles): Size N Stage 1A static fusimotor commands compensated by 1 + tau_7B*s + 0.5*tau_7B^2*s^2
        SimTK::Vector expected_normalized_fiber_lengths;           // Branch 7B (8.0ms A-gamma -> Sub 3 Spindles): Size N expected L_hat(t) compensated by 1 + tau_7B*s + 0.5*tau_7B^2*s^2
        SimTK::Vector expected_normalized_fiber_velocities;        // Branch 7B (8.0ms A-gamma -> Sub 3 Spindles): Size N expected dL_hat/dt(t) compensated by 1 + tau_7B*s + 0.5*tau_7B^2*s^2
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
