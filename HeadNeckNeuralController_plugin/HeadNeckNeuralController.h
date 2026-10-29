#ifndef OPENSIM_HEAD_NECK_NEURAL_CONTROLLER_H_
#define OPENSIM_HEAD_NECK_NEURAL_CONTROLLER_H_

#include <OpenSim/OpenSim.h>
#include <deque>
#include <vector>
#include <map>

namespace OpenSim {

class HeadNeckNeuralController : public Controller {
OpenSim_DECLARE_CONCRETE_OBJECT(HeadNeckNeuralController, Controller);

public:

    OpenSim_DECLARE_PROPERTY(delay_vcr, double, "Vestibulo-Collic Reflex delay in seconds (default: 0.015)");
    OpenSim_DECLARE_PROPERTY(delay_ccr, double, "Cervico-Collic Reflex delay in seconds (default: 0.020)");
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
    OpenSim_DECLARE_PROPERTY(p_phas, double, "Phasic otolith TF Pole");
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
    OpenSim_DECLARE_PROPERTY(use_smith_predictor, bool, "If true, utilizes Smith Predictor delay compensation for Mahony proprioceptive fusion (default: false)");
    OpenSim_DECLARE_PROPERTY(delay_smith_proprio, double, "Proprioceptive transport delay in seconds for Smith compensation (default: 0.015)");
    OpenSim_DECLARE_PROPERTY(delay_afferent_proprio, double, "Proprioceptive afferent transport delay in seconds arriving at the central observer (default: 0.0)");

    HeadNeckNeuralController();
    
    OpenSim_DECLARE_OUTPUT(omega_recon_pitch, double, getReconstructedCanalAngularVelocityPitch, SimTK::Stage::Dynamics);
    double getReconstructedCanalAngularVelocityPitch(const SimTK::State& state) const {
        if (history.empty()) return 0.0;
        auto iterator = history.lower_bound(state.getTime());
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
        SimTK::Vec3 estimated_head_orientation{0.0};               // (roll, yaw, pitch) - Head global orientation
        SimTK::Vec3 reconstructed_canal_angular_velocity{0.0};     // Head global angular velocity (gain-normalized canal firing with biological phase lead)
        SimTK::Vec3 luenberger_angular_velocity{0.0};              // Clean Luenberger-inverted physical angular velocity (roll, yaw, pitch)
        SimTK::Mat33 rotation_world_to_head{1.0};                  // True skull rotation from world to head at record timestamp
        SimTK::Mat33 estimated_rotation_head_to_world{1.0};        // Observer estimated rotation from head to world at record timestamp
        SimTK::Vec3 trunk_orientation{0.0};                        // (roll, yaw, pitch) - Trunk global orientation
        SimTK::Vec3 trunk_angular_velocity{0.0};                   // (roll, yaw, pitch) - Trunk global angular velocity
        SimTK::Vec3 head_linear_velocity{0.0};                     // Linear velocity of the head in ground frame
        SimTK::Vector primary_afferent_firing_rate;                // Size N (Spindle primary Ia firing rate above baseline)
        SimTK::Vec3 estimated_neck_relative_angle{0.0};            // Size 3 (pitch, roll, yaw)
        SimTK::Vec3 estimated_neck_relative_speed{0.0};            // Size 3 (pitch, roll, yaw)
    };
    
    struct DelayedData {
        HistoryRecord record;
        SimTK::Vec3 linear_acceleration{0.0};                      // Extracted linear acceleration
        SimTK::Vec3 reconstructed_angular_acceleration{0.0};       // Extracted angular acceleration
    };
    
    mutable std::map<double, HistoryRecord> history;
    
    DelayedData getDelayedData(double current_time, double delay_duration) const;
    
    mutable SimTK::Matrix moment_arm_jacobian_3dof;                // 3 x N (pitch, roll, yaw)
    mutable SimTK::Matrix moment_arm_jacobian_6dof;                // 6 x N (pitch1, pitch2, roll1, roll2, yaw1, yaw2)
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
    mutable std::vector<std::string> spindle_length_change_state_names;
    mutable std::vector<std::string> spindle_velocity_state_names;
    mutable std::vector<std::string> spindle_dynamic_fusimotor_state_names;
    mutable std::vector<std::string> spindle_static_fusimotor_state_names;
    mutable std::vector<const StateVariable*> state_variable_cervico_collic;
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
    mutable const StateVariable* state_variable_observer_quaternion[4] = {nullptr};
    mutable const StateVariable* state_variable_observer_velocity[3] = {nullptr};
    mutable const StateVariable* state_variable_observer_position[3] = {nullptr};
    mutable const StateVariable* state_variable_tonic_otolith[3] = {nullptr};
    mutable const StateVariable* state_variable_semicircular_canal_reflex[3] = {nullptr};
    mutable const StateVariable* state_variable_phasic_otolith[3] = {nullptr};
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
    mutable std::vector<double> baseline_muscle_activations;
    mutable std::vector<double> baseline_primary_afferent_rates;
    mutable std::vector<double> baseline_secondary_afferent_rates;
    mutable std::vector<double> last_alpha_motor_excitations;
    mutable bool is_initialized;
    mutable double last_jacobian_update_time;
    mutable SimTK::Vec3 baseline_spatial_orientation;              // (roll, yaw, pitch)
    
    void initializeGeometryAndBaseline(const SimTK::State& state) const;
};

} // namespace OpenSim

#endif // OPENSIM_HEAD_NECK_NEURAL_CONTROLLER_H_
