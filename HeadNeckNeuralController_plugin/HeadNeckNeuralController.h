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
    OpenSim_DECLARE_PROPERTY(delay_time, double, "Neural delay in seconds (default: 0.015)");
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
    
    // Continuous Vigor logic
    OpenSim_DECLARE_PROPERTY(k_vigor, double, "Continuous Vigor multiplier for velocity gating");
    OpenSim_DECLARE_PROPERTY(Ki_task, double, "Integral gain for gravity compensation (default: 6.54)");
    // Cerebellar dynamic approximation toggle
    OpenSim_DECLARE_PROPERTY(use_dynamic_jacobian, bool, "If true, recalculates the TNT Metric Tensor at every step");
    OpenSim_DECLARE_PROPERTY(K_eso, double, "Gain for Extended State Observer disturbance rejection (default: 0.0)");
    OpenSim_DECLARE_PROPERTY(enable_eye_afferents, bool, "If true, includes extraocular eye muscles (r_*, l_*) in controller state allocation (default: false)");

    HeadNeckNeuralController();
    
    OpenSim_DECLARE_OUTPUT(omega_recon_pitch, double, getOmegaReconPitch, SimTK::Stage::Dynamics);
    double getOmegaReconPitch(const SimTK::State& s) const {
        if(history.empty()) return 0.0;
        auto it = history.lower_bound(s.getTime());
        if(it == history.end()) {
            if (history.empty()) return 0.0;
            --it;
        }
        if (!it->second.valid) return 0.0;
        return it->second.omega_recon[2];
    }

    // Shared 3D Vestibular Angular Velocity Output [roll, yaw, pitch] (rad/s) for VOR FixationController
    OpenSim_DECLARE_OUTPUT(vestibular_omega, SimTK::Vec3, getVestibularOmega, SimTK::Stage::Velocity);
    SimTK::Vec3 getVestibularOmega(const SimTK::State& s) const {
        double wr = sv_luen_omega[0] ? sv_luen_omega[0]->getValue(s) : getStateVariableValue(s, "luen_omega_roll");
        double wy = sv_luen_omega[1] ? sv_luen_omega[1]->getValue(s) : getStateVariableValue(s, "luen_omega_yaw");
        double wp = sv_luen_omega[2] ? sv_luen_omega[2]->getValue(s) : getStateVariableValue(s, "luen_omega_pitch");
        return SimTK::Vec3(wr, wy, wp);
    }

    // Decoded proprioceptive relative state OpenSim Outputs (accessible by reporters and python scripts)
    OpenSim_DECLARE_OUTPUT(est_neck_pitch, double, getOutputNeckPitch, SimTK::Stage::Velocity);
    double getOutputNeckPitch(const SimTK::State& s) const { return est_neck_rel_angle[0]; }

    OpenSim_DECLARE_OUTPUT(est_neck_roll, double, getOutputNeckRoll, SimTK::Stage::Velocity);
    double getOutputNeckRoll(const SimTK::State& s) const { return est_neck_rel_angle[1]; }

    OpenSim_DECLARE_OUTPUT(est_neck_yaw, double, getOutputNeckYaw, SimTK::Stage::Velocity);
    double getOutputNeckYaw(const SimTK::State& s) const { return est_neck_rel_angle[2]; }

    OpenSim_DECLARE_OUTPUT(est_neck_pitch_speed, double, getOutputNeckPitchSpeed, SimTK::Stage::Velocity);
    double getOutputNeckPitchSpeed(const SimTK::State& s) const { return est_neck_rel_speed[0]; }

    OpenSim_DECLARE_OUTPUT(est_neck_roll_speed, double, getOutputNeckRollSpeed, SimTK::Stage::Velocity);
    double getOutputNeckRollSpeed(const SimTK::State& s) const { return est_neck_rel_speed[1]; }

    OpenSim_DECLARE_OUTPUT(est_neck_yaw_speed, double, getOutputNeckYawSpeed, SimTK::Stage::Velocity);
    double getOutputNeckYawSpeed(const SimTK::State& s) const { return est_neck_rel_speed[2]; }

    // Mahony Spatial Observer OpenSim Outputs (accessible by reporters and python scripts)
    OpenSim_DECLARE_OUTPUT(mahony_pitch, double, getMahonyPitch, SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(mahony_roll, double, getMahonyRoll, SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(mahony_yaw, double, getMahonyYaw, SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(mahony_pitch_speed, double, getMahonyPitchSpeed, SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(mahony_roll_speed, double, getMahonyRollSpeed, SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(mahony_yaw_speed, double, getMahonyYawSpeed, SimTK::Stage::Velocity);

    double getMahonyPitch(const SimTK::State& s) const;
    double getMahonyRoll(const SimTK::State& s) const;
    double getMahonyYaw(const SimTK::State& s) const;
    double getMahonyPitchSpeed(const SimTK::State& s) const;
    double getMahonyRollSpeed(const SimTK::State& s) const;
    double getMahonyYawSpeed(const SimTK::State& s) const;

    // Decoded proprioceptive relative state (passive observer / multisensory fusion)
    const SimTK::Vec3& getEstimatedNeckRelativeAngle() const { return est_neck_rel_angle; }
    const SimTK::Vec3& getEstimatedNeckRelativeSpeed() const { return est_neck_rel_speed; }
    double getEstimatedNeckPitch() const { return est_neck_rel_angle[0]; }
    double getEstimatedNeckRoll() const { return est_neck_rel_angle[1]; }
    double getEstimatedNeckYaw() const { return est_neck_rel_angle[2]; }
    double getEstimatedNeckPitchSpeed() const { return est_neck_rel_speed[0]; }
    double getEstimatedNeckRollSpeed() const { return est_neck_rel_speed[1]; }
    double getEstimatedNeckYawSpeed() const { return est_neck_rel_speed[2]; }

    void computeControls(const SimTK::State& s, SimTK::Vector& controls) const override;
    void computeStateVariableDerivatives(const SimTK::State& s) const override;
    
protected:
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    void extendConnectToModel(Model& model) override;
    void extendInitStateFromProperties(SimTK::State& s) const override;

private:
    struct FilterParams {
        double a0, a1, b1, c0, c1, d;
    };
    FilterParams fparams[2];

    struct HistRecord {
        bool valid = false;
        SimTK::Vec3 theta{0.0};        // (roll, yaw, pitch) - Head global orientation
        SimTK::Vec3 omega_recon{0.0};  // Head global angular velocity
        SimTK::Vec3 theta_trunk{0.0};  // (roll, yaw, pitch) - Trunk global orientation
        SimTK::Vec3 omega_trunk{0.0};  // (roll, yaw, pitch) - Trunk global angular velocity
        SimTK::Vec3 v_lin{0.0};        // Linear velocity of the head
        SimTK::Vector Ia;              // size N (Spindle Firing Rate)
        SimTK::Vec3 est_neck_rel_angle{0.0}; // size 3 (pitch, roll, yaw)
        SimTK::Vec3 est_neck_rel_speed{0.0}; // size 3 (pitch, roll, yaw)
    };
    
    struct DelayedData {
        HistRecord record;
        SimTK::Vec3 a_lin{0.0};        // Extracted linear acceleration
        SimTK::Vec3 alpha_recon{0.0};  // Extracted angular acceleration
    };
    
    mutable std::map<double, HistRecord> history;
    
    DelayedData getDelayedData(double t, double delay) const;
    
    mutable SimTK::Matrix J_moment_arms; // 3 x N (pitch, roll, yaw)
    mutable SimTK::Matrix J_6dof_moment_arms; // 6 x N (pitch1, pitch2, roll1, roll2, yaw1, yaw2)
    mutable SimTK::Matrix K_proprio_decode; // 3 x N reconstruction matrix: -(J * J^T)^{-1} * J
    mutable SimTK::Vec3 est_neck_rel_angle; // Decoded relative neck angle [pitch, roll, yaw] (rad)
    mutable SimTK::Vec3 est_neck_rel_speed; // Decoded relative neck speed [pitch, roll, yaw] (rad/s)
    mutable SimTK::Vec3 mahony_angle;       // Mahony estimated head attitude [pitch, roll, yaw] (rad)
    mutable SimTK::Vec3 mahony_speed;       // Mahony estimated angular velocity [pitch, roll, yaw] (rad/s)
    
    void updateDecoderMatrix() const;
    
    mutable SimTK::Vector last_tau_des; // Store commanded torque for Efference Copy
    
    mutable std::vector<int> active_muscle_indices;
    mutable std::vector<std::string> muscle_names;
    mutable std::vector<std::string> name_ccr_state;
    mutable std::vector<std::string> name_sp_dL;
    mutable std::vector<std::string> name_sp_v;
    mutable std::vector<std::string> name_sp_fdyn;
    mutable std::vector<std::string> name_sp_fstat;
    mutable std::vector<const StateVariable*> sv_ccr_state;
    mutable std::vector<const StateVariable*> sv_sp_dL;
    mutable std::vector<const StateVariable*> sv_sp_v;
    mutable std::vector<const StateVariable*> sv_sp_fdyn;
    mutable std::vector<const StateVariable*> sv_sp_fstat;
    mutable const StateVariable* sv_canal[6][2][2] = {{{nullptr}}};
    mutable const StateVariable* sv_traj_pos[3] = {nullptr};
    mutable const StateVariable* sv_traj_vel[3] = {nullptr};
    mutable const StateVariable* sv_err_int[3] = {nullptr};
    mutable const StateVariable* sv_vol_filt[3] = {nullptr};
    mutable const StateVariable* sv_luen_omega[3] = {nullptr};
    mutable const StateVariable* sv_obs_q[4] = {nullptr};
    mutable const StateVariable* sv_obs_v[3] = {nullptr};
    mutable const StateVariable* sv_obs_p[3] = {nullptr};
    mutable const StateVariable* sv_ton[3] = {nullptr};
    mutable const StateVariable* sv_sc[3] = {nullptr};
    mutable const StateVariable* sv_phas[3] = {nullptr};
    mutable const Coordinate* cached_coord_p1 = nullptr;
    mutable const Coordinate* cached_coord_p2 = nullptr;
    mutable const Coordinate* cached_coord_r1 = nullptr;
    mutable const Coordinate* cached_coord_r2 = nullptr;
    mutable const Coordinate* cached_coord_y1 = nullptr;
    mutable const Coordinate* cached_coord_y2 = nullptr;
    mutable const Coordinate* cached_coord_gnd_p = nullptr;
    mutable const Coordinate* cached_coord_gnd_r = nullptr;
    mutable const Coordinate* cached_coord_gnd_y = nullptr;
    mutable const Body* cached_skull = nullptr;
    SimTK::Mat33 B_R_mat;
    SimTK::Mat33 B_L_mat;
    SimTK::Mat33 B_R_inv_mat;
    SimTK::Mat33 B_L_inv_mat;
    mutable std::vector<const OpenSim::Output<double>*> cached_Ia_out;
    mutable std::vector<const OpenSim::Output<double>*> cached_II_out;
    mutable std::vector<double> cached_scale_L;
    mutable std::vector<double> cached_scale_Ia;
    mutable std::vector<double> cached_F_max;
    mutable std::vector<double> base_activations;
    mutable std::vector<double> base_Ia;
    mutable std::vector<double> base_II;
    mutable std::vector<double> last_u_alpha;
    mutable bool is_initialized;
    mutable double last_jacobian_update_time;
    mutable SimTK::Vec3 baseline_spatial_orientation; // roll, yaw, pitch
    
    void initializeGeometryAndBaseline(const SimTK::State& s) const;
};

} // namespace OpenSim

#endif // OPENSIM_HEAD_NECK_NEURAL_CONTROLLER_H_
