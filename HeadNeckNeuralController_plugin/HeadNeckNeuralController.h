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
    OpenSim_DECLARE_PROPERTY(delay_time, double, "Neural delay in seconds (default: 0.013)");
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






    // Vestibular & Reflex properties
    OpenSim_DECLARE_PROPERTY(K_ton, double, "Tonic otolith TF Gain");
    OpenSim_DECLARE_PROPERTY(p_ton, double, "Tonic otolith TF Pole");
    OpenSim_DECLARE_PROPERTY(K_sc, double, "Semicircular canal TF Gain");
    OpenSim_DECLARE_PROPERTY(z_sc, double, "Semicircular canal TF Zero");
    OpenSim_DECLARE_PROPERTY(p_sc, double, "Semicircular canal TF Pole");
    OpenSim_DECLARE_PROPERTY(K_phas, double, "Phasic otolith TF Gain");
    OpenSim_DECLARE_PROPERTY(p_phas, double, "Phasic otolith TF Pole");
    OpenSim_DECLARE_PROPERTY(k_p, double, "Muscle-level CCR proportional gain");
    OpenSim_DECLARE_PROPERTY(k_v, double, "Muscle-level CCR derivative gain");

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

    HeadNeckNeuralController();
    
    OpenSim_DECLARE_OUTPUT(omega_recon_pitch, double, getOmegaReconPitch, SimTK::Stage::Dynamics);
    double getOmegaReconPitch(const SimTK::State& s) const {
        if(history.empty()) return 0.0;
        auto it = history.lower_bound(s.getTime());
        if(it == history.end()) {
            if (history.empty()) return 0.0;
            --it;
        }
        if (it->second.omega_recon.size() < 3) return 0.0;
        return it->second.omega_recon[2];
    }

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
        SimTK::Vector theta;       // size 3 (roll, yaw, pitch)
        SimTK::Vector omega_recon; // size 3
        SimTK::Vec3 v_lin;         // Linear velocity of the head
        SimTK::Vector L;           // size N
        SimTK::Vector L_dot;       // size N
        SimTK::Vector Ia;          // size N (Spindle Firing Rate)
    };
    
    struct DelayedData {
        HistRecord record;
        SimTK::Vec3 a_lin;         // Extracted linear acceleration
        SimTK::Vector alpha_recon; // Extracted angular acceleration
    };
    
    mutable std::map<double, HistRecord> history;
    
    DelayedData getDelayedData(double t, double delay) const;
    
    mutable SimTK::Matrix J_moment_arms; // 6 x N (pitch1, pitch2, roll1, roll2, yaw1, yaw2)
    
    mutable SimTK::Vector last_tau_des; // Store commanded torque for Efference Copy
    
    mutable std::vector<int> active_muscle_indices;
    mutable std::vector<std::string> muscle_names;
    mutable std::vector<double> base_activations;
    mutable std::vector<double> base_lengths;
    mutable bool is_initialized;
    mutable double last_jacobian_update_time;
    mutable SimTK::Vec3 baseline_spatial_orientation; // roll, yaw, pitch
    
    void initializeGeometryAndBaseline(const SimTK::State& s) const;
};

} // namespace OpenSim

#endif // OPENSIM_HEAD_NECK_NEURAL_CONTROLLER_H_
