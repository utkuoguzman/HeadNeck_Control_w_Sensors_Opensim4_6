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
    OpenSim_DECLARE_PROPERTY(Kg_proprioception, double, "Gain for GTO Ib");
    OpenSim_DECLARE_PROPERTY(K_vestibular, double, "Gain for Vestibular descending drive");
    OpenSim_DECLARE_PROPERTY(K_otolith, double, "Gain for Otolith descending drive (Static posture)");

    // Voluntary command
    OpenSim_DECLARE_PROPERTY(desired_yaw, double, "Voluntary desired yaw angle for the head (rad)");
    OpenSim_DECLARE_PROPERTY(desired_pitch, double, "Voluntary desired pitch angle for the head (rad)");
    OpenSim_DECLARE_PROPERTY(desired_roll, double, "Voluntary desired roll angle for the head (rad)");

    // Vestibular & Reflex properties
    OpenSim_DECLARE_PROPERTY(G_sc, double, "Vestibular semicircular canal gain");
    OpenSim_DECLARE_PROPERTY(G_ton, double, "Tonic otolith gain");
    OpenSim_DECLARE_PROPERTY(G_phas, double, "Phasic otolith gain");
    OpenSim_DECLARE_PROPERTY(k_p, double, "Muscle-level CCR proportional gain");
    OpenSim_DECLARE_PROPERTY(k_v, double, "Muscle-level CCR derivative gain");

    // Gamma motor neuron drive (Alpha-Gamma Coactivation)
    OpenSim_DECLARE_PROPERTY(K_gamma_dyn, double, "Gain for Vestibular-driven Gamma_dyn");
    OpenSim_DECLARE_PROPERTY(K_gamma_stat, double, "Gain for Alpha-Gamma Coactivation");
    OpenSim_DECLARE_PROPERTY(K_gamma, double, "Gain for Gamma motor neuron dynamic setpoint shift (default: 1.0)");
    OpenSim_DECLARE_PROPERTY(Kp_task, double, "Task-Space Proportional Gain (Kp) for 3D Torque computation");
    OpenSim_DECLARE_PROPERTY(Ki_task, double, "Task-Space Integral Gain (Ki) for posture maintenance against gravity");
    OpenSim_DECLARE_PROPERTY(Kd_task, double, "Task-Space Derivative Gain (Kd) for 3D Torque computation");

    // Cerebellar dynamic approximation toggle
    OpenSim_DECLARE_PROPERTY(use_dynamic_jacobian, bool, "If true, recalculates the TNT Metric Tensor at every step");

    HeadNeckNeuralController();

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
    
    void initializeGeometryAndBaseline(const SimTK::State& s) const;
};

} // namespace OpenSim

#endif // OPENSIM_HEAD_NECK_NEURAL_CONTROLLER_H_
