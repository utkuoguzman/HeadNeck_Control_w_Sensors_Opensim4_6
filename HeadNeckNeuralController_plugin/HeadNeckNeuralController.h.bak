#ifndef OPENSIM_HEAD_NECK_NEURAL_CONTROLLER_H_
#define OPENSIM_HEAD_NECK_NEURAL_CONTROLLER_H_

#include <OpenSim/OpenSim.h>
#include <deque>
#include <vector>

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

    // Gamma motor neuron drive (Alpha-Gamma Coactivation)
    OpenSim_DECLARE_PROPERTY(K_gamma, double, "Gain for Gamma motor neuron dynamic setpoint shift (default: 1.0)");
    OpenSim_DECLARE_PROPERTY(Kp_task, double, "Task-Space Proportional Gain (Kp) for 3D Torque computation");
    OpenSim_DECLARE_PROPERTY(Kd_task, double, "Task-Space Derivative Gain (Kd) for 3D Torque computation");

    // Cerebellar dynamic approximation toggle
    OpenSim_DECLARE_PROPERTY(use_dynamic_jacobian, bool, "If true, recalculates the TNT Metric Tensor at every step");

    HeadNeckNeuralController();

    void computeControls(const SimTK::State& s, SimTK::Vector& controls) const override;
    
protected:
    void extendConnectToModel(Model& model) override;
    void extendInitStateFromProperties(SimTK::State& s) const override;

private:
    struct Record {
        double time;
        SimTK::Vec3 vestibular_signal; // Canals
        SimTK::Vec3 otolith_signal;    // Otoliths
        std::vector<double> Ia_signals;
        std::vector<double> Ib_signals;
    };
    mutable std::deque<Record> delay_buffer;
    
    mutable SimTK::Matrix J_moment_arms; // 6 x N (pitch1, pitch2, roll1, roll2, yaw1, yaw2)
    
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
