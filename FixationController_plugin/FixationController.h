#ifndef FIXATION_CONTROLLER_H
#define FIXATION_CONTROLLER_H

#include "osimFixationDLL.h"
#include <OpenSim/Simulation/Control/Controller.h>
#include <SimTKmath.h>

namespace OpenSim {

class OSIMFIXATION_API FixationController : public Controller {
    OpenSim_DECLARE_CONCRETE_OBJECT(FixationController, Controller);

public:
    OpenSim_DECLARE_PROPERTY(target_location, SimTK::Vec3, "3D location of the fixation target in Skull frame (obsolete if hardcoded).");
    
    OpenSim_DECLARE_PROPERTY(kp_horizontal, double, "Proportional tracking gain (Horizontal)");
    OpenSim_DECLARE_PROPERTY(kd_horizontal, double, "Derivative tracking gain (Horizontal)");
    OpenSim_DECLARE_PROPERTY(kp_vertical, double, "Proportional tracking gain (Vertical)");
    OpenSim_DECLARE_PROPERTY(kd_vertical, double, "Derivative tracking gain (Vertical)");
    OpenSim_DECLARE_PROPERTY(kp_torsion, double, "Proportional tracking gain (Torsion)");
    OpenSim_DECLARE_PROPERTY(kd_torsion, double, "Derivative tracking gain (Torsion)");
    
    OpenSim_DECLARE_PROPERTY(K_vor, double, "Vestibulo-Ocular Reflex Gain");
    OpenSim_DECLARE_PROPERTY(K_cor, double, "Cervico-Ocular Reflex Gain");

    FixationController();
    ~FixationController() override = default;

    void computeControls(const SimTK::State& s, SimTK::Vector& controls) const override;
    void extendConnectToModel(Model& model) override;
    void extendInitStateFromProperties(SimTK::State& s) const override;

private:
    void constructProperties();
    void initializeGeometryAndBaseline(const SimTK::State& s) const;
    
    mutable bool is_initialized{false};
    mutable double last_jacobian_update_time{-1.0};
    
    mutable std::vector<int> r_eye_muscle_indices;
    mutable std::vector<int> l_eye_muscle_indices;
    mutable SimTK::Matrix J_r;
    mutable SimTK::Matrix J_l;
};

}

#endif
