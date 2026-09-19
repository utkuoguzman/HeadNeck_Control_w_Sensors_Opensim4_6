#ifndef FIXATION_CONTROLLER_H
#define FIXATION_CONTROLLER_H

#include "osimFixationDLL.h"
#include <OpenSim/Simulation/Control/Controller.h>

namespace OpenSim {

/**
 * \brief Binocular 3D Fixation Controller (Vergence Tracking)
 *
 * This controller simulates the neural mechanism for binocular convergence and tracking.
 * Rather than generating a pre-computed saccade trajectory, this is a "pure" continuous
 * feedback controller based on biological vergence models (e.g., dual-mode or leaky-integrator 
 * tracking transfer functions). 
 * 
 * It continuously calculates the required Horizontal (Yaw) and Vertical (Pitch) angles 
 * for BOTH the left and right eyes to fixate on a single 3D point in space, driving 
 * vergence (crossing) dynamically based on target depth.
 */
class OSIMFIXATION_API FixationController : public Controller {
    OpenSim_DECLARE_CONCRETE_OBJECT(FixationController, Controller);

public:
    OpenSim_DECLARE_PROPERTY(target_location, SimTK::Vec3, "3D location of the fixation target in Ground frame.");
    
    // Neural gains representing the burst-slide (transfer function) components
    OpenSim_DECLARE_PROPERTY(kp_horizontal, double, "Proportional tracking gain (Horizontal)");
    OpenSim_DECLARE_PROPERTY(kd_horizontal, double, "Derivative tracking gain (Horizontal)");
    OpenSim_DECLARE_PROPERTY(ki_horizontal, double, "Integral tracking gain (Horizontal)");
    OpenSim_DECLARE_PROPERTY(kp_vertical, double, "Proportional tracking gain (Vertical)");
    OpenSim_DECLARE_PROPERTY(kd_vertical, double, "Derivative tracking gain (Vertical)");
    OpenSim_DECLARE_PROPERTY(ki_vertical, double, "Integral tracking gain (Vertical)");
    OpenSim_DECLARE_PROPERTY(kp_torsion, double, "Proportional tracking gain (Torsion - to keep eye level)");
    OpenSim_DECLARE_PROPERTY(kd_torsion, double, "Derivative tracking gain (Torsion)");
    OpenSim_DECLARE_PROPERTY(ki_torsion, double, "Integral tracking gain (Torsion)");
    
    // VOR and COR feed-forward gains
    OpenSim_DECLARE_PROPERTY(K_vor, double, "Vestibulo-Ocular Reflex Gain");
    OpenSim_DECLARE_PROPERTY(K_cor, double, "Cervico-Ocular Reflex Gain");

    FixationController();
    ~FixationController() override = default;

    void computeControls(const SimTK::State& s, SimTK::Vector& controls) const override;

private:
    void constructProperties();
    
    mutable double last_time{0.0};
    mutable double int_err_yaw[2]{0.0, 0.0};
    mutable double int_err_pitch[2]{0.0, 0.0};
    mutable double int_err_torsion[2]{0.0, 0.0};
};

} // namespace OpenSim

#endif // FIXATION_CONTROLLER_H

