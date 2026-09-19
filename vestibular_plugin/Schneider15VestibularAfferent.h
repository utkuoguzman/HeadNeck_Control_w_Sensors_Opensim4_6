#ifndef _Schneider15VestibularAfferent_h_
#define _Schneider15VestibularAfferent_h_

#include <OpenSim/OpenSim.h>
#include "osimVestibularDLL.h"

namespace OpenSim {

class OSIMVESTIBULAR_API Schneider15VestibularAfferent : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(Schneider15VestibularAfferent, ModelComponent);

public:
    // ========================================================================
    // PROPERTIES
    // ========================================================================
    // Semicircular Canal Parameters (Defaults to Regular Afferents)
    OpenSim_DECLARE_PROPERTY(canal_k, double, "Sensitivity (spikes/s) / (deg/s)");
    OpenSim_DECLARE_PROPERTY(canal_T1, double, "Time constant T1 (s)");
    OpenSim_DECLARE_PROPERTY(canal_T2, double, "Time constant T2 (s)");
    OpenSim_DECLARE_PROPERTY(canal_Tc, double, "Time constant Tc (s)");

    // Otolith Parameters (Defaults to Regular Afferents)
    OpenSim_DECLARE_PROPERTY(otolith_KOTO, double, "Otolith sensitivity (ips/g)");
    OpenSim_DECLARE_PROPERTY(otolith_kA, double, "Acceleration sensitivity kA");
    OpenSim_DECLARE_PROPERTY(otolith_tauA, double, "Time constant tauA (s)");
    OpenSim_DECLARE_PROPERTY(otolith_tauM, double, "Time constant tauM (s)");
    OpenSim_DECLARE_PROPERTY(otolith_kv, double, "Velocity sensitivity/fractional exponent kv");
    OpenSim_DECLARE_PROPERTY(otolith_tauV, double, "Time constant tauV (s)");

    // Oustaloup Approximation Parameter
    OpenSim_DECLARE_PROPERTY(oustaloup_order, int,
        "Order of the Oustaloup fractional derivative approximation (default: 3).");

    // Threshold (Deadband) Properties for Leans Illusion
    OpenSim_DECLARE_PROPERTY(canal_threshold, double,
        "Detection threshold for the Semicircular Canals in deg/s (default: 2.0).");
    OpenSim_DECLARE_PROPERTY(otolith_threshold, double,
        "Detection threshold for the Otoliths in Gs (default: 0.005).");

    // Anatomical Orientation & Position Properties
    OpenSim_DECLARE_PROPERTY(location_in_skull, SimTK::Vec3,
        "Physical location of the vestibular sensor in the skull frame (default: 0 0 0).");
    OpenSim_DECLARE_PROPERTY(normal_anterior, SimTK::Vec3,
        "Normal vector for the Anterior semicircular canal (default: right anterior).");
    OpenSim_DECLARE_PROPERTY(normal_posterior, SimTK::Vec3,
        "Normal vector for the Posterior semicircular canal (default: right posterior).");
    OpenSim_DECLARE_PROPERTY(normal_horizontal, SimTK::Vec3,
        "Normal vector for the Horizontal semicircular canal (default: right horizontal).");

    // ========================================================================
    // SOCKETS
    // ========================================================================
    OpenSim_DECLARE_SOCKET(head_frame, PhysicalFrame, "The frame (e.g. skull/head) this vestibular sensor is attached to.");

    // ========================================================================
    // OUTPUTS
    // ========================================================================
    // Semicircular Canal Outputs (Firing rates in 3D local frame: X, Y, Z)
    OpenSim_DECLARE_OUTPUT(canal_firing_rate, SimTK::Vec3, getCanalFiringRate, SimTK::Stage::Dynamics);

    // Otolith Outputs (Firing rates in 3D local frame: X, Y, Z)
    OpenSim_DECLARE_OUTPUT(otolith_firing_rate, SimTK::Vec3, getOtolithFiringRate, SimTK::Stage::Dynamics);

    // ========================================================================
    // METHODS
    // ========================================================================
    Schneider15VestibularAfferent();

    SimTK::Vec3 getCanalFiringRate(const SimTK::State& s) const;
    SimTK::Vec3 getOtolithFiringRate(const SimTK::State& s) const;

protected:
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    void extendInitStateFromProperties(SimTK::State& s) const override;
    void computeStateVariableDerivatives(const SimTK::State& s) const override;

private:
    void constructProperties();

    // Helper functions to precalculate Oustaloup filter coefficients
    void computeOustaloupFilter(double alpha, double tauV, int N, std::vector<double>& zeros, std::vector<double>& poles, double& gain) const;

    // Cache variable indices or names can be stored here if needed, 
    // but in OpenSim 4 we can just use State variables directly.
};

} // end namespace OpenSim

#endif // _Schneider15VestibularAfferent_h_

