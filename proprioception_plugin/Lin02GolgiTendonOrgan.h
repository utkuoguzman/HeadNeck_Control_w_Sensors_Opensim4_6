#pragma once
#include "OpenSim/Simulation/Model/ModelComponent.h"
#include "OpenSim/Simulation/Model/Model.h"
#include <OpenSim/Simulation/Model/Muscle.h>

namespace OpenSim {

/**
 * Implements an aggregate / single-afferent model of Golgi Tendon Organs (Ib)
 * based on Lin & Crago (2002), Ann. Biomed. Eng. 30:54-67:
 *   1) Logarithmic static nonlinearity driven by total tendon force F_T >= 0.
 *   2) Exact derivative-free 2-state partial-fraction realization of Eq. (2)
 *      H(s) = S * (1.7 - (17/15)/(s + 2.0) - (2/75)/(s + 0.2)) followed by
 *      a fast 2.0 ms (500 rad/s) strictly-proper transduction filter.
 */
class Lin02GolgiTendonOrgan : public ModelComponent {
OpenSim_DECLARE_CONCRETE_OBJECT(Lin02GolgiTendonOrgan, ModelComponent);
friend class Millard12EqMuscleWithAfferents;
friend class Millard12EqMuscleWithIntermediateAfferent;
friend class AfferentAnalysis;

public:
    OpenSim_DECLARE_PROPERTY(lpf_tau, double, "Legacy time constant property preserved for XML compatibility");
    OpenSim_DECLARE_PROPERTY(enable_GTO, bool, "Whether to add and integrate GTO ODE states (default: false)");
    OpenSim_DECLARE_PROPERTY(gf_fraction, double, "GTO force scale Gf as fraction of owner F_max (<=0 uses legacy 4.0 N; default: 0.08)");
    OpenSim_DECLARE_PROPERTY(single_afferent_scale, bool, "If true outputs single-Ib rate in imp/s (1x); if false outputs 40x aggregate rate (default: true)");

    Lin02GolgiTendonOrgan();

    void setOwnerMuscleName(std::string OwnerMuscleName);

    double getGTOout(const SimTK::State& s) const;

    OpenSim_DECLARE_OUTPUT(aggregate_firing_rate, double, getGTOout, SimTK::Stage::Velocity);

    double getLPFtau() const { return get_lpf_tau(); }
    void setLPFtau(double aLPFtau);

protected:
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    void extendInitStateFromProperties(SimTK::State& s) const override;
    void extendSetPropertiesFromState(const SimTK::State& s) override;
    void extendConnectToModel(Model& aModel) override;

    double Gg, Gf;
    double thr;

    bool isGTOActive() const;
    void initFromMuscle(SimTK::State& s) const;
    void computeStateVariableDerivatives(const SimTK::State& s) const override;

private:
    void constructProperties();
    double computeNonlinearDrive(const SimTK::State& s) const;

    std::string ownerMuscleName;
    const Muscle* musclePtr = nullptr;

    mutable const StateVariable* sv_gto_w1 = nullptr;
    mutable const StateVariable* sv_gto_w2 = nullptr;
    mutable const StateVariable* sv_gto_out = nullptr;
    void ensureStatePointers() const;
};

} // end of namespace OpenSim
