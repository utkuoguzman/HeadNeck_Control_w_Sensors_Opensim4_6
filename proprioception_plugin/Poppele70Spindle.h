#pragma once
#include "OpenSim/Simulation/Model/ModelComponent.h"
#include "OpenSim/Simulation/Model/Model.h"
#include <OpenSim/Simulation/Model/Muscle.h>

namespace OpenSim {

class Poppele70Spindle : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(Poppele70Spindle, ModelComponent);

friend class Millard12EqMuscleWithIntermediateAfferent;

public:
    Poppele70Spindle();

    // Outputs
    OpenSim_DECLARE_OUTPUT(primary_Ia, double, getIaOutput, SimTK::Stage::Dynamics);

    // Get output (Firing Rate)
    double getIaOutput(const SimTK::State& s) const;

    // State Accessors
    double getPoppeleX1(const SimTK::State& s) const;
    void setPoppeleX1(SimTK::State& s, double val) const;
    double getPoppeleX2(const SimTK::State& s) const;
    void setPoppeleX2(SimTK::State& s, double val) const;
    double getPoppeleX3(const SimTK::State& s) const;
    void setPoppeleX3(SimTK::State& s, double val) const;
    double getPoppeleX4(const SimTK::State& s) const;
    void setPoppeleX4(SimTK::State& s, double val) const;

    void setOwnerMuscleName(std::string name);

protected:
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    void extendInitStateFromProperties(SimTK::State& s) const override;
    void extendConnectToModel(Model& aModel) override;
    void computeStateVariableDerivatives(const SimTK::State& s) const override;

private:
    std::string ownerMuscleName;
    SimTK::ReferencePtr<const Muscle> musclePtr;
    void setIaOutput(const SimTK::State& s, double IaOutput) const;
};

} // namespace OpenSim

