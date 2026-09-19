#ifndef OPENSIM_MILLARD_12_EQ_MUSCLE_WITH_SIMPLIFIED_AFFERENT_H_
#define OPENSIM_MILLARD_12_EQ_MUSCLE_WITH_SIMPLIFIED_AFFERENT_H_

#include <OpenSim/OpenSim.h>

#ifdef _WIN32
    #ifdef OSIMMILLARD12EQWITHAFF_EXPORTS
        #define OSIMMILLARD12EQWITHAFF_API __declspec(dllexport)
    #else
        #define OSIMMILLARD12EQWITHAFF_API __declspec(dllimport)
    #endif
#else
    #define OSIMMILLARD12EQWITHAFF_API
#endif

namespace OpenSim {

class OSIMMILLARD12EQWITHAFF_API Millard12EqMuscleWithSimplifiedAfferent : public Millard2012EquilibriumMuscle {
    OpenSim_DECLARE_CONCRETE_OBJECT(Millard12EqMuscleWithSimplifiedAfferent, Millard2012EquilibriumMuscle);

public:
    OpenSim_DECLARE_PROPERTY(lpf_tau, double, "time constant for all the low-pass filters");

private:
    mutable SimTK::Vec<3> vel, ts; 
    mutable SimTK::Mat33 C0;
    mutable SimTK::Mat44 C1;
    
public:
    Millard12EqMuscleWithSimplifiedAfferent();
    Millard12EqMuscleWithSimplifiedAfferent(const std::string &name, double maxIsometricForce, 
                                            double optimalFiberLength, double tendonSlackLength, double pennationAngle);

    void constructProperties();

    // Muscle states and their derivatives 
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    void extendInitStateFromProperties(SimTK::State& s) const override;
    void extendSetPropertiesFromState(const SimTK::State& s) override;

    void computeInitialFiberEquilibrium(SimTK::State& s) const override;
    void computeStateVariableDerivatives(const SimTK::State& s) const override;

    // Afferent Outputs
    double getIaAfferent(const SimTK::State& s) const;
    double getIbAfferent(const SimTK::State& s) const;

    // Auxiliary
    double getLPFvelocity(const SimTK::State& s) const;
    void setLPFvelocity(SimTK::State& s, double Velocity) const;
    double getLPFacceleration(const SimTK::State& s) const;
    void setLPFacceleration(SimTK::State& s, double Acceleration) const;
    double approxFiberAcceleration(const SimTK::State& s) const;
};

} // end of namespace OpenSim

#endif // OPENSIM_MILLARD_12_EQ_MUSCLE_WITH_SIMPLIFIED_AFFERENT_H_

