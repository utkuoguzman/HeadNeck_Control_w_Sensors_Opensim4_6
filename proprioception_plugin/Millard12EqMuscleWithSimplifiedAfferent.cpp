#include "Millard12EqMuscleWithSimplifiedAfferent.h"
#include <iostream>
#include <cmath>

using namespace std;
using namespace OpenSim;

Millard12EqMuscleWithSimplifiedAfferent::Millard12EqMuscleWithSimplifiedAfferent()
{
    constructProperties();
}

Millard12EqMuscleWithSimplifiedAfferent::Millard12EqMuscleWithSimplifiedAfferent(const std::string &name, double maxIsometricForce, double optimalFiberLength, double tendonSlackLength, double pennationAngle)
:Super(name, maxIsometricForce, optimalFiberLength, tendonSlackLength, pennationAngle)
{
    constructProperties();
}

void Millard12EqMuscleWithSimplifiedAfferent::constructProperties()
{
    constructProperty_lpf_tau(0.01);
}

void Millard12EqMuscleWithSimplifiedAfferent::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);
    addStateVariable("LPF_velocity");
    addStateVariable("LPF_acceleration");
}

void Millard12EqMuscleWithSimplifiedAfferent::extendInitStateFromProperties(SimTK::State& s) const
{
    Super::extendInitStateFromProperties(s);
    setLPFvelocity(s, 0.0);
    setLPFacceleration(s, 0.0);
}

void Millard12EqMuscleWithSimplifiedAfferent::extendSetPropertiesFromState(const SimTK::State& s)
{
    Super::extendSetPropertiesFromState(s);
}

void Millard12EqMuscleWithSimplifiedAfferent::computeInitialFiberEquilibrium(SimTK::State& s) const
{
    Super::computeInitialFiberEquilibrium(s);
    setLPFvelocity(s, getFiberVelocity(s));
    setLPFacceleration(s, 0.0); 
}

void Millard12EqMuscleWithSimplifiedAfferent::computeStateVariableDerivatives(const SimTK::State& s) const
{
    Super::computeStateVariableDerivatives(s);
    
    // Cascaded filter to get smooth velocity and acceleration
    double curr_accel = (getFiberVelocity(s) - getLPFvelocity(s)) / get_lpf_tau();
    setStateVariableDerivativeValue(s, "LPF_velocity", curr_accel);
    setStateVariableDerivativeValue(s, "LPF_acceleration", (curr_accel - getLPFacceleration(s)) / get_lpf_tau());
}

double Millard12EqMuscleWithSimplifiedAfferent::approxFiberAcceleration(const SimTK::State& s) const
{
    return (getFiberVelocity(s) - getLPFvelocity(s)) / get_lpf_tau();
}

double Millard12EqMuscleWithSimplifiedAfferent::getLPFvelocity(const SimTK::State& s) const { return getStateVariableValue(s, "LPF_velocity"); }
void Millard12EqMuscleWithSimplifiedAfferent::setLPFvelocity(SimTK::State& s, double Velocity) const { setStateVariableValue(s, "LPF_velocity", Velocity); }
double Millard12EqMuscleWithSimplifiedAfferent::getLPFacceleration(const SimTK::State& s) const { return getStateVariableValue(s, "LPF_acceleration"); }
void Millard12EqMuscleWithSimplifiedAfferent::setLPFacceleration(SimTK::State& s, double Acceleration) const { setStateVariableValue(s, "LPF_acceleration", Acceleration); }

// -------------------------------------------------------------------------
// Simplified Afferent Models from Zhang et al. 2020
// -------------------------------------------------------------------------
double Millard12EqMuscleWithSimplifiedAfferent::getIaAfferent(const SimTK::State& s) const
{
    // Extract normalized kinematics
    double lopt = getOptimalFiberLength();
    double L = getFiberLength(s) / lopt;
    double V = getLPFvelocity(s) / lopt;
    double A = getLPFacceleration(s) / lopt;

    // Parameters from Zhang 2020 (based on Mileusnic 2006 bag1)
    double M = 0.0002;
    double beta = 0.0605;
    double C = 1.0; 
    double a = 0.3;
    double R = 0.46;
    double Lsr0 = 0.04;
    double Lpr0 = 0.76;
    double Kpr = 0.15;
    double Ksr = 10.464;
    double LsrN = 0.0423;
    double G = 4000.0; // 20% of 20000

    // Smooth absolute value and sign for velocity
    double eps = 1e-4;
    double V_reg = std::sqrt(V * V + eps);
    double V_sign = V / V_reg;

    // Tension Calculation
    double term2 = beta * C * (L - R - Lsr0) * V_sign * std::pow(V_reg, a);
    double term3 = Kpr * (L - Lpr0 - Lsr0);
    double T = M * A + term2 + term3;
    
    // Smooth Max for T < 0
    T = 0.5 * (T + std::sqrt(T * T + eps));

    // Ia Primary Afferent Firing
    double Ia = G * ( (T / Ksr) - (LsrN - Lsr0) );
    
    // Smooth Max for Ia < 0
    return 0.5 * (Ia + std::sqrt(Ia * Ia + eps));
}

double Millard12EqMuscleWithSimplifiedAfferent::getIbAfferent(const SimTK::State& s) const
{
    // GTO Model from Zhang 2020
    double F_norm = getActiveFiberForce(s) / getMaxIsometricForce();
    double p = 60.0; // max fusimotor frequency for human from Zhang 2020

    // Smooth step function around F_norm = 0.7
    // Using sigmoid to smoothly transition from 0 to p*F_norm
    double k = 1000.0; // Steepness of the transition
    double smooth_step = 0.5 * (1.0 + std::tanh(k * (F_norm - 0.7)));

    return p * F_norm * smooth_step;
}
