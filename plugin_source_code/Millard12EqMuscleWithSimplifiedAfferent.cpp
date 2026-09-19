#include "Millard12EqMuscleWithSimplifiedAfferent.h"
#include <iostream>

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
    ts[0] = -0.02;
    ts[1] = -0.01;
    ts[2] = 0.0;
    vel = 0;
    C0 = 0.0; C1 = 0.0;
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
    vel = 0;
    ts[2] = 0.0; ts[1] = -0.01; ts[0] = -0.02; 
    C0 = 0.0; C1 = 0.0;
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
    
    vel[0] = vel[1] = vel[2] = getFiberVelocity(s);
    ts[2] = s.getTime();
    ts[1] = ts[2] - 0.001;
    ts[0] = ts[1] - 0.001;
}

void Millard12EqMuscleWithSimplifiedAfferent::computeStateVariableDerivatives(const SimTK::State& s) const
{
    Super::computeStateVariableDerivatives(s);
    setStateVariableDerivativeValue(s, "LPF_velocity", (getFiberVelocity(s) - getLPFvelocity(s)) / get_lpf_tau());
    setStateVariableDerivativeValue(s, "LPF_acceleration", (approxFiberAcceleration(s) - getLPFacceleration(s)) / get_lpf_tau());
}

double Millard12EqMuscleWithSimplifiedAfferent::approxFiberAcceleration(const SimTK::State& s) const
{
    double accel;
    double curr_vel = getLPFvelocity(s);
    double curr_time = s.getTime();
    
    if( curr_time > ts(2) )
    {
        ts(0) = ts(0) - curr_time;
        ts(1) = ts(1) - curr_time;
        ts(2) = ts(2) - curr_time;
        
        C0(0,1) = ts(1)/(ts(1)-ts(0));
        C0(1,1) = ts(0)/(ts(0)-ts(1));
        C0(0,2) = ts(2)*C0(0,1)/(ts(2)-ts(0));
        C0(1,2) = ts(2)*C0(1,1)/(ts(2)-ts(1));
        C0(2,2) = ts(1)*ts(0)/((ts(2)-ts(0))*(ts(2)-ts(1)));
        C1(0,1) = 1/(ts(0)-ts(1));
        C1(1,1) = -C1(0,1);
        C1(2,2) = ((ts(1)-ts(0))/( (ts(2)-ts(1))*(ts(2)-ts(0)) ))*(C0(1,1) - ts(1)*C1(1,1));
        C1(0,3) = C0(0,2)/ts(0);
        C1(1,3) = C0(1,2)/ts(1);
        C1(2,3) = C0(2,2)/ts(2);
        C1(3,3) = ( (ts(1)-ts(2))*(ts(2)-ts(0))/(ts(0)*ts(1)*ts(2)) )*(C0(2,2) - ts(2)*C1(2,2));
                  
        accel = C1(3,3)*curr_vel + C1(2,3)*vel(2) + C1(1,3)*vel(1) + C1(0,3)*vel(0);
        
        vel(0) = vel(1); vel(1) = vel(2); vel(2) = curr_vel;
        ts(0) = ts(1) + curr_time;
        ts(1) = ts(2) + curr_time;
        ts(2) = curr_time;
    } 
    else
    {
        if( curr_time > ts(1) ) {
            accel = ( 3*curr_vel - 4*vel(1) + vel(0) )/(curr_time - ts(0));
            vel(2) = curr_vel; ts(2) = curr_time;
        } else if( s.getTime() > ts(0) ) {
            accel = (curr_vel - vel(0))/(curr_time - ts(0));
            vel(2) = curr_vel; vel(1) = vel(0);
            ts(2) = curr_time; ts(1) = ts(0); ts(0) = ts(1) - 1.0e-5;
        } else {
            accel = getLPFacceleration(s);
            vel(2) = curr_vel; ts(2) = curr_time;
            vel(1) = vel(2); ts(1) = ts(2) - 1.0e-5;
            vel(0) = vel(1); ts(0) = ts(1) - 1.0e-5;
        }
    }
    return accel;
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

    // Tension Calculation
    double term2 = beta * C * (L - R - Lsr0) * ((V > 0) ? 1.0 : -1.0) * std::pow(std::abs(V), a);
    double term3 = Kpr * (L - Lpr0 - Lsr0);
    double T = M * A + term2 + term3;
    
    if (T < 0) T = 0; // Muscle fiber can't push

    // Ia Primary Afferent Firing
    double Ia = G * ( (T / Ksr) - (LsrN - Lsr0) );
    return std::max(0.0, Ia);
}

double Millard12EqMuscleWithSimplifiedAfferent::getIbAfferent(const SimTK::State& s) const
{
    // GTO Model from Zhang 2020
    double F_norm = getActiveFiberForce(s) / getMaxIsometricForce();
    double p = 60.0; // max fusimotor frequency for human from Zhang 2020

    if (F_norm < 0.7) {
        return 0.0;
    } else {
        return p * F_norm;
    }
}

