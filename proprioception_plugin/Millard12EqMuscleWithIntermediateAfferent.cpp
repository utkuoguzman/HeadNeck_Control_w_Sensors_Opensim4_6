// INCLUDES
#include "Millard12EqMuscleWithIntermediateAfferent.h"
#include <iostream>  // remove later

// STATICS
using namespace std;
using namespace OpenSim;

/* METHODS */
// Default constructor. Not used.
Millard12EqMuscleWithIntermediateAfferent::Millard12EqMuscleWithIntermediateAfferent()
{
	constructProperties();
	upd_spindle().setOwnerMuscleName("no_spindle_name");
	upd_GTO().setOwnerMuscleName("no_GTO_name");
}

// Constructor. Mainly used.
Millard12EqMuscleWithIntermediateAfferent::Millard12EqMuscleWithIntermediateAfferent(const std::string &name, double maxIsometricForce, double optimalFiberLength, double tendonSlackLength, double pennationAngle)
:Super(name, maxIsometricForce, optimalFiberLength, tendonSlackLength, pennationAngle)
{
	constructProperties();
	upd_spindle().setOwnerMuscleName(getName());
	upd_GTO().setOwnerMuscleName(getName());
	
	// Initialize the work variables to calculate the acceleration
	// removed mutable init
}

// GET & SET "state variables" and their "derivatives"
double Millard12EqMuscleWithIntermediateAfferent::getLPFvelocity(const SimTK::State& s) const
{
	return getStateVariableValue(s, "LPF_velocity");
}	

void Millard12EqMuscleWithIntermediateAfferent::setLPFvelocity(SimTK::State& s, double Velocity) const
{
	setStateVariableValue(s, "LPF_velocity", Velocity);
}

double Millard12EqMuscleWithIntermediateAfferent::getLPFacceleration(const SimTK::State& s) const
{
	return getStateVariableValue(s, "LPF_acceleration");
}

void Millard12EqMuscleWithIntermediateAfferent::setLPFacceleration(SimTK::State& s, double Acceleration) const
{
	setStateVariableValue(s, "LPF_acceleration", Acceleration);
}

// Get & Set the Properties
void Millard12EqMuscleWithIntermediateAfferent::setLPFtau(double aLPFtau) {
	set_lpf_tau(aLPFtau);
}

// construct the new properties and set their default values without crowding the constructor
void Millard12EqMuscleWithIntermediateAfferent::constructProperties()
{
	setAuthors("Utku Oguzman");
	constructProperty_lpf_tau(0.01); // LPF time constant
	constructProperty_spindle(Poppele70Spindle());
    constructProperty_GTO(Lin02GolgiTendonOrgan());
	//All properties are added to the property set. Once added, they can be read in and written to files.
}

/* "MODEL COMPONENT" INTERFACES */
// Define new ODE muscle states and their derivatives 
void Millard12EqMuscleWithIntermediateAfferent::extendAddToSystem(SimTK::MultibodySystem& system) const
{
	// Allow Millard2012EquilibriumMuscle to add its states, cache, etc. to the MultiBody Solver
	Super::extendAddToSystem(system);
	
	// low-pass filtered state variables used to calculate derivatives 
	addStateVariable("LPF_velocity"); // fiber velocity
	addStateVariable("LPF_acceleration"); // fiber acceleration
	//addCacheVariable("LPF_velocity", 0.0, SimTK::Stage::Dynamics);
}

// the way to initialize muscle state variables by using properties.
void Millard12EqMuscleWithIntermediateAfferent::extendInitStateFromProperties(SimTK::State& s) const
{
    Super::extendInitStateFromProperties(s);
	
	// here we init the state directly, but not from any properties
	setLPFvelocity(s, 0.0);
	setLPFacceleration(s, 0.0);

}

// use the current values of the muscle states to update the properties
void Millard12EqMuscleWithIntermediateAfferent::extendSetPropertiesFromState(const SimTK::State& s)
{
    Super::extendSetPropertiesFromState(s);
}

// the way to declare the spindle as a subcomponent
void Millard12EqMuscleWithIntermediateAfferent::extendConnectToModel(Model& aModel)
{
	// The afferents need the name of their owner muscle
	upd_spindle().setOwnerMuscleName(getName());
	upd_GTO().setOwnerMuscleName(getName());
	Super::extendConnectToModel(aModel);
}

/* COMPUTATIONS */
// This function finds the initial states for the starting tension & then it calls the same-named method of the parent class
void Millard12EqMuscleWithIntermediateAfferent::computeInitialFiberEquilibrium(SimTK::State& s) const
{
	// First let the muscle find an equilibrium state
	Super::computeInitialFiberEquilibrium(s);
	
	setLPFvelocity(s, getFiberVelocity(s));
	// a simplifying assumption is a steady state
	setLPFacceleration(s, 0.0); 
	
	// get a reasonable initial value for the GTO nonlinearity
	get_GTO().initFromMuscle(s);
}

// If added any states, their derivatives must be updated here
void Millard12EqMuscleWithIntermediateAfferent::computeStateVariableDerivatives(const SimTK::State& s) const
{
	Super::computeStateVariableDerivatives(s);
	
	// Cascaded filter to get smooth velocity and acceleration
	double curr_accel = (getFiberVelocity(s) - getLPFvelocity(s)) / getLPFtau();
	setStateVariableDerivativeValue(s, "LPF_velocity", curr_accel);
	setStateVariableDerivativeValue(s, "LPF_acceleration", (curr_accel - getLPFacceleration(s)) / getLPFtau());
}

//--------------------------------------------------------------------------
// Approximate the muscle fiber acceleration
//--------------------------------------------------------------------------
double Millard12EqMuscleWithIntermediateAfferent::approxFiberAcceleration(const SimTK::State& s) const
{
	return (getFiberVelocity(s) - getLPFvelocity(s)) / getLPFtau();
}
