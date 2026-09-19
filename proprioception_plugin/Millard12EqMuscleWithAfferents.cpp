// INCLUDES
#include "Millard12EqMuscleWithAfferents.h"
#include <iostream>  // remove later

// STATICS
using namespace std;
using namespace OpenSim;

/* METHODS */
// Default constructor. Not used.
Millard12EqMuscleWithAfferents::Millard12EqMuscleWithAfferents()
{
	constructProperties();
	upd_spindle().setOwnerMuscleName("no_spindle_name");
	upd_GTO().setOwnerMuscleName("no_GTO_name");
}

// Constructor. Mainly used.
Millard12EqMuscleWithAfferents::Millard12EqMuscleWithAfferents(const std::string &name, double maxIsometricForce, double optimalFiberLength, double tendonSlackLength, double pennationAngle)
:Super(name, maxIsometricForce, optimalFiberLength, tendonSlackLength, pennationAngle)
{
	constructProperties();
	upd_spindle().setOwnerMuscleName(getName());
	upd_GTO().setOwnerMuscleName(getName());
	
	// Initialize the work variables to calculate the acceleration
	ts[0] = -0.02;
	ts[1] = -0.01;
	ts[2] = 0.0;
	vel = 0;
	C0 = 0.0; C1 = 0.0;
}

// GET & SET "state variables" and their "derivatives"
double Millard12EqMuscleWithAfferents::getLPFvelocity(const SimTK::State& s) const
{
	return getStateVariableValue(s, "LPF_velocity");
}	

void Millard12EqMuscleWithAfferents::setLPFvelocity(SimTK::State& s, double Velocity) const
{
	setStateVariableValue(s, "LPF_velocity", Velocity);
}

double Millard12EqMuscleWithAfferents::getLPFacceleration(const SimTK::State& s) const
{
	return getStateVariableValue(s, "LPF_acceleration");
}

void Millard12EqMuscleWithAfferents::setLPFacceleration(SimTK::State& s, double Acceleration) const
{
	setStateVariableValue(s, "LPF_acceleration", Acceleration);
}

// Get & Set the Properties
void Millard12EqMuscleWithAfferents::setLPFtau(double aLPFtau) {
	set_lpf_tau(aLPFtau);
}

// construct the new properties and set their default values without crowding the constructor
void Millard12EqMuscleWithAfferents::constructProperties()
{
	setAuthors("Utku Oguzman");
	constructProperty_lpf_tau(0.01); // LPF time constant
	constructProperty_spindle(Mileusnic06Spindle());
    constructProperty_GTO(Lin02GolgiTendonOrgan());
	//All properties are added to the property set. Once added, they can be read in and written to files.
}

/* "MODEL COMPONENT" INTERFACES */
// Define new ODE muscle states and their derivatives 
void Millard12EqMuscleWithAfferents::extendAddToSystem(SimTK::MultibodySystem& system) const
{
	// Allow Millard2012EquilibriumMuscle to add its states, cache, etc. to the MultiBody Solver
	Super::extendAddToSystem(system);
	
	// low-pass filtered state variables used to calculate derivatives 
	addStateVariable("LPF_velocity"); // fiber velocity
	addStateVariable("LPF_acceleration"); // fiber acceleration
	//addCacheVariable("LPF_velocity", 0.0, SimTK::Stage::Dynamics);
}

// the way to initialize muscle state variables by using properties.
void Millard12EqMuscleWithAfferents::extendInitStateFromProperties(SimTK::State& s) const
{
    Super::extendInitStateFromProperties(s);
	
	// here we init the state directly, but not from any properties
	setLPFvelocity(s, 0.0);
	setLPFacceleration(s, 0.0);


	// Initialize the work variables to calculate the acceleration
	vel = 0;
	ts[2] = 0.0; ts[1] = -0.01; ts[0] = -0.02; 
	C0 = 0.0; C1 = 0.0;
}

// use the current values of the muscle states to update the properties
void Millard12EqMuscleWithAfferents::extendSetPropertiesFromState(const SimTK::State& s)
{
    Super::extendSetPropertiesFromState(s);
}

// the way to declare the spindle as a subcomponent
void Millard12EqMuscleWithAfferents::extendConnectToModel(Model& aModel)
{
	// The afferents need the name of their owner muscle
	upd_spindle().setOwnerMuscleName(getName());
	upd_GTO().setOwnerMuscleName(getName());
	Super::extendConnectToModel(aModel);
}

/* COMPUTATIONS */
// This function finds the initial states for the starting tension & then it calls the same-named method of the parent class
void Millard12EqMuscleWithAfferents::computeInitialFiberEquilibrium(SimTK::State& s) const
{
	// First let the muscle find an equilibrium state
	Super::computeInitialFiberEquilibrium(s);
	
	setLPFvelocity(s, getFiberVelocity(s));
	// a simplifying assumption is a steady state
	setLPFacceleration(s, 0.0); 
	
	// the spindle's initial conditions depend on the muscle,
	// so this method should be called last.
	get_spindle().computeInitialSpindleEquilibrium(s);
	
	// get a reasonable initial value for the GTO nonlinearity
	get_GTO().initFromMuscle(s);
	
	// update the work vectors assuming no acceleration
	vel[0] = vel[1] = vel[2] = getFiberVelocity(s); //no acceleration
	ts[2] = s.getTime();
	ts[1] = ts[2] - 0.001; //go back a little in time domain
	ts[0] = ts[1] - 0.001;
}

// If added any states, their derivatives must be updated here
void Millard12EqMuscleWithAfferents::computeStateVariableDerivatives(const SimTK::State& s) const
{
	// This is a "carefree" version of that:
	Super::computeStateVariableDerivatives(s);
	
	// next state is the LPF velocity ("if the LPF velocity is equal to fiber velocity, don't change")
	setStateVariableDerivativeValue(s, "LPF_velocity",(getFiberVelocity(s) - getLPFvelocity(s))/getLPFtau());
	 
	// the LPF acceleration
	setStateVariableDerivativeValue(s, "LPF_acceleration",(approxFiberAcceleration(s) - getLPFacceleration(s)) / getLPFtau());
}

//--------------------------------------------------------------------------
// Approximate the muscle fiber acceleration
//--------------------------------------------------------------------------
double Millard12EqMuscleWithAfferents::approxFiberAcceleration(const SimTK::State& s) const
{
	double accel;   // muscle fiber acceleration 
	double curr_vel;	//  muscle fiber velocity		
	double curr_time = s.getTime();	// time in the simulation
	
	curr_vel = getLPFvelocity(s);
	
	if( curr_time > ts(2) )
	{	// vel and ts are not ahead of current time.
		// Using 4-point Fornberg's method.
		// The formula below assumes current time = 0
		ts(0) = ts(0) - curr_time;
		ts(1) = ts(1) - curr_time;
		ts(2) = ts(2) - curr_time;
		
		// calculate coefficients
		C0(0,1) = ts(1)/(ts(1)-ts(0));
		C0(1,1) = ts(0)/(ts(0)-ts(1));
		C0(0,2) = ts(2)*C0(0,1)/(ts(2)-ts(0));
		C0(1,2) = ts(2)*C0(1,1)/(ts(2)-ts(1));
		C0(2,2) = ts(1)*ts(0)/((ts(2)-ts(0))*(ts(2)-ts(1)));
		C1(0,1) = 1/(ts(0)-ts(1));
		C1(1,1) = -C1(0,1);
		C1(2,2) = ((ts(1)-ts(0))/( (ts(2)-ts(1))*(ts(2)-ts(0)) ))
		          *(C0(1,1) - ts(1)*C1(1,1));
		C1(0,3) = C0(0,2)/ts(0);
		C1(1,3) = C0(1,2)/ts(1);
		C1(2,3) = C0(2,2)/ts(2);
		C1(3,3) = ( (ts(1)-ts(2))*(ts(2)-ts(0))/(ts(0)*ts(1)*ts(2)) )*(C0(2,2) - ts(2)*C1(2,2));
				  
		// use the coefficients
		accel = C1(3,3)*curr_vel + C1(2,3)*vel(2) + C1(1,3)*vel(1) + C1(0,3)*vel(0);
		
		// shift velocities and times
		vel(0) = vel(1); vel(1) = vel(2); vel(2) = curr_vel;
		ts(0) = ts(1) + curr_time; // changing to absolute times
		ts(1) = ts(2) + curr_time;
		ts(2) = curr_time;
	} 
	else  // computeStateVariableDerivatives was called before for a more advanced time
	{
		if( curr_time > ts(1) )
		{  // vel(1) and vel(0) still useful.
			// Using a 3-point rule for differentiation
			accel = ( 3*curr_vel - 4*vel(1) + vel(0) )/(curr_time - ts(0));
			
			// shift velocities and times
			vel(2) = curr_vel; ts(2) = curr_time;
		}
		else if( s.getTime() > ts(0) )
		{ // We only have one value before current time.
			// Using a 2-point rule for differentiation
			accel = (curr_vel - vel(0))/(curr_time - ts(0));
			
			// shift velocities and times
			vel(2) = curr_vel; vel(1) = vel(0);
			ts(2) = curr_time; ts(1) = ts(0); ts(0) = ts(1) - 1.0e-5;
		}
		else // we have no data to do this calculation
		{
			accel = getLPFacceleration(s);
			vel(2) = curr_vel; ts(2) = curr_time;
			vel(1) = vel(2); ts(1) = ts(2) - 1.0e-5;
			vel(0) = vel(1); ts(0) = ts(1) - 1.0e-5;
			//std::cout << "computed acceleration with no data \n";
		}
	}
	return accel;
}
