/*
 * TugOfWarController.cpp
 *
 * This file is a modified version of the file ControllerExample.cpp, which
 * is part of an OpenSim tutorial:
 * http://simtk-confluence.stanford.edu:8080/display/OpenSim/Creating+a+Controller
 *
 */
 
// Parameters for the prescribed functions
#define AMP 0.0014
#define MAX_Z 0.0142
#define MIN_Z  -0.002
#define RISE_T 0.12   // 1.2, 0.2, 0.12
 
 // Include OpenSim and functions
#include <OpenSim/OpenSim.h>
#include "TugOfWarController.h"

// This allows us to use OpenSim functions, classes, etc., without having to
// prefix the names of those things with "OpenSim::".
using namespace OpenSim;

// This allows us to use SimTK functions, classes, etc., without having to
// prefix the names of those things with "SimTK::".
using namespace SimTK;


//______________________________________________________________________________
/**
 * The controller will try to make the model follow this position
 * in the z direction.
 */
double desiredModelZPosition( double t ) {
/*    // TEST FUNCTION
	// z(t) = AMP * sin( pi * t )
	//return AMP * sin( Pi * t );
*/
/*
	// Sinusoidal stretches (Fig. 6 Mileusnic06a)
	return AMP * ( cos( 2*Pi*t ) - 0.8);
*/

	// Ramp
	if( t > 1.0 )
	{
		if(  t < 1.0 + RISE_T )
		{
			double slope = 1.05*(MAX_Z - MIN_Z)/(RISE_T);
			return slope*(t - 1.0);
		}
		else
			return MAX_Z + 0.005;
	}
	else
		return 0.0;

}

double desiredModelZVelocity( double t ) {
/*  //TEST FUNCTION	
	// z'(t) = (AMP*pi) cos( pi * t )
	return AMP * Pi * cos( Pi * t );
*/
/*
	// Sinusoidal stretches (Fig. 6 Mileusnic06a)
	return -AMP * 2 * Pi * sin( 2*Pi*t );
*/
	// Ramp
	if( (t > 1.0) && (t < 1.0 + RISE_T) )
	{
		return (MAX_Z - MIN_Z)/(RISE_T) + 0.0095;
	}
	else 
		return 0.0;

}
//______________________________________________________________________________
/**
 * The controller will try to make the model follow this acceleration
 * in the z direction.
 */
double desiredModelZAcceleration( double t ) {
/*  //TEST FUNCTION	
	// z''(t) = -(AMP*pi^2) sin( pi * t )
	return -AMP * Pi * Pi * sin( Pi * t );
*/
/*
	// Sinusoidal stretches (Fig. 6 Mileusnic06a)
	return -AMP * 4 * Pi * Pi * cos( 2*Pi*t );
*/

	// Ramp. 
	return 0.0;

}

	/**
	 * Constructor
	 *
	 * @param aModel Model to be controlled
	 * @param aKp Position gain by which the position error will be multiplied
	 */
	TugOfWarController::TugOfWarController(double aKp, double aKv) : Controller(), kp( aKp ), kv( aKv ) 
	{
	}

	/**
	 * This function is called at every time step for every actuator.
	 *
	 * @param s Current state of the system
	 * @param controls Controls being calculated
	 */
void TugOfWarController::computeControls(const SimTK::State& s, SimTK::Vector &controls) const
{
	// Get the current time in the simulation.
	double t = s.getTime();
		// Read the mass of the block.
	double blockMass = getModel().getBodySet().get( "block" ).getMass();
		// Get pointers to each of the muscles in the model.
	const Muscle* leftMuscle = dynamic_cast<const Muscle*>( &getConnectee<Actuator>("actuators", 0) );
	const Muscle* rightMuscle = dynamic_cast<const Muscle*>( &getConnectee<Actuator>("actuators", 1) );	
	// Compute the desired position of the block in the tug-of-war
	// model.
	double zdes  = desiredModelZPosition(t);
	
    // Compute desired velocity
	double zdesv = desiredModelZVelocity(t);
	
	// Compute the desired acceleration of the block in the tug-
	// of-war model.	
	double zdesa = desiredModelZAcceleration(t);
	
	// Get the ztranslation coordinate in the model.
	// If 5 is not the index of the coordinate, then all it's lost
	//const Coordinate zCoord = _model->getCoordinateSet().get(5);
	//const Coordinate& zCoord = _model->getCoordinateSet().get(5);
	const Coordinate& zCoord = _model->getCoordinateSet().get(5);
		
	// Get the crrent position of the block in the tug-of-war
	// model.	
	double z  = zCoord.getValue(s);
	
	// Get the current velocity
	double zv  = zCoord.getSpeedValue(s);
	
	// Compute the correction to the desired acceleration arising
	// from the deviation of the block's current position from its
	// desired position (this deviation is the "position error").
	double pErrTerm = kp * ( zdes  - z  );
	
	double vErrTerm = kv * ( zdesv - zv );
	
	// Compute the total desired acceleration based on the initial
	// desired acceleration plus the position and velocity error 
	// terms we computed above.
	double desAcc = zdesa + pErrTerm + vErrTerm;
	
	// Compute the desired force on the block as the mass of the
	// block times the total desired acceleration of the block.
	double desFrc = desAcc * blockMass;
	
	// Get the maximum isometric force for the left muscle.
	double FoptL = leftMuscle->getMaxIsometricForce();
	
	// Get the maximum isometric force for the right muscle.
	double FoptR = rightMuscle->getMaxIsometricForce();
	
	// If desired force is in direction of one muscle's pull
	// direction, then set that muscle's control based on desired
	// force.  Otherwise, set the muscle's control to zero.
	double leftControl = 0.0, rightControl = 0.0;
	if( desFrc < 0 ) {
		leftControl = abs( desFrc ) / FoptL;
		rightControl = 0.0;
	}
	else if( desFrc > 0 ) {
		leftControl = 0.0;
		rightControl = abs( desFrc ) / FoptR;
	}
	// Don't allow any control value to be greater than one.
	if( leftControl > 1.0 ) leftControl = 1.0;
	if( rightControl > 1.0 ) rightControl = 1.0;
	
	// Instead of all that I can just use a sloppy ramp
	/*
	 * leftControl = 0.02*t;
	 * rightControl = 0.004*t;
	 */
	
	//leftControl = 0.0; rightControl = 0.0;
	// Millard12EqMuscleWithAfferents has 3 controls
	Vector muscleControl(3, 0.0);
	muscleControl[0] = leftControl;
	muscleControl[1] = 0.01;  // dynamic gamma   inputs
	muscleControl[2] = 0.01;   // static gamma motoneuron inputs
	// Add in the controls computed for this muscle to the set of all model controls
	leftMuscle->addInControls(muscleControl, controls);
	// Specify control for other actuator (muscle) controlled by this controller
	muscleControl[0] = rightControl;
	rightMuscle->addInControls(muscleControl, controls);
}
