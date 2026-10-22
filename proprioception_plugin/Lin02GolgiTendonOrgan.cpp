/* 
* The Lin02GolgiTendonOrgan class implements a model for the aggregate
* population response of the Golgi tendon organs in a muscle.
* The model is taken from: Lin & Crago, "Neural and Mechanical Contributions
* to the Stretch Reflex: A Model Synthesis" Ann Biomed Eng 30:54-67.
*
* Objects of this class are meant to be contained in particular muscle
* objects, such as those of the Millard12EqMuscleWithAfferents class.
*
* @author Sergio Verduzco Flores
*/

#include <iostream>  // remove later
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include "Millard12EqMuscleWithAfferents.h"
#include <cmath>  // for the log() function

using namespace OpenSim;

Lin02GolgiTendonOrgan::Lin02GolgiTendonOrgan()
{
	constructProperties();
	
	// Set parameter values
	Gg = 60;  	// pulses/s
	Gf = 4;		// Newtons
	thr = 0;	// pulses/s
	// Initialize work variables
	// removed historical variables
}

//=============================================================================
// get & set properties
//=============================================================================
void Lin02GolgiTendonOrgan::setLPFtau(double aLPFtau) {
	set_lpf_tau(aLPFtau);
}

//=============================================================================
// get & set state variables
//=============================================================================
void Lin02GolgiTendonOrgan::ensureStatePointers() const {
	if (!sv_nonlinear) {
		sv_nonlinear = traverseToStateVariable("nonlinear");
		sv_nonlinear_deriv = traverseToStateVariable("nonlinear_deriv");
		sv_filter_out = traverseToStateVariable("filter_out");
		sv_filter_out_deriv = traverseToStateVariable("filter_out_deriv");
	}
}

// LPF output of the log nonlinearity
double Lin02GolgiTendonOrgan::getX(const SimTK::State& s) const {
	if (!get_enable_GTO()) return 0.0;
	ensureStatePointers();
	return sv_nonlinear ? sv_nonlinear->getValue(s) : getStateVariableValue(s, "nonlinear");
}
void Lin02GolgiTendonOrgan::setX(SimTK::State& s, double X) const {
	if (!get_enable_GTO()) return;
	ensureStatePointers();
	if (sv_nonlinear) sv_nonlinear->setValue(s, X);
	else setStateVariableValue(s, "nonlinear", X);
}
// derivative of the LPF output of the log nonlinearity
double Lin02GolgiTendonOrgan::getXp(const SimTK::State& s) const {
	if (!get_enable_GTO()) return 0.0;
	ensureStatePointers();
	return sv_nonlinear_deriv ? sv_nonlinear_deriv->getValue(s) : getStateVariableValue(s, "nonlinear_deriv");
}
void Lin02GolgiTendonOrgan::setXp(SimTK::State& s, double Xp) const {
	if (!get_enable_GTO()) return;
	ensureStatePointers();
	if (sv_nonlinear_deriv) sv_nonlinear_deriv->setValue(s, Xp);
	else setStateVariableValue(s, "nonlinear_deriv", Xp);
}
// Intermediate variable for the transfer function filter
double Lin02GolgiTendonOrgan::getY(const SimTK::State& s) const {
	if (!get_enable_GTO()) return 0.0;
	ensureStatePointers();
	return sv_filter_out ? sv_filter_out->getValue(s) : getStateVariableValue(s, "filter_out");
}
void Lin02GolgiTendonOrgan::setY(SimTK::State& s, double Y) const {
	if (!get_enable_GTO()) return;
	ensureStatePointers();
	if (sv_filter_out) sv_filter_out->setValue(s, Y);
	else setStateVariableValue(s, "filter_out", Y);
}
// output variable of the transfer function
double Lin02GolgiTendonOrgan::getZ(const SimTK::State& s) const {
	if (!get_enable_GTO()) return 0.0;
	ensureStatePointers();
	return sv_filter_out_deriv ? sv_filter_out_deriv->getValue(s) : getStateVariableValue(s, "filter_out_deriv");
}
void Lin02GolgiTendonOrgan::setZ(SimTK::State& s, double Z) const {
	if (!get_enable_GTO()) return;
	ensureStatePointers();
	if (sv_filter_out_deriv) sv_filter_out_deriv->setValue(s, Z);
	else setStateVariableValue(s, "filter_out_deriv", Z);
}
// output of the Golgi tendon organ
double Lin02GolgiTendonOrgan::getGTOout(const SimTK::State& s) const {
	if (!get_enable_GTO()) return 0.0;
	return cached_gto_out; 
}
void Lin02GolgiTendonOrgan::setGTOout(const SimTK::State& s, 
									  double output) const {
	cached_gto_out = output;
}

//=============================================================================
// MODEL COMPONENT INTERFACE
//=============================================================================

void Lin02GolgiTendonOrgan::extendAddToSystem(SimTK::MultibodySystem& system) const
{
	Super::extendAddToSystem(system);
	if (!get_enable_GTO()) return;
	
	// adding state variables
	addStateVariable("nonlinear", SimTK::Stage::Velocity); // LPF output of Eq. 1
	addStateVariable("nonlinear_deriv", SimTK::Stage::Velocity);
	addStateVariable("filter_out", SimTK::Stage::Velocity);
	addStateVariable("filter_out_deriv", SimTK::Stage::Velocity); // output of Eq. 2
	
	// a cache variable for the aggregate output of the GTOs
	addCacheVariable("gto_out", 0.0, SimTK::Stage::Dynamics);
	
	// ensuring that the owner muscle is in the system	
	const ForceSet& fSet = getModel().getForceSet();
	try {
		fSet.get(ownerMuscleName);
	}
	catch (OpenSim::Exception e) {
		std::cout << "WARNING - Lin02GolgiTendonOrgan::addToSystem() could not find ";
		std::cout << "the muscle with name" << ownerMuscleName << '\n';
		std::cout << "Exception: " << e.getMessage() << '\n';
		return;
	}
	
	// ensuring that the owner muscle is of the right type
	std::string forceClassName = fSet.get(ownerMuscleName).getConcreteClassName();
	if( forceClassName != "Millard12EqMuscleWithAfferents" )
	{
		std::cout << "WARNING - In Lin02GolgiTendonOrgan::addToSystem() \n";
		std::cout << "Lin02GolgiTendonOrgan is owned by a force that is not " ;
		std::cout << "of the Millard12EqMuscleWithAfferents class \n" ;
	}		
}
	
void Lin02GolgiTendonOrgan::extendInitStateFromProperties(SimTK::State& s) const
{
    Super::extendInitStateFromProperties(s);
	sv_nonlinear = nullptr;
	sv_nonlinear_deriv = nullptr;
	sv_filter_out = nullptr;
	sv_filter_out_deriv = nullptr;
	if (!get_enable_GTO()) return;
	setX(s,0.0);
	setXp(s,0.0);
	setY(s, 0.0);
	setZ(s, 0.0);
	// Initialize the work variables 
	// removed historical variables
}

void Lin02GolgiTendonOrgan::extendSetPropertiesFromState(const SimTK::State& s) 
{
    Super::extendSetPropertiesFromState(s);
}

void Lin02GolgiTendonOrgan::extendConnectToModel(Model& aModel) 
{
	Super::extendConnectToModel(aModel);
	
	// connectToModel is called several times, some of them before
	// the owner muscle is in the model. We can only initialize
	// musclePtr once the owner muscle is in the model.
	if( (aModel.getMuscles()).contains(ownerMuscleName) )
		musclePtr = &((aModel.getMuscles()).get(ownerMuscleName));
}

void Lin02GolgiTendonOrgan::constructProperties()
{
	setAuthors("Sergio Verduzco");
	constructProperty_lpf_tau(0.01); // LPF time constant
	constructProperty_enable_GTO(false); // Disabled by default during neck study; set true when GTO is needed
}

void Lin02GolgiTendonOrgan::setOwnerMuscleName(std::string OwnerMuscleName)
{
	ownerMuscleName = OwnerMuscleName;
}

//=============================================================================
// Derivative computation
//=============================================================================
void Lin02GolgiTendonOrgan::computeStateVariableDerivatives(const SimTK::State& s) const
{
	if (!get_enable_GTO()) return;
	ensureStatePointers();
	double non_lin = Gg*std::log((musclePtr->getFiberForce(s) / Gf) + 1);
	double x_val = getX(s);
	double xp_val = getXp(s);
	double y_val = getY(s);
	double z_val = getZ(s);
	
	// Cascaded filter for smooth derivatives without finite-differencing
	double curr_diff0 = (non_lin - x_val) / getLPFtau();
	double curr_diff1 = (curr_diff0 - xp_val) / getLPFtau();
	
	if (sv_nonlinear) sv_nonlinear->setDerivative(s, curr_diff0);
	else setStateVariableDerivativeValue(s, "nonlinear", curr_diff0);
	if (sv_nonlinear_deriv) sv_nonlinear_deriv->setDerivative(s, curr_diff1);
	else setStateVariableDerivativeValue(s, "nonlinear_deriv", curr_diff1);
	
	double Xpp = curr_diff1;
	
	if (sv_filter_out) sv_filter_out->setDerivative(s, z_val);
	else setStateVariableDerivativeValue(s, "filter_out", z_val);
	if (sv_filter_out_deriv) sv_filter_out_deriv->setDerivative(s, -2.2*z_val - 0.4*y_val + 68.0*Xpp + 103.2*xp_val + 16.0*x_val);
	else setStateVariableDerivativeValue(s, "filter_out_deriv", -2.2*z_val - 0.4*y_val + 68.0*Xpp + 103.2*xp_val + 16.0*x_val);
			  
	// Smooth Max for GTO output
	double y_diff = y_val - thr;
	double smooth_out = 0.5 * (y_val + thr + std::sqrt(y_diff*y_diff + 1e-4));
	setGTOout(s, smooth_out);
}

//=============================================================================
// Initializing values
//=============================================================================
void Lin02GolgiTendonOrgan::initFromMuscle(SimTK::State& s) const
{
	if (!get_enable_GTO()) return;
	setXp(s, 0.0);
	double nonLin = Gg*std::log( (musclePtr->getFiberForce(s) / Gf) + 1 );
	setX(s, nonLin);
	setY(s, 40.0*nonLin);  // this is the fixed point
	setZ(s, 0.0);
	// removed historical variables
}


//--------------------------------------------------------------------------
// Approximate the derivatives required by Eq. 2
//--------------------------------------------------------------------------
SimTK::Vec<2> Lin02GolgiTendonOrgan::
calculateDerivatives(const SimTK::State& s) const
{
	SimTK::Vec<2> diff;
	// diff(0) = LPF derivative of nonlinear output
	// diff(1) = derivative of diff(0)'s variable 
	
	double curr_nl = getX(s);
	double curr_Dnl = getXp(s);
	double curr_time = s.getTime();	// time in the simulation

	if( curr_time > ts(2) )
	{	// data vectors are not ahead of current time.
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
		C1(3,3) = ( (ts(1)-ts(2))*(ts(2)-ts(0))/(ts(0)*ts(1)*ts(2)) )
		          *(C0(2,2) - ts(2)*C1(2,2));
				  
		// use the coefficients
		double nolo = 0.925*curr_nl + 0.075*nl(2);
		diff(0) = C1(3,3)*nolo + C1(2,3)*nl(2) + C1(1,3)*nl(1) + C1(0,3)*nl(0);
		// using a simpler rule for the 2nd derivative
		double dino = ( Dnl(1) + Dnl(2) )/2;
		diff(1) = 2*(curr_Dnl - dino) / ( -(ts(1)+ts(2)) );
	
		// shift work vectors and times
		nl(0) = nl(1); nl(1) = nl(2); nl(2) = curr_nl;
		Dnl(0) = Dnl(1); Dnl(1) = Dnl(2); Dnl(2) = curr_Dnl;
		ts(0) = ts(1) + curr_time; // changing to absolute times
		ts(1) = ts(2) + curr_time;
		ts(2) = curr_time;
	} 
	else  // computeStateVariableDerivatives was called before for a more advanced time
	{
		if( curr_time > ts(1) )
		{  // nl(1) and nl(0) still useful.
			// Using a 3-point rule for differentiation
			diff(0) = ( 3.0*curr_nl - 4.0*nl(1) + nl(0) )/(curr_time - ts(0));
			double dino = ( Dnl(0) + Dnl(1) )/2;
			diff(1) = 2*(curr_Dnl - dino) / ( -(ts(0)+ts(1)) );
			
			// shift velocities and times
			nl(2) = curr_nl; Dnl(2) = curr_Dnl;
			ts(2) = curr_time;	 
		}
		else if( curr_time > ts(0) )
		{ // We only have one value before current time.
			// Using a 2-point rule for differentiation
			diff(0) = (curr_nl - nl(0))/(curr_time - ts(0));
			diff(1) = (curr_Dnl - Dnl(0))/(curr_time - ts(0));
			
			// shift velocities and times
			nl(2) = curr_nl; nl(1) = nl(0);
			Dnl(2) = curr_Dnl; Dnl(1) = Dnl(0);
			ts(2) = curr_time; ts(1) = ts(0); ts(0) = ts(1) - 1.0e-6;
		}
		else // we interpolate backwards
		{
			diff(0) = (nl(0) - curr_nl) / (ts(0) - curr_time);
			diff(1) = (Dnl(0) - curr_Dnl) / (ts(0) - curr_time); 
			
			nl(2) = curr_nl; nl(1) = nl(2); nl(0) = nl(1);
			Dnl(2) = curr_Dnl; Dnl(1) = Dnl(2); Dnl(0) = Dnl(1);
			ts(2) = curr_time; ts(1) = ts(2) - 1.0e-6; ts(0) = ts(1) - 1.0e-6;
		}
	}
	return diff;
}