
#pragma once
#include "OpenSim/Simulation/Model/ModelComponent.h"
#include "OpenSim/Simulation/Model/Model.h"
#include <OpenSim/Simulation/Model/Muscle.h>

namespace OpenSim {
	
// FUNCTIONALITY
// The Lin02GolgiTendonOrgan class implements a model for the aggregate
// population response of the Golgi tendon organs in a muscle.
// 
// MODEL SOURCE
// The model is taken from: Lin & Crago, "Neural and Mechanical Contributions
// to the Stretch Reflex: A Model Synthesis" Ann Biomed Eng 30:54-67.
// 
// PROPERTYNESS
// Objects of this class are meant to be contained in particular muscle
// objects, such as those of the Millard12EqMuscleWithAfferents class.

// Implementing this model requires 4 state variables and 1 cache variable is
// used to provide the output, corresponding to the aggregate firing rate of all 
// Golgi tendon organs connected to the muscle.
class Lin02GolgiTendonOrgan : public ModelComponent {
OpenSim_DECLARE_CONCRETE_OBJECT(Lin02GolgiTendonOrgan, ModelComponent);
friend class Millard12EqMuscleWithAfferents;
	friend class Millard12EqMuscleWithIntermediateAfferent;
friend class AfferentAnalysis;

public:
	OpenSim_DECLARE_PROPERTY(lpf_tau, double, "time constant for the low-pass filters");

	Lin02GolgiTendonOrgan();  // the constructor
	
	/* Specify the name of the muscle that contains this object */
	void setOwnerMuscleName(std::string OwnerMuscleName);
	
	//-------------------------------------------------------------------------
	// GET & SET state variables and cache variables
	//-------------------------------------------------------------------------
	// low-pass filtered output of the nonlinearity
	double getX(const SimTK::State& s) const;
	void setX(SimTK::State& s, double X) const;
	// derivative of the low-pass filtered output of the nonlinearity
	double getXp(const SimTK::State& s) const;
	void setXp(SimTK::State& s, double Xp) const;
	// Intermediate variable for the transfer function filter
	double getY(const SimTK::State& s) const;
	void setY(SimTK::State& s, double Y) const;
	// output variable of the transfer function
	double getZ(const SimTK::State& s) const;
	void setZ(SimTK::State& s, double Z) const;
	// output of the Golgi tendon organ
	OpenSim_DECLARE_OUTPUT(aggregate_firing_rate, double, getGTOout, SimTK::Stage::Dynamics);
	double getGTOout(const SimTK::State& s) const;
	void setGTOout(const SimTK::State& s, double output) const;
	
	//-------------------------------------------------------------------------
	// GET & SET Properties
	//-------------------------------------------------------------------------
	double getLPFtau() const { return get_lpf_tau(); }
	void setLPFtau(double aLPFtau);
	
protected:
	//--------------------------------------------------------------------------
	// MODEL COMPONENT INTERFACE
	//--------------------------------------------------------------------------
    /// Currently you need to call all these from the owner muscle.
	
	/** Add any Symbody elements(state variables, cache variables, etc.) 
	 * to the System. To be called after connectToModel calls. 
	 * Due to reasons not entirely clear, this method has to be invoked
	 * from the addToSystem method of the owner muscle. At the end. */
	void extendAddToSystem(SimTK::MultibodySystem& system) const override;
	
	/** initialize state variables from properties */
	void extendInitStateFromProperties(SimTK::State& s) const override;
	
	/** set current values of state into the properties */
	void extendSetPropertiesFromState(const SimTK::State& s) override;
	
	/** Extension of the parent class method. 
	 * This method must be called from the connectToModel method of
	 * the owner muscle, before the Super::connectToModel statement. */
	void extendConnectToModel(Model& aModel) override;
	
	//-------------------------------------------------------------------------
	// PARAMETERS
	//-------------------------------------------------------------------------	
	// Values used in Eq. 1 (initialized in the constuctor) 
	double Gg, Gf; // pulses/s and Newtons, respectively
	// Threshold for Eq. 3
	double thr;  // pulses / s
	
	/** Set an inital value for the output of the nonlinearity,
	  * and set the other variables to zero */
	void initFromMuscle(SimTK::State& s) const;
	
	//-------------------------------------------------------------------------
	// COMPUTATIONS
	//-------------------------------------------------------------------------
	void computeStateVariableDerivatives(const SimTK::State& s) const override;
	
	/** This method calculates the first and second derivatives of 
	 *  the log nonlinearity. 
	 *  It uses a version of the method in: Fornberg 1998 "Calculation of 
	 *  Weights in Finite Difference Formulas" SIAM Rev.40(3):685-691,
	 *  as well as simpler rules.
	 *  The stored data points to apply the method are in the vectors
	 *  ts, vel, F, and dF .  */
	SimTK::Vec<2> calculateDerivatives(const SimTK::State& s) const;
	
	
	
private:
	void constructProperties();
	
	/** The name of the muscle that owns the instance of this object.
	 *  Initialized in the muscle's connectToModel method, or in its constructor.
	 *  Checked in addToSystem. */
	std::string ownerMuscleName;
	
	/** This pointer leads to the the owner muscle. 
	 *  Initialized in Lin02GolgiTendonOrgan::connectToModel */
	const Muscle *musclePtr;
	
	/** These auxiliary vectors are used to approximate the 
	 *  derivatives of the log nonlinearity. 
	 *  'nl' and 'Dnl' are the stored values of the output of the
	 *  nonlinearity (Eq. 1) and its derivative, respectively. 
	 *  'ts' contains the time when they were stored.
	 *  Since computeStateVariableDerivatives is const, mutable is needed. */
	mutable SimTK::Vec<3> ts, nl, Dnl; 
	mutable SimTK::Mat33 C0;
	mutable SimTK::Mat44 C1;

}; // end of class Lin02GolgiTendonOrgan
	
} // end of namespace OpenSim
