
// INCLUDE
#include <OpenSim/OpenSim.h>
#include "Poppele70Spindle.h"
#include "Lin02GolgiTendonOrgan.h"

namespace OpenSim {
/**
 * FUNCTIONALITY
 * This class extends a Millard2012EquilibriumMuscle by including including
 * inputs and outputs to represent spindle afferents and Golgi tendon organs. 
 * 
 * MODEL SOURCE
 * The spindle afferent model comes from: Mileusnic et al 2006 "Mathematical 
 * models of proprioceptors. I. Control and transduction in the muscle spindle"
 * J Neurophysiol 96:1772-1788.
 * 
 * The GTO model is the one used in: Lin, Crago 2002 "Neural and mechanical 
 * contributions to the stretch reflex: A model synthesis" 
 * Ann Biomed Eng 30:54-67.
 *
 * HOW THE CODE ACHIEVED
 * The implementation of this class was achieved by modifying the file
 * FatigableMuscle.h, which is part of the OpenSim API examples.
 * The Muscle.h base class specifies the interface that must be implemented 
 * by the derived muscle classes.
 * 
 * CLASS EXTENSION
 * The Millard12EqMuscleWithIntermediateAfferent derives from Millard2012EquilibriumMuscle,
 * which is a concrete implementation of the  Muscle interface (interfaces are done).
 * 
 * AFFERENTS SPINDLES
 * The dynamics for spindle afferents are handled by a "member object" of the
 * Poppele70Spindle class.
 * The Poppele70Spindle object requires muscle fiber "acceleration", which is 
 * not available from the Millard2012EquilibriumMuscle class, so this has to
 * be approximated numerically. To aid this, this muscle class provides a
 * LPF'ed version of the fiber velocity and LPF'ed version of the acceleration is 
 * provided as a state variable.
 * 
 * AFFERENTS TENDON ORGANS
 * The dynamics of the GTO are handled by an object of the Lin02GolgiTendonOrgan class.
 * 
 * INPUTS
 * Unlike most muscle types, Millard12EqWithAfferents expects 3 inputs. One input causes
 * the muscle activation, and the other 2 static and dynamic gamma motoneuron input
 * that modulates the response of a muscle spindle. The inputs can be specified using objects
 * of the SpindleController, or the TugOfWarController classes.
 */
class Millard12EqMuscleWithIntermediateAfferent : public Millard2012EquilibriumMuscle {OpenSim_DECLARE_CONCRETE_OBJECT(Millard12EqMuscleWithIntermediateAfferent, Millard2012EquilibriumMuscle);

friend class Poppele70Spindle;

public:
	/* PROPERTIES */
	OpenSim_DECLARE_PROPERTY(lpf_tau, double, "time constant for all the low-pass filters");

	// the afferents
	OpenSim_DECLARE_PROPERTY(spindle, Poppele70Spindle, "Spindle model");
	OpenSim_DECLARE_PROPERTY(GTO, Lin02GolgiTendonOrgan, "GTO model");

	// This storage object records the afferent outputs
	Storage *afferents;
	
private:

	/** These auxiliary vectors are used to approximate the derivatives of muscle variables. 
	 *  'vel' and 'ts' are the "velocity" and "time values" used in the method
	 *  to calculate the acceleration. 
	 *  Since computeStateVariableDerivatives is const, mutable is needed. */
// No mutable arrays needed, handled by cascaded filter		
public:
	/* METHODS */
	// The class constructors 
	Millard12EqMuscleWithIntermediateAfferent();
	Millard12EqMuscleWithIntermediateAfferent(const std::string &name, double maxIsometricForce, 
								double optimalFiberLength,	double tendonSlackLength,
								double pennationAngle);

	// Get & Set "state variables" and their "derivatives"
	double getLPFvelocity(const SimTK::State& s) const;
	void   setLPFvelocity(SimTK::State& s, double Velocity) const;
	double getLPFacceleration(const SimTK::State& s) const;
	void   setLPFacceleration(SimTK::State& s, double Acceleration) const;
	
	// Get & Set the Properties
	double getLPFtau() const { return get_lpf_tau(); }
	void setLPFtau(double aLPFtau);

	double getIaAfferent(const SimTK::State& s) const { return get_spindle().getIaOutput(s); }
	double getIIAfferent(const SimTK::State& s) const { return 0.0; } // Poppele doesn't have II yet
	double getIbAfferent(const SimTK::State& s) const { return get_GTO().getGTOout(s); }

    OpenSim_DECLARE_OUTPUT(primary_Ia, double, getIaAfferent, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(secondary_II, double, getIIAfferent, SimTK::Stage::Dynamics);

private:
	/** construct the new properties and set their default values */
	void constructProperties();

public:
	/* OTHER SHORT METHODS */
	// This function should override Actuator::numControls() 
	int numControls() const override {return 3;};
	
	// This function allows to peek at the Poppele70Spindle object, Mostly for the sake of Analysis objects
	const Poppele70Spindle* getSpindle() const { return &get_spindle(); }
	
	// This one allows to peek at the Lin02GolgiTendonOrgan object, Mostly the same.
	const Lin02GolgiTendonOrgan* getGTO() const { return &get_GTO(); }
	
protected:
	/* "MODEL COMPONENT" INTERFACES */
	// the way to add new ODE muscle states to the multibody system corresponding to this muscle
	void extendAddToSystem(SimTK::MultibodySystem& system) const override;

	// the way to initialize muscle state variables by using properties.
	void extendInitStateFromProperties(SimTK::State& s) const override;

	// use the current values of the muscle states to update the properties
	void extendSetPropertiesFromState(const SimTK::State& s) override;

	// the way to declare the spindle as a subcomponent. 
	void extendConnectToModel(Model& aModel) override;

	/* COMPUTATIONS */
	// This function finds the initial state for the tension & then it calls the same-named method of the parent class
	void computeInitialFiberEquilibrium(SimTK::State& s) const override;
	
	// Compute the derivatives for muscle state variables of this muscle
	void computeStateVariableDerivatives(const SimTK::State& s) const override; 
		
	/** The following function calculates the derivative of the fiber velocity
	 *  using the method in: Fornberg 1998 "Calculation of Weights in Finite
	 *  Difference Formulas" SIAM Rev.40(3):685-691.
	 *  The method is adapted to use with four points and to always evaluate at the
	 *  last point.  */
	double approxFiberAcceleration(const SimTK::State& s) const;
};	
}
