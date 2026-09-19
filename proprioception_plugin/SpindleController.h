#ifndef OPENSIM_SPINDLE_CONTROLLER_H_
#define OPENSIM_SPINDLE_CONTROLLER_H_


#include <OpenSim/Simulation/Control/Controller.h>
#include <OpenSim/Common/FunctionSet.h>
//#include "osimPluginDLL.h" // for registering as plugin


namespace OpenSim { 

class Function;

//=============================================================================
//=============================================================================
/**
 * SpindleController is a concrete Controller that specifies functions to 
 * prescribe the control values of muscles with spindle afferents as a function 
 * of time. A spindle afferent is an object of  class Mileusnic06Spindle, and
 * expects 2 inputs: corresponding to static and dynamic gamma motoneurons.
 *
 * The class SpindleController is a version of the class PrescribedController.h,
 * where modifications have been made to allow three control functions for a 
 * single actuator (an object of the Millard12EqMuscleWithAfferents class) that 
 * contains a Mileusnic06Spindle object.
 */

class SpindleController : public Controller {
OpenSim_DECLARE_CONCRETE_OBJECT(SpindleController, Controller);

//=============================================================================
// DATA
//=============================================================================

public:
	/** FunctionSet of prescribed controls associated with each actuator. */
	OpenSim_DECLARE_PROPERTY(ControlFunctions, FunctionSet,
		"Functions (one per control) describing the controls for spindle actuators"
		"specified for this controller." );
		
	/** FunctionSet describing the inputs corresponding to static gamma motoneruons */
	OpenSim_DECLARE_PROPERTY(SpindleFunctionsStatic, FunctionSet,
		"Functions (one per control) describing the inputs to nuclear chain fibers and"
		"static nuclear bag fibers." );
		
	/** FunctionSet describing the inputs corresponding to dynamic gamma motoneruons */
	OpenSim_DECLARE_PROPERTY(SpindleFunctionsDynamic, FunctionSet,
		"Functions (one per control) describing the inputs to dynamic nuclear bag fibers." );

	/** (Optional) prescribed controls from a storage file  */
	/** NOT YET ADAPTED */
	OpenSim_DECLARE_OPTIONAL_PROPERTY(controls_file, std::string,
		"Controls storage (.sto) file containing controls for individual "
		"actuators in the model. Column labels must match actuator names."
		"NOT ADAPTED YET FROM PrescribedController");

	/** (Optional) interpolation method for controls in storage.  */
	OpenSim_DECLARE_OPTIONAL_PROPERTY(interpolation_method, int,
		"Interpolate the controls file data using piecewise: '0-constant', "
		"'1-linear', '3-cubic' or '5-quintic' functions.");

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION AND DESTRUCTION
	//--------------------------------------------------------------------------
public:
	/** Default constructor */
	SpindleController();

	/** Convenience constructor get controls from file
	 * @param controlsFileName  string containing the controls storage (.sto) 
	 * @param interpMethodType	int 0-constant, 1-linear, 3-cubic, 5-quintic
	 *                          defaults to linear.
	 */
	SpindleController(const std::string& controlsFileName, 
						 int interpMethodType = 1);

	/** Destructor */
	virtual ~SpindleController();

	//--------------------------------------------------------------------------
	// CONTROL
	//--------------------------------------------------------------------------
	/**
	 * Compute the control values for all actuators under the control of this
	 * Controller.
	 *
	 * @param s             system state 
	 * @param controls      model controls  
	 */
	void computeControls(const SimTK::State& s, SimTK::Vector& controls) const override;

	/**
	 *	Assign prescribed control functions for the desired actuator identified 
	 *  by its index. Controller takes ownership of the function.
	 *  @param index                the actuator's index in the controller's set
	 *	@param prescribedFunction   the actuator's control function
	 *  @param prescribedFunctionStatic     function describing static gamma motoneurons
	 *  @param prescribedFunctionDynamic    function describing dynamic gamma motoneurons
	 */
	void prescribeControlForActuator(int index, 
									Function *prescribedFunction,
									Function *prescribedFunctionStatic,
									Function *prescribedFunctionDynamic);

	/**
	 *	Assign prescribed control functions for the desired actuator identified
	 *  by its name. Controller takes ownership of the function.
	 *  @param actName                the actuator's name in the controller's set
	 *	@param prescribedFunction   the actuator's control function
	 *  @param prescribedFunctionStatic     function describing static gamma motoneurons
	 *  @param prescribedFunctionDynamic    function describing dynamic gamma motoneurons
	 */
	void prescribeControlForActuator(const std::string actName,
									 Function *prescribedFunction,
									 Function *prescribedFunctionStatic,
									 Function *prescribedFunctionDynamic);

public:
	/** Model component interface */
	void extendConnectToModel(Model& aModel) override;
private:
	// construct and initialize properties
	void constructProperties();

	// utility
	Function* createFunctionFromData(const std::string& name,
		const Array<double>& time, const Array<double>& data);

	// This method sets all member variables to default (e.g., NULL) values.
	void setNull();

//=============================================================================
};	// END of class SpindleController

}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_SPINDLE_CONTROLLER_H_


