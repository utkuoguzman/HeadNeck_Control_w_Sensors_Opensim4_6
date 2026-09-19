/* SpindleController.cpp
*/

//=============================================================================
// INCLUDES
//=============================================================================
#include "SpindleController.h"
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/GCVSpline.h>
#include <OpenSim/Common/PiecewiseConstantFunction.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/Actuator.h>

//=============================================================================
// STATICS
//=============================================================================

// This command indicates that any identifier (class, variable, method, etc.)
// defined within the OpenSim namespace can be used in this file without the
// "OpenSim::" prefix.
using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
/*
 * Default constructor.
 */
SpindleController::SpindleController() :
	Controller()
{
	setNull();
	constructProperties();
}

/*
 * Convenience constructor.
 */
SpindleController::
	SpindleController(const std::string& controlsFileName, 
						 int interpMethodType) : Controller()
{
	setNull();
	constructProperties();
	set_controls_file(controlsFileName);
	set_interpolation_method(interpMethodType);
}

/*
 * Destructor.
 */
SpindleController::~SpindleController()
{
}

/*
 * Set NULL values for all member variables.
 */
void SpindleController::setNull()
{
	setAuthors("Sergio Verduzco from code by Ajay Seth");
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SpindleController::constructProperties()
{
	constructProperty_ControlFunctions(FunctionSet());
	constructProperty_SpindleFunctionsStatic(FunctionSet());
	constructProperty_SpindleFunctionsDynamic(FunctionSet());
	constructProperty_controls_file();
	constructProperty_interpolation_method();
}


void SpindleController::extendConnectToModel(Model& model)
{
	Super::extendConnectToModel(model);
	if(!getProperty_controls_file().empty()){
		Storage controls(get_controls_file());
		const Array<string>& columns = controls.getColumnLabels();

		int ncols = columns.getSize();

		int tcol = columns.findIndex("time");
		if(tcol < 0){
			tcol = columns.findIndex("t");
			if(tcol < 0){
				throw Exception("SpindleController::connectToModel pescribed "
				"controls file was not specified as functions of time.",
					__FILE__, __LINE__);
			}
		}
		int nrows = controls.getSize();
		Array<double> time(0.0, nrows);
		Array<double> data(0.0, nrows);
		controls.getTimeColumn(time);

		FunctionSet& controlFuncs = upd_ControlFunctions();
		const Set<Actuator>& modelActuators = getModel().getActuators();

		//Set<Actuator>& controllerActuators = updActuators();

		for(int i=0; i<ncols; ++i){
			if(i == tcol) continue;
			const string& actName = columns[i];
			int found = controlFuncs.getIndex(actName);
			// if the columns is for a control already part of the set,
			// or is time, ignore it.
			if(found < 0){  // not found in the controllers set of functions
				// find a corresponding actuator in the model
				found = modelActuators.getIndex(actName);
				if(found >= 0){ // found a corresponding actuator
    				controls.getDataColumn(controls.getStateIndex(actName), data);
    				Function* pfunc = createFunctionFromData(actName, time, data);
    
    				// Connect the actuator to this controller's socket
    				appendSocketConnectee_actuators(modelActuators.get(actName));
    				prescribeControlForActuator(actName, pfunc, pfunc, pfunc); 
				}
				else{
					cout << "SpindleController::connectToModel() could not "
						"find actuator '" << actName << "' in the model." <<endl;
				}
			}// if found in functions, it has already been prescribed
		}// end looping through columns
	}// if no constrols storage specified, do nothing
}


// compute the control value for an actuator
void SpindleController::computeControls(const SimTK::State& s, SimTK::Vector& controls) const
{
	SimTK::Vector actControls(3, 0.0);
	SimTK::Vector time(1, s.getTime());

	for(int i = 0; i < getSocket<Actuator>("actuators").getNumConnectees(); i++){
		actControls[0] = get_ControlFunctions()[i].calcValue(time);
		actControls[1] = get_SpindleFunctionsStatic()[i].calcValue(time);
		actControls[2] = get_SpindleFunctionsDynamic()[i].calcValue(time);
		
		const Actuator& act = getConnectee<Actuator>("actuators", i);
		act.addInControls(actControls, controls);
	}
}


//=============================================================================
// GET AND SET
//=============================================================================

void SpindleController::prescribeControlForActuator(int index, 
								Function *prescribedFunction,
								Function *prescribedFunctionStatic,
								Function *prescribedFunctionDynamic)
{
	SimTK_ASSERT( index < getActuatorSet().getSize(), 
		"SpindleController::computeControl:  index > number of actuators" );
	SimTK_ASSERT( index >= 0,  
		"SpindleController::computeControl:  index < 0" );
	// In here, assert that the actuator can can take 3 controls ...
	SimTK_ASSERT( getActuatorSet().get(index).numControls() == 3,
		"SpindleController::prescribeControlForActuator: Actuator can't use 3 controls");
	
	if(index >= get_ControlFunctions().getSize())
	{
		upd_ControlFunctions().setSize(index+1);
		upd_SpindleFunctionsStatic().setSize(index+1);
		upd_SpindleFunctionsDynamic().setSize(index+1);
	}
	upd_ControlFunctions().set(index, prescribedFunction);
	upd_SpindleFunctionsStatic().set(index, prescribedFunctionStatic);
	upd_SpindleFunctionsDynamic().set(index, prescribedFunctionDynamic);
}

void SpindleController::prescribeControlForActuator(const std::string actName, 
								Function *prescribedFunction,
								Function *prescribedFunctionStatic,
								Function *prescribedFunctionDynamic)
{
	int index = -1;
	for(int i = 0; i < getSocket<Actuator>("actuators").getNumConnectees(); ++i) {
		if(getConnectee<Actuator>("actuators", i).getName() == actName) {
			index = i; 
			break;
		}
	}
	if(index < 0 )
		throw Exception("SpindleController does not have "+actName+" in its list of actuators to control.");
	prescribeControlForActuator(index, prescribedFunction, 
								prescribedFunctionStatic, prescribedFunctionDynamic);
}

// utility
Function* SpindleController::createFunctionFromData(const std::string& name,
						const Array<double>& time, const Array<double>& data)
{
	int method = 1;
	if(!getProperty_interpolation_method().empty())
		method = get_interpolation_method();

	if(method > 0)
		return new GCVSpline(method, time.getSize(), &time[0], &data[0], name);
	else if(method ==0)
		return new PiecewiseConstantFunction(time.getSize(), 
													&time[0], &data[0], name);
	else
		throw Exception("SpindleController- Invalid interpolation method.");
}