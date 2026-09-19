/*
 * TugOfWarController.h
 *
 * This file is a modified version of the file ControllerExample.cpp, which
 * is part of an OpenSim tutorial:
 * http://simtk-confluence.stanford.edu:8080/display/OpenSim/Creating+a+Controller
 *
 */
 
 #include <OpenSim/Simulation/Control/Controller.h>
 
 namespace OpenSim {
	 
 class TugOfWarController : public Controller {
OpenSim_DECLARE_CONCRETE_OBJECT(TugOfWarController, Controller);

// This section contains methods that can be called in this controller class.
public:
	/**
	 * Constructor
	 *
	 * @param aModel Model to be controlled
	 * @param aKp Position gain by which the position error will be multiplied
	 */
	TugOfWarController(double aKp, double aKv);
	
	void computeControls(const SimTK::State& s, SimTK::Vector &controls) const override;
	
private:

	/** Position gain for this controller */
	double kp;
	
	/** Velocity gain for this controller */
	double kv;
	
};

}; // namespace OpenSim