/* -------------------------------------------------------------------------- *
 *                       OpenSim:  AfferentAnalysis.cpp                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Sergio Verduzco, from MyAnalysis.cpp                            *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

 #define NV 2   // Number of variables to store from the Mileusnic06Spindle object
 
//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/Muscle.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include "AfferentAnalysis.h"
#include "Millard12EqMuscleWithAfferents.h"

using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S) AND SETUP
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor
 */
AfferentAnalysis::AfferentAnalysis() : Analysis()
{
    setNull();
    constructProperties();
}

/** 
 * A constructor that receives the model as argument
 */
AfferentAnalysis::AfferentAnalysis(Model *aModel) : Analysis()
{
	setNull();
	constructProperties();
	setModel(*aModel); /// will also set the model in the parent Analysis
	                   /// Should always call setModel before the simulation,
					   /// so the arrays ctrlInputs and spinVars get initialized
}

//_____________________________________________________________________________
/**
 * SetNull()
 */
void AfferentAnalysis::setNull()
{
    _muscleInputs = NULL;
}


//_____________________________________________________________________________
/*
 * Connect properties to local pointers.
 */
void AfferentAnalysis::constructProperties()
{
    Array<string> defaultMuscleNames;
    defaultMuscleNames.append("all");
    constructProperty_muscleListProp(defaultMuscleNames);

}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Construct a description for the body kinematics files.
 */
void AfferentAnalysis::
constructDescription()
{
	string descrip;

	descrip = "\nThis file contains values retrieved from within.\n";
	descrip += "an Millard12EqMuscleWithAfferents object, either ";
	descrip += "inputs or cache and state variables. \n\n";

	setDescription(descrip);
}

//_____________________________________________________________________________
/**
 * Set up storage objects.
 *
 * In general, the storage objects in your analysis are used to record
 * the results of your analysis and write them to file.  You will often
 * have a number of storage objects, each for recording a different
 * kind of result.
 */
void AfferentAnalysis::setupStorage()
{
	if(_model==NULL) return;
	
	// CLEAR EXISTING WORK ARRAYS
	/// _storageList comes from the Analysis class. I believe
	/// this will cause it to clear _storageList in the destructor. 
	_storageList.setMemoryOwner(true);
	_storageList.setSize(0); 
	//_muscleList.setSize(0);
	_muscleArray.setMemoryOwner(false);
	_muscleArray.setSize(0);
	
	
	_muscleInputs = new Storage(1000,"MuscleInputs");
	_muscleInputs->setDescription(getDescription());
	_storageList.append(_muscleInputs);
	
	_spindleVariables = new Storage(1000,"spindleVariables");
	_spindleVariables->setDescription(getDescription());
	_storageList.append(_spindleVariables);
	
	_gtoVariables = new Storage(1000,"gtoVariables");
	_gtoVariables->setDescription(getDescription());
	_storageList.append(_gtoVariables);
	
	// POPULATE _muscleList FOR "all" in _muscleListProp
	ForceSet& fSet = _model->updForceSet();
	
	int nm = getProperty_muscleListProp().size();
	_muscleList.setSize(0);
	
	for(int i = 0; i < nm; i++)
	{
		 _muscleList.append(get_muscleListProp(i));
	}
	if((nm==1) && (_muscleList.get(0)=="all")) {
		_muscleList.setSize(0);
		int nf = fSet.getSize();
		for(int i=0;i<nf;i++) {
			Muscle *m = dynamic_cast<Muscle*>(&fSet.get(i));
            if( m ) _muscleList.append(m->getName());
		}
	}
	// POPULATE ACTIVE MUSCLE ARRAY
	Array<string> tmpMuscleList("");
	nm = _muscleList.getSize();
	_muscleArray.setSize(0);
	for(int i=0; i<nm; i++) {
		if(fSet.contains(_muscleList[i])) {
    		Muscle* mus = dynamic_cast<Muscle*>( &fSet.get(_muscleList[i]) );
			if(mus){
				_muscleArray.append(mus);
				tmpMuscleList.append(mus->getName());
			}
		}
	}
	_muscleList = tmpMuscleList;

	// CONSTRUCT AND SET COLUMN LABELS
	// I have some heterogeneous storage objects, so I'll
	// construct the labels differently for each one
	int size = _storageList.getSize();
	for(int i=0;i<size;i++) {
		if(_storageList[i]==NULL) continue;
		Array<string> lable = columnLabels(i);
		_storageList[i]->setColumnLabels(lable);
	}

}

//_____________________________________________________________________________
/**
 * Construct column labels for the output results.
 *
 * For analyses that run during a simulation, the first column is almost
 * always time.  For the purpose of example, the code below adds labels
 * appropriate for recording the translation and orientation of each
 * body in the model.
 *
 * This method needs to be called as necessary to update the column labels.
 */
/**
 * Construct the column labels for the MuscleAnalysis storage files.
 * DEPRECATED
 */
void AfferentAnalysis::constructColumnLabels()
{
	if(!_model) return;
	int size = _muscleList.getSize();
	Array<string> labels("",size+1);
	labels[0] = "time";
	for(int i=0; i<size; i++) {
		labels.append( _muscleList[i] + "_I1" );
		labels.append( _muscleList[i] + "_I2" );
		labels.append( _muscleList[i] + "_I3" );
	}
	setColumnLabels(labels);
}

/**
 * Construct the column labels for the MuscleAnalysis storage files.
 * @param stor which of the storage objects' labels to return
 */
Array<string> AfferentAnalysis::columnLabels(int stor)
{
	int size = _muscleList.getSize();
	Array<string> labels("", size + 1);
	labels[0] = "time";
	
	if(stor == 0) // labels for the inputs storage
	{
		for(int i=0; i<size; i++) {
			labels.append( _muscleList[i] + "_I1" );
			labels.append( _muscleList[i] + "_I2" );
			labels.append( _muscleList[i] + "_I3" );
		}
	} else if( stor == 1 ) // spindle 
	{	
		for(int i=0; i<size; i++) {
			labels.append( _muscleList[i] + "_Ia" );
			labels.append( _muscleList[i] + "_II" );
		}
	}
	else if ( stor == 2 ) // GTO
	{
		for(int i=0; i<size; i++) {
			labels.append( _muscleList[i] + "_GTO" );
		}
	}
	else {
		cout << "WARNING: tried to build labels for invalid storage \n";
		cout << "in AfferentAnalysis::columnLabels. \n";
	}
	return labels;

}

/**
 * Specify a single muscle for which the analysis will be 
 * performed. Currently it's one muscle or all muscles.
 * Make sure the muscle is of the Millard12EqMuscleWithAfferents class.
 */
void AfferentAnalysis::specifyMuscle(const std::string &muscName)
{
	if(!_model) return;
	
	ForceSet& fSet = _model->updForceSet();
	try {
		fSet.get(muscName);
	}
	catch (OpenSim::Exception e) {
		cout << "WARNING - MyAnalyis::specifyMuscle() could not find ";
		cout << "the muscle with name" << muscName << '\n';
		cout << "Exception: " << e.getMessage() << '\n';
		return;
	}
	Array<string> newMuscleName;
    newMuscleName.append(muscName);
	set_muscleListProp(newMuscleName);
	
}
 
//_____________________________________________________________________________
/**
 * Set the model for which this analysis is to be run.
 *
 * Sometimes the model on which an analysis should be run is not available
 * at the time an analysis is created.  Or, you might want to change the
 * model.  This method is used to set the model on which the analysis is
 * to be run.
 *
 * @param aModel Model pointer
 */
void AfferentAnalysis::
setModel(Model& aModel)
{
	// SET THE MODEL IN THE BASE CLASS
	Super::setModel(aModel);

	// UPDATE VARIABLES IN THIS CLASS
	constructDescription();
	
	setupStorage();

	// Setup size of work array to hold muscle controls
	int numMuscles = _muscleList.getSize();
	ctrlInputs.setSize(3*numMuscles); // TODO generalize that 3
	
	// Setup size of work arrays to hold variables
	
	// NV is defined at the top of the file
	spinVars.setSize(NV*numMuscles);
	gtoVars.setSize(numMuscles);
}


//=============================================================================
// ANALYSIS
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute and record the results.
 *
 */
int AfferentAnalysis::record(const SimTK::State& s)
{
	if(_model==NULL) return(-1);
	
	int nm = _muscleArray.getSize();
	//double nan = SimTK::NaN;
	SimTK::Vector ctrlVec;  // temporary container for muscle controls
	SimTK::Vec<NV> stVars;	// temporary container for spindle Vars.
	double gtoOut;  // temporary container for GTO output
	
	bool inpWarning = false; // flag, so there are no repeated warnings
	
	for(int i=0; i<nm; ++i) {
		// storing controls
		try{
			ctrlVec = _muscleArray[i]->getControls(s);
		}
		catch (const std::exception& e) {
			if(!inpWarning){
				cout << "WARNING- AfferentAnalysis::record() unable to evaluate ";
				cout << "muscle controls at time " << s.getTime() << " for reason: ";
				cout << e.what() << endl;
				inpWarning = true;
			}
			continue;
		}
		int I = 3*i;
		memcpy(&ctrlInputs[I],&ctrlVec[0],3*sizeof(double)); // risky...
		
		// storing spindle variables
		// first make sure cache variables are valid
		if( !(dynamic_cast<Millard12EqMuscleWithAfferents*>
		     (_muscleArray[i])->getSpindle()->isCacheVariableValid(s, "primaryIa")) ||
			!(dynamic_cast<Millard12EqMuscleWithAfferents*>
		     (_muscleArray[i])->getSpindle()->isCacheVariableValid(s, "secondaryII")) )
		{
			_model->getMultibodySystem().realize(s,SimTK::Stage::Acceleration);
		}
		stVars[0] = dynamic_cast<Millard12EqMuscleWithAfferents*>
                        (_muscleArray[i])->getSpindle()->getIaOutput(s);		
		stVars[1] = dynamic_cast<Millard12EqMuscleWithAfferents*>
                        (_muscleArray[i])->getSpindle()->getIIOutput(s);
		
		int J = NV*i;
		memcpy(&spinVars[J],&stVars,NV*sizeof(double));
		
		// storing the output from the GTO
		if( !(dynamic_cast<Millard12EqMuscleWithAfferents*>
		     (_muscleArray[i])->getGTO()->isCacheVariableValid(s, "gto_out")) )
		{
			_model->getMultibodySystem().realize(s,SimTK::Stage::Acceleration);
		}
			
		gtoOut = dynamic_cast<Millard12EqMuscleWithAfferents*>
                             (_muscleArray[i])->getGTO()->getGTOout(s);

		int K = i;
		memcpy(&gtoVars[K],&gtoOut,sizeof(double));
	}
	   
	// APPEND TO STORAGE  
	_muscleInputs->append(s.getTime(),ctrlInputs.getSize(),&ctrlInputs[0]);
	_spindleVariables->append(s.getTime(),spinVars.getSize(),&spinVars[0]);
	_gtoVariables->append(s.getTime(),gtoVars.getSize(),&gtoVars[0]);
	
	return(0);
}
//_____________________________________________________________________________
/**
 * This method is called at the beginning of an analysis so that any
 * necessary initializations may be performed.
 *
 * This method is meant to be called at the begining of an integration in
 * Model::integBeginCallback() and has the same argument list.
 *
 * @param aStep Step number of the integration.
 * @param aDT Size of the time step that will be attempted.
 * @param aT Current time in the integration.
 * @param aX Current control values.
 * @param aY Current states.
 * @param aYP Current pseudo states.
 * @param aDYDT Current state derivatives.
 *
 * @return -1 on error, 0 otherwise.
 */
int AfferentAnalysis::begin(const SimTK::State& s)
{
	if(!proceed()) return(0);
	
	/// Allocation of storage objects happens in setModel() 

	// RESET STORAGE
	Storage *store;
	int size = _storageList.getSize();
	for(int i=0;i<size;i++) {
		store = _storageList[i];
		if(store==NULL) continue;
		store->purge();
	}

	// RECORD
	int status = 0;
	if(_storageList.getSize()> 0 && _storageList.get(0)->getSize() <= 0) status = record(s);

	return(status);
}
//_____________________________________________________________________________
/**
 * This method is called to perform the analysis.  It can be called during
 * the execution of a forward integrations or after the integration by
 * feeding it the necessary data.
 *
 * When called during an integration, this method is meant to be called in
 * Model::integStepCallback(), which has the same argument list.
 *
 * @param aXPrev Controls at the beginining of the current time step.
 * @param aYPrev States at the beginning of the current time step.
 * @param aYPPrev Pseudo states at the beginning of the current time step.
 * @param aStep Step number of the integration.
 * @param aDT Size of the time step that was just taken.
 * @param aT Current time in the integration.
 * @param aX Current control values.
 * @param aY Current states.
 * @param aYP Current pseudo states.
 * @param aDYDT Current state derivatives.
 *
 * @return -1 on error, 0 otherwise.
 */
int AfferentAnalysis::step(const SimTK::State& s, int stepNumber)
{
	if(!proceed(stepNumber)) return(0);

	record(s);

	return(0);
}
//_____________________________________________________________________________
/**
 * This method is called at the end of an analysis so that any
 * necessary finalizations may be performed.
 *
 * This method is meant to be called at the end of an integration in
 * Model::integEndCallback() and has the same argument list.
 *
 * @param aStep Step number of the integration.
 * @param aDT Size of the time step that was just completed.
 * @param aT Current time in the integration.
 * @param aX Current control values.
 * @param aY Current states.
 * @param aYP Current pseudo states.
 * @param aDYDT Current state derivatives.
 *
 * @return -1 on error, 0 otherwise.
 */
int AfferentAnalysis::end(const SimTK::State& s)
{
	if(!proceed()) return(0);

	record(s);

	return(0);
}

//=============================================================================
// IO
//=============================================================================
//_____________________________________________________________________________
/**
 * Print results.
 * 
 * The file names are constructed as
 * aDir + "/" + aBaseName + "_" + ComponentName + aExtension
 *
 * @param aDir Directory in which the results reside.
 * @param aBaseName Base file name.
 * @param aDT Desired time interval between adjacent storage vectors.  Linear
 * interpolation is used to print the data out at the desired interval.
 * @param aExtension File extension.
 *
 * @return 0 on success, -1 on error.
 */
int AfferentAnalysis::
printResults(const string &aBaseName,const string &aDir,double aDT,
				 const string &aExtension)
{
	std::string prefix = aBaseName + "_" + getName() + "_";
	for(int i=0; i<_storageList.getSize(); ++i){
		Storage::printResult(_storageList[i],prefix+_storageList[i]->getName(),aDir,aDT,aExtension);
	}

	return(0);
}


