#include "Poppele70Spindle.h"

using namespace std;
using namespace OpenSim;

Poppele70Spindle::Poppele70Spindle() 
{
}

double Poppele70Spindle::getPoppeleX1(const SimTK::State& s) const { return getStateVariableValue(s, "poppele_x1"); }
void Poppele70Spindle::setPoppeleX1(SimTK::State& s, double val) const { setStateVariableValue(s, "poppele_x1", val); }
double Poppele70Spindle::getPoppeleX2(const SimTK::State& s) const { return getStateVariableValue(s, "poppele_x2"); }
void Poppele70Spindle::setPoppeleX2(SimTK::State& s, double val) const { setStateVariableValue(s, "poppele_x2", val); }
double Poppele70Spindle::getPoppeleX3(const SimTK::State& s) const { return getStateVariableValue(s, "poppele_x3"); }
void Poppele70Spindle::setPoppeleX3(SimTK::State& s, double val) const { setStateVariableValue(s, "poppele_x3", val); }
double Poppele70Spindle::getPoppeleX4(const SimTK::State& s) const { return getStateVariableValue(s, "poppele_x4"); }
void Poppele70Spindle::setPoppeleX4(SimTK::State& s, double val) const { setStateVariableValue(s, "poppele_x4", val); }

double Poppele70Spindle::getIaOutput(const SimTK::State& s) const
{
    return getCacheVariableValue<double>(s, "primaryIa"); 
}

void Poppele70Spindle::setIaOutput(const SimTK::State& s, double IaOutput) const
{
    setCacheVariableValue(s, "primaryIa", IaOutput);
}

void Poppele70Spindle::setOwnerMuscleName(std::string OwnerMuscleName)
{
    ownerMuscleName = OwnerMuscleName;
}

void Poppele70Spindle::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);
    
    // Poppele Spindle Model States
    addStateVariable("poppele_x1");
    addStateVariable("poppele_x2");
    addStateVariable("poppele_x3");
    addStateVariable("poppele_x4");
    
    // adding the output in cache variables
    addCacheVariable("primaryIa", 0.0, SimTK::Stage::Dynamics);
}

void Poppele70Spindle::extendInitStateFromProperties(SimTK::State& s) const
{
    Super::extendInitStateFromProperties(s);
    
    setPoppeleX1(s, 0.0);
    setPoppeleX2(s, 0.0);
    setPoppeleX3(s, 0.0);
    setPoppeleX4(s, 0.0);
}

void Poppele70Spindle::extendConnectToModel(Model& aModel) 
{
    Super::extendConnectToModel(aModel);
    
    if( (aModel.getMuscles()).contains(ownerMuscleName) )
        musclePtr = &((aModel.getMuscles()).get(ownerMuscleName));
}

void Poppele70Spindle::computeStateVariableDerivatives(const SimTK::State& s) const
{
    if (!musclePtr) {
        setStateVariableDerivativeValue(s, "poppele_x1", 0.0);
        setStateVariableDerivativeValue(s, "poppele_x2", 0.0);
        setStateVariableDerivativeValue(s, "poppele_x3", 0.0);
        setStateVariableDerivativeValue(s, "poppele_x4", 0.0);
        setIaOutput(s, 0.0);
        return;
    }

    // 2. Extract Muscle Stretch
    // Poppele 1970 uses stretch in mm. We will use L_norm - 1.0, scaled to roughly match biological firing rates.
    // We will multiply delta_L by 100 to scale from 0.01 normalized stretch to 1.0 (equivalent to 1mm)
    double L_norm = musclePtr->getNormalizedFiberLength(s);
    double u = (L_norm - 1.0) * 100.0;
    
    // 3. Poppele State Space Matrices
    double a0 = 2937.6;
    double a1 = 77059.584;
    double a2 = 90513.633;
    double a3 = 600.856;
    
    double c0 = -264384000.0;
    double c1 = -6915673440.0;
    double c2 = -8099289057.6;
    double c3 = -49060440.0;
    double d = 90000.0;
    
    // 4. Get Current Poppele States
    double x1 = getPoppeleX1(s);
    double x2 = getPoppeleX2(s);
    double x3 = getPoppeleX3(s);
    double x4 = getPoppeleX4(s);
    
    // 5. Calculate Derivatives
    double dx1 = x2;
    double dx2 = x3;
    double dx3 = x4;
    double dx4 = -a0*x1 - a1*x2 - a2*x3 - a3*x4 + u;
    
    setStateVariableDerivativeValue(s, "poppele_x1", dx1);
    setStateVariableDerivativeValue(s, "poppele_x2", dx2);
    setStateVariableDerivativeValue(s, "poppele_x3", dx3);
    setStateVariableDerivativeValue(s, "poppele_x4", dx4);
    
    // 6. Calculate Output (Firing Rate)
    double y = c0*x1 + c1*x2 + c2*x3 + c3*x4 + d*u;
    
    // Ensure firing rate is non-negative and biologically bounded
    if (y < 0.0) y = 0.0;
    if (y > 500.0) y = 500.0;
    
    setIaOutput(s, y);
}
