#include "RegisterTypes_osimHeadNeckNeuralAdaptive.h"
#include "HeadNeckNeuralControllerAdaptive.h"

using namespace OpenSim;

static osimHeadNeckNeuralAdaptiveInstantiator osimHeadNeckNeuralAdaptiveInstantiatorVar;

OSIMHEADNECKNEURALADAPTIVE_API void RegisterTypes_osimHeadNeckNeuralAdaptive() {
    try {
        Object::registerType(HeadNeckNeuralControllerAdaptive());
    } catch (const std::exception& e) {
        std::cerr << "Error registering HeadNeckNeuralControllerAdaptive: " << e.what() << std::endl;
    }
}

osimHeadNeckNeuralAdaptiveInstantiator::osimHeadNeckNeuralAdaptiveInstantiator() {
    registerDllClasses();
}

void osimHeadNeckNeuralAdaptiveInstantiator::registerDllClasses() {
    RegisterTypes_osimHeadNeckNeuralAdaptive();
}

