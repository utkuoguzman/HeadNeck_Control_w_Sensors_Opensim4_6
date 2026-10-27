#include "RegisterTypes_osimHeadNeckNeural.h"
#include "HeadNeckNeuralController.h"

using namespace OpenSim;

static osimHeadNeckNeuralInstantiator osimHeadNeckNeuralInstantiatorVar;

OSIMHEADNECKNEURAL_API void RegisterTypes_osimHeadNeckNeural() {
    try {
        Object::registerType(HeadNeckNeuralController());
    } catch (const std::exception& e) {
        std::cerr << "Error registering HeadNeckNeuralController: " << e.what() << std::endl;
    }
}

osimHeadNeckNeuralInstantiator::osimHeadNeckNeuralInstantiator() {
    registerDllClasses();
}

void osimHeadNeckNeuralInstantiator::registerDllClasses() {
    RegisterTypes_osimHeadNeckNeural();
}
