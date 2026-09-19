#include <OpenSim/OpenSim.h>
#include "RegisterTypes_osimVestibular.h"
#include "Schneider15VestibularAfferent.h"

static osimVestibularInstantiator vestibular_instantiator;

OSIMVESTIBULAR_API void RegisterTypes_osimVestibular()
{
    try {
        OpenSim::Object::registerType(OpenSim::Schneider15VestibularAfferent());
    } catch (const std::exception& e) {
        std::cerr << "Error during osimVestibular registration: " << e.what() << std::endl;
    }
}

osimVestibularInstantiator::osimVestibularInstantiator()
{
    registerDllClasses();
}

void osimVestibularInstantiator::registerDllClasses()
{
    RegisterTypes_osimVestibular();
}

