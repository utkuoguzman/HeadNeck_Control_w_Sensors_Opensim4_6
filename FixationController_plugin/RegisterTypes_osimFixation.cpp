#include <OpenSim/OpenSim.h>
#include "RegisterTypes_osimFixation.h"
#include "FixationController.h"

using namespace OpenSim;

class osimFixationInstantiator {
public:
    osimFixationInstantiator();
private:
    void registerDllClasses();
};

osimFixationInstantiator::osimFixationInstantiator() {
    registerDllClasses();
}

void osimFixationInstantiator::registerDllClasses() {
    RegisterTypes_osimFixation();
}

static osimFixationInstantiator osimFixationInstantiator_instance;

OSIMFIXATION_API void RegisterTypes_osimFixation() {
    try {
        Object::registerType(FixationController());
    } catch (const std::exception& e) {
        std::cerr << "Error during osimFixation registration: " << e.what() << std::endl;
    }
}
