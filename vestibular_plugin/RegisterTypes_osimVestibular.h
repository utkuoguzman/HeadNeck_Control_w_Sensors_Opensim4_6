#ifndef _RegisterTypes_osimVestibular_h_
#define _RegisterTypes_osimVestibular_h_

#include "osimVestibularDLL.h"

extern "C" {
    OSIMVESTIBULAR_API void RegisterTypes_osimVestibular();
}

class osimVestibularInstantiator {
public:
    osimVestibularInstantiator();
private:
    void registerDllClasses();
};

#endif // _RegisterTypes_osimVestibular_h_
