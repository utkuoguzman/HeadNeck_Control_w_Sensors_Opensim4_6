#ifndef OPENSIM_REGISTERTYPES_OSIM_HEAD_NECK_NEURAL_ADAPTIVE_H_
#define OPENSIM_REGISTERTYPES_OSIM_HEAD_NECK_NEURAL_ADAPTIVE_H_

#include <OpenSim/OpenSim.h>

#ifdef _WIN32
    #ifdef OSIMHEADNECKNEURALADAPTIVE_EXPORTS
        #define OSIMHEADNECKNEURALADAPTIVE_API __declspec(dllexport)
    #else
        #define OSIMHEADNECKNEURALADAPTIVE_API __declspec(dllimport)
    #endif
#else
    #define OSIMHEADNECKNEURALADAPTIVE_API
#endif

extern "C" {
    OSIMHEADNECKNEURALADAPTIVE_API void RegisterTypes_osimHeadNeckNeuralAdaptive();
}

class osimHeadNeckNeuralAdaptiveInstantiator {
public:
    osimHeadNeckNeuralAdaptiveInstantiator();
private:
    void registerDllClasses();
};

#endif // OPENSIM_REGISTERTYPES_OSIM_HEAD_NECK_NEURAL_ADAPTIVE_H_

