#ifndef OPENSIM_REGISTERTYPES_OSIM_HEAD_NECK_NEURAL_H_
#define OPENSIM_REGISTERTYPES_OSIM_HEAD_NECK_NEURAL_H_

#include <OpenSim/OpenSim.h>

#ifdef _WIN32
    #ifdef OSIMHEADNECKNEURAL_EXPORTS
        #define OSIMHEADNECKNEURAL_API __declspec(dllexport)
    #else
        #define OSIMHEADNECKNEURAL_API __declspec(dllimport)
    #endif
#else
    #define OSIMHEADNECKNEURAL_API
#endif

extern "C" {
    OSIMHEADNECKNEURAL_API void RegisterTypes_osimHeadNeckNeural();
}

class osimHeadNeckNeuralInstantiator {
public:
    osimHeadNeckNeuralInstantiator();
private:
    void registerDllClasses();
};

#endif // OPENSIM_REGISTERTYPES_OSIM_HEAD_NECK_NEURAL_H_

