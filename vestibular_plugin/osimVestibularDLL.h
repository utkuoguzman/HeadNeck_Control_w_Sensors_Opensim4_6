#ifndef _osimVestibularDLL_h_
#define _osimVestibularDLL_h_

// UNIX
#ifndef _WIN32
    #define OSIMVESTIBULAR_API
// WINDOWS
#else
    #ifdef OSIMVESTIBULAR_EXPORTS
        #define OSIMVESTIBULAR_API __declspec(dllexport)
    #else
        #define OSIMVESTIBULAR_API __declspec(dllimport)
    #endif
#endif

#endif // _osimVestibularDLL_h_

