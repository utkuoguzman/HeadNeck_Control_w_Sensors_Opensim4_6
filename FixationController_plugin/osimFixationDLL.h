#ifndef OSIMFIXATION_DLL_H_
#define OSIMFIXATION_DLL_H_

#ifndef _WIN32
    #define OSIMFIXATION_API
#else
    #ifdef OSIMFIXATION_EXPORTS
        #define OSIMFIXATION_API __declspec(dllexport)
    #else
        #define OSIMFIXATION_API __declspec(dllimport)
    #endif
#endif

#endif // OSIMFIXATION_DLL_H_

