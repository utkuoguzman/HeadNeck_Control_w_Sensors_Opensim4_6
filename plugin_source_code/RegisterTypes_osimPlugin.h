#ifndef _RegisterTypes_osimPlugin_h_
#define _RegisterTypes_osimPlugin_h_

#ifdef _WIN32
    #define OSIMPLUGIN_API __declspec(dllexport)
#else
    #define OSIMPLUGIN_API
#endif

extern "C" {
    OSIMPLUGIN_API void RegisterTypes_osimMillard12EqWithAff();
}

#endif // _RegisterTypes_osimPlugin_h_