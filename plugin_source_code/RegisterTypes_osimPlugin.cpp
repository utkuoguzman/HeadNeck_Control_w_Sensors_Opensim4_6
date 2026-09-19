#include "RegisterTypes_osimPlugin.h"
#include "Millard12EqMuscleWithAfferents.h"
#include "Millard12EqMuscleWithSimplifiedAfferent.h"
#include "Mileusnic06Spindle.h"
#include "Lin02GolgiTendonOrgan.h"
#include "SpindleController.h"
#include "AfferentAnalysis.h"
#include <OpenSim/Common/Object.h>

using namespace OpenSim;

extern "C" OSIMPLUGIN_API void RegisterTypes_osimMillard12EqWithAff() {
    std::ofstream log("D:/plugin_log.txt");
    try{
        log << "Registering Millard12EqMuscleWithAfferents..." << std::endl;
        Object::registerType(Millard12EqMuscleWithAfferents());
        
        log << "Registering Millard12EqMuscleWithSimplifiedAfferent..." << std::endl;
        Object::registerType(Millard12EqMuscleWithSimplifiedAfferent());

        log << "Registering Mileusnic06Spindle..." << std::endl;
        Object::registerType(Mileusnic06Spindle());

        log << "Registering Lin02GolgiTendonOrgan..." << std::endl;
        Object::registerType(Lin02GolgiTendonOrgan());

        log << "Registering SpindleController..." << std::endl;
        Object::registerType(SpindleController());

        log << "Registering AfferentAnalysis..." << std::endl;
        Object::registerType(AfferentAnalysis());

        // If it succeeds, it writes this file
        log << "SUCCESS: All classes registered perfectly!" << std::endl;

    } catch (const std::exception& e) {
        // If it crashes, it catches the hidden OpenSim 4.6 error and writes it here
        std::ofstream log("D:/plugin_log.txt");
        log << "CRITICAL EXCEPTION: " << e.what() << std::endl;
    } catch (...) {
        std::ofstream log("D:/plugin_log.txt");
        log << "CRITICAL EXCEPTION: Unknown C++ Error!" << std::endl;
    }
}

// The trigger that forces the GUI to run the registration above
class PluginInstantiator {
public:
    PluginInstantiator() {
        RegisterTypes_osimMillard12EqWithAff();
    }
};

static PluginInstantiator autoRunner;