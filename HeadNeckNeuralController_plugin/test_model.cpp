#include <OpenSim/OpenSim.h>
#include <iostream>

using namespace OpenSim;
using namespace SimTK;

int main() {
    try {
        LoadOpenSimLibrary("D:/Akademik/PhD_Thesis/OpenSim/Proprioception_Plugin_Opensim4_6_Explicit/plugin_source_code/build/Release/osimMillard12EqWithAff");
        LoadOpenSimLibrary("D:/Akademik/PhD_Thesis/OpenSim/Proprioception_Plugin_Opensim4_6_Explicit/vestibular_plugin/build/Release/osimVestibularDLL");
        LoadOpenSimLibrary("D:/Akademik/PhD_Thesis/OpenSim/Proprioception_Plugin_Opensim4_6_Explicit/HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural");
        
        Model model("D:/Akademik/PhD_Thesis/OpenSim/Proprioception_Plugin_Opensim4_6_Explicit/HeadNeckNeuralController_plugin/build/test.osim");
        State& s = model.initSystem();
        
        // The HeadNeckNeuralController should initialize and print its debug info during computeControls or initSystem
        // Let's force it by taking a step
        Manager manager(model);
        manager.initialize(s);
        manager.integrate(0.01);
        
        std::cout << "Successfully stepped model." << std::endl;
    } catch (const std::exception& e) {
        std::cout << "Error: " << e.what() << std::endl;
    }
    return 0;
}
