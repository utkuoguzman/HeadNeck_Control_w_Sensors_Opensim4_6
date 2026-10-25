#include <OpenSim/OpenSim.h>
#include <iostream>

using namespace OpenSim;
using namespace SimTK;

int main() {
    try {
        LoadOpenSimLibrary("proprioception_plugin/build/Release/osimMillard12EqWithAff.dll");
        LoadOpenSimLibrary("vestibular_plugin/build/Release/osimVestibular.dll");
        LoadOpenSimLibrary("HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll");
        LoadOpenSimLibrary("FixationController_plugin/build/Release/osimFixation.dll");
        
        Model model("osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim");
        State& s = model.initSystem();
        
        const OpenSim::Body& skull = model.getBodySet().get("skull");
        SimTK::Transform X_mob = skull.getMobilizedBody().getBodyTransform(s);
        SimTK::Transform X_gnd = skull.getTransformInGround(s);
        std::cout << "X_mob R:" << std::endl << X_mob.R() << std::endl;
        std::cout << "X_gnd R:" << std::endl << X_gnd.R() << std::endl;
        
        Manager manager(model);
        manager.initialize(s);
        manager.integrate(0.01);
        std::cout << "Successfully stepped model to 0.01s." << std::endl;
    } catch (const std::exception& e) {
        std::cout << "Error: " << e.what() << std::endl;
    }
    return 0;
}
