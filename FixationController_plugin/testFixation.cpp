#include <OpenSim/OpenSim.h>
#include <iostream>

using namespace OpenSim;

int main() {
    try {
        // Load required plugins
        LoadOpenSimLibrary("osimMillard12EqWithAff");
        LoadOpenSimLibrary("osimVestibular");
        LoadOpenSimLibrary("osimHeadNeckNeural");
        LoadOpenSimLibrary("osimFixation");

        // Load the model
        std::string modelPath = "D:/Akademik/PhD_Thesis/OpenSim/Proprioception_Plugin_Opensim4_6_Explicit/osim_files/HYOID_HeadNeckNeural_Model.osim";
        Model model(modelPath);
        model.setUseVisualizer(false);

        SimTK::State& state = model.initSystem();
        
        std::cout << "Starting 3-second Forward Dynamics Simulation..." << std::endl;

        // Run simulation for 3 seconds
        Manager manager(model);
        manager.setIntegratorAccuracy(1e-3); // relaxed tolerance for speed and to avoid stiff hangs
        manager.initialize(state);
        
        // This will print progress to the console
        manager.integrate(3.0);

        // Save the results
        auto statesTable = manager.getStatesTable();
        STOFileAdapter::write(statesTable, "forward_simulation_results.sto");

        std::cout << "Success! Saved states to: forward_simulation_results.sto" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}

