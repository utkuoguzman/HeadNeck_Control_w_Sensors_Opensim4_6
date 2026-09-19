#include <OpenSim/OpenSim.h>
#include "FixationController.h"

using namespace OpenSim;

int main() {
    try {
        LoadOpenSimLibrary("osimMillard12EqWithAff");
        LoadOpenSimLibrary("osimVestibular");
        // Load the model
        std::string modelPath = "../../osim_files/HYOID_ScaledStrength_v1.2andEye_v1_IndependentEyes_Afferents.osim";
        Model model(modelPath);

        // Add the new FixationController
        FixationController* fixCon = new FixationController();
        fixCon->setName("Binocular_Fixation_Controller");
        
        // Set an initial target point 1 meter straight ahead and slightly down (e.g. looking at an object)
        fixCon->set_target_location(SimTK::Vec3(1.0, -0.2, 0.0));

        // Connect eye muscles to the controller's actuators socket to prevent GUI errors
        const std::string muscles[] = {
            "r_Lateral_Rectus", "r_Medial_Rectus", "r_Superior_Rectus", "r_Inferior_Rectus", "r_Superior_Oblique", "r_Inferior_Oblique",
            "l_Lateral_Rectus", "l_Medial_Rectus", "l_Superior_Rectus", "l_Inferior_Rectus", "l_Superior_Oblique", "l_Inferior_Oblique"
        };
        for (const auto& mName : muscles) {
            if (model.getMuscles().contains(mName)) {
                fixCon->addActuator(model.getMuscles().get(mName));
            }
        }
        
        // Add to the model's controller set
        model.addController(fixCon);

        // Finalize connections so the new actuators socket links are saved
        model.finalizeConnections();

        // Save the updated model
        std::string outPath = "../../osim_files/HYOID_ScaledStrength_v1.2andEye_v1_IndependentEyes_Afferents_Fixation.osim";
        model.print(outPath);
        std::cout << "Successfully added Binocular Fixation Controller and saved to: " << outPath << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
