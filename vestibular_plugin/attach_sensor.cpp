#include <OpenSim/OpenSim.h>
#include "Schneider15VestibularAfferent.h"
#include <iostream>

using namespace OpenSim;

#include "RegisterTypes_osimVestibular.h"

int main() {
    try {
        // Load the afferent plugins so we can parse the muscles and register types
        LoadOpenSimLibrary("osimMillard12EqWithAff");
        RegisterTypes_osimVestibular();

        // Load the model
        std::string model_path = "D:/Akademik/PhD_Thesis/OpenSim/Proprioception_Plugin_Opensim4_6_Explicit/osim_files/HYOID_ScaledStrength_v1.2andEye_v1_IndependentEyes_Afferents.osim";
        Model model(model_path);

        // Create the Vestibular Afferent sensor
        Schneider15VestibularAfferent* vestibular = new Schneider15VestibularAfferent();
        vestibular->setName("vestibular_sensor");

        // Try to find the head body
        const Body* head_frame = nullptr;
        if (model.getBodySet().contains("skull")) {
            head_frame = &model.getBodySet().get("skull");
        } else if (model.getBodySet().contains("head")) {
            head_frame = &model.getBodySet().get("head");
        } else {
            std::cerr << "Could not find 'skull' or 'head' body in the model!" << std::endl;
            return 1;
        }

        // Connect the sensor to the head frame
        vestibular->updSocket("head_frame").connect(*head_frame);

        // Add the sensor to the model's MiscModelComponentSet so the GUI can see it
        model.updMiscModelComponentSet().adoptAndAppend(vestibular);

        // Finalize connections so they can be printed to XML
        model.finalizeConnections();

        // Save the model
        model.print(model_path);
        std::cout << "Successfully attached vestibular_sensor to '" << head_frame->getName() << "' and saved " << model_path << "!" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
