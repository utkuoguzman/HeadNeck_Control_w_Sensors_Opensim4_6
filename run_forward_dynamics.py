import os
import sys
try:
    script_dir = os.path.dirname(os.path.abspath(__file__))
except:
    script_dir = r"D:\Akademik\PhD_Thesis\OpenSim\HeadNeck_Control_w_Sensors_Opensim4_6"
os.chdir(script_dir)

import opensim as osim
import os

def run_forward_dynamics():
    try:
        print("Loading OpenSim libraries...")
        # Paths to your compiled DLLs
        # Adjust these paths if your DLLs are located elsewhere
        base_dir = r"."
        
        osim.Model.LoadOpenSimLibrary(os.path.join(base_dir, r"proprioception_plugin\build\Release\osimMillard12EqWithAff.dll"))
        osim.Model.LoadOpenSimLibrary(os.path.join(base_dir, r"vestibular_plugin\build\Release\osimVestibular.dll"))
        osim.Model.LoadOpenSimLibrary(os.path.join(base_dir, r"HeadNeckNeuralController_plugin\build\Release\osimHeadNeckNeural.dll"))
        osim.Model.LoadOpenSimLibrary(os.path.join(base_dir, r"FixationController_plugin\build\Release\osimFixation.dll"))

        model_path = os.path.join(base_dir, r"osim_files\HYOID_HeadNeckNeural_Model.osim")
        print("Loading model: %s" % model_path)
        model = osim.Model(model_path)
        model.setUseVisualizer(False)

        print("Initializing system...")
        state = model.initSystem()

        # You can set initial states here if you want
        # e.g., model.getCoordinateSet().get("r_eye_add_abd").setValue(state, 0.0)

        print("Starting 3-second Forward Dynamics Simulation...")
        manager = osim.Manager(model)
        
        # Relaxing integrator accuracy to prevent freezing on stiff muscle/neural dynamics
        manager.setIntegratorAccuracy(1e-3)
        manager.initialize(state)
        
        # Run simulation
        manager.integrate(3.0)

        # Save results
        states_table = manager.getStatesTable()
        output_file = os.path.join(base_dir, r"state_files\forward_simulation_results.sto")
        osim.STOFileAdapter.write(states_table, output_file)

        print("Success! Saved states to: %s" % output_file)

    except Exception as e:
        print("An error occurred: %s" % e)

if __name__ == "__main__":
    run_forward_dynamics()

