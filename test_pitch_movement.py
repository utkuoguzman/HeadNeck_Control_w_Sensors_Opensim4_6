import org.opensim.modeling as modeling
import os

model_path = r"D:\Akademik\PhD_Thesis\OpenSim\Proprioception_Plugin_Opensim4_6_Explicit\osim_files\HYOID_HeadNeckNeural_Model_WithLimits.osim"
model = modeling.Model(model_path)

# Find the controller
found_controller = False
for i in range(model.getControllerSet().getSize()):
    ctrl = model.getControllerSet().get(i)
    if ctrl.getConcreteClassName() == "HeadNeckNeuralController":
        # Force the desired pitch to 0.7 radians (approx 40 degrees flexion)
        ctrl.setPropertyValue_desired_pitch(0.7)
        found_controller = True
        print "Set desired_pitch to 0.7"
        break

if not found_controller:
    print "Controller not found!"

state = model.initSystem()
manager = modeling.Manager(model)
manager.setIntegratorAccuracy(1e-4)
manager.initialize(state)

print "Integrating for 1.0 second..."
manager.integrate(1.0)

states_table = manager.getStatesTable()
output_dir = r"D:\Akademik\PhD_Thesis\OpenSim\Proprioception_Plugin_Opensim4_6_Explicit"
output_file = os.path.join(output_dir, "test_pitch_0_7.sto")
modeling.STOFileAdapter.write(states_table, output_file)

print "Done! Saved to: " + output_file

