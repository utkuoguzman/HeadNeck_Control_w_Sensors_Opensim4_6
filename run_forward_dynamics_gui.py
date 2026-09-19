import org.opensim.modeling as modeling
import os

# Get the current model from the OpenSim GUI
model = getCurrentModel()

if model == None:
    print "No model loaded! Please load the model in the GUI first."
else:
    print "Running Forward Dynamics for model: " + model.getName()

    # Initialize the system to get a fresh state
    state = model.initSystem()
    
    # Set up the Manager
    manager = modeling.Manager(model)
    
    # Use the user's preferred integrator settings for accurate muscle/neural physics
    manager.setIntegratorAccuracy(0.00008)
    manager.setIntegratorMinimumStepSize(0.00008)
    manager.setIntegratorMaximumStepSize(1.0)
    manager.setIntegratorInternalStepLimit(50000)
    
    manager.initialize(state)
    
    print "Integrating for 4.0 seconds in steps of 0.2s..."
    
    # Step-by-step integration to print progress
    current_t = 0.0
    while current_t < 4.0:
        current_t += 0.2
        # Use round to avoid floating point errors like 0.600000001
        current_t = round(current_t, 2) 
        manager.integrate(current_t)
        print "Successfully integrated to " + str(current_t) + " seconds"
    
    print "Simulation finished!"
    
    # Extract the results
    states_table = manager.getStatesTable()
    
    # Save to a .sto file
    output_dir = r"D:\Akademik\PhD_Thesis\OpenSim\Proprioception_Plugin_Opensim4_6_Explicit"
    output_file = os.path.join(output_dir, "forward_simulation_results.sto")
    modeling.STOFileAdapter.write(states_table, output_file)
    
    print "Success! Saved states to: " + output_file
    
    # Automatically load the resulting motion back into the GUI!
    try:
        loadMotion(output_file)
        print "Motion automatically loaded into the GUI! Hit play in the visualizer."
    except Exception, e:
        print "Could not automatically load the motion. You can load it manually via File -> Load Motion."
        print str(e)

