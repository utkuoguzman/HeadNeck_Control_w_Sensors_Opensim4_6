import opensim as osim
import numpy as np
import matplotlib.pyplot as plt
import os

# Load plugins
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'proprioception_plugin/build/Release/osimMillard12EqWithAff.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'FixationController_plugin/build/Release/osimFixation.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'vestibular_plugin/build/Release/osimVestibular.dll'))

def run_rotation_test(axis="pitch"):
    model = osim.Model("osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim")
    model.setUseVisualizer(False)

    # Remove muscles
    model.updForceSet().clearAndDestroy()

    # Remove fixation controller
    cset = model.updControllerSet()
    for i in range(cset.getSize()):
        if cset.get(i).getName() == "Binocular_Fixation_Controller":
            cset.remove(i)
            break
            
    if axis == "pitch":
        coords = [model.updCoordinateSet().get("pitch1"), model.updCoordinateSet().get("pitch2")]
    elif axis == "roll":
        coords = [model.updCoordinateSet().get("roll1"), model.updCoordinateSet().get("roll2")]
    elif axis == "yaw":
        coords = [model.updCoordinateSet().get("yaw1"), model.updCoordinateSet().get("yaw2")]

    dt = 0.005
    t_end = 1.0
    times = np.arange(0, t_end, dt)
    
    velocity_deg = 50.0
    velocity_rad = velocity_deg * (np.pi / 180.0)
    
    pos_signal = times * velocity_rad
    
    splines = [osim.SimmSpline(), osim.SimmSpline()]
    for i in range(len(times)):
        splines[0].addPoint(times[i], pos_signal[i] / 2.0)
        splines[1].addPoint(times[i], pos_signal[i] / 2.0)
        
    coords[0].setPrescribedFunction(splines[0])
    coords[1].setPrescribedFunction(splines[1])
    coords[0].setDefaultIsPrescribed(True)
    coords[1].setDefaultIsPrescribed(True)
    
    state = model.initSystem()
    
    # We will record the 6 continuous state variables manually!
    # The state variables are canal_{i}_filter_0_state_0 and canal_{i}_filter_0_state_1
    # Actually, we know the input is u[i]. The filter is a linear system.
    # To see the steady state responses or step responses, we just integrate.
    
    manager = osim.Manager(model)
    manager.initialize(state)
    
    t_hist = []
    responses = {0:[], 1:[], 2:[], 3:[], 4:[], 5:[]}
    
    # Names for canals based on C++ array order
    canal_names = {
        0: "Right Anterior", 1: "Right Posterior", 2: "Right Horizontal",
        3: "Left Anterior", 4: "Left Posterior", 5: "Left Horizontal"
    }

    print(f"Running pure {axis} rotation...")
    manager.integrate(1.0)
    
    # Wait, if we use TableReporter we can get all states!
    # But since it's only 1 second, we can just do a loop to avoid reporter string parsing
    manager.initialize(state)
    for t in times[1:]:
        manager.integrate(t)
        state = manager.getState()
        model.realizeDynamics(state)
        
        t_hist.append(t)
        
        # Calculate canal output from state variables
        controller = model.updControllerSet().get(0)
        for i in range(6):
            x2 = controller.getStateVariableValue(state, f"canal_{i}_filter_0_state_1")
            responses[i].append(x2)
            
    return t_hist, responses, canal_names

# Run all 3
results = {}
for ax in ["pitch", "roll", "yaw"]:
    t_hist, resp, names = run_rotation_test(ax)
    results[ax] = (t_hist, resp, names)

# Plotting
fig, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True)

for i, ax_name in enumerate(["pitch", "roll", "yaw"]):
    t_hist, resp, names = results[ax_name]
    ax = axs[i]
    for c_idx in range(6):
        ax.plot(t_hist, resp[c_idx], label=names[c_idx])
    
    ax.set_title(f"Pure {ax_name.capitalize()} Rotation (50 deg/s step)")
    ax.set_ylabel("Filter State x2")
    if i == 0:
        ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')

axs[-1].set_xlabel("Time (s)")
plt.tight_layout()
os.makedirs("logs", exist_ok=True)
plt.savefig("logs/test_vestibular_orientations.png")
print("Saved orientation plots to logs/test_vestibular_orientations.png")
