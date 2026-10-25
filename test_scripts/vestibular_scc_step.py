import opensim as osim
import numpy as np
import matplotlib.pyplot as plt
import os
import sys

# Load plugins
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'proprioception_plugin/build/Release/osimMillard12EqWithAff.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'FixationController_plugin/build/Release/osimFixation.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'vestibular_plugin/build/Release/osimVestibular.dll'))

model = osim.Model("osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim")
model.setUseVisualizer(False)

# Get Controller and Isolate Semicircular Canals
ctrl = model.getControllerSet().get(0)

def set_prop(name, value):
    try:
        osim.PropertyHelper.setValueDouble(value, ctrl.getPropertyByName(name))
    except Exception:
        pass

# Set all gains to 0 to perfectly isolate the Semicircular Canals (SCC)
set_prop("Kp_task", 0.0)
set_prop("Ki_task", 0.0)
set_prop("Kd_task", 0.0)
set_prop("G_ton", 0.0)
set_prop("G_phas", 0.0)
set_prop("k_p", 0.0)
set_prop("k_v", 0.0)
set_prop("K_gamma_dyn", 0.0)
set_prop("K_gamma_stat", 0.0)
set_prop("Kp_proprioception", 0.0)

# TURN ON SCC
set_prop("G_sc", 10.0) 

# Prescribe a Velocity Step in Pitch (0 to 60 deg/sec suddenly)
coord_p1 = model.updCoordinateSet().get("pitch1")
coord_p2 = model.updCoordinateSet().get("pitch2")

times = osim.ArrayDouble()
values_p1 = osim.ArrayDouble()
values_p2 = osim.ArrayDouble()

velocity_deg_per_sec = 60.0
velocity_rad_per_sec = velocity_deg_per_sec * (np.pi / 180.0)

for i in range(1501):
    t = i / 100.0
    times.append(t)
    
    # 0 to 1s: Rest
    # 1s to 15s: Constant Velocity
    if t < 1.0:
        angle = 0.0
    else:
        angle = velocity_rad_per_sec * (t - 1.0)
        
    values_p1.append(angle / 2.0)
    values_p2.append(angle / 2.0)

spline_p1 = osim.SimmSpline()
spline_p2 = osim.SimmSpline()
for i in range(times.getSize()):
    spline_p1.addPoint(times.get(i), values_p1.get(i))
    spline_p2.addPoint(times.get(i), values_p2.get(i))

coord_p1.setPrescribedFunction(spline_p1)
coord_p2.setPrescribedFunction(spline_p2)
coord_p1.setDefaultIsPrescribed(True)
coord_p2.setDefaultIsPrescribed(True)

state = model.initSystem()
model.equilibrateMuscles(state)
manager = osim.Manager(model)
manager.initialize(state)

t_hist = []
vel_hist = []
exc_hist = []

# Monitor a Neck Extensor muscle (since pitch is positive forward, SCC will resist by firing extensors)
muscles = model.getMuscles()
splenius_name = muscles.get(0).getName()
splenius = muscles.get(0)
splenius = osim.Muscle.safeDownCast(splenius)

print("Running Vestibular SCC Velocity Step Test (15 seconds)...")
for i in range(1, 1501):
    t = i / 100.0
    manager.integrate(t)
    model.realizeDynamics(state)
    state = manager.getState()
    
    t_hist.append(t)
    vel_hist.append(coord_p1.getSpeedValue(state)*2.0 * (180.0/np.pi))
    exc_hist.append(splenius.getExcitation(state))

fig, ax1 = plt.subplots(figsize=(10,6))
ax2 = ax1.twinx()
ax1.plot(t_hist, vel_hist, 'k--', label="Head Pitch Velocity (deg/s)")
ax2.plot(t_hist, exc_hist, 'b-', label=f"SCC Excitation ({splenius_name})")

ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Head Velocity (deg/s)")
ax2.set_ylabel("Vestibular Excitation")
plt.title("Vestibular Semicircular Canal Dynamics: Velocity Step Test")
fig.legend(loc="upper right", bbox_to_anchor=(0.85, 0.85))

os.makedirs("logs", exist_ok=True)
plt.savefig(f"logs/vestibular_scc_step.png")
print("Saved plot to logs/vestibular_scc_step.png")
