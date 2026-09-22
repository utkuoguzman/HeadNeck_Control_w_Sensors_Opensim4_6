import opensim as osim
import numpy as np
import math
import os
import matplotlib.pyplot as plt

# Load all custom C++ DLLs before loading the model so it can deserialize all custom components!
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'proprioception_plugin/build/Release/osimMillard12EqWithAff.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'vestibular_plugin/build/Release/osimVestibular.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'FixationController_plugin/build/Release/osimFixation.dll'))
model = osim.Model("osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim")

# 1. SETUP CONTROLLERS
print("Configuring Controllers...")
controller = None
for i in range(model.getControllerSet().getSize()):
    ctrl = model.getControllerSet().get(i)
    if ctrl.getConcreteClassName() == "FixationController":
        ctrl.set_enabled(False) # Disable eyes
    elif ctrl.getConcreteClassName() == "HeadNeckNeuralController":
        controller = ctrl
        
if controller is None:
    raise Exception("HeadNeckNeuralController not found in model!")

# Set the Happee Table 1 B1 (Eyes Closed) parameters via generic Property API
osim.PropertyHelper.setValueDouble(2.0, controller.updPropertyByName("G_ton"))
osim.PropertyHelper.setValueDouble(1.73, controller.updPropertyByName("G_sc"))
osim.PropertyHelper.setValueDouble(0.45, controller.updPropertyByName("k_p"))
osim.PropertyHelper.setValueDouble(0.13, controller.updPropertyByName("k_v"))

# Add the Voluntary Postural PID Drive to represent Na_post (baseline gravity compensation)
osim.PropertyHelper.setValueDouble(49.5, controller.updPropertyByName("Kp_task"))
osim.PropertyHelper.setValueDouble(41.2, controller.updPropertyByName("Ki_task"))
osim.PropertyHelper.setValueDouble(5.3, controller.updPropertyByName("Kd_task"))

# Instead of locking (which can cause Simbody redundant constraint errors),
# we explicitly prescribe a constant 0.0 motion to all other base coordinates.
# We MUST explicitly unlock them first because they are locked in the .osim file!
zero_func = osim.Constant(0.0)
for coord_name in ["gndpitch", "gndroll", "gndyaw", "spine_ty", "spine_tz"]:
    c = model.updCoordinateSet().get(coord_name)
    c.set_locked(False)
    c.setDefaultIsPrescribed(True)
    c.setPrescribedFunction(zero_func)

# 2. CREATE SEAT PERTURBATION (Happee simulated seat multisine)
# Happee states B1 has RMS velocity = 0.08 m/s, and RMS T1 acceleration = 0.42 m/s^2
# For a pure sine wave, this exactly equates to an amplitude of 2.15 cm at 0.835 Hz.
amplitude = 0.0215 # 2.15 cm
frequency = 0.835 # 0.835 Hz (Middle of B1)

sine_function = osim.Sine(amplitude, 2 * math.pi * frequency, 0)
coord_tx = model.updCoordinateSet().get("spine_tx")
# OpenSim 4.0+ sets prescribed motion directly on the coordinate!
coord_tx.set_locked(False) # MUST UNLOCK FIRST
coord_tx.setDefaultIsPrescribed(True)
coord_tx.setPrescribedFunction(sine_function)

# 3. INITIALIZE STATE
state = model.initSystem()

# Ensure the head starts at exactly 0.0 posture
model.updCoordinateSet().get("pitch1").setValue(state, 0.0)
model.updCoordinateSet().get("pitch2").setValue(state, 0.0)
model.updCoordinateSet().get("roll1").setValue(state, 0.0)
model.updCoordinateSet().get("roll2").setValue(state, 0.0)
model.updCoordinateSet().get("yaw1").setValue(state, 0.0)
model.updCoordinateSet().get("yaw2").setValue(state, 0.0)

# Set the initial velocity of the sled to match the prescribed sine wave derivative at t=0
# v(0) = A * omega * cos(0)
initial_velocity = amplitude * 2.0 * math.pi * frequency
model.updCoordinateSet().get("spine_tx").setSpeedValue(state, initial_velocity)

model.equilibrateMuscles(state)

# 4. SET UP INTEGRATOR (SemiExplicitEuler2 at 1e-4 accuracy is required for the muscle solver)
manager = osim.Manager(model)
manager.setIntegratorMethod(5) # SemiExplicitEuler2
manager.setIntegratorAccuracy(1e-4)
manager.setIntegratorMinimumStepSize(1e-8)
manager.setIntegratorMaximumStepSize(0.01)

# Arrays for plotting
time_arr = []
sled_arr = []
head_arr = []

# 5. RUN SIMULATION
print("Running Happee Seat Perturbation Simulation...")
state.setTime(0.0)
manager.initialize(state)

final_time = 3.0
log_interval = 0.05

if not os.path.exists("logs"):
    os.makedirs("logs")

with open("logs/log_happee_perturbation.txt", "w") as f:
    f.write("Time\tSpine_Tx\tHead_Pitch\tHead_Pitch_Vel\n")
    
    while state.getTime() < final_time:
        manager.integrate(state.getTime() + log_interval)
        state = manager.getState()
        
        t = state.getTime()
        tx = coord_tx.getValue(state)
        
        pitch1 = model.getCoordinateSet().get("pitch1").getValue(state)
        pitch2 = model.getCoordinateSet().get("pitch2").getValue(state)
        p_vel1 = model.getCoordinateSet().get("pitch1").getSpeedValue(state)
        p_vel2 = model.getCoordinateSet().get("pitch2").getSpeedValue(state)
        
        head_pitch = pitch1 + pitch2
        head_pitch_vel = p_vel1 + p_vel2
        
        log_str = "{:.3f}\t{:.4f}\t{:.4f}\t{:.4f}\n".format(t, tx, head_pitch, head_pitch_vel)
        f.write(log_str)
        print("T: {:.2f}s | Seat Tx: {:.3f}m | Head Pitch: {:.2f} deg".format(t, tx, head_pitch * 180.0/math.pi))
        
        time_arr.append(t)
        sled_arr.append(tx)
        head_arr.append(head_pitch)

print("Simulation finished! Output saved to logs/log_happee_perturbation.txt")

# Plotting the results!
print("Generating plot...")
fig, ax1 = plt.subplots(figsize=(10, 5))

color = 'tab:blue'
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Sled Displacement (m)', color=color)
ax1.plot(time_arr, sled_arr, color=color, label='Seat (T1) Displacement')
ax1.tick_params(axis='y', labelcolor=color)

ax2 = ax1.twinx()  
color = 'tab:red'
ax2.set_ylabel('Head Pitch (rad)', color=color)  
ax2.plot(time_arr, head_arr, color=color, label='Head Global Pitch')
ax2.tick_params(axis='y', labelcolor=color)

fig.suptitle('Happee 2017 Validation: Head Pitch vs Sled Perturbation')
fig.tight_layout()  
plt.grid(True)
plt.savefig("logs/happee_plot.png")
plt.show()

