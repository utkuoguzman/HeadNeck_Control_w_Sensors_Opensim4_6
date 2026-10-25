import opensim as osim
import numpy as np
import os

osim.Logger.setLevelString("Warn")
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'proprioception_plugin/build/Release/osimMillard12EqWithAff.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'FixationController_plugin/build/Release/osimFixation.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'vestibular_plugin/build/Release/osimVestibular.dll'))

model = osim.Model("osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim")
model.setUseVisualizer(False)

ctrl = None
fixation = None
for i in range(model.getControllerSet().getSize()):
    c = model.getControllerSet().get(i)
    if c.getConcreteClassName() == "HeadNeckNeuralController":
        ctrl = c
    elif c.getConcreteClassName() == "FixationController":
        fixation = c

if fixation is not None:
    try:
        osim.PropertyHelper.setValueDouble(0.0, fixation.updPropertyByName("Kp_fixation"))
    except:
        pass

def set_prop(name, value):
    osim.PropertyHelper.setValueDouble(value, ctrl.updPropertyByName(name))

# RESTORE CMA-ES GAINS
set_prop("Kp_task", 65.6)
set_prop("Ki_task", 36.6)
set_prop("Kd_task", 14.1)
set_prop("G_ton", 12.15)
set_prop("G_sc", 1.48)
set_prop("G_phas", 4.89)

state = model.initSystem()
model.equilibrateMuscles(state)

manager = osim.Manager(model)
manager.setIntegratorMethod(5)
manager.setIntegratorAccuracy(1e-3)
manager.initialize(state)

def get_targets(t):
    p_targets, r_targets = (0.0,0.0,0.0), (0.0,0.0,0.0)
    
    # Step function testing for Yaw
    if t < 2.0:
        y_targets = (0.0, 0.0, 0.0)
        r_targets = (0.0, 0.0, 0.0)
    elif t < 5.0:
        y_targets = (0.0, 0.0, 0.0)
        r_targets = (np.radians(35.0), 0.0, 0.0)
    else:
        y_targets = (0.0, 0.0, 0.0)
        r_targets = (0.0, 0.0, 0.0)
        
    return p_targets, r_targets, y_targets

dt = 0.02
total_time = 8.0

print("Starting Roll Step Function Simulation...")
log_file = open("logs/roll_step_log.txt", "w")
log_file.write("t,p_des,r_des,y_des,p_act,r_act,y_act,p_vel,r_vel,y_vel\n")

cset = model.getCoordinateSet()

for step in range(1, int(total_time / dt) + 2):
    t = step * dt
    
    (dp, dp_v, dp_a), (dr, dr_v, dr_a), (dy, dy_v, dy_a) = get_targets(t)
        
    set_prop("desired_pitch", dp)
    set_prop("desired_roll", dr)
    set_prop("desired_yaw", dy)
    set_prop("desired_pitch_v", dp_v)
    set_prop("desired_roll_v", dr_v)
    set_prop("desired_yaw_v", dy_v)

    manager.integrate(t)
    state = manager.getState()
    
    act_p = cset.get("pitch1").getValue(state) + cset.get("pitch2").getValue(state)
    act_r = cset.get("roll1").getValue(state) + cset.get("roll2").getValue(state)
    act_y = cset.get("yaw1").getValue(state) + cset.get("yaw2").getValue(state)
    
    act_p_v = cset.get("pitch1").getSpeedValue(state) + cset.get("pitch2").getSpeedValue(state)
    act_r_v = cset.get("roll1").getSpeedValue(state) + cset.get("roll2").getSpeedValue(state)
    act_y_v = cset.get("yaw1").getSpeedValue(state) + cset.get("yaw2").getSpeedValue(state)
            
    if step % 5 == 0:
        print(f"t={t:.2f}s | Des(R): {np.degrees(dr):.1f} | Act(P,R,Y): {np.degrees(act_p):.1f}, {np.degrees(act_r):.1f}, {np.degrees(act_y):.1f} | Vel(P,R,Y): {np.degrees(act_p_v):.0f}, {np.degrees(act_r_v):.0f}, {np.degrees(act_y_v):.0f} deg/s")
        
    log_file.write(f"{t},{dp},{dr},{dy},{act_p},{act_r},{act_y},{act_p_v},{act_r_v},{act_y_v}\n")

log_file.close()
print("Done! Saved log to logs/roll_step_log.txt")

try:
    import matplotlib.pyplot as plt
    data = np.loadtxt("logs/roll_step_log.txt", delimiter=",", skiprows=1)
    t = data[:,0]
    dp, dr, dy = data[:,1], data[:,2], data[:,3]
    ap, ar, ay = data[:,4], data[:,5], data[:,6]
    pv, rv, yv = data[:,7], data[:,8], data[:,9]
    
    plt.figure(figsize=(10, 8))
    plt.subplot(2,1,1)
    plt.plot(t, dp*180/np.pi, 'r--', label='Des Pitch')
    plt.plot(t, ap*180/np.pi, 'r-', label='Act Pitch')
    plt.plot(t, dr*180/np.pi, 'g--', label='Des Roll')
    plt.plot(t, ar*180/np.pi, 'g-', label='Act Roll')
    plt.plot(t, dy*180/np.pi, 'b--', label='Des Yaw')
    plt.plot(t, ay*180/np.pi, 'b-', label='Act Yaw')
    plt.legend(); plt.ylabel('Angle (deg)'); plt.grid(True)
    
    plt.subplot(2,1,2)
    plt.plot(t, np.abs(pv)*180/np.pi, 'r-', label='Pitch Vel')
    plt.plot(t, np.abs(rv)*180/np.pi, 'g-', label='Roll Vel')
    plt.plot(t, np.abs(yv)*180/np.pi, 'b-', label='Yaw Vel')
    plt.legend(); plt.ylabel('Absolute Velocity (deg/s)'); plt.xlabel('Time (s)'); plt.grid(True)
    
    plt.savefig('logs/roll_step_plot.png')
except Exception as e:
    print("Could not plot:", e)


