import opensim as osim
import numpy as np
import os

# Suppress OpenSim integration spam!
osim.Logger.setLevelString("Warn")

# Load plugins
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'proprioception_plugin/build/Release/osimMillard12EqWithAff.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'FixationController_plugin/build/Release/osimFixation.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'vestibular_plugin/build/Release/osimVestibular.dll'))

model = osim.Model("osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim")
model.setUseVisualizer(False)

# Get the Neural Controller and Fixation Controller dynamically
ctrl = None
fixation = None
for i in range(model.getControllerSet().getSize()):
    c = model.getControllerSet().get(i)
    if c.getConcreteClassName() == "HeadNeckNeuralController":
        ctrl = c
    elif c.getConcreteClassName() == "FixationController":
        fixation = c

if ctrl is None:
    print("ERROR: HeadNeckNeuralController not found!")

def set_prop(name, value):
    try:
        osim.PropertyHelper.setValueDouble(value, ctrl.updPropertyByName(name))
    except Exception as e:
        print(f"Error setting {name}: {e}")

# Use best parameters (feel free to update if CMA-ES finds better)
# Eval SUCCESS: Kp=65.6, Ki=36.6, Kd=14.1, Gton=12.15, Gsc=1.48, Gphas=4.89, Kp_prop=0.052, K_gamma_dyn=36.0, K_gamma_stat=97.2
set_prop("Kp_task", 65.6)
set_prop("Ki_task", 36.6)
set_prop("Kd_task", 14.1)
set_prop("G_ton", 12.15)
set_prop("G_sc", 1.48)
set_prop("G_phas", 4.89)
set_prop("Kp_proprioception", 0.052)
set_prop("K_gamma_dyn", 36.0)
set_prop("K_gamma_stat", 97.2)
set_prop("K_recip_inhib", 0.0)

# Disable Fixation so head is free
if fixation is not None:
    try:
        osim.PropertyHelper.setValueDouble(0.0, fixation.updPropertyByName("Kp_fixation"))
    except:
        pass

state = model.initSystem()
model.equilibrateMuscles(state)

manager = osim.Manager(model)
manager.setIntegratorMethod(5) # RungeKuttaMerson
manager.setIntegratorAccuracy(1e-3)
manager.initialize(state)

def min_jerk(t, t_start, t_duration, start_val, end_val):
    if t <= t_start:
        return start_val, 0.0, 0.0
    if t >= t_start + t_duration:
        return end_val, 0.0, 0.0
    
    tau = (t - t_start) / t_duration
    pos = start_val + (end_val - start_val) * (10*tau**3 - 15*tau**4 + 6*tau**5)
    vel = (end_val - start_val) / t_duration * (30*tau**2 - 60*tau**3 + 30*tau**4)
    acc = (end_val - start_val) / (t_duration**2) * (60*tau - 180*tau**2 + 120*tau**3)
    return pos, vel, acc

def get_targets(t):
    # Returns (pos, vel, acc) for Pitch, Roll, Yaw
    p_targets = (0.0, 0.0, 0.0)
    r_targets = (0.0, 0.0, 0.0)
    y_targets = (0.0, 0.0, 0.0)
    
    def move_profile(t_local, max_deg):
        max_rad = np.radians(max_deg)
        if t_local <= 0.25:
            return (0.0, 0.0, 0.0)
        elif t_local <= 1.25:
            return min_jerk(t_local, 0.25, 1.0, 0.0, max_rad)
        elif t_local <= 1.50:
            return (max_rad, 0.0, 0.0)
        elif t_local <= 3.50:
            return min_jerk(t_local, 1.50, 2.0, max_rad, -max_rad)
        elif t_local <= 3.75:
            return (-max_rad, 0.0, 0.0)
        elif t_local <= 4.75:
            return min_jerk(t_local, 3.75, 1.0, -max_rad, 0.0)
        else:
            return (0.0, 0.0, 0.0)

    if t <= 4.75:
        p_targets = move_profile(t, -20.0) # Pitch: +/- 45 deg (stay away from 60 deg limit)
    elif t <= 9.50:
        r_targets = move_profile(t - 4.75, 20.0) # Roll: +/- 30 deg (stay away from 39.9 deg limit)
    elif t <= 14.25:
        y_targets = move_profile(t - 9.50, 20.0) # Yaw: +/- 50 deg
        
    return p_targets, r_targets, y_targets

print("Starting 14.25s Volitional Tracking Simulation...")
log_file = open("logs/volitional_tracking_log.txt", "w")
log_file.write("t,p_des,r_des,y_des,p_act,r_act,y_act\n")

cset = model.getCoordinateSet()
dt = 0.02
for step in range(1, int(14.25 / dt) + 2):
    t = step * dt
    
    (dp, dp_v, dp_a), (dr, dr_v, dr_a), (dy, dy_v, dy_a) = get_targets(t)
    
    # Send Position, Velocity, Acceleration to the controller
    set_prop("desired_pitch", dp)
    set_prop("desired_roll", dr)
    set_prop("desired_yaw", dy)
    set_prop("desired_pitch_v", dp_v)
    set_prop("desired_roll_v", dr_v)
    set_prop("desired_yaw_v", dy_v)
    # Note: ensure C++ properties exist for _a, if not these will safely except via pass
    set_prop("desired_pitch_a", dp_a)
    set_prop("desired_roll_a", dr_a)
    set_prop("desired_yaw_a", dy_a)

    manager.integrate(t)
    state = manager.getState()
    
    # Measure actual kinematics
    act_p = cset.get("pitch1").getValue(state) + cset.get("pitch2").getValue(state)
    act_r = cset.get("roll1").getValue(state) + cset.get("roll2").getValue(state)
    act_y = cset.get("yaw1").getValue(state) + cset.get("yaw2").getValue(state)
    
    max_exc = 0.0
    muscles = model.getMuscles()
    for i in range(muscles.getSize()):
        exc = muscles.get(i).getExcitation(state)
        if exc > max_exc:
            max_exc = exc
            
    if step % 10 == 0:
        print(f"t={t:.2f}s | Des(P,R,Y): {np.degrees(dp):.1f}, {np.degrees(dr):.1f}, {np.degrees(dy):.1f} | Act(P,R,Y): {np.degrees(act_p):.1f}, {np.degrees(act_r):.1f}, {np.degrees(act_y):.1f} | MaxExc: {max_exc:.3f}")
        
    log_file.write(f"{t},{dp},{dr},{dy},{act_p},{act_r},{act_y}\n")

log_file.close()
print("Done! Saved log to logs/volitional_tracking_log.txt")

try:
    import matplotlib.pyplot as plt
    import numpy as np
    
    data = np.loadtxt("logs/volitional_tracking_log.txt", delimiter=",", skiprows=1)
    t = data[:,0]
    dp, dr, dy = data[:,1], data[:,2], data[:,3]
    ap, ar, ay = data[:,4], data[:,5], data[:,6]
    
    plt.figure(figsize=(10, 6))
    plt.plot(t, dp*180/np.pi, 'r--', label='Des Pitch')
    plt.plot(t, ap*180/np.pi, 'r-', label='Act Pitch')
    plt.plot(t, dr*180/np.pi, 'g--', label='Des Roll')
    plt.plot(t, ar*180/np.pi, 'g-', label='Act Roll')
    plt.plot(t, dy*180/np.pi, 'b--', label='Des Yaw')
    plt.plot(t, ay*180/np.pi, 'b-', label='Act Yaw')
    
    plt.legend()
    plt.title('Volitional Tracking (Min Jerk Trajectories)')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (deg)')
    plt.grid(True)
    plt.savefig('logs/volitional_tracking_plot.png')
    plt.show()
except Exception as e:
    print(f"Could not plot: {e}")
