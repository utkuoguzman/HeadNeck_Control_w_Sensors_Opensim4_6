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
for i in range(model.getControllerSet().getSize()):
    c = model.getControllerSet().get(i)
    if c.getConcreteClassName() == "HeadNeckNeuralController":
        ctrl = c

def set_prop(name, value):
    osim.PropertyHelper.setValueDouble(value, ctrl.updPropertyByName(name))

# ---------------------------------------------------------
# Set biologically realistic relaxed gains (Gen 8 best)
# ---------------------------------------------------------
set_prop("Kp_task", 29.1)
set_prop("Ki_task", 1.5)
set_prop("Kd_task", 24.7)

set_prop("G_ton", 8.76)
set_prop("G_sc", 4.33)
set_prop("G_phas", 6.06)

set_prop("Kp_proprioception", 0.441)
set_prop("K_gamma_dyn", 52.36)
set_prop("K_gamma_stat", 92.37)
# ---------------------------------------------------------

state = model.initSystem()
model.equilibrateMuscles(state)

manager = osim.Manager(model)
manager.setIntegratorMethod(5)
manager.setIntegratorAccuracy(1e-3)
manager.initialize(state)

# Perfect SE(3) Observer Initialization
skull = model.getBodySet().get("skull")
R_head_to_world = skull.getTransformInGround(state).R()
q = osim.Rotation(R_head_to_world).convertRotationToQuaternion()
model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q0", q.get(0))
model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q1", q.get(1))
model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q2", q.get(2))
model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q3", q.get(3))


def min_jerk(t, t0, tf, pos0, posf):
    if t <= t0:
        return pos0, 0.0, 0.0
    elif t >= tf:
        return posf, 0.0, 0.0
    else:
        D = tf - t0
        tau = (t - t0) / D
        delta = posf - pos0
        
        pos = pos0 + delta * (10*tau**3 - 15*tau**4 + 6*tau**5)
        vel = (delta / D) * (30*tau**2 - 60*tau**3 + 30*tau**4)
        acc = (delta / (D**2)) * (60*tau - 180*tau**2 + 120*tau**3)
        return pos, vel, acc

def get_targets(t):
    y_targets = (0.0, 0.0, 0.0)
    r_targets = (0.0, 0.0, 0.0)
    
    # 45 degree Pitch forward over 0.6 seconds (human-like saccade speed)
    if t < 5.0:
        p_targets = min_jerk(t, 2.0, 2.6, 0.0, np.radians(45.0))
    else:
        # Return to center over 0.6 seconds
        p_targets = min_jerk(t, 5.0, 5.6, np.radians(45.0), 0.0)
        
    return p_targets, r_targets, y_targets

dt = 0.01  # Tighter timestep for smooth tracking
total_time = 8.0

print("Starting Minimum-Jerk Pitch Simulation...")
log_file = open("logs/pitch_minjerk_log.txt", "w")
log_file.write("t,p_des,r_des,y_des,p_act,r_act,y_act,p_vel_des,p_vel_act,p_acc_des\n")

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
    
    set_prop("desired_pitch_a", dp_a)
    set_prop("desired_roll_a", dr_a)
    set_prop("desired_yaw_a", dy_a)

    manager.integrate(t)
    state = manager.getState()
    
    act_p = cset.get("pitch1").getValue(state) + cset.get("pitch2").getValue(state)
    act_r = cset.get("roll1").getValue(state) + cset.get("roll2").getValue(state)
    act_y = cset.get("yaw1").getValue(state) + cset.get("yaw2").getValue(state)
    
    act_p_v = cset.get("pitch1").getSpeedValue(state) + cset.get("pitch2").getSpeedValue(state)
            
    if step % 20 == 0:
        print(f"t={t:.2f}s | Des: {np.degrees(dp):.1f} | Act: {np.degrees(act_p):.1f} | Des_V: {np.degrees(dp_v):.0f}/s | Act_V: {np.degrees(act_p_v):.0f}/s | Des_A: {np.degrees(dp_a):.0f}/s2")
        
    log_file.write(f"{t},{dp},{dr},{dy},{act_p},{act_r},{act_y},{dp_v},{act_p_v},{dp_a}\n")

log_file.close()
print("Done! Saved log to logs/pitch_minjerk_log.txt")

try:
    import matplotlib.pyplot as plt
    data = np.loadtxt("logs/pitch_minjerk_log.txt", delimiter=",", skiprows=1)
    t = data[:,0]
    dp = data[:,1]
    ap = data[:,4]
    dp_v = data[:,7]
    ap_v = data[:,8]
    dp_a = data[:,9]
    
    plt.figure(figsize=(10, 10))
    
    plt.subplot(3,1,1)
    plt.plot(t, dp*180/np.pi, 'r--', label='Des Pitch')
    plt.plot(t, ap*180/np.pi, 'r-', label='Act Pitch')
    plt.legend(); plt.ylabel('Angle (deg)'); plt.grid(True)
    plt.title("Minimum-Jerk Pitch Tracking")
    
    plt.subplot(3,1,2)
    plt.plot(t, dp_v*180/np.pi, 'g--', label='Des Vel')
    plt.plot(t, ap_v*180/np.pi, 'g-', label='Act Vel')
    plt.legend(); plt.ylabel('Velocity (deg/s)'); plt.grid(True)
    
    plt.subplot(3,1,3)
    plt.plot(t, dp_a*180/np.pi, 'b--', label='Des Accel (Feedforward)')
    plt.legend(); plt.ylabel('Acceleration (deg/s^2)'); plt.xlabel('Time (s)'); plt.grid(True)
    
    plt.tight_layout()
    plt.savefig('logs/pitch_minjerk_plot.png')
except Exception as e:
    print("Could not plot:", e)

