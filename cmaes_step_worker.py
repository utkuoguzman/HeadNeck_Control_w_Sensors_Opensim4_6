import opensim as osim
import numpy as np
import os
import sys

sys.stdout.reconfigure(line_buffering=True)

os.environ["SIMB_NUM_THREADS"] = "1"
os.environ["OMP_NUM_THREADS"] = "3"

osim.Logger.setLevelString("Warn")
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'proprioception_plugin/build/Release/osimMillard12EqWithAff.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'FixationController_plugin/build/Release/osimFixation.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'vestibular_plugin/build/Release/osimVestibular.dll'))

def min_jerk(t, t0, tf, pos0, posf):
    if t <= t0:
        return pos0, 0.0, 0.0
    if t >= tf:
        return posf, 0.0, 0.0
    
    tau = (t - t0) / (tf - t0)
    
    pos = pos0 + (posf - pos0) * (10*tau**3 - 15*tau**4 + 6*tau**5)
    
    dtau_dt = 1.0 / (tf - t0)
    vel = (posf - pos0) * (30*tau**2 - 60*tau**3 + 30*tau**4) * dtau_dt
    
    d2tau_dt2 = 1.0 / ((tf - t0)**2)
    acc = (posf - pos0) * (60*tau - 180*tau**2 + 120*tau**3) * d2tau_dt2
    
    return pos, vel, acc

def evaluate_params(Kp, Ki, Kd, G_ton, G_sc, G_phas, kp_prop, k_gamma_dyn, k_gamma_stat, worker_id):
    model = osim.Model("osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim")
    model.setUseVisualizer(False)

    ctrl = None
    for i in range(model.getControllerSet().getSize()):
        c = model.getControllerSet().get(i)
        if c.getConcreteClassName() == "HeadNeckNeuralController":
            ctrl = c
            break
    if ctrl is None:
        return 100000.0

    osim.PropertyHelper.setValueDouble(Kp, ctrl.updPropertyByName("Kp_task"))
    osim.PropertyHelper.setValueDouble(Ki, ctrl.updPropertyByName("Ki_task"))
    osim.PropertyHelper.setValueDouble(Kd, ctrl.updPropertyByName("Kd_task"))

    osim.PropertyHelper.setValueDouble(G_ton, ctrl.updPropertyByName("G_ton"))
    osim.PropertyHelper.setValueDouble(G_sc, ctrl.updPropertyByName("G_sc"))
    osim.PropertyHelper.setValueDouble(G_phas, ctrl.updPropertyByName("G_phas"))

    osim.PropertyHelper.setValueDouble(kp_prop, ctrl.updPropertyByName("Kp_proprioception"))
    osim.PropertyHelper.setValueDouble(k_gamma_dyn, ctrl.updPropertyByName("K_gamma_dyn"))
    osim.PropertyHelper.setValueDouble(k_gamma_stat, ctrl.updPropertyByName("K_gamma_stat"))

    osim.PropertyHelper.setValueDouble(0.45, ctrl.updPropertyByName("k_p"))
    osim.PropertyHelper.setValueDouble(0.13, ctrl.updPropertyByName("k_v"))

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

    cset = model.getCoordinateSet()
    dt = 0.05
    total_error = 0.0

    duration = 0.6 # 600ms duration for human saccade head movements
    
    for step in range(1, int(6.0 / dt) + 1):
        t = step * dt
        
        # Determine current target states based on minimum jerk trajectories
        dp, dp_v, dp_a = 0.0, 0.0, 0.0
        dr, dr_v, dr_a = 0.0, 0.0, 0.0
        dy, dy_v, dy_a = 0.0, 0.0, 0.0
        
        # Pitch: 0 to 45 deg between t=1.0 and t=1.6, then hold
        if t >= 1.0:
            dp, dp_v, dp_a = min_jerk(t, 1.0, 1.0 + duration, 0.0, np.radians(45.0))
        
        # Roll: 0 to 35 deg between t=3.0 and t=3.6, then hold
        if t >= 3.0:
            dr, dr_v, dr_a = min_jerk(t, 3.0, 3.0 + duration, 0.0, np.radians(35.0))
            
        # Yaw: 0 to 50 deg between t=5.0 and t=5.6, then hold
        if t >= 5.0:
            dy, dy_v, dy_a = min_jerk(t, 5.0, 5.0 + duration, 0.0, np.radians(50.0))

        osim.PropertyHelper.setValueDouble(dp, ctrl.updPropertyByName("desired_pitch"))
        osim.PropertyHelper.setValueDouble(dr, ctrl.updPropertyByName("desired_roll"))
        osim.PropertyHelper.setValueDouble(dy, ctrl.updPropertyByName("desired_yaw"))
        
        osim.PropertyHelper.setValueDouble(dp_v, ctrl.updPropertyByName("desired_pitch_v"))
        osim.PropertyHelper.setValueDouble(dr_v, ctrl.updPropertyByName("desired_roll_v"))
        osim.PropertyHelper.setValueDouble(dy_v, ctrl.updPropertyByName("desired_yaw_v"))
        
        osim.PropertyHelper.setValueDouble(dp_a, ctrl.updPropertyByName("desired_pitch_a"))
        osim.PropertyHelper.setValueDouble(dr_a, ctrl.updPropertyByName("desired_roll_a"))
        osim.PropertyHelper.setValueDouble(dy_a, ctrl.updPropertyByName("desired_yaw_a"))

        try:
            state = manager.integrate(t)
        except Exception:
            return 100000.0

        p = cset.get("pitch1").getValue(state) + cset.get("pitch2").getValue(state)
        r = cset.get("roll1").getValue(state) + cset.get("roll2").getValue(state)
        y = cset.get("yaw1").getValue(state) + cset.get("yaw2").getValue(state)

        # Track tracking error + penalize extreme velocities/activations to prevent instability
        err = (p - dp)**2 + (r - dr)**2 + (y - dy)**2
        total_error += err * dt

        if step % int(0.2 / dt) == 0:
            print(f"[{worker_id}] t={t:>4.2f}s | Des_P: {np.degrees(dp):>5.1f} | Des_R: {np.degrees(dr):>5.1f} | Des_Y: {np.degrees(dy):>5.1f} | Act_P: {np.degrees(p):>6.1f} | Act_R: {np.degrees(r):>6.1f} | Act_Y: {np.degrees(y):>6.1f}")

    print(f"[{worker_id}] Kp={Kp:.1f}, Gton={G_ton:.3f} | Err={total_error:.4f}")
    return total_error

if __name__ == "__main__":
    if len(sys.argv) < 11:
        print("100000.0")
        sys.exit(1)

    Kp = float(sys.argv[1])
    Ki = float(sys.argv[2])
    Kd = float(sys.argv[3])
    G_ton = float(sys.argv[4])
    G_sc = float(sys.argv[5])
    G_phas = float(sys.argv[6])
    kp_prop = float(sys.argv[7])
    k_gamma_dyn = float(sys.argv[8])
    k_gamma_stat = float(sys.argv[9])
    worker_id = sys.argv[10]

    err = evaluate_params(Kp, Ki, Kd, G_ton, G_sc, G_phas, kp_prop, k_gamma_dyn, k_gamma_stat, worker_id)
    print(f"SCORE:{err}")
