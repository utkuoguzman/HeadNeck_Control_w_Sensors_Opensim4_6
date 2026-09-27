import opensim as osim
import numpy as np
import os
import sys

# Force line-buffered stdout so the master process sees progress in real time
sys.stdout.reconfigure(line_buffering=True)

osim.Logger.setLevelString("Warn")
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'proprioception_plugin/build/Release/osimMillard12EqWithAff.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'FixationController_plugin/build/Release/osimFixation.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'vestibular_plugin/build/Release/osimVestibular.dll'))

def evaluate_params(Kp, Ki, Kd, worker_id):
    model = osim.Model("osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim")
    model.setUseVisualizer(False)

    ctrl = None
    for i in range(model.getControllerSet().getSize()):
        c = model.getControllerSet().get(i)
        if c.getConcreteClassName() == "HeadNeckNeuralController":
            ctrl = c
            break
    if ctrl is None:
        raise RuntimeError("HeadNeckNeuralController not found in model")

    # Set gains
    osim.PropertyHelper.setValueDouble(Kp, ctrl.updPropertyByName("Kp_task"))
    osim.PropertyHelper.setValueDouble(Ki, ctrl.updPropertyByName("Ki_task"))
    osim.PropertyHelper.setValueDouble(Kd, ctrl.updPropertyByName("Kd_task"))
    osim.PropertyHelper.setValueDouble(12.15, ctrl.updPropertyByName("G_ton"))
    osim.PropertyHelper.setValueDouble(1.48, ctrl.updPropertyByName("G_sc"))
    osim.PropertyHelper.setValueDouble(4.89, ctrl.updPropertyByName("G_phas"))

    state = model.initSystem()
    model.equilibrateMuscles(state)

    manager = osim.Manager(model)
    manager.setIntegratorMethod(5)
    manager.setIntegratorAccuracy(1e-3)
    manager.initialize(state)

    cset = model.getCoordinateSet()
    dt = 0.05
    total_error = 0.0

    for step in range(1, int(6.0 / dt) + 1):
        t = step * dt
        dp, dr, dy = 0.0, 0.0, 0.0
        if t < 1.0:
            pass
        elif t < 2.0:
            dp = np.radians(45.0)   # Pitch step
        elif t < 3.0:
            pass
        elif t < 4.0:
            dr = np.radians(35.0)   # Roll step
        elif t < 5.0:
            pass
        elif t < 6.0:
            dy = np.radians(50.0)   # Yaw step

        # Set desired values
        osim.PropertyHelper.setValueDouble(dp, ctrl.updPropertyByName("desired_pitch"))
        osim.PropertyHelper.setValueDouble(dr, ctrl.updPropertyByName("desired_roll"))
        osim.PropertyHelper.setValueDouble(dy, ctrl.updPropertyByName("desired_yaw"))
        osim.PropertyHelper.setValueDouble(0.0, ctrl.updPropertyByName("desired_pitch_v"))
        osim.PropertyHelper.setValueDouble(0.0, ctrl.updPropertyByName("desired_roll_v"))
        osim.PropertyHelper.setValueDouble(0.0, ctrl.updPropertyByName("desired_yaw_v"))

        try:
            manager.integrate(t)
        except RuntimeError as e:
            print(f"[{worker_id}] CRASH at t={t:.2f}s: {e}")
            # Return huge penalty so CMA-ES avoids this region
            return 1e6

        state = manager.getState()

        act_p = cset.get("pitch1").getValue(state) + cset.get("pitch2").getValue(state)
        act_r = cset.get("roll1").getValue(state) + cset.get("roll2").getValue(state)
        act_y = cset.get("yaw1").getValue(state) + cset.get("yaw2").getValue(state)

        # Log every 0.2 s (every 4 steps of dt=0.05)
        if round(t * 100) % 20 == 0:
            print(f"[{worker_id}] t={t:.2f}s | Des(P,R,Y): {np.degrees(dp):5.1f},{np.degrees(dr):5.1f},{np.degrees(dy):5.1f} | Act(P,R,Y): {np.degrees(act_p):5.1f},{np.degrees(act_r):5.1f},{np.degrees(act_y):5.1f}")

        total_error += abs(dp - act_p) + abs(dr - act_r) + abs(dy - act_y)

    return total_error

if __name__ == "__main__":
    if len(sys.argv) != 5:
        print("Usage: python cmaes_step_worker.py Kp Ki Kd worker_id")
        sys.exit(1)
    Kp = float(sys.argv[1])
    Ki = float(sys.argv[2])
    Kd = float(sys.argv[3])
    worker_id = sys.argv[4]
    err = evaluate_params(Kp, Ki, Kd, worker_id)
    # Final line: the raw score for the master to read
    print(f"SCORE:{err}")
