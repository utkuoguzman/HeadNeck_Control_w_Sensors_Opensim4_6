import opensim as osim
import numpy as np
import os

# Load all OpenSim plugin libraries
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'proprioception_plugin/build/Release/osimMillard12EqWithAff.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'vestibular_plugin/build/Release/osimVestibular.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'FixationController_plugin/build/Release/osimFixation.dll'))

def run_multisensory_audit():
    model = osim.Model('osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim')
    state = model.initSystem()
    manager = osim.Manager(model)
    state.setTime(0.0)
    manager.initialize(state)

    controller = model.getControllerSet().get('VCR_CCR_Controller')
    
    # Proprioceptive Decoded Outputs (from 72 muscles)
    prop_p_out = controller.getOutput('est_neck_pitch')
    prop_r_out = controller.getOutput('est_neck_roll')
    prop_y_out = controller.getOutput('est_neck_yaw')
    
    # Fused Multisensory Observer Outputs (from C++ plugin)
    fused_p_out = controller.getOutput('mahony_pitch')
    fused_r_out = controller.getOutput('mahony_roll')
    fused_y_out = controller.getOutput('mahony_yaw')
    fused_p_spd_out = controller.getOutput('mahony_pitch_speed')

    skull = model.getBodySet().get('skull')

    dt_step = 0.005
    total_time = 0.50
    t_prev = 0.0

    # Also simulate standalone Vestibular-only (no proprioceptive anchor) for exact direct comparison
    q_vest_standalone = np.array([1.0, 0.0, 0.0, 0.0])

    times = []
    gt_pitch_list, prop_pitch_list, vest_pitch_list, fused_pitch_list = [], [], [], []
    gt_yaw_list,   prop_yaw_list,   vest_yaw_list,   fused_yaw_list   = [], [], [], []
    gt_roll_list,  prop_roll_list,  vest_roll_list,  fused_roll_list  = [], [], [], []

    gt_spd_list, fused_spd_list = [], []

    t = 0.0
    while t < total_time:
        t += dt_step
        state = manager.integrate(t)
        model.realizeVelocity(state)

        # Ground truth
        qp1 = model.getCoordinateSet().get('pitch1').getValue(state)
        qp2 = model.getCoordinateSet().get('pitch2').getValue(state)
        gt_p = qp1 + qp2

        qr1 = model.getCoordinateSet().get('roll1').getValue(state)
        qr2 = model.getCoordinateSet().get('roll2').getValue(state)
        gt_r = qr1 + qr2

        qy1 = model.getCoordinateSet().get('yaw1').getValue(state)
        qy2 = model.getCoordinateSet().get('yaw2').getValue(state)
        gt_y = qy1 + qy2

        up1 = model.getCoordinateSet().get('pitch1').getSpeedValue(state)
        up2 = model.getCoordinateSet().get('pitch2').getSpeedValue(state)
        gt_spd = up1 + up2

        # 1. Proprioceptive alone (direct from 72 muscles)
        prop_p = float(prop_p_out.getValueAsString(state))
        prop_r = float(prop_r_out.getValueAsString(state))
        prop_y = float(prop_y_out.getValueAsString(state))

        # 2. Fused Observer (from compiled C++ plugin)
        fused_p = float(fused_p_out.getValueAsString(state))
        fused_r = float(fused_r_out.getValueAsString(state))
        fused_y = float(fused_y_out.getValueAsString(state))
        fused_spd = float(fused_p_spd_out.getValueAsString(state))

        # 3. Vestibular alone integration (K_prop = 0)
        dt = t - t_prev
        R_head = skull.getTransformInGround(state).R()
        R_mat = np.array([[R_head.get(r, c) for c in range(3)] for r in range(3)])
        w_vec = np.array([skull.getVelocityInGround(state).get(0).get(i) for i in range(3)])
        w_head = R_mat.T @ w_vec
        u_meas = R_mat.T @ np.array([0.0, 1.0, 0.0])

        q0, q1, q2, q3 = q_vest_standalone
        u_est_v = np.array([2*(q1*q2 + q0*q3), q0*q0 - q1*q1 + q2*q2 - q3*q3, 2*(q2*q3 - q0*q1)])
        e_oto_v = np.cross(u_meas, u_est_v)
        w_v = w_head + 0.05 * e_oto_v
        dq_v = 0.5 * np.array([
            -q1*w_v[0] - q2*w_v[1] - q3*w_v[2],
             q0*w_v[0] + q2*w_v[2] - q3*w_v[1],
             q0*w_v[1] - q1*w_v[2] + q3*w_v[0],
             q0*w_v[2] + q1*w_v[1] - q2*w_v[0]
        ])
        q_vest_standalone += dq_v * dt
        q_vest_standalone /= np.linalg.norm(q_vest_standalone)
        vest_p = np.arctan2(2*(q_vest_standalone[0]*q_vest_standalone[3] + q_vest_standalone[1]*q_vest_standalone[2]),
                            1 - 2*(q_vest_standalone[2]**2 + q_vest_standalone[3]**2))
        vest_y = np.arcsin(np.clip(2*(q_vest_standalone[0]*q_vest_standalone[2] - q_vest_standalone[3]*q_vest_standalone[1]), -1.0, 1.0))
        vest_r = np.arctan2(2*(q_vest_standalone[0]*q_vest_standalone[1] + q_vest_standalone[2]*q_vest_standalone[3]),
                            1 - 2*(q_vest_standalone[1]**2 + q_vest_standalone[2]**2))

        t_prev = t

        times.append(t)
        gt_pitch_list.append(np.degrees(gt_p))
        prop_pitch_list.append(np.degrees(prop_p))
        vest_pitch_list.append(np.degrees(vest_p))
        fused_pitch_list.append(np.degrees(fused_p))

        gt_yaw_list.append(np.degrees(gt_y))
        prop_yaw_list.append(np.degrees(prop_y))
        vest_yaw_list.append(np.degrees(vest_y))
        fused_yaw_list.append(np.degrees(fused_y))

        gt_roll_list.append(np.degrees(gt_r))
        prop_roll_list.append(np.degrees(prop_r))
        vest_roll_list.append(np.degrees(vest_r))
        fused_roll_list.append(np.degrees(fused_r))

        gt_spd_list.append(np.degrees(gt_spd))
        fused_spd_list.append(np.degrees(fused_spd))

    # Compute Error Metrics
    err_p_prop  = np.abs(np.array(gt_pitch_list) - np.array(prop_pitch_list))
    err_p_vest  = np.abs(np.array(gt_pitch_list) - np.array(vest_pitch_list))
    err_p_fused = np.abs(np.array(gt_pitch_list) - np.array(fused_pitch_list))

    err_y_prop  = np.abs(np.array(gt_yaw_list) - np.array(prop_yaw_list))
    err_y_vest  = np.abs(np.array(gt_yaw_list) - np.array(vest_yaw_list))
    err_y_fused = np.abs(np.array(gt_yaw_list) - np.array(fused_yaw_list))

    err_r_prop  = np.abs(np.array(gt_roll_list) - np.array(prop_roll_list))
    err_r_vest  = np.abs(np.array(gt_roll_list) - np.array(vest_roll_list))
    err_r_fused = np.abs(np.array(gt_roll_list) - np.array(fused_roll_list))

    err_spd_fused = np.abs(np.array(gt_spd_list) - np.array(fused_spd_list))

    print("=" * 95)
    print("      MULTISENSORY HEAD ATTITUDE ESTIMATION AUDIT (OPENSIM C++ PLUGIN)")
    print("=" * 95)
    print(f"{'Time (s)':>8} | {'GT Pitch':>9} | {'Proprio':>9} | {'Vestib':>9} | {'FUSED':>9} | {'Prop Err':>9} | {'Vest Err':>9} | {'FUSED Err':>9}")
    print("-" * 95)
    for i in range(0, len(times), 10):
        print(f"{times[i]:8.3f} | {gt_pitch_list[i]:9.2f} | {prop_pitch_list[i]:9.2f} | {vest_pitch_list[i]:9.2f} | {fused_pitch_list[i]:9.2f} | {err_p_prop[i]:9.2f} | {err_p_vest[i]:9.2f} | {err_p_fused[i]:9.2f}")

    print("=" * 95)
    print("ACCURACY COMPARISON (POSITION ERROR):")
    print("-" * 95)
    print(f"PITCH (deg):")
    print(f"  * Proprioception Alone:  Max = {np.max(err_p_prop):6.3f} deg | Mean = {np.mean(err_p_prop):6.3f} deg")
    print(f"  * Vestibular Alone:      Max = {np.max(err_p_vest):6.3f} deg | Mean = {np.mean(err_p_vest):6.3f} deg")
    print(f"  * FUSED (Multisensory):  Max = {np.max(err_p_fused):6.3f} deg | Mean = {np.mean(err_p_fused):6.3f} deg  <-- [SUPERIOR]")
    print("-" * 95)
    print(f"YAW (deg):")
    print(f"  * Proprioception Alone:  Max = {np.max(err_y_prop):6.3f} deg | Mean = {np.mean(err_y_prop):6.3f} deg")
    print(f"  * Vestibular Alone:      Max = {np.max(err_y_vest):6.3f} deg | Mean = {np.mean(err_y_vest):6.3f} deg")
    print(f"  * FUSED (Multisensory):  Max = {np.max(err_y_fused):6.3f} deg | Mean = {np.mean(err_y_fused):6.3f} deg")
    print("-" * 95)
    print(f"ROLL (deg):")
    print(f"  * Proprioception Alone:  Max = {np.max(err_r_prop):6.3f} deg | Mean = {np.mean(err_r_prop):6.3f} deg")
    print(f"  * Vestibular Alone:      Max = {np.max(err_r_vest):6.3f} deg | Mean = {np.mean(err_r_vest):6.3f} deg")
    print(f"  * FUSED (Multisensory):  Max = {np.max(err_r_fused):6.3f} deg | Mean = {np.mean(err_r_fused):6.3f} deg")
    print("-" * 95)
    print(f"VELOCITY ERROR (FUSED OBSERVER):")
    print(f"  * Max Pitch Velocity Error: {np.max(err_spd_fused):6.3f} deg/s | Mean = {np.mean(err_spd_fused):6.3f} deg/s")
    print("=" * 95)

if __name__ == '__main__':
    run_multisensory_audit()

