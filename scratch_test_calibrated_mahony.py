import opensim as osim
import numpy as np
import os

osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'proprioception_plugin/build/Release/osimMillard12EqWithAff.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'vestibular_plugin/build/Release/osimVestibular.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'FixationController_plugin/build/Release/osimFixation.dll'))

model = osim.Model('osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim')
state = model.initSystem()
manager = osim.Manager(model)
state.setTime(0.0)
manager.initialize(state)

# Let's test a calibrated Mahony filter
q = np.array([1.0, 0.0, 0.0, 0.0]) # [q0, q1, q2, q3]
t_prev = 0.0

print(f"{'Time':>6} | {'GT Pitch':>10} | {'Calibrated Mahony':>18} | {'Error (deg)':>12}")
print('-' * 55)

max_error = 0.0

for step in range(1, 11):
    t_target = step * 0.05
    state = manager.integrate(t_target)
    model.realizeDynamics(state)
    
    qp1 = model.getCoordinateSet().get('pitch1').getValue(state)
    qp2 = model.getCoordinateSet().get('pitch2').getValue(state)
    gt_pitch = qp1 + qp2
    
    # Read skull kinematics:
    skull = model.getBodySet().get('skull')
    R_head = skull.getTransformInGround(state).R()
    R_mat = np.array([[R_head.get(r, c) for c in range(3)] for r in range(3)])
    
    omega_gnd = skull.getVelocityInGround(state).get(0)
    w_vec = np.array([omega_gnd.get(i) for i in range(3)])
    # In head frame:
    w_head = R_mat.T @ w_vec
    
    # Simulation sub-stepping of Mahony filter:
    dt = t_target - t_prev
    sub_steps = 50
    dt_sub = dt / sub_steps
    
    for _ in range(sub_steps):
        q0, q1, q2, q3 = q
        
        # Semicircular canal integration in body frame:
        gx, gy, gz = w_head[0], w_head[1], w_head[2]
        
        dq0 = 0.5 * (-q1*gx - q2*gy - q3*gz)
        dq1 = 0.5 * ( q0*gx + q2*gz - q3*gy)
        dq2 = 0.5 * ( q0*gy - q1*gz + q3*gx)
        dq3 = 0.5 * ( q0*gz + q1*gy - q2*gx)
        
        q[0] += dq0 * dt_sub
        q[1] += dq1 * dt_sub
        q[2] += dq2 * dt_sub
        q[3] += dq3 * dt_sub
        q /= np.linalg.norm(q)
        
    t_prev = t_target
    
    est_pitch = np.arctan2(2.0*(q[0]*q[3] + q[1]*q[2]), 1.0 - 2.0*(q[2]**2 + q[3]**2))
    err = abs(np.degrees(gt_pitch) - np.degrees(est_pitch))
    if err > max_error: max_error = err
    print(f"{t_target:6.2f} | {np.degrees(gt_pitch):10.2f} | {np.degrees(est_pitch):18.2f} | {err:12.2f}")

print(f"\nMax Error: {max_error:.2f} deg")

