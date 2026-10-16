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

# High-rate continuous tracking
q = np.array([1.0, 0.0, 0.0, 0.0]) # [q0, q1, q2, q3]
t_prev = 0.0

dt_step = 0.005 # 5ms sampling
total_time = 0.50

times = []
gt_pitches = []
mahony_pitches = []
gt_pitch_speeds = []
mahony_pitch_speeds = []

t = 0.0
while t < total_time:
    t += dt_step
    state = manager.integrate(t)
    model.realizeDynamics(state)
    
    qp1 = model.getCoordinateSet().get('pitch1').getValue(state)
    qp2 = model.getCoordinateSet().get('pitch2').getValue(state)
    gt_pitch = qp1 + qp2
    
    up1 = model.getCoordinateSet().get('pitch1').getSpeedValue(state)
    up2 = model.getCoordinateSet().get('pitch2').getSpeedValue(state)
    gt_speed = up1 + up2
    
    # Skull transform in Ground
    skull = model.getBodySet().get('skull')
    R_head = skull.getTransformInGround(state).R()
    R_mat = np.array([[R_head.get(r, c) for c in range(3)] for r in range(3)])
    
    # Angular velocity in Ground:
    omega_gnd = skull.getVelocityInGround(state).get(0)
    w_vec = np.array([omega_gnd.get(i) for i in range(3)])
    w_head = R_mat.T @ w_vec
    
    # Acceleration in Ground:
    # Otolith measures: f_oto = R^T * (a_lin - g)
    # With tilt-translation acceleration gating:
    # We estimate gravity UP vector:
    # In rest, f_oto points UP. During high acceleration (|a| > threshold), gravity is corrupted.
    
    # Mahony integration:
    dt = t - t_prev
    q0, q1, q2, q3 = q
    gx, gy, gz = w_head[0], w_head[1], w_head[2]
    
    # Otolith gravity correction (slow, biological):
    # Expected UP in head frame:
    u_est = np.array([
        2.0*(q1*q2 + q0*q3),
        q0*q0 - q1*q1 + q2*q2 - q3*q3,
        2.0*(q2*q3 - q0*q1)
    ])
    
    # True UP in head frame:
    u_meas = R_mat.T @ np.array([0.0, 1.0, 0.0])
    
    # Error vector:
    e = np.cross(u_meas, u_est)
    
    # Complementary filter gain (biological slow otolith integration ~ 0.05 to 0.1 rad/s)
    Kp = 0.05
    gx += Kp * e[0]
    gy += Kp * e[1]
    gz += Kp * e[2]
    
    dq0 = 0.5 * (-q1*gx - q2*gy - q3*gz)
    dq1 = 0.5 * ( q0*gx + q2*gz - q3*gy)
    dq2 = 0.5 * ( q0*gy - q1*gz + q3*gx)
    dq3 = 0.5 * ( q0*gz + q1*gy - q2*gx)
    
    q[0] += dq0 * dt
    q[1] += dq1 * dt
    q[2] += dq2 * dt
    q[3] += dq3 * dt
    q /= np.linalg.norm(q)
    t_prev = t
    
    est_pitch = np.arctan2(2.0*(q[0]*q[3] + q[1]*q[2]), 1.0 - 2.0*(q[2]**2 + q[3]**2))
    
    times.append(t)
    gt_pitches.append(np.degrees(gt_pitch))
    mahony_pitches.append(np.degrees(est_pitch))
    gt_pitch_speeds.append(np.degrees(gt_speed))
    mahony_pitch_speeds.append(np.degrees(gz))

pos_errors = np.abs(np.array(gt_pitches) - np.array(mahony_pitches))
vel_errors = np.abs(np.array(gt_pitch_speeds) - np.array(mahony_pitch_speeds))

print(f"{'Time':>6} | {'GT Pitch':>10} | {'Mahony Pitch':>12} | {'Pos Err':>8} | {'GT Speed':>10} | {'Mahony Spd':>10} | {'Vel Err':>8}")
print('-' * 80)
for i in range(0, len(times), 10):
    print(f"{times[i]:6.2f} | {gt_pitches[i]:10.2f} | {mahony_pitches[i]:12.2f} | {pos_errors[i]:8.2f} | {gt_pitch_speeds[i]:10.2f} | {mahony_pitch_speeds[i]:10.2f} | {vel_errors[i]:8.2f}")

print(f"\nMax Position Error: {np.max(pos_errors):.3f} deg")
print(f"Mean Position Error: {np.mean(pos_errors):.3f} deg")
print(f"Max Velocity Error: {np.max(vel_errors):.3f} deg/s")
print(f"Mean Velocity Error: {np.mean(vel_errors):.3f} deg/s")

