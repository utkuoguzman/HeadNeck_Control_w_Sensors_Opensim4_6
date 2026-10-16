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

q = np.array([1.0, 0.0, 0.0, 0.0])
t_prev = 0.0

print(f"{'Time':>6} | {'GT Pitch':>10} | {'Canal Gyro Pitch':>18}")
print('-' * 42)

for step in range(1, 11):
    t_target = step * 0.05
    state = manager.integrate(t_target)
    
    qp1 = model.getCoordinateSet().get('pitch1').getValue(state)
    qp2 = model.getCoordinateSet().get('pitch2').getValue(state)
    gt_pitch = qp1 + qp2
    
    # Read skull angular velocity in Ground:
    omega_ground = model.getBodySet().get('skull').getVelocityInGround(state).get(0)
    wz = omega_ground.get(2) # pitch rate
    
    dt = t_target - t_prev
    dq3 = 0.5 * q[0] * wz
    dq0 = -0.5 * q[3] * wz
    q[0] += dq0 * dt
    q[3] += dq3 * dt
    q /= np.linalg.norm(q)
    t_prev = t_target
    
    est_pitch = np.arctan2(2.0*q[0]*q[3], 1.0 - 2.0*q[3]**2)
    print(f"{t_target:6.2f} | {np.degrees(gt_pitch):10.2f} | {np.degrees(est_pitch):18.2f}")

