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

controller = model.getControllerSet().get('VCR_CCR_Controller')
mah_pitch_out = controller.getOutput('mahony_pitch')
mah_roll_out = controller.getOutput('mahony_roll')
mah_yaw_out = controller.getOutput('mahony_yaw')
mah_pitch_spd_out = controller.getOutput('mahony_pitch_speed')
mah_roll_spd_out = controller.getOutput('mahony_roll_speed')
mah_yaw_spd_out = controller.getOutput('mahony_yaw_speed')

dt_step = 0.005
total_time = 0.50

times = []
gt_pitches, mah_pitches = [], []
gt_rolls, mah_rolls = [], []
gt_yaws, mah_yaws = [], []

gt_pitch_spds, mah_pitch_spds = [], []
gt_roll_spds, mah_roll_spds = [], []
gt_yaw_spds, mah_yaw_spds = [], []

t = 0.0
while t < total_time:
    t += dt_step
    state = manager.integrate(t)
    model.realizeVelocity(state)
    
    # Ground truth relative neck kinematics
    qp1 = model.getCoordinateSet().get('pitch1').getValue(state)
    qp2 = model.getCoordinateSet().get('pitch2').getValue(state)
    up1 = model.getCoordinateSet().get('pitch1').getSpeedValue(state)
    up2 = model.getCoordinateSet().get('pitch2').getSpeedValue(state)
    gt_pitch = qp1 + qp2
    gt_pitch_spd = up1 + up2

    qr1 = model.getCoordinateSet().get('roll1').getValue(state)
    qr2 = model.getCoordinateSet().get('roll2').getValue(state)
    ur1 = model.getCoordinateSet().get('roll1').getSpeedValue(state)
    ur2 = model.getCoordinateSet().get('roll2').getSpeedValue(state)
    gt_roll = qr1 + qr2
    gt_roll_spd = ur1 + ur2

    qy1 = model.getCoordinateSet().get('yaw1').getValue(state)
    qy2 = model.getCoordinateSet().get('yaw2').getValue(state)
    uy1 = model.getCoordinateSet().get('yaw1').getSpeedValue(state)
    uy2 = model.getCoordinateSet().get('yaw2').getSpeedValue(state)
    gt_yaw = qy1 + qy2
    gt_yaw_spd = uy1 + uy2
    
    # Mahony outputs from C++ plugin
    m_p = float(mah_pitch_out.getValueAsString(state))
    m_r = float(mah_roll_out.getValueAsString(state))
    m_y = float(mah_yaw_out.getValueAsString(state))
    
    m_p_spd = float(mah_pitch_spd_out.getValueAsString(state))
    m_r_spd = float(mah_roll_spd_out.getValueAsString(state))
    m_y_spd = float(mah_yaw_spd_out.getValueAsString(state))
    
    times.append(t)
    gt_pitches.append(np.degrees(gt_pitch))
    mah_pitches.append(np.degrees(m_p))
    gt_rolls.append(np.degrees(gt_roll))
    mah_rolls.append(np.degrees(m_r))
    gt_yaws.append(np.degrees(gt_yaw))
    mah_yaws.append(np.degrees(m_y))
    
    gt_pitch_spds.append(np.degrees(gt_pitch_spd))
    mah_pitch_spds.append(np.degrees(m_p_spd))
    gt_roll_spds.append(np.degrees(gt_roll_spd))
    mah_roll_spds.append(np.degrees(m_r_spd))
    gt_yaw_spds.append(np.degrees(gt_yaw_spd))
    mah_yaw_spds.append(np.degrees(m_y_spd))

pos_err_p = np.abs(np.array(gt_pitches) - np.array(mah_pitches))
vel_err_p = np.abs(np.array(gt_pitch_spds) - np.array(mah_pitch_spds))

pos_err_r = np.abs(np.array(gt_rolls) - np.array(mah_rolls))
vel_err_r = np.abs(np.array(gt_roll_spds) - np.array(mah_roll_spds))

pos_err_y = np.abs(np.array(gt_yaws) - np.array(mah_yaws))
vel_err_y = np.abs(np.array(gt_yaw_spds) - np.array(mah_yaw_spds))

print("=" * 85)
print(f"{'Time (s)':>8} | {'GT Pitch':>10} | {'Mahony P':>10} | {'Pos Err':>8} | {'GT Spd':>10} | {'Mahony Spd':>10} | {'Vel Err':>8}")
print("-" * 85)
for i in range(0, len(times), 10):
    print(f"{times[i]:8.3f} | {gt_pitches[i]:10.2f} | {mah_pitches[i]:10.2f} | {pos_err_p[i]:8.2f} | {gt_pitch_spds[i]:10.2f} | {mah_pitch_spds[i]:10.2f} | {vel_err_p[i]:8.2f}")

print("=" * 85)
print(f"PITCH  - Max Position Error: {np.max(pos_err_p):.3f} deg | Mean: {np.mean(pos_err_p):.3f} deg")
print(f"PITCH  - Max Velocity Error: {np.max(vel_err_p):.3f} deg/s | Mean: {np.mean(vel_err_p):.3f} deg/s")
print(f"ROLL   - Max Position Error: {np.max(pos_err_r):.3f} deg | Mean: {np.mean(pos_err_r):.3f} deg")
print(f"ROLL   - Max Velocity Error: {np.max(vel_err_r):.3f} deg/s | Mean: {np.mean(vel_err_r):.3f} deg/s")
print(f"YAW    - Max Position Error: {np.max(pos_err_y):.3f} deg | Mean: {np.mean(pos_err_y):.3f} deg")
print(f"YAW    - Max Velocity Error: {np.max(vel_err_y):.3f} deg/s | Mean: {np.mean(vel_err_y):.3f} deg/s")
print("=" * 85)
