with open('cmaes_worker_p22.py', 'r', encoding='utf-8') as f:
    lines = f.readlines()

new_block = [
    '    # Direct gentle task space PD parameters (disregards W values)\n',
    '    # Analytically derived from inverted pendulum 1-sec settling time:\n',
    '    #   Pitch: Kp = 10.24 Nm/rad, Kd = 1.31 Nms/rad\n',
    '    #   Roll : Kp = 10.21 Nm/rad, Kd = 1.29 Nms/rad\n',
    '    #   Yaw  : Kp =  0.24 Nm/rad, Kd = 0.12 Nms/rad\n',
    '    kp_task_pitch = float(os.environ.get("KP_TASK_PITCH", "10.24"))\n',
    '    kd_task_pitch = float(os.environ.get("KD_TASK_PITCH", "1.31"))\n',
    '    kp_task_roll  = float(os.environ.get("KP_TASK_ROLL",  "10.21"))\n',
    '    kd_task_roll  = float(os.environ.get("KD_TASK_ROLL",  "1.29"))\n',
    '    kp_task_yaw   = float(os.environ.get("KP_TASK_YAW",   "0.24"))\n',
    '    kd_task_yaw   = float(os.environ.get("KD_TASK_YAW",   "0.12"))\n',
    '    osim.PropertyHelper.setValueDouble(kp_task_pitch, controller.updPropertyByName("Kp_task_pitch"))\n',
    '    osim.PropertyHelper.setValueDouble(kd_task_pitch, controller.updPropertyByName("Kd_task_pitch"))\n',
    '    osim.PropertyHelper.setValueDouble(kp_task_roll,  controller.updPropertyByName("Kp_task_roll"))\n',
    '    osim.PropertyHelper.setValueDouble(kd_task_roll,  controller.updPropertyByName("Kd_task_roll"))\n',
    '    osim.PropertyHelper.setValueDouble(kp_task_yaw,   controller.updPropertyByName("Kp_task_yaw"))\n',
    '    osim.PropertyHelper.setValueDouble(kd_task_yaw,   controller.updPropertyByName("Kd_task_yaw"))\n'
]

# Replace lines 122 through 141 (inclusive)
lines[122:142] = new_block

with open('cmaes_worker_p22.py', 'w', encoding='utf-8') as f:
    f.writelines(lines)

print('Successfully patched cmaes_worker_p22.py with direct gentle gains!')

