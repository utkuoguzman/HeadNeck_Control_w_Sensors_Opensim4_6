with open("cmaes_worker_p22.py", "r", encoding="utf-8") as f:
    src = f.read()

old_vol_init = """    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_pitch", ki * 0.10528)

    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_roll", 0.0)

    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_yaw", 0.0)"""

new_vol_init = """    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_pitch", ki * 0.10528)

    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_roll", 0.0)

    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_yaw", 0.0)

    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt2_pitch", ki * 0.10528)

    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt2_roll", 0.0)

    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt2_yaw", 0.0)"""

assert old_vol_init in src, "old_vol_init not found"
src = src.replace(old_vol_init, new_vol_init, 1)

# Default KP_TASK_PITCH and KD_TASK_PITCH to 0.0 during postural stabilization
old_kp_kd = """    kp_task_pitch = float(os.environ.get("KP_TASK_PITCH", "10.24"))
    kd_task_pitch = float(os.environ.get("KD_TASK_PITCH", "1.31"))"""
new_kp_kd = """    kp_task_pitch = float(os.environ.get("KP_TASK_PITCH", "0.0"))
    kd_task_pitch = float(os.environ.get("KD_TASK_PITCH", "0.0"))"""
assert old_kp_kd in src, "old_kp_kd not found"
src = src.replace(old_kp_kd, new_kp_kd, 1)

with open("cmaes_worker_p22.py", "w", encoding="utf-8", newline="\r\n") as f:
    f.write(src)

print("Patched cmaes_worker_p22.py successfully!")

