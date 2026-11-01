with open('cmaes_worker_p22.py', 'r', encoding='utf-8') as f:
    text = f.read()

target1 = """    # Voluntary LQR costs:
    # Pitch voluntary control is explicitly DISABLED (pure reflex: VCR + CCR, matching Forbes 2013b & Happee 2017)
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("Wp_base_pitch"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("Wv_base_pitch"))
    # Roll and Yaw voluntary costs maintained to lock 3D lateral stability
    for ax in ['roll', 'yaw']:
        osim.PropertyHelper.setValueDouble(400.0, controller.updPropertyByName(f"Wp_base_{ax}"))
        osim.PropertyHelper.setValueDouble(40.0, controller.updPropertyByName(f"Wv_base_{ax}"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("k_vigor"))
    osim.PropertyHelper.setValueDouble(0.1, controller.updPropertyByName("trajectory_tau"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("K_eso"))

    # Ki_task disabled for pitch (0.0 voluntary integral torque)
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("Ki_task"))"""

replacement1 = """    # Voluntary LQR costs:
    # Relaxed voluntary pitch costs (Wp_base_pitch=0.0 by default for pure reflex compliance)
    wp_pitch = float(os.environ.get("WP_BASE_PITCH", "0.0"))
    wv_pitch = float(os.environ.get("WV_BASE_PITCH", str(2.0 * math.sqrt(wp_pitch) if wp_pitch > 0 else 0.0)))
    osim.PropertyHelper.setValueDouble(wp_pitch, controller.updPropertyByName("Wp_base_pitch"))
    osim.PropertyHelper.setValueDouble(wv_pitch, controller.updPropertyByName("Wv_base_pitch"))
    # Roll and Yaw voluntary costs maintained to lock 3D lateral stability
    for ax in ['roll', 'yaw']:
        osim.PropertyHelper.setValueDouble(400.0, controller.updPropertyByName(f"Wp_base_{ax}"))
        osim.PropertyHelper.setValueDouble(40.0, controller.updPropertyByName(f"Wv_base_{ax}"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("k_vigor"))
    osim.PropertyHelper.setValueDouble(0.1, controller.updPropertyByName("trajectory_tau"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("K_eso"))

    # Ki_task loaded for gravity compensation trim
    osim.PropertyHelper.setValueDouble(ki, controller.updPropertyByName("Ki_task"))"""

target2 = """    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/pitch_error_integral", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/roll_error_integral", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/yaw_error_integral", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_pitch", 0.0)"""

replacement2 = """    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/pitch_error_integral", 0.10528)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/roll_error_integral", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/yaw_error_integral", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_pitch", ki * 0.10528)"""

text_lf = text.replace('\r\n', '\n')
assert target1 in text_lf, "Target 1 not found"
assert target2 in text_lf, "Target 2 not found"

text_lf = text_lf.replace(target1, replacement1)
text_lf = text_lf.replace(target2, replacement2)

with open('cmaes_worker_p22.py', 'w', encoding='utf-8', newline='\r\n') as f:
    f.write(text_lf.replace('\n', '\r\n'))

print("SUCCESSFULLY RESTORED LOADED KI AND CONFIGURED RELAXED VOLUNTARY PITCH IN cmaes_worker_p22.py")

