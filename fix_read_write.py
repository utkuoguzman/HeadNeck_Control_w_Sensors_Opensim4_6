import os

with open("HeadNeckNeuralController_plugin/HeadNeckNeuralController.cpp", "r") as f:
    code = f.read()

old_block = '''        double x_phas_pitch = getStateVariableValue(s, "phas_state_pitch");
        double x_phas_roll  = getStateVariableValue(s, "phas_state_roll");
        double x_phas_yaw   = getStateVariableValue(s, "phas_state_yaw");
        
        setStateVariableDerivativeValue(s, "ton_state_pitch", -get_p_ton() * x_ton_pitch + u_ton_pitch);'''

new_block = '''        double x_phas_pitch = getStateVariableValue(s, "phas_state_pitch");
        double x_phas_roll  = getStateVariableValue(s, "phas_state_roll");
        double x_phas_yaw   = getStateVariableValue(s, "phas_state_yaw");
        
        setStateVariableDerivativeValue(s, "ton_state_pitch", -get_p_ton() * x_ton_pitch + u_ton_pitch);'''

# Wait, the problem is that `getStateVariableValue(s, "ton_state_pitch")` comes AFTER `setStateVariableDerivativeValue(s, "yaw_error_integral", yaw_err);`
# I should move the `getStateVariableValue` calls to BEFORE `setStateVariableDerivativeValue(s, "pitch_error_integral", pitch_err);`

old_injection = '''    setStateVariableDerivativeValue(s, "pitch_error_integral", pitch_err);
    setStateVariableDerivativeValue(s, "roll_error_integral", roll_err);
    setStateVariableDerivativeValue(s, "yaw_error_integral", yaw_err);

    // --- Neural Filter State Derivatives ---
    DelayedData vcr_data = getDelayedData(s.getTime(), get_delay_time());'''

# I will replace it so that the state variables are read BEFORE setting any derivatives!
# Actually, I can just replace the whole section starting from `// PID Integrator derivatives`

import re
match = re.search(r'// PID Integrator derivatives.*?setStateVariableDerivativeValue\(s, "yaw_error_integral", yaw_err\);', code, re.DOTALL)
if match:
    old_pid_block = match.group(0)
    
    # We will read states before old_pid_block, and put the Neural Filter State Derivatives right after reading states!
    # Wait, it's safer to just do all `getStateVariableValue` at the top of the function, or right before the first `setStateVariableDerivativeValue`.
    
    # The first setStateVariableDerivativeValue is in the canal filters:
    # setStateVariableDerivativeValue(s, prefix + "_state_0", dx1);
    
    pass

