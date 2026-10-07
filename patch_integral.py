import re
from pathlib import Path

content = Path('cmaes_worker.py').read_text(encoding='utf-8')

# Change delay from 2.5 to 0.5
content = content.replace('delay = 2.5', 'delay = 0.5')

# Insert pre-loaded integral states
insert_target = 'model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q3", q.get(3))'
pre_load_code = """
    # NEW OPTIMIZATION: Pre-load the PID integral terms so we don't have to wait 2.5s to settle!
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/pitch_error_integral", -0.01906)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/roll_error_integral", -0.01184)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/yaw_error_integral", 0.06309)
"""

content = content.replace(insert_target, insert_target + '\n' + pre_load_code)

Path('cmaes_worker.py').write_text(content, encoding='utf-8')
print("Successfully patched cmaes_worker.py")
