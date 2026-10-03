import os

injection = """manager.initialize(state)
    
    # Perfect SE(3) Observer Initialization
    skull = model.getBodySet().get("skull")
    R_world_to_head = skull.getMobilizedBody().getBodyTransform(state).R().invert()
    q = R_world_to_head.convertRotationToQuaternion()
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q0", q[0])
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q1", q[1])
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q2", q[2])
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q3", q[3])
"""

# 1. Update test_pitch_minjerk.py
test_path = r'test_scripts\test_pitch_minjerk.py'
with open(test_path, 'r') as f:
    content = f.read()
if "Perfect SE(3) Observer Initialization" not in content:
    content = content.replace("manager.initialize(state)", injection)
    with open(test_path, 'w') as f:
        f.write(content)

# 2. Update cmaes_worker.py
worker_path = r'cmaes_worker.py'
with open(worker_path, 'r') as f:
    content = f.read()
if "Perfect SE(3) Observer Initialization" not in content:
    # Need to match indentation, cmaes_worker.py has manager.initialize(state) indented by 4 spaces
    indented_injection = injection.replace('\n', '\n    ')
    content = content.replace("    manager.initialize(state)", "    " + indented_injection.strip())
    with open(worker_path, 'w') as f:
        f.write(content)

print("Injected perfect initialization!")
