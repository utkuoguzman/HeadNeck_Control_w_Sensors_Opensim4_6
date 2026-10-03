import os

old_line = 'R_world_to_head = skull.getMobilizedBody().getBodyTransform(state).R().invert()'
new_line = 'R_world_to_head = skull.getTransformInGround(state).R().invert()'

# 1. Update test_pitch_minjerk.py
test_path = r'test_scripts\test_pitch_minjerk.py'
with open(test_path, 'r') as f:
    content = f.read()
content = content.replace(old_line, new_line)
with open(test_path, 'w') as f:
    f.write(content)

# 2. Update cmaes_worker.py
worker_path = r'cmaes_worker.py'
with open(worker_path, 'r') as f:
    content = f.read()
content = content.replace(old_line, new_line)
with open(worker_path, 'w') as f:
    f.write(content)

print("Fixed OpenSim Python API call!")
