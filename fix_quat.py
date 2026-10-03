import os

old_line = 'q = R_world_to_head.convertRotationToQuaternion()'
new_line = 'q = osim.Rotation(R_world_to_head).convertRotationToQuaternion()'

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

print("Fixed Rotation to Quaternion!")
