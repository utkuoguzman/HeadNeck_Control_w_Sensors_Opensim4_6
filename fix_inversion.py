import os

old_line1 = 'R_world_to_head = skull.getTransformInGround(state).R().invert()'
old_line2 = 'q = osim.Rotation(R_world_to_head).convertRotationToQuaternion()'

new_line1 = 'R_head_to_world = skull.getTransformInGround(state).R()'
new_line2 = 'q = osim.Rotation(R_head_to_world).convertRotationToQuaternion()'

def replace_in_file(filepath):
    with open(filepath, 'r') as f:
        content = f.read()
    content = content.replace(old_line1, new_line1).replace(old_line2, new_line2)
    with open(filepath, 'w') as f:
        f.write(content)

replace_in_file(r'test_scripts\test_pitch_minjerk.py')
replace_in_file(r'cmaes_worker.py')

print("Fixed Quaternion Inversion!")
