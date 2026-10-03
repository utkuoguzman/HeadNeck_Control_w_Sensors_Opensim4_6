import os

file_path = r'cmaes_worker.py'
with open(file_path, 'r') as f:
    lines = f.readlines()

for i, line in enumerate(lines):
    if "# Perfect SE(3) Observer Initialization" in line:
        # We know lines i to i+8 are incorrectly indented by 8 spaces instead of 4
        for j in range(i, i+8):
            if lines[j].startswith("        "):
                lines[j] = lines[j][4:]

with open(file_path, 'w') as f:
    f.writelines(lines)

print("Fixed indentation in cmaes_worker.py")
