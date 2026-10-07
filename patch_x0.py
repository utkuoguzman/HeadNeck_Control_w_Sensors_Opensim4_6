import os

with open('optimize_PID_cmaes_parallel.py', 'r', encoding='utf-8') as f:
    lines = f.readlines()

for i, line in enumerate(lines):
    if "x0 = [" in line and "9.0, 0.425," in line:
        lines[i] = "    x0 = [16.9099, 0.4115, 27.5753, 5.0202, 4.1992, 2.5413, 0.0782, 64.6769, 100.4010]\n"
        break

with open('optimize_PID_cmaes_parallel.py', 'w', encoding='utf-8') as f:
    f.writelines(lines)

print("Successfully updated x0!")
