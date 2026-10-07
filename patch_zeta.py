import os

with open('optimize_PID_cmaes_parallel.py', 'r', encoding='utf-8') as f:
    lines = f.readlines()

for i, line in enumerate(lines):
    if "[ 4.0,  0.3, 10.0,   5.0,  0.5,   2.0,    0.01,        20.0,         50.0  ]" in line:
        lines[i] = line.replace("0.3", "0.1")
        break

with open('optimize_PID_cmaes_parallel.py', 'w', encoding='utf-8') as f:
    f.writelines(lines)

print("Successfully lowered zeta bound to 0.1!")
