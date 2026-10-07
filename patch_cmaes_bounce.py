import os

with open('optimize_PID_cmaes_parallel.py', 'r', encoding='utf-8') as f:
    lines = f.readlines()

for i, line in enumerate(lines):
    if "x0 = [" in line:
        lines[i] = "    x0 = [19.97, 0.266, 15.2, 13.407, 3.810, 3.236, 2.482, 79.99, 136.49]\n"
    if "sigma0 =" in line:
        lines[i] = "    sigma0 = 0.5 \n"
    if "udom =" in line or "# UPPER" in line or "[20.0," in line:
        if "[20.0," in line: # Only replace the 20.0 upper bound for w_n
            lines[i] = line.replace("20.0,", "25.0,", 1)

with open('optimize_PID_cmaes_parallel.py', 'w', encoding='utf-8') as f:
    f.writelines(lines)

print("Patched x0, sigma0, and w_n upper bound!")
