import os
import re

file_path = r'optimize_PID_cmaes_parallel.py'
with open(file_path, 'r') as f:
    content = f.read()

content = re.sub(r"sigma0 = 0\.5", "sigma0 = 1.5", content)

with open(file_path, 'w') as f:
    f.write(content)

print("Updated sigma0 to 1.5 in optimize_PID_cmaes_parallel.py!")
