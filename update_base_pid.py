import os
import re

file_path = r'optimize_PID_cmaes_parallel.py'
with open(file_path, 'r') as f:
    content = f.read()

content = re.sub(r"BASE_KP = [\d\.]+", "BASE_KP = 786.01", content)
content = re.sub(r"BASE_KI = [\d\.]+", "BASE_KI = 128.66", content)
content = re.sub(r"BASE_KD = [\d\.]+", "BASE_KD = 15.68", content)

with open(file_path, 'w') as f:
    f.write(content)

print("Updated Base PID values in optimize_PID_cmaes_parallel.py!")
