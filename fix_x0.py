import os
import re

file_path = r'optimize_PID_cmaes_parallel.py'
with open(file_path, 'r') as f:
    content = f.read()

# Replace the initial guess for alpha
content = re.sub(
    r"x0 = \[4\.89, 8\.76", 
    "x0 = [2.0, 8.76", 
    content
)

with open(file_path, 'w') as f:
    f.write(content)

print("Fixed x0 initial guess for alpha!")
