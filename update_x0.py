import os
import re

file_path = r'run_cmaes_step.py'
with open(file_path, 'r') as f:
    content = f.read()

content = re.sub(r"x0 = \[.*?\]", "x0 = [696.79, 179.12, 120.92]", content)
content = re.sub(r"sigma0 = 15\.0", "sigma0 = 50.0", content)

with open(file_path, 'w') as f:
    f.write(content)

print("Updated x0 initialization!")
