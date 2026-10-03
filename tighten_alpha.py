import os
import re

file_path = r'optimize_PID_cmaes_parallel.py'
with open(file_path, 'r') as f:
    content = f.read()

# Replace the upper bound array again
content = re.sub(
    r"\[\s*5\.0,\s*25\.0,\s*5\.0,\s*10\.0,\s*2\.50,\s*80\.0,\s*150\.0\s*\]", 
    "[ 3.0,  25.0,   5.0, 10.0,    2.50,    80.0,       150.0  ]", 
    content
)

with open(file_path, 'w') as f:
    f.write(content)

print("Updated alpha upper bound to 3.0!")
