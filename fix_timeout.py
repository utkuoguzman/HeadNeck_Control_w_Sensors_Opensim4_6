import os
import re

file_path = r'optimize_PID_cmaes_parallel.py'
with open(file_path, 'r', encoding='utf-8') as f:
    content = f.read()

# Replace capture_output=True with stdout=subprocess.PIPE, stderr=sys.stderr
content = re.sub(
    r"capture_output=True,\s*text=True,\s*timeout=120",
    "stdout=subprocess.PIPE, stderr=sys.stderr, text=True, timeout=300",
    content
)

with open(file_path, 'w', encoding='utf-8') as f:
    f.write(content)

print("Updated timeout to 300s and fixed stderr passthrough in optimize_PID_cmaes_parallel.py")
