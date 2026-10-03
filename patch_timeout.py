import os

with open('optimize_PID_cmaes_parallel.py', 'r', encoding='utf-8') as f:
    content = f.read()

content = content.replace("timeout=300", "timeout=600")

with open('optimize_PID_cmaes_parallel.py', 'w', encoding='utf-8') as f:
    f.write(content)

print("Successfully increased timeout to 600s!")
