import os

file_path = 'cmaes_worker.py'
with open(file_path, 'r', encoding='utf-8') as f:
    content = f.read()

content = content.replace("zeta = 1.0", "zeta = 0.8")

with open(file_path, 'w', encoding='utf-8') as f:
    f.write(content)

print("Updated zeta to 0.8!")
