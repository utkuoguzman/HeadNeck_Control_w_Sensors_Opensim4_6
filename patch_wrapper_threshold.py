import os

file_path = 'optimize_PID_cmaes_parallel.py'
with open(file_path, 'r', encoding='utf-8') as f:
    content = f.read()

old_line = "if score >= 10000.0 or total_sse > 25000.0:"
new_line = "if score >= 90000.0: # ONLY abort on physical crash (100000.0) or timeout"

if old_line in content:
    content = content.replace(old_line, new_line)
    with open(file_path, 'w', encoding='utf-8') as f:
        f.write(content)
    print("Successfully patched the early abort threshold!")
else:
    print("Could not find the target line to replace.")
