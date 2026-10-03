import os

# 1. Update test_pitch_minjerk.py
test_path = r'test_scripts\test_pitch_minjerk.py'
with open(test_path, 'r') as f:
    content = f.read()
content = content.replace("q[0]", "q.get(0)").replace("q[1]", "q.get(1)").replace("q[2]", "q.get(2)").replace("q[3]", "q.get(3)")
with open(test_path, 'w') as f:
    f.write(content)

# 2. Update cmaes_worker.py
worker_path = r'cmaes_worker.py'
with open(worker_path, 'r') as f:
    content = f.read()
content = content.replace("q[0]", "q.get(0)").replace("q[1]", "q.get(1)").replace("q[2]", "q.get(2)").replace("q[3]", "q.get(3)")
with open(worker_path, 'w') as f:
    f.write(content)

print("Fixed Quaternion indexing!")
