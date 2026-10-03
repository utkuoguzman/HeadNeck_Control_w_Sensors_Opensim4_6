import os

with open('optimize_PID_cmaes_parallel.py', 'r', encoding='utf-8') as f:
    content = f.read()

old_lower = "[ 1.0,  0.3,  0.0,   5.0,  0.5,   2.0,    0.01,        20.0,         50.0  ]"
new_lower = "[ 1.0,  0.3, 10.0,   5.0,  0.5,   2.0,    0.01,        20.0,         50.0  ]"
content = content.replace(old_lower, new_lower)

with open('optimize_PID_cmaes_parallel.py', 'w', encoding='utf-8') as f:
    f.write(content)

print("Successfully updated Ki lower limit to 10.0!")
