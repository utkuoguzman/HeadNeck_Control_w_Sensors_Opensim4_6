import os

# 1. Update cmaes_worker.py (Delay and Fast-Kill Threshold)
with open('cmaes_worker.py', 'r', encoding='utf-8') as f:
    content = f.read()

content = content.replace("delay = 2.0", "delay = 2.5")
content = content.replace("abs(head_pitch_deg) > 30.0 or abs(head_roll_deg) > 30.0 or abs(head_yaw_deg) > 30.0", 
                          "abs(head_pitch_deg) > 45.0 or abs(head_roll_deg) > 45.0 or abs(head_yaw_deg) > 45.0")

with open('cmaes_worker.py', 'w', encoding='utf-8') as f:
    f.write(content)

# 2. Update optimize_PID_cmaes_parallel.py (Ki upper bound)
with open('optimize_PID_cmaes_parallel.py', 'r', encoding='utf-8') as f:
    opt_content = f.read()

old_bounds = "[20.0, 50.0,  25.0,  5.0,  10.0,    2.50,        80.0,        150.0  ]"
new_bounds = "[20.0, 300.0,  25.0,  5.0,  10.0,    2.50,        80.0,        150.0  ]"

opt_content = opt_content.replace(old_bounds, new_bounds)

with open('optimize_PID_cmaes_parallel.py', 'w', encoding='utf-8') as f:
    f.write(opt_content)

print("Successfully increased delay to 2.5s, Fast-Kill to 45deg, and Ki max to 300!")
