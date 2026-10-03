import os

with open('optimize_PID_cmaes_parallel.py', 'r', encoding='utf-8') as f:
    content = f.read()

old_x0 = "x0 = [9.0, 0.425, 5.0, 11.2, 4.3, 2.6, 0.2, 58.7, 94.8]"
new_x0 = "x0 = [9.0, 0.425, 15.0, 11.2, 4.3, 2.6, 0.2, 58.7, 94.8]"
content = content.replace(old_x0, new_x0)

content = content.replace("(8 Dimensions)", "(9 Dimensions)")

with open('optimize_PID_cmaes_parallel.py', 'w', encoding='utf-8') as f:
    f.write(content)

print("Successfully updated x0 to match the new bounds!")
