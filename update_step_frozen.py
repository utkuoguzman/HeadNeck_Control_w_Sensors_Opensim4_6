import os
import re

file_path = r'run_cmaes_step.py'
with open(file_path, 'r') as f:
    content = f.read()

# Replace the Frozen variables block
old_block = """# Frozen reflex parameters (Round 2 Best, Gen 19):
FROZEN_G_TON = 16.84
FROZEN_G_SC = 3.13
FROZEN_G_PHAS = 6.22
FROZEN_KP_PROP = 0.56
FROZEN_K_GAMMA_DYN = 38.11
FROZEN_K_GAMMA_STAT = 87.47"""

new_block = """# Frozen reflex parameters (Round 2c: Gen 8 Biologically Excellent):
FROZEN_G_TON = 8.76
FROZEN_G_SC = 4.33
FROZEN_G_PHAS = 6.06
FROZEN_KP_PROP = 0.441
FROZEN_K_GAMMA_DYN = 52.36
FROZEN_K_GAMMA_STAT = 92.37"""

content = content.replace(old_block, new_block)
with open(file_path, 'w') as f:
    f.write(content)
print("Updated run_cmaes_step.py!")
