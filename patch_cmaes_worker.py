import os

with open('cmaes_worker.py', 'r', encoding='utf-8') as f:
    lines = f.readlines()

for i, line in enumerate(lines):
    # 1. Add Emojis to 0.2s print
    if 'print(f"[{worker_id} | {frequency:.2f}Hz] t={current_t:.2f}s' in line:
        lines[i] = line.replace('print(f"[', 'print(f"⏳ [{')
    
    # 2. Add Emojis to KILLED print
    if 'print(f"[{worker_id} | {frequency:.2f}Hz] KILLED:' in line:
        lines[i] = line.replace('print(f"[', 'print(f"❌ [{')

    # 3. Modify the Error Calculation to handle deg/m and degrees
    if 'error = (sim_gain - target_gain)**2 + 0.5*(sim_phase - target_phase)**2' in line:
        # We need to insert the math conversions before this line
        indent = line[:len(line) - len(line.lstrip())]
        
        conversion_code = f"""{indent}# NEW MATH: Convert amplitude_rad to deg/m, and phase_rad to degrees
{indent}sled_amplitude = 0.0215 * (0.835 / freq)**2
{indent}sim_gain_deg = sim_gain * (180.0 / math.pi)
{indent}sim_gain_deg_per_m = sim_gain_deg / sled_amplitude
{indent}sim_phase_deg = sim_phase * (180.0 / math.pi)

{indent}# Compute Error using the correct units
{indent}error = ((sim_gain_deg_per_m - target_gain)/target_gain)**2 * 10000.0 + 0.5*((sim_phase_deg - target_phase)/180.0)**2 * 10000.0
"""
        lines[i] = conversion_code

with open('cmaes_worker.py', 'w', encoding='utf-8') as f:
    f.writelines(lines)

print("Patched cmaes_worker.py with Emojis and Unit Conversions!")
