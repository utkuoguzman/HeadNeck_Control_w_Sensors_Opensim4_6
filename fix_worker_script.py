import re
from pathlib import Path

file_path = Path('cmaes_worker.py')
content = file_path.read_text(encoding='utf-8')

# 1. Fix the double curly braces syntax error
content = content.replace('{{worker_id}', '{worker_id}')

# 2. Fix the Emojis that might have been mangled
content = content.replace('? [{worker_id}', '⏳ [{worker_id}')
content = content.replace('KILLED: Head collapsed', '❌ KILLED: Head collapsed')

# 3. Clean up the double conversion. 
# Inside run_simulation, it already returns gain_deg_m and phase_deg.
# So sim_gain and sim_phase are ALREADY in the correct units!
# Let's completely remove the NEW MATH block.

if '# NEW MATH' in content:
    lines = content.split('\n')
    new_lines = []
    skip = False
    for line in lines:
        if '# NEW MATH' in line:
            skip = True
        elif 'error = ((sim_gain' in line or 'error = (sim_gain' in line:
            skip = False
            # Insert the correct normalized error
            indent = line[:len(line) - len(line.lstrip())]
            new_lines.append(f"{indent}error = ((sim_gain - target_gain)/target_gain)**2 * 10000.0 + 0.5*((sim_phase - target_phase)/180.0)**2 * 10000.0")
            continue
            
        if not skip:
            new_lines.append(line)
    
    content = '\n'.join(new_lines)

file_path.write_text(content, encoding='utf-8')
print("Successfully cleaned up cmaes_worker.py!")
