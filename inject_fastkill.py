import os
import re

file_path = r'cmaes_worker.py'
with open(file_path, 'r', encoding='utf-8') as f:
    content = f.read()

# Add the early abort logic inside the while loop
abort_logic = """
        t_history.append(current_t)
        pitch_history.append(p1 + p2)
        
        # EARLY ABORT: If the head collapses or explodes, kill the simulation instantly!
        if abs(head_pitch_deg) > 30.0 or abs(head_roll_deg) > 30.0 or abs(head_yaw_deg) > 30.0:
            print(f"[{worker_id} | {frequency:.2f}Hz] KILLED: Head collapsed! P:{head_pitch_deg:.1f} R:{head_roll_deg:.1f} Y:{head_yaw_deg:.1f}", file=sys.stderr, flush=True)
            print("100000.0")
            sys.exit(0)
"""

content = re.sub(
    r"\s*t_history\.append\(current_t\)\s*pitch_history\.append\(p1 \+ p2\)",
    abort_logic,
    content
)

with open(file_path, 'w', encoding='utf-8') as f:
    f.write(content)

print("Injected Fast-Kill logic into cmaes_worker.py!")
