import os
import re

file_path = r'cmaes_step_worker.py'
with open(file_path, 'r') as f:
    content = f.read()

# The old print block to replace
old_block_pattern = r"if step % int\(0\.2 / dt\) == 0:.*?print\(f\"\[\{worker_id\}\] t=\{t:\.2f\}s \| Des: \{np\.degrees\(dp\):\.1f\} \| Act: \{np\.degrees\(p\):\.1f\} \| Des_V: \{np\.degrees\(dp_v\):\.0f\}/s \| Act_V: \{np\.degrees\(act_p_v\):\.0f\}/s \| Des_A: \{np\.degrees\(dp_a\):\.0f\}/s2\"\)"

new_block = """if step % int(0.2 / dt) == 0:
            if t >= 5.0:
                des_str = f"Des(Y): {np.degrees(dy):>6.1f}"
            elif t >= 3.0:
                des_str = f"Des(R): {np.degrees(dr):>6.1f}"
            elif t >= 1.0:
                des_str = f"Des(P): {np.degrees(dp):>6.1f}"
            else:
                des_str = f"Des(-): {0.0:>6.1f}"
                
            print(f"[{worker_id}] t={t:>4.2f}s | {des_str} | Act_P: {np.degrees(p):>6.1f} | Act_R: {np.degrees(r):>6.1f} | Act_Y: {np.degrees(y):>6.1f}")"""

content = re.sub(old_block_pattern, new_block, content, flags=re.DOTALL)

with open(file_path, 'w') as f:
    f.write(content)

print("Updated print formatting in cmaes_step_worker.py!")
