import os

file_path = r'cmaes_step_worker.py'
with open(file_path, 'r') as f:
    content = f.read()

# Find the spot after error calculation to insert the print
insert_target = "total_error += err * dt"
print_block = """total_error += err * dt

        if step % int(0.2 / dt) == 0:
            act_p_v = cset.get("pitch1").getSpeedValue(state) + cset.get("pitch2").getSpeedValue(state)
            print(f"[{worker_id}] t={t:.2f}s | Des: {np.degrees(dp):.1f} | Act: {np.degrees(p):.1f} | Des_V: {np.degrees(dp_v):.0f}/s | Act_V: {np.degrees(act_p_v):.0f}/s | Des_A: {np.degrees(dp_a):.0f}/s2")"""

if "if step % int(0.2 / dt) == 0:" not in content:
    content = content.replace(insert_target, print_block)
    with open(file_path, 'w') as f:
        f.write(content)

print("Progress prints added to cmaes_step_worker.py!")
