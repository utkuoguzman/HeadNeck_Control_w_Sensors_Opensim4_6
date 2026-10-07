import os

with open('optimize_PID_cmaes_parallel.py', 'r', encoding='utf-8') as f:
    lines = f.readlines()

for i, line in enumerate(lines):
    if "return total_sse" in line and "def " not in line and "    return total_sse" in line:
        # Check if we are in the evaluate() function and it's the final return
        if i > 0 and "    return total_sse" in line:
            # We want to insert the print statement right before the return total_sse
            # Need to get the indentation right. It's usually 4 spaces for the return.
            indent = line[:len(line) - len(line.lstrip())]
            print_stmt = indent + 'print(f"✅ SUCCESS [G{gen_idx:02d}-W{w_idx:02d}]: Completed all frequencies! Final Score = {total_sse:.1f}")\n'
            lines.insert(i, print_stmt)
            break

with open('optimize_PID_cmaes_parallel.py', 'w', encoding='utf-8') as f:
    f.writelines(lines)

print("Successfully added SUCCESS print statement!")
