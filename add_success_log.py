from pathlib import Path

p = Path('optimize_6_independent.py')
content = p.read_text(encoding='utf-8')

old_block = """            if score >= 90000.0:
                print(f"🛑 POOR FIT [F{freq_idx+1}-W{w_idx+1}] at {freq:.2f}Hz (Score: {score:.1f})")
            return freq_idx, w_idx, score"""

new_block = """            if score >= 90000.0:
                print(f"🛑 POOR FIT [F{freq_idx+1}-W{w_idx+1}] at {freq:.2f}Hz (Score: {score:.1f})")
            else:
                print(f"✅ SUCCESS [F{freq_idx+1}-W{w_idx+1}] at {freq:.2f}Hz (Score: {score:.1f})")
            return freq_idx, w_idx, score"""

if old_block in content:
    content = content.replace(old_block, new_block)
    p.write_text(content, encoding='utf-8')
    print("Success patch applied!")
else:
    print("Could not find the block to patch. Maybe it was already patched?")
