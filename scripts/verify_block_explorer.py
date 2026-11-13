import re

HTML_PATH = "docs/neuromotor_block_explorer.html"

with open(HTML_PATH, "r", encoding="utf-8") as f:
    text = f.read()

# Check tab buttons
tabs = re.findall(r'<button onclick="selectTab\(\'([^\']+)\'\)"', text)
print("Tab buttons found:", tabs)

# Check DIAGRAMS keys
d_match = re.search(r'const DIAGRAMS = \{(.*?)\n    \};', text, re.DOTALL)
if d_match:
    diag_keys = re.findall(r'^\s{6}([a-zA-Z0-9_]+):\s*\{', d_match.group(1), re.MULTILINE)
    print("DIAGRAMS keys found:", diag_keys)

# Check BLOCK_INFO keys
b_match = re.search(r'const BLOCK_INFO = \{(.*?)\n    \};', text, re.DOTALL)
if b_match:
    block_keys = re.findall(r'^\s{6}([a-zA-Z0-9_]+):\s*\{', b_match.group(1), re.MULTILINE)
    print(f"Total BLOCK_INFO keys: {len(block_keys)}")
    print("Sample keys:", block_keys[:4], "...", block_keys[-4:])

# Check presence of properness keywords
for b in ['s5_sc', 's5_phas', 's5_ton', 's5_sum', 's6_rect', 's6_ccr', 's1_pid']:
    bm = re.search(rf'{b}:\s*\{{(.*?)\n      \}},', text, re.DOTALL)
    if bm:
        has_proper = 'Strictly Proper' in bm.group(1) or 'D = 0' in bm.group(1) or 'Proper' in bm.group(1)
        has_val = any(v in bm.group(1) for v in ['4.1019', '15.4205', '5.2598', '0.4435', '3.7901', '99.9196'])
        print(f"✓ {b}: found! has_proper={has_proper}, has_calibrated_values={has_val}")
    else:
        print(f"✗ {b}: NOT FOUND in BLOCK_INFO!")
