import re

with open('test_scripts/fig3_ramp_stretches.py', 'r') as f:
    content = f.read()

content = content.replace('xml_content.replace("<objects/>", f"<objects>{spindle_ctrl}</objects>")', 're.sub(r"<objects\\s*/>", f"<objects>{spindle_ctrl}</objects>", xml_content)')

# Also we need to import re! It's already in the file or we can just use import re
if 'import re' not in content:
    content = 'import re\n' + content

with open('test_scripts/fig3_ramp_stretches.py', 'w') as f:
    f.write(content)
print('DONE')

