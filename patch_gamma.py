with open('test_scripts/fig3_ramp_stretches.py', 'r') as f:
    text = f.read()

text = text.replace('{gamma["stat"]}', 'GAM_STAT')
text = text.replace('{gamma["dyn"]}', 'GAM_DYN')

# Counteract the bug in the C++ plugin where actControls[1] (static) is mapped to ctrlVec[1] (which the spindle thinks is dynamic)
text = text.replace('GAM_STAT', '{gamma["dyn"]}')
text = text.replace('GAM_DYN', '{gamma["stat"]}')

with open('test_scripts/fig3_ramp_stretches.py', 'w') as f:
    f.write(text)
