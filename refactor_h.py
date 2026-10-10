import sys
import re

def update_h(filename):
    with open(filename, 'r') as f:
        content = f.read()
        
    prop_inj = """    OpenSim_DECLARE_PROPERTY(trajectory_tau, double, "Time constant for the internal 2nd-order trajectory planner");\n"""
    
    idx = content.find('OpenSim_DECLARE_PROPERTY(desired_roll, double')
    if idx != -1:
        end_idx = content.find('\n', idx) + 1
        content = content[:end_idx] + prop_inj + content[end_idx:]
        
    # Remove old ones
    content = re.sub(r'\s*OpenSim_DECLARE_PROPERTY\(desired_[a-z]+_[va].*\n', '\n', content)
        
    with open(filename, 'w') as f:
        f.write(content)

if __name__ == '__main__':
    update_h(sys.argv[1])

