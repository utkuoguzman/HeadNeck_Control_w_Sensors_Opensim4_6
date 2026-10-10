import re
import sys

def get_closing_brace(content, start_idx):
    brace_count = 0
    in_str = False
    in_char = False
    in_comment = False
    in_mline_comment = False
    i = start_idx
    while i < len(content):
        c = content[i]
        if not in_str and not in_char and not in_comment and not in_mline_comment:
            if c == '/' and i+1 < len(content) and content[i+1] == '/':
                in_comment = True
                i += 1
            elif c == '/' and i+1 < len(content) and content[i+1] == '*':
                in_mline_comment = True
                i += 1
            elif c == '"':
                in_str = True
            elif c == "'":
                in_char = True
            elif c == '{':
                brace_count += 1
            elif c == '}':
                brace_count -= 1
                if brace_count == 0:
                    return i
        elif in_str and c == '"' and content[i-1] != '\\':
            in_str = False
        elif in_char and c == "'" and content[i-1] != '\\':
            in_char = False
        elif in_comment and c == '\n':
            in_comment = False
        elif in_mline_comment and c == '*' and i+1 < len(content) and content[i+1] == '/':
            in_mline_comment = False
            i += 1
        i += 1
    return -1

def update_cpp(filename):
    with open(filename, 'r') as f:
        content = f.read()

    # Replace constructor defaults
    content = re.sub(r'\s*constructProperty_desired_[a-z]+_[va]\([^\)]+\);', '', content)
    
    idx_constr = content.find('constructProperty_desired_roll(0.0);')
    if idx_constr != -1:
        end_idx = content.find('\n', idx_constr) + 1
        content = content[:end_idx] + "    constructProperty_trajectory_tau(0.1);\n" + content[end_idx:]

    content = re.sub(r'get_desired_pitch\(\)', 'traj_pos_pitch', content)
    content = re.sub(r'get_desired_pitch_v\(\)', 'traj_vel_pitch', content)
    content = re.sub(r'get_desired_pitch_a\(\)', 'traj_acc_pitch', content)

    content = re.sub(r'get_desired_roll\(\)', 'traj_pos_roll', content)
    content = re.sub(r'get_desired_roll_v\(\)', 'traj_vel_roll', content)
    content = re.sub(r'get_desired_roll_a\(\)', 'traj_acc_roll', content)

    content = re.sub(r'get_desired_yaw\(\)', 'traj_pos_yaw', content)
    content = re.sub(r'get_desired_yaw_v\(\)', 'traj_vel_yaw', content)
    content = re.sub(r'get_desired_yaw_a\(\)', 'traj_acc_yaw', content)

    injection = """
    double tau_traj = get_trajectory_tau();
    if (tau_traj < 1e-4) tau_traj = 1e-4;
    double traj_pos_pitch = getStateVariableValue(s, "traj_pos_pitch");
    double traj_vel_pitch = getStateVariableValue(s, "traj_vel_pitch");
    double traj_acc_pitch = (1.0 / (tau_traj * tau_traj)) * (get_desired_pitch() - traj_pos_pitch) - (2.0 / tau_traj) * traj_vel_pitch;
    
    double traj_pos_roll = getStateVariableValue(s, "traj_pos_roll");
    double traj_vel_roll = getStateVariableValue(s, "traj_vel_roll");
    double traj_acc_roll = (1.0 / (tau_traj * tau_traj)) * (get_desired_roll() - traj_pos_roll) - (2.0 / tau_traj) * traj_vel_roll;

    double traj_pos_yaw = getStateVariableValue(s, "traj_pos_yaw");
    double traj_vel_yaw = getStateVariableValue(s, "traj_vel_yaw");
    double traj_acc_yaw = (1.0 / (tau_traj * tau_traj)) * (get_desired_yaw() - traj_pos_yaw) - (2.0 / tau_traj) * traj_vel_yaw;
"""

    idx1 = content.find('void HeadNeckNeuralController::computeControls(const SimTK::State& s, SimTK::Vector& controls) const {')
    if idx1 != -1:
        insert_pos = content.find('{', idx1) + 1
        content = content[:insert_pos] + injection + content[insert_pos:]

    idx2 = content.find('void HeadNeckNeuralController::computeStateVariableDerivatives(const SimTK::State& s) const {')
    if idx2 != -1:
        insert_pos = content.find('{', idx2) + 1
        content = content[:insert_pos] + injection + content[insert_pos:]

        # find closing brace
        close_idx = get_closing_brace(content, content.find('{', idx2))
        
        integration = """
    setStateVariableDerivativeValue(s, "traj_pos_pitch", traj_vel_pitch);
    setStateVariableDerivativeValue(s, "traj_vel_pitch", traj_acc_pitch);
    setStateVariableDerivativeValue(s, "traj_pos_roll", traj_vel_roll);
    setStateVariableDerivativeValue(s, "traj_vel_roll", traj_acc_roll);
    setStateVariableDerivativeValue(s, "traj_pos_yaw", traj_vel_yaw);
    setStateVariableDerivativeValue(s, "traj_vel_yaw", traj_acc_yaw);
"""
        content = content[:close_idx] + integration + content[close_idx:]


    add_states = """
    addStateVariable("traj_pos_pitch", SimTK::Stage::Velocity);
    addStateVariable("traj_vel_pitch", SimTK::Stage::Velocity);
    addStateVariable("traj_pos_roll", SimTK::Stage::Velocity);
    addStateVariable("traj_vel_roll", SimTK::Stage::Velocity);
    addStateVariable("traj_pos_yaw", SimTK::Stage::Velocity);
    addStateVariable("traj_vel_yaw", SimTK::Stage::Velocity);
"""
    idx4 = content.find('addStateVariable("ton_state_pitch", SimTK::Stage::Velocity);')
    if idx4 != -1:
        content = content[:idx4] + add_states + content[idx4:]

    init_states = """
    setStateVariableValue(s, "traj_pos_pitch", 0.0);
    setStateVariableValue(s, "traj_vel_pitch", 0.0);
    setStateVariableValue(s, "traj_pos_roll", 0.0);
    setStateVariableValue(s, "traj_vel_roll", 0.0);
    setStateVariableValue(s, "traj_pos_yaw", 0.0);
    setStateVariableValue(s, "traj_vel_yaw", 0.0);
"""
    idx5 = content.find('setStateVariableValue(s, "omega_filt_yaw", 0.0);')
    if idx5 != -1:
        end_idx = content.find('\n', idx5) + 1
        content = content[:end_idx] + init_states + content[end_idx:]

    with open(filename, 'w') as f:
        f.write(content)

if __name__ == '__main__':
    update_cpp(sys.argv[1])

