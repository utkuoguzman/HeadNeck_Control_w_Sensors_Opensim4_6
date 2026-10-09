import re

filepath = r"d:\Akademik\PhD_Thesis\OpenSim\HeadNeck_Control_w_Sensors_Opensim4_6\HeadNeckNeuralController_plugin\HeadNeckNeuralController.cpp"

with open(filepath, "r") as f:
    content = f.read()

# Find the start and end of computeStateVariableDerivatives
start_str = "void HeadNeckNeuralController::computeStateVariableDerivatives(const SimTK::State& s) const {"
end_str = "void HeadNeckNeuralController::computeControls(const SimTK::State& s, SimTK::Vector& controls) const {"

start_idx = content.find(start_str)
end_idx = content.find(end_str)

func_body = content[start_idx:end_idx]

# We need to insert the reads right after the start_str and model declaration.
reads = """
    // -------------------------------------------------------------------------
    // READ ALL STATE VARIABLES BEFORE SETTING ANY DERIVATIVES
    // (OpenSim cache rule: writing derivatives dirties the cache for reads)
    // -------------------------------------------------------------------------
    double canal_states[6][2][2];
    for(int i=0; i<6; ++i) {
        for(int j=0; j<2; ++j) {
            std::string prefix = "canal_" + std::to_string(i) + "_filter_" + std::to_string(j);
            canal_states[i][j][0] = getStateVariableValue(s, prefix + "_state_0");
            canal_states[i][j][1] = getStateVariableValue(s, prefix + "_state_1");
        }
    }
    double p_int = getStateVariableValue(s, "pitch_error_integral");
    double r_int = getStateVariableValue(s, "roll_error_integral");
    double y_int = getStateVariableValue(s, "yaw_error_integral");
    
    double local_eff_tau_pitch = getStateVariableValue(s, "eff_tau_pitch");
    double local_eff_tau_roll = getStateVariableValue(s, "eff_tau_roll");
    double local_eff_tau_yaw = getStateVariableValue(s, "eff_tau_yaw");

    double tau_dist_pitch = getStateVariableValue(s, "eso_tau_dist_pitch");
    double tau_dist_roll = getStateVariableValue(s, "eso_tau_dist_roll");
    double tau_dist_yaw = getStateVariableValue(s, "eso_tau_dist_yaw");

    double local_obs_q0 = getStateVariableValue(s, "obs_q0");
    double local_obs_q1 = getStateVariableValue(s, "obs_q1");
    double local_obs_q2 = getStateVariableValue(s, "obs_q2");
    double local_obs_q3 = getStateVariableValue(s, "obs_q3");

    double local_obs_vx = getStateVariableValue(s, "obs_vx");
    double local_obs_vy = getStateVariableValue(s, "obs_vy");
    double local_obs_vz = getStateVariableValue(s, "obs_vz");

    double local_obs_px = getStateVariableValue(s, "obs_px");
    double local_obs_py = getStateVariableValue(s, "obs_py");
    double local_obs_pz = getStateVariableValue(s, "obs_pz");
    // -------------------------------------------------------------------------
"""

# Replace canal filter reads
func_body = re.sub(
    r'double x1 = getStateVariableValue\(s, prefix \+ "_state_0"\);',
    r'double x1 = canal_states[i][j][0];',
    func_body
)
func_body = re.sub(
    r'double x2 = getStateVariableValue\(s, prefix \+ "_state_1"\);',
    r'double x2 = canal_states[i][j][1];',
    func_body
)

# Replace PID Integrators
func_body = re.sub(
    r'double p_int = getStateVariableValue\(s, "pitch_error_integral"\);',
    r'',
    func_body
)
func_body = re.sub(
    r'double r_int = getStateVariableValue\(s, "roll_error_integral"\);',
    r'',
    func_body
)
func_body = re.sub(
    r'double y_int = getStateVariableValue\(s, "yaw_error_integral"\);',
    r'',
    func_body
)

# Replace eff_tau
func_body = re.sub(
    r'double eff_tau_pitch = getStateVariableValue\(s, "eff_tau_pitch"\);',
    r'double eff_tau_pitch = local_eff_tau_pitch;',
    func_body
)
func_body = re.sub(
    r'double eff_tau_roll = getStateVariableValue\(s, "eff_tau_roll"\);',
    r'double eff_tau_roll = local_eff_tau_roll;',
    func_body
)
func_body = re.sub(
    r'double eff_tau_yaw = getStateVariableValue\(s, "eff_tau_yaw"\);',
    r'double eff_tau_yaw = local_eff_tau_yaw;',
    func_body
)

# Replace eso_tau_dist
func_body = re.sub(
    r'double tau_dist_pitch = getStateVariableValue\(s, "eso_tau_dist_pitch"\);',
    r'',
    func_body
)
func_body = re.sub(
    r'double tau_dist_roll = getStateVariableValue\(s, "eso_tau_dist_roll"\);',
    r'',
    func_body
)
func_body = re.sub(
    r'double tau_dist_yaw = getStateVariableValue\(s, "eso_tau_dist_yaw"\);',
    r'',
    func_body
)

# Replace obs_q
func_body = re.sub(r'double q0 = getStateVariableValue\(s, "obs_q0"\);', r'double q0 = local_obs_q0;', func_body)
func_body = re.sub(r'double q1 = getStateVariableValue\(s, "obs_q1"\);', r'double q1 = local_obs_q1;', func_body)
func_body = re.sub(r'double q2 = getStateVariableValue\(s, "obs_q2"\);', r'double q2 = local_obs_q2;', func_body)
func_body = re.sub(r'double q3 = getStateVariableValue\(s, "obs_q3"\);', r'double q3 = local_obs_q3;', func_body)

# Replace obs_v
func_body = re.sub(r'double vx = getStateVariableValue\(s, "obs_vx"\);', r'double vx = local_obs_vx;', func_body)
func_body = re.sub(r'double vy = getStateVariableValue\(s, "obs_vy"\);', r'double vy = local_obs_vy;', func_body)
func_body = re.sub(r'double vz = getStateVariableValue\(s, "obs_vz"\);', r'double vz = local_obs_vz;', func_body)

# Replace obs_p
func_body = re.sub(r'double px = getStateVariableValue\(s, "obs_px"\);', r'double px = local_obs_px;', func_body)
func_body = re.sub(r'double py = getStateVariableValue\(s, "obs_py"\);', r'double py = local_obs_py;', func_body)
func_body = re.sub(r'double pz = getStateVariableValue\(s, "obs_pz"\);', r'double pz = local_obs_pz;', func_body)

# Also K_eff_pitch
func_body = re.sub(
    r'SimTK::Vec3 a_expected\(K_eff_pitch \* getStateVariableValue\(s, "eff_tau_pitch"\), 0\.0, 0\.0\);',
    r'SimTK::Vec3 a_expected(K_eff_pitch * local_eff_tau_pitch, 0.0, 0.0);',
    func_body
)

# Insert the reads block at the start of the function body
insert_pos = func_body.find("const Model& model = getModel();") + len("const Model& model = getModel();")
func_body = func_body[:insert_pos] + "\n" + reads + func_body[insert_pos:]

# Replace the original content
new_content = content[:start_idx] + func_body + content[end_idx:]

with open(filepath, "w") as f:
    f.write(new_content)

print("Refactored HeadNeckNeuralController.cpp to group all getStateVariableValue calls at the top.")

