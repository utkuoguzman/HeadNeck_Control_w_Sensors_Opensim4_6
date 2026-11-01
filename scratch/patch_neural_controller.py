# Patch HeadNeckNeuralController.h
with open('HeadNeckNeuralController_plugin/HeadNeckNeuralController.h', 'r', encoding='utf-8') as f:
    h_content = f.read()

target_h = '    OpenSim_DECLARE_PROPERTY(Wv_base_yaw, double, "Base LQR velocity cost for yaw");'
replacement_h = '''    OpenSim_DECLARE_PROPERTY(Wv_base_yaw, double, "Base LQR velocity cost for yaw");

    // Direct Task Space Stiffness and Damping (Disregards Wp/Wv when set >= 0.0)
    OpenSim_DECLARE_PROPERTY(Kp_task_pitch, double, "Explicit Task Kp for pitch in Nm/rad (overrides Wp if >= 0.0, default: -1.0)");
    OpenSim_DECLARE_PROPERTY(Kd_task_pitch, double, "Explicit Task Kd for pitch in Nms/rad (overrides Wv if >= 0.0, default: -1.0)");
    OpenSim_DECLARE_PROPERTY(Kp_task_roll, double, "Explicit Task Kp for roll in Nm/rad (overrides Wp if >= 0.0, default: -1.0)");
    OpenSim_DECLARE_PROPERTY(Kd_task_roll, double, "Explicit Task Kd for roll in Nms/rad (overrides Wv if >= 0.0, default: -1.0)");
    OpenSim_DECLARE_PROPERTY(Kp_task_yaw, double, "Explicit Task Kp for yaw in Nm/rad (overrides Wp if >= 0.0, default: -1.0)");
    OpenSim_DECLARE_PROPERTY(Kd_task_yaw, double, "Explicit Task Kd for yaw in Nms/rad (overrides Wv if >= 0.0, default: -1.0)");'''

assert target_h in h_content, 'target_h not found'
h_content = h_content.replace(target_h, replacement_h, 1)

with open('HeadNeckNeuralController_plugin/HeadNeckNeuralController.h', 'w', encoding='utf-8') as f:
    f.write(h_content)
print('Patched HeadNeckNeuralController.h successfully!')

# Patch HeadNeckNeuralController.cpp
with open('HeadNeckNeuralController_plugin/HeadNeckNeuralController.cpp', 'r', encoding='utf-8') as f:
    cpp_content = f.read()

# 1. Constructor constructProperty_
target_cpp_ctor = '    constructProperty_Wv_base_yaw(40.0);'
replacement_cpp_ctor = '''    constructProperty_Wv_base_yaw(40.0);
    constructProperty_Kp_task_pitch(-1.0);
    constructProperty_Kd_task_pitch(-1.0);
    constructProperty_Kp_task_roll(-1.0);
    constructProperty_Kd_task_roll(-1.0);
    constructProperty_Kp_task_yaw(-1.0);
    constructProperty_Kd_task_yaw(-1.0);'''

assert target_cpp_ctor in cpp_content, 'target_cpp_ctor not found'
cpp_content = cpp_content.replace(target_cpp_ctor, replacement_cpp_ctor, 1)

# 2. computeControls kp and kd calculation
target_cpp_calc = '''    double vigor_pitch = 1.0 + get_k_vigor() * std::abs(traj_vel_pitch);
    double kp_pitch = std::sqrt(get_Wp_base_pitch() * vigor_pitch);
    double kd_pitch = std::sqrt(get_Wv_base_pitch() * vigor_pitch + 2.0 * kp_pitch);
    double tau_vol_raw_pitch = kp_pitch * (traj_pos_pitch - vol_rel_pitch) 
                             + kd_pitch * (traj_vel_pitch - vol_rel_pitch_dot) 
                             + get_Ki_task() * p_int 
                             + I_head_pitch * traj_acc_pitch;

    double vigor_roll = 1.0 + get_k_vigor() * std::abs(traj_vel_roll);
    double kp_roll = std::sqrt(get_Wp_base_roll() * vigor_roll);
    double kd_roll = std::sqrt(get_Wv_base_roll() * vigor_roll + 2.0 * kp_roll);
    double tau_vol_raw_roll = kp_roll * (traj_pos_roll - vol_rel_roll) 
                            + kd_roll * (traj_vel_roll - vol_rel_roll_dot) 
                            + get_Ki_task() * r_int 
                            + I_head_roll * traj_acc_roll;

    double vigor_yaw = 1.0 + get_k_vigor() * std::abs(traj_vel_yaw);
    double kp_yaw = std::sqrt(get_Wp_base_yaw() * vigor_yaw);
    double kd_yaw = std::sqrt(get_Wv_base_yaw() * vigor_yaw + 2.0 * kp_yaw);
    double tau_vol_raw_yaw = kp_yaw * (traj_pos_yaw - vol_rel_yaw) 
                           + kd_yaw * (traj_vel_yaw - vol_rel_yaw_dot) 
                           + get_Ki_task() * y_int 
                           + I_head_yaw * traj_acc_yaw;'''

replacement_cpp_calc = '''    double vigor_pitch = 1.0 + get_k_vigor() * std::abs(traj_vel_pitch);
    double kp_pitch = (get_Kp_task_pitch() >= 0.0) ? get_Kp_task_pitch() : std::sqrt(get_Wp_base_pitch() * vigor_pitch);
    double kd_pitch = (get_Kd_task_pitch() >= 0.0) ? get_Kd_task_pitch() : std::sqrt(get_Wv_base_pitch() * vigor_pitch + 2.0 * kp_pitch);
    double tau_vol_raw_pitch = kp_pitch * (traj_pos_pitch - vol_rel_pitch) 
                             + kd_pitch * (traj_vel_pitch - vol_rel_pitch_dot) 
                             + get_Ki_task() * p_int 
                             + I_head_pitch * traj_acc_pitch;

    double vigor_roll = 1.0 + get_k_vigor() * std::abs(traj_vel_roll);
    double kp_roll = (get_Kp_task_roll() >= 0.0) ? get_Kp_task_roll() : std::sqrt(get_Wp_base_roll() * vigor_roll);
    double kd_roll = (get_Kd_task_roll() >= 0.0) ? get_Kd_task_roll() : std::sqrt(get_Wv_base_roll() * vigor_roll + 2.0 * kp_roll);
    double tau_vol_raw_roll = kp_roll * (traj_pos_roll - vol_rel_roll) 
                            + kd_roll * (traj_vel_roll - vol_rel_roll_dot) 
                            + get_Ki_task() * r_int 
                            + I_head_roll * traj_acc_roll;

    double vigor_yaw = 1.0 + get_k_vigor() * std::abs(traj_vel_yaw);
    double kp_yaw = (get_Kp_task_yaw() >= 0.0) ? get_Kp_task_yaw() : std::sqrt(get_Wp_base_yaw() * vigor_yaw);
    double kd_yaw = (get_Kd_task_yaw() >= 0.0) ? get_Kd_task_yaw() : std::sqrt(get_Wv_base_yaw() * vigor_yaw + 2.0 * kp_yaw);
    double tau_vol_raw_yaw = kp_yaw * (traj_pos_yaw - vol_rel_yaw) 
                           + kd_yaw * (traj_vel_yaw - vol_rel_yaw_dot) 
                           + get_Ki_task() * y_int 
                           + I_head_yaw * traj_acc_yaw;'''

assert target_cpp_calc in cpp_content, 'target_cpp_calc not found'
cpp_content = cpp_content.replace(target_cpp_calc, replacement_cpp_calc, 1)

with open('HeadNeckNeuralController_plugin/HeadNeckNeuralController.cpp', 'w', encoding='utf-8') as f:
    f.write(cpp_content)
print('Patched HeadNeckNeuralController.cpp successfully!')

