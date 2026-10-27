# Patch HeadNeckNeuralController.h and HeadNeckNeuralController.cpp to add Ki_task_pitch, Ki_task_roll, Ki_task_yaw

h_path = "HeadNeckNeuralController_plugin/HeadNeckNeuralController.h"
with open(h_path, "r", encoding="utf-8") as f:
    h_content = f.read()

old_h = 'OpenSim_DECLARE_PROPERTY(Ki_task, double, "Integral gain for gravity compensation (default: 6.54)");'
new_h = '''OpenSim_DECLARE_PROPERTY(Ki_task, double, "Integral gain for gravity compensation (default: 6.54)");
    OpenSim_DECLARE_PROPERTY(Ki_task_pitch, double, "Explicit Task Ki for pitch in Nm/(rad*s) (default: -1.0)");
    OpenSim_DECLARE_PROPERTY(Ki_task_roll, double, "Explicit Task Ki for roll in Nm/(rad*s) (default: 0.0)");
    OpenSim_DECLARE_PROPERTY(Ki_task_yaw, double, "Explicit Task Ki for yaw in Nm/(rad*s) (default: 0.0)");'''

assert old_h in h_content, "old_h not found in header"
h_content = h_content.replace(old_h, new_h, 1)

# Also update constructor in HeadNeckNeuralController.cpp
cpp_path = "HeadNeckNeuralController_plugin/HeadNeckNeuralController.cpp"
with open(cpp_path, "r", encoding="utf-8") as f:
    cpp_content = f.read()

old_ctor = 'constructProperty_Ki_task(6.54);'
new_ctor = '''constructProperty_Ki_task(6.54);
    constructProperty_Ki_task_pitch(-1.0);
    constructProperty_Ki_task_roll(0.0);
    constructProperty_Ki_task_yaw(0.0);'''

assert old_ctor in cpp_content, "old_ctor not found in cpp"
cpp_content = cpp_content.replace(old_ctor, new_ctor, 1)

old_tau_vol = '''    double tau_vol_raw_pitch = kp_pitch * (traj_pos_pitch - vol_rel_pitch) 
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

new_tau_vol = '''    double ki_pitch = (get_Ki_task_pitch() >= 0.0) ? get_Ki_task_pitch() : get_Ki_task();
    double ki_roll  = (get_Ki_task_roll()  >= 0.0) ? get_Ki_task_roll()  : 0.0;
    double ki_yaw   = (get_Ki_task_yaw()   >= 0.0) ? get_Ki_task_yaw()   : 0.0;

    double tau_vol_raw_pitch = kp_pitch * (traj_pos_pitch - vol_rel_pitch) 
                             + kd_pitch * (traj_vel_pitch - vol_rel_pitch_dot) 
                             + ki_pitch * p_int 
                             + I_head_pitch * traj_acc_pitch;

    double vigor_roll = 1.0 + get_k_vigor() * std::abs(traj_vel_roll);
    double kp_roll = (get_Kp_task_roll() >= 0.0) ? get_Kp_task_roll() : std::sqrt(get_Wp_base_roll() * vigor_roll);
    double kd_roll = (get_Kd_task_roll() >= 0.0) ? get_Kd_task_roll() : std::sqrt(get_Wv_base_roll() * vigor_roll + 2.0 * kp_roll);
    double tau_vol_raw_roll = kp_roll * (traj_pos_roll - vol_rel_roll) 
                            + kd_roll * (traj_vel_roll - vol_rel_roll_dot) 
                            + ki_roll * r_int 
                            + I_head_roll * traj_acc_roll;

    double vigor_yaw = 1.0 + get_k_vigor() * std::abs(traj_vel_yaw);
    double kp_yaw = (get_Kp_task_yaw() >= 0.0) ? get_Kp_task_yaw() : std::sqrt(get_Wp_base_yaw() * vigor_yaw);
    double kd_yaw = (get_Kd_task_yaw() >= 0.0) ? get_Kd_task_yaw() : std::sqrt(get_Wv_base_yaw() * vigor_yaw + 2.0 * kp_yaw);
    double tau_vol_raw_yaw = kp_yaw * (traj_pos_yaw - vol_rel_yaw) 
                           + kd_yaw * (traj_vel_yaw - vol_rel_yaw_dot) 
                           + ki_yaw * y_int 
                           + I_head_yaw * traj_acc_yaw;'''

assert old_tau_vol in cpp_content, "old_tau_vol not found in cpp"
cpp_content = cpp_content.replace(old_tau_vol, new_tau_vol, 1)

with open(h_path, "w", encoding="utf-8") as f:
    f.write(h_content)
with open(cpp_path, "w", encoding="utf-8") as f:
    f.write(cpp_content)

print("Successfully patched HeadNeckNeuralController.h and HeadNeckNeuralController.cpp!")

