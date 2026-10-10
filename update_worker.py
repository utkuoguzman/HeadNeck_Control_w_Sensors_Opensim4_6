import re

def update_worker():
    with open('worker_peng_target.py', 'r') as f:
        content = f.read()

    # Remove min_jerk function
    content = re.sub(r'def min_jerk\(.*?return pos, vel, acc\n', '', content, flags=re.DOTALL)
    
    # Run simulation arguments: `duration` becomes `tau`
    content = content.replace("run_simulation(Wp_base, Wv_base, k_vigor, duration, worker_name", "run_simulation(Wp_base, Wv_base, k_vigor, tau_traj, worker_name")
    
    # Update properties
    content = content.replace('osim.PropertyHelper.setValueDouble(k_vigor, controller.updPropertyByName("k_vigor"))',
                              'osim.PropertyHelper.setValueDouble(k_vigor, controller.updPropertyByName("k_vigor"))\n    osim.PropertyHelper.setValueDouble(tau_traj, controller.updPropertyByName("trajectory_tau"))')

    # Remove min_jerk in the loop and set desired_pitch once
    old_loop = """    while current_t < target_time - 1e-6:
        # The intent (reference) is a minimum jerk curve, which provides the desired velocity bell curve to trigger k_vigor
        p_pos, p_vel, p_acc = min_jerk(current_t - delay, duration, amp_p)
        
        osim.PropertyHelper.setValueDouble(p_pos, controller.updPropertyByName("desired_pitch"))
        osim.PropertyHelper.setValueDouble(p_vel, controller.updPropertyByName("desired_pitch_v"))
        osim.PropertyHelper.setValueDouble(p_acc, controller.updPropertyByName("desired_pitch_a"))

        next_t = current_t + step"""
        
    new_loop = """    while current_t < target_time - 1e-6:
        # Send a single step command at t = delay
        p_pos = amp_p if current_t >= delay else 0.0
        osim.PropertyHelper.setValueDouble(p_pos, controller.updPropertyByName("desired_pitch"))

        next_t = current_t + step"""
        
    content = content.replace(old_loop, new_loop)
    
    with open('worker_peng_target.py', 'w') as f:
        f.write(content)

if __name__ == '__main__':
    update_worker()

