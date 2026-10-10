import re

def update_opt():
    with open('optimize_peng_target.py', 'r') as f:
        content = f.read()

    content = content.replace("4. T_duration (Brain's internal planned min-jerk duration in seconds)", "4. tau_traj (Brain's internal 2nd-order critically damped trajectory planner time constant)")
    content = content.replace("T_duration", "tau_traj")
    
    with open('optimize_peng_target.py', 'w') as f:
        f.write(content)

if __name__ == '__main__':
    update_opt()

