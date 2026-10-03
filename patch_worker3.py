import os
import re

file_path = 'cmaes_worker.py'
with open(file_path, 'r', encoding='utf-8') as f:
    content = f.read()

pattern = re.compile(
    r"kp = float\(sys\.argv\[1\]\).*?worker_name = sys\.argv\[13\].*?os\.getpid\(\)\"\}?", 
    re.DOTALL
)

new_block = """omega_n = float(sys.argv[1])
    ki = float(sys.argv[2])
    g_ton = float(sys.argv[3])
    g_sc = float(sys.argv[4])
    g_phas = float(sys.argv[5])
    kp_prop = float(sys.argv[6])
    k_gamma_dyn = float(sys.argv[7])
    k_gamma_stat = float(sys.argv[8])
    
    freq = float(sys.argv[9])
    target_gain = float(sys.argv[10])
    target_phase = float(sys.argv[11])
    
    zeta = 1.0
    I_eff = 1.0
    kp = I_eff * (omega_n ** 2)
    kd = 2.0 * zeta * I_eff * omega_n
    
    try:
        worker_name = sys.argv[12] if len(sys.argv) >= 13 else f"W-{os.getpid()}\""""

content = pattern.sub(new_block, content)

with open(file_path, 'w', encoding='utf-8') as f:
    f.write(content)

print("Forcibly updated cmaes_worker.py with regex DOTALL!")
