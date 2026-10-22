import os

path = "cmaes_worker_p15.py"
with open(path, "r", encoding="utf-8") as f:
    text = f.read()

target = """    K_sc = float(sys.argv[6])
    z_sc = float(sys.argv[7])
    p_sc = float(sys.argv[8])
    
    # Biological Ratio Coupling: r_ton = K_ton / K_sc
    if raw_p0 <= 5.0:
        r_ton = raw_p0
        K_ton = r_ton * K_sc
    else:
        K_ton = raw_p0
        r_ton = K_ton / K_sc
    
    K_phas = float(sys.argv[9])
    p_phas = float(sys.argv[10])
    
    kp_prop = float(sys.argv[11])
    z_ccr = float(sys.argv[12])
    p_ccr = float(sys.argv[13])"""

replacement = """    K_sc = float(sys.argv[6])
    raw_z_sc = float(sys.argv[7])
    p_sc = float(sys.argv[8])
    
    # Biological Ratio Coupling: r_ton = K_ton / K_sc
    if raw_p0 <= 5.0:
        r_ton = raw_p0
        K_ton = r_ton * K_sc
    else:
        K_ton = raw_p0
        r_ton = K_ton / K_sc
        
    # Biological Lead Ratio Coupling: z_sc = r_lead_sc * p_sc
    if raw_z_sc <= 1.0:
        r_lead_sc = raw_z_sc
        z_sc = r_lead_sc * p_sc
    else:
        z_sc = raw_z_sc
        r_lead_sc = z_sc / p_sc
    
    K_phas = float(sys.argv[9])
    p_phas = float(sys.argv[10])
    
    kp_prop = float(sys.argv[11])
    raw_z_ccr = float(sys.argv[12])
    p_ccr = float(sys.argv[13])
    
    # Biological Lead Ratio Coupling: z_ccr = r_lead_ccr * p_ccr
    if raw_z_ccr <= 1.0:
        r_lead_ccr = raw_z_ccr
        z_ccr = r_lead_ccr * p_ccr
    else:
        z_ccr = raw_z_ccr
        r_lead_ccr = z_ccr / p_ccr"""

norm_text = text.replace("\r\n", "\n")
norm_target = target.replace("\r\n", "\n")
norm_replacement = replacement.replace("\r\n", "\n")

if norm_target in norm_text:
    new_norm_text = norm_text.replace(norm_target, norm_replacement)
    with open(path, "w", encoding="utf-8", newline="\r\n") as f:
        f.write(new_norm_text)
    print("cmaes_worker_p15.py successfully updated!")
else:
    print("Target string not found in cmaes_worker_p15.py")

