import subprocess, sys, json

with open('logs/candidates/W7_P24G1.json') as fp:
    cand = json.load(fp)

# In candidate json:
# params are [K_ton, p_ton, K_sc, z_sc, p_sc, K_phas, p_phas, kp_prop, z_ccr, p_ccr, k_dyn, k_stat]
# But cmaes_worker_p22 expects candidate vector from CMA space:
# [r_ton, p_ton, K_sc, r_lead_sc, p_sc, K_phas, p_phas, kp, r_lead_ccr, p_ccr, k_dyn, k_stat]
p = cand['params']
K_ton, p_ton, K_sc, z_sc, p_sc, K_phas, p_phas, kp, z_ccr, p_ccr, k_dyn, k_stat = p
r_ton = K_ton / K_sc
r_lead_sc = z_sc / p_sc
r_lead_ccr = z_ccr / p_ccr

cand_cma = [r_ton, p_ton, K_sc, r_lead_sc, p_sc, K_phas, p_phas, kp, r_lead_ccr, p_ccr, k_dyn, k_stat]

freq = 0.783
t_gain = 236.93
t_phase = -96.96

cmd = [
    sys.executable, "cmaes_worker_p22.py",
    "22.0", "0.707", "40.2786",
    *[str(v) for v in cand_cma],
    str(freq), str(t_gain), str(t_phase),
    "W_TEST"
]

print("Running test worker with gentle gains...")
res = subprocess.run(cmd, capture_output=True, text=True)
print("Return code:", res.returncode)
print("Stdout:\n", res.stdout)
if res.stderr:
    print("Stderr:\n", res.stderr[-500:])

