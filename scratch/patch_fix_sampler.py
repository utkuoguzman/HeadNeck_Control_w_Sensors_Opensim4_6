import re

# ---------------------------------------------------------------- worker
wp = "cmaes_worker_p22.py"
w = open(wp, "r", encoding="utf-8", newline="").read()

pat_ton = re.compile(r"    if raw_p0 <= 5\.0:\s+r_ton = raw_p0\s+K_ton = r_ton \* K_sc\s+else:\s+K_ton = raw_p0\s+r_ton = K_ton / K_sc")
pat_sc = re.compile(r"    if raw_z_sc <= 1\.0:\s+r_lead_sc = raw_z_sc\s+z_sc = r_lead_sc \* p_sc\s+else:\s+z_sc = raw_z_sc\s+r_lead_sc = z_sc / p_sc")
pat_ccr = re.compile(r"    if raw_z_ccr <= 1\.0:\s+r_lead_ccr = raw_z_ccr\s+z_ccr = r_lead_ccr \* p_ccr\s+else:\s+z_ccr = raw_z_ccr\s+r_lead_ccr = z_ccr / p_ccr")
for name, pat, rep in [
    ("ton", pat_ton, "    # UNCONDITIONAL ratio mapping (no value-based ratio/absolute guessing)\r\n    r_ton = raw_p0\r\n    K_ton = r_ton * K_sc"),
    ("sc", pat_sc, "    r_lead_sc = raw_z_sc\r\n    z_sc = r_lead_sc * p_sc"),
    ("ccr", pat_ccr, "    r_lead_ccr = raw_z_ccr\r\n    z_ccr = r_lead_ccr * p_ccr"),
]:
    assert len(pat.findall(w)) == 1, f"worker pattern {name} not found exactly once"
    w = pat.sub(rep, w)
open(wp, "w", encoding="utf-8", newline="").write(w)
print("worker: ratio mapping made unconditional")

# ---------------------------------------------------------------- campaign
cp = "scripts/run_phase2_4_campaign.py"
c = open(cp, "r", encoding="utf-8", newline="").read().replace("\r\n", "\n")

# 1. sampler: apply CMA_stds scaling (es.sigma_vec) everywhere
t = "    D = es.sm.D\n"
assert c.count(t) == 1
c = c.replace(t, t + "    stds = np.asarray(es.sigma_vec.scaling, dtype=float) * np.ones(len(mean))  # per-parameter CMA_stds\n")
t = "z_tpa = (B.T @ d0) / (D * sigma)"
assert c.count(t) == 1
c = c.replace(t, "z_tpa = (B.T @ (d0 / stds)) / (D * sigma)")
t = "dx = sigma * (B @ (D * zk))"
assert c.count(t) == 2, c.count(t)
c = c.replace(t, "dx = sigma * stds * (B @ (D * zk))")

# 2. physical floors + caps
t = """PHYSICAL_LOWER_LIMITS = np.array([
    0.001,  # r_ton
    0.1,    # p_ton
    0.001,  # K_sc
    0.0001, # r_lead_sc
    1.0,    # p_sc
    0.01,   # K_phas
    1.0,    # p_phas
    0.01,   # kp
    0.0001, # r_lead_ccr
    1.0,    # p_ccr
    0.01,   # k_dyn
    0.001   # k_stat
])"""
assert t in c
c = c.replace(t, """PHYSICAL_LOWER_LIMITS = np.array([
    0.05,   # r_ton   (K_ton = r_ton*K_sc; keep tonic path alive)
    0.1,    # p_ton
    0.05,   # K_sc    (floor > 0 so canal/tonic paths cannot be deleted)
    0.001,  # r_lead_sc
    1.0,    # p_sc
    0.01,   # K_phas
    1.0,    # p_phas
    0.01,   # kp
    0.001,  # r_lead_ccr
    1.0,    # p_ccr
    0.05,   # k_dyn
    0.05    # k_stat
])

# Physical ceilings: a dynamic expansion may never exceed these
PHYSICAL_UPPER_LIMITS = np.array([
    3.0,    # r_ton
    300.0,  # p_ton
    3.0,    # K_sc
    1.0,    # r_lead_sc  (zero must stay below pole -> lead)
    300.0,  # p_sc
    15.0,   # K_phas
    80.0,   # p_phas
    5.0,    # kp
    1.0,    # r_lead_ccr
    400.0,  # p_ccr
    2.0,    # k_dyn
    2.0     # k_stat
])""")

t = """            delta = 0.10 * R
            target_U = U + delta
            new_upper[i] = target_U
            expansions.append(f"{PARAM_NAMES[i]} upper expanded: {U:.4f} -> {target_U:.4f} (+10% range)")"""
assert t in c
c = c.replace(t, """            delta = 0.10 * R
            cap = PHYSICAL_UPPER_LIMITS[i]
            target_U = min(U + delta, cap)
            if target_U > U + 1e-12:
                new_upper[i] = target_U
                expansions.append(f"{PARAM_NAMES[i]} upper expanded: {U:.4f} -> {target_U:.4f} (+10% range, cap {cap})")
            else:
                expansions.append(f"{PARAM_NAMES[i]} reached physical ceiling ({cap})")""")

open(cp, "w", encoding="utf-8", newline="").write(c.replace("\n", "\r\n"))
print("campaign: sampler scaling fixed, floors/caps added")

