import json, re

# ---- worker: K_ton absolute & independent
wp = "cmaes_worker_p22.py"
w = open(wp, "r", encoding="utf-8", newline="").read()
old = "    # UNCONDITIONAL ratio mapping (no value-based ratio/absolute guessing)\r\n    r_ton = raw_p0\r\n    K_ton = r_ton * K_sc"
assert w.count(old) == 1
w = w.replace(old, "    # K_ton is an ABSOLUTE, independent parameter (lead ratios below stay unconditional ratios)\r\n    K_ton = raw_p0\r\n    r_ton = K_ton / K_sc")
open(wp, "w", encoding="utf-8", newline="").write(w)

# ---- campaign
cp = "scripts/run_phase2_4_campaign.py"
c = open(cp, "r", encoding="utf-8", newline="").read().replace("\r\n", "\n")
def rep(old, new, n=1):
    global c
    assert c.count(old) == n, (old, c.count(old))
    c = c.replace(old, new)

rep('    "r_ton", "p_ton", "K_sc", "r_lead_sc", "p_sc", "K_phas",', '    "K_ton", "p_ton", "K_sc", "r_lead_sc", "p_sc", "K_phas",')
rep("    0.05,   # r_ton   (K_ton = r_ton*K_sc; keep tonic path alive)", "    0.01,   # K_ton   (absolute, independent of K_sc)")
rep("    3.0,    # r_ton\n", "    3.0,    # K_ton\n")
rep("            0.5769298,   # 0: r_ton", "            0.58,        # 0: K_ton")
rep("            0.08,    # r_ton", "            0.07,    # K_ton")
rep("        r_ton = best_cand[0]\n", "        K_ton = best_cand[0]\n")
rep("        K_ton = r_ton * K_sc\n", "        r_ton = K_ton / K_sc  # reporting only\n")
open(cp, "w", encoding="utf-8", newline="").write(c.replace("\n", "\r\n"))

# ---- seed file
pp = "data/prescreen_rational_champion.json"
p = json.load(open(pp))
p["x0"][0] = 0.58; p["sigma0"][0] = 0.07
p["lower_bounds"][0] = 0.01; p["upper_bounds"][0] = 3.0
p["description"] = "Converged 12-Parameter Adaptive Seed (40,854.91 pts); K_ton absolute; sampler-fixed"
json.dump(p, open(pp, "w"), indent=2)
print("OK", p["x0"][:3], p["sigma0"][:3], p["lower_bounds"][:3], p["upper_bounds"][:3])

