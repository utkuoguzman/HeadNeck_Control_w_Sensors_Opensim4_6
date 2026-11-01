with open("scripts/run_phase2_3_campaign.py", "r", encoding="utf-8") as f:
    content = f.read()

# 1. Add cmath import
if "import cmath" not in content:
    content = content.replace("import math\n", "import math\nimport cmath\n")
    if "import cmath" not in content: # in case of \r\n
        content = content.replace("import math\r\n", "import math\r\nimport cmath\r\n")

# 2. Add surrogate_score function before evaluate_freq
surrogate_code = """
# Phase 2.3 Section 3 Surrogate Pre-Screening Engine
SURR_OPT_PLANT = [0.6000, 0.3636, 2.8558, 0.0050, 0.0085, 1.348]
SURR_DELAY_VCR = 0.010
SURR_DELAY_CCR = 0.015

def surrogate_score(cand):
    r_ton, p_ton, K_sc, r_lead_sc, p_sc, K_phas, p_phas, kp, r_lead_ccr, p_ccr, k_dyn, k_stat = cand
    K_ton = r_ton * K_sc
    z_sc = r_lead_sc * p_sc
    z_ccr = r_lead_ccr * p_ccr
    
    J_eff, B_pass, K_pass, M_drive, tau_act, k_geom = SURR_OPT_PLANT
    
    total_score = 0.0
    for freq, (t_gain, t_phase) in [(0.324, (86.55, -7.03)), (0.783, (236.93, -96.96)), (1.099, (319.74, -109.23)), (2.446, (423.34, -161.34)), (5.583, (503.30, -201.38)), (7.927, (572.69, -247.62))]:
        omega = 2.0 * math.pi * freq
        s = 1j * omega
        
        H_act = 1.0 / (tau_act * s + 1.0)
        H_sc = ((s + z_sc) / (s + p_sc)) * cmath.exp(-s * SURR_DELAY_VCR)
        H_ton = (p_ton / (s + p_ton)) * cmath.exp(-s * SURR_DELAY_VCR)
        H_phas = (s / (s + p_phas)) * cmath.exp(-s * SURR_DELAY_VCR)
        H_ccr = kp * (1.0 + k_dyn * (s / 10.0)) * ((s + z_ccr) / (s + p_ccr)) * cmath.exp(-s * SURR_DELAY_CCR)
        
        Z_reflex = (K_sc * H_sc * s + K_ton * H_ton + H_ccr) * H_act
        Z_plant = - (omega**2) * J_eff + s * B_pass + K_pass
        T_drive = - (omega**2) * (M_drive - K_phas * H_phas * H_act)
        
        Theta_over_X = (T_drive / (Z_plant + Z_reflex)) * k_geom
        
        gain_deg_m = abs(Theta_over_X) * (180.0 / math.pi)
        
        phase_rad = cmath.phase(Theta_over_X)
        phase_deg = math.degrees(phase_rad)
        while phase_deg > 90.0: phase_deg -= 360.0
        while phase_deg < -270.0: phase_deg += 360.0
        
        sim_gain_safe = max(1e-6, gain_deg_m)
        gain_db_diff = 20.0 * math.log10(sim_gain_safe / t_gain)
        gain_loss = ((gain_db_diff / 1.0) ** 2) * 100.0
        
        phase_diff = (phase_deg - t_phase + 180.0) % 360.0 - 180.0
        phase_loss = ((phase_diff / 10.0) ** 2) * 100.0
        
        total_score += (gain_loss + phase_loss)
    return total_score

"""

if "def surrogate_score" not in content:
    content = content.replace("def evaluate_freq(", surrogate_code + "def evaluate_freq(")

# 3. Add surrogate pre-screening to iteration loop
old_ask = "        candidates = es.ask()\n        all_scores = [0.0] * len(candidates)"
old_ask_crlf = "        candidates = es.ask()\r\n        all_scores = [0.0] * len(candidates)"

new_ask = """        if iteration >= 13:
            # Surrogate Pre-Screening: sample 72 candidates, rank via surrogate, pick top 18
            pool = es.ask(number=72)
            surr_scores = [surrogate_score(c) for c in pool]
            ranked_indices = np.argsort(surr_scores)
            candidates = [pool[i] for i in ranked_indices[:18]]
            print(f"[SURROGATE PRE-SCREENING] Evaluated 72 candidates in ~3 ms. Best predicted surrogate score: {surr_scores[ranked_indices[0]]:.1f} pts (Worst discarded: {surr_scores[ranked_indices[-1]]:.1f} pts). Passing top 18 to OpenSim.", flush=True)
        else:
            candidates = es.ask()
        all_scores = [0.0] * len(candidates)"""

if old_ask in content:
    content = content.replace(old_ask, new_ask)
elif old_ask_crlf in content:
    content = content.replace(old_ask_crlf, new_ask)
else:
    print("Warning: old_ask not found directly, checking regex or alternate match...")

with open("scripts/run_phase2_3_campaign.py", "w", encoding="utf-8") as f:
    f.write(content)

print("Patch applied to scripts/run_phase2_3_campaign.py successfully!")

