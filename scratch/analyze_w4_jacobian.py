import os
import glob
import json
import math
import cmath
import numpy as np

FREQS = [0.324, 0.783, 1.099, 2.446, 5.583, 7.927]
TARGETS = [
    (0.324,  86.55,   -7.03),
    (0.783, 236.93,  -96.96),
    (1.099, 319.74, -109.23),
    (2.446, 423.34, -161.34),
    (5.583, 503.30, -201.38),
    (7.927, 572.69, -247.62)
]
PARAM_NAMES = [
    "K_ton", "p_ton", "K_sc", "z_sc", "p_sc",
    "K_phas", "p_phas", "kp_prop", "z_ccr", "p_ccr",
    "K_gamma_dyn", "K_gamma_stat"
]

def eval_raw_bode(freq_hz, th):
    omega = 2.0 * math.pi * freq_hz
    s = complex(0.0, omega)
    s2 = complex(-omega * omega, 0.0)

    D_vcr = cmath.exp(-s * 0.010)
    D_ccr = cmath.exp(-s * 0.015)
    D_vol = cmath.exp(-s * 0.040)
    H_act = 1.0 / ((1.0 + 0.012 * s) * (1.0 + 0.035 * s))

    I_pivot = 0.0953
    B_pass  = 0.35
    K_pass  = 3.80
    Ki_pitch = 40.2786
    omega_vol = 2.0 * math.pi * 2.0
    Z_int = (Ki_pitch / s) * (omega_vol / (s + omega_vol)) * D_vol
    D_mech = I_pivot * s2 + B_pass * s + K_pass + Z_int

    K_ton = max(1e-4, th[0])
    p_ton = max(1.0,  th[1])
    Z_ton = K_ton * (p_ton / (s + p_ton)) * D_vcr * H_act

    K_sc = max(1e-4, th[2])
    z_sc = max(1e-3, th[3])
    p_sc = max(1.0,  th[4])
    norm_sc = 1.0 / math.sqrt(max(1e-4, z_sc / p_sc))
    H_canal = 1.0 + 0.02442 * s
    Z_sc = 18.0 * K_sc * norm_sc * ((s + z_sc) / (s + p_sc)) * H_canal * s * D_vcr * H_act

    K_phas = max(1e-4, th[5])
    p_phas = max(0.5,  th[6])
    r_neck = 0.13
    H_phas = K_phas * (p_phas / (s + p_phas)) * D_vcr * H_act
    Z_phas_rot = -r_neck * s2 * H_phas

    kp_prop = max(1e-4, th[7])
    z_ccr   = max(1e-3, th[8])
    p_ccr   = max(1.0,  th[9])
    k_gdyn  = max(0.01, th[10])
    k_gstat = max(0.01, th[11])
    norm_ccr = 1.0 / math.sqrt(max(1e-4, z_ccr / p_ccr))
    H_spindle = 9.5 * ((1.0 + 0.35 * k_gstat) + (0.055 + 0.045 * k_gdyn) * s)
    Z_ccr = kp_prop * norm_ccr * ((s + z_ccr) / (s + p_ccr)) * H_spindle * D_ccr * H_act

    D_CL = D_mech + Z_ton + Z_sc + Z_phas_rot + Z_ccr
    m_head_r = 4.30 * 0.13
    N_CL = (-s2) * (m_head_r + 0.025 * H_phas) * (180.0 / math.pi)
    return N_CL / D_CL

files = glob.glob("logs/candidates/W*_P24G*.json") + glob.glob("logs/candidates/W_ADAPT_LYAP*.json") + glob.glob("logs/candidates/W_NEW_KP0*.json")
by_worker = {}
for f in files:
    base = os.path.basename(f)
    if "Hz.json" in base:
        w = base.rsplit("_", 1)[0]
        d = json.load(open(f, encoding="utf-8"))
        by_worker.setdefault(w, {}).setdefault("freqs", {})[d["freq"]] = d
        if "params" in d:
            by_worker[w]["params"] = d["params"]
    else:
        w = base[:-5]
        d = json.load(open(f, encoding="utf-8"))
        if "params" in d:
            by_worker.setdefault(w, {})["params"] = d["params"]

clean_workers = []
for w, v in by_worker.items():
    if "params" in v and len(v.get("freqs", {})) == 6:
        tot = sum(x["score"] for x in v["freqs"].values())
        max_g = max(x["sim_gain"] for x in v["freqs"].values())
        if tot < 100000 and max_g < 40000:
            clean_workers.append((tot, w))
clean_workers.sort()

w_anchor = "W4_P24G3"
th_anchor = np.array(by_worker[w_anchor]["params"], dtype=float)

def get_residual_vec(w):
    # 12-vector: 6 gain_db_diff / 1.0, 6 phase_diff / 10.0
    res = np.zeros(12)
    for k, (f, tg, tp) in enumerate(TARGETS):
        d = by_worker[w]["freqs"][f]
        gdb = d["gain_db_diff"]
        sp = d["sim_phase"]
        pdiff = (sp - tp + 180.0) % 360.0 - 180.0
        res[k] = gdb / 1.0
        res[k + 6] = pdiff / 10.0
    return res

r_anchor = get_residual_vec(w_anchor)
print(f"Anchor {w_anchor} score = {100.0 * np.sum(r_anchor**2):.2f} pts")
for k, (f, tg, tp) in enumerate(TARGETS):
    print(f"  {f:.3f} Hz: gain_err = {r_anchor[k]:+6.2f} dB ({100*r_anchor[k]**2:6.1f} pts), phase_err = {r_anchor[k+6]*10:+6.2f} deg ({100*r_anchor[k+6]**2:6.1f} pts)")

# Build normalized design matrix X = (th - th_anchor) / th_anchor and delta residual Y = r - r_anchor
X_list = []
Y_list = []
W_weights = []
names_list = []

for tot, w in clean_workers:
    if w == w_anchor:
        continue
    th = np.array(by_worker[w]["params"], dtype=float)
    dx = (th - th_anchor) / th_anchor
    dist = np.linalg.norm(dx)
    r = get_residual_vec(w)
    # Unwrap phase change relative to r_anchor so there are no +-360 jumps
    dr = r - r_anchor
    for k in range(6):
        while dr[k + 6] > 18.0: dr[k + 6] -= 36.0
        while dr[k + 6] < -18.0: dr[k + 6] += 36.0
    X_list.append(dx)
    Y_list.append(dr)
    # Gaussian kernel weight centered at W4_P24G3
    W_weights.append(math.exp(-0.5 * (dist / 0.35)**2) * (20000.0 / max(12000.0, tot)))
    names_list.append((tot, dist, w))

X = np.array(X_list) # (N, 12)
Y = np.array(Y_list) # (N, 12)
W = np.array(W_weights) # (N,)

# Also compute analytical Space-Mapped Jacobian J_ana at th_anchor (12 residuals x 12 normalized params)
J_ana = np.zeros((12, 12))
for j in range(12):
    d_th = 1e-4 * abs(th_anchor[j])
    th_p = th_anchor.copy(); th_p[j] += d_th
    th_m = th_anchor.copy(); th_m[j] -= d_th
    for k, (f, tg, tp) in enumerate(TARGETS):
        Gp = eval_raw_bode(f, th_p)
        Gm = eval_raw_bode(f, th_m)
        d_lnG = cmath.log(Gp / Gm) / (2.0 * d_th)
        J_ana[k, j]     = th_anchor[j] * (20.0 / math.log(10.0)) * d_lnG.real / 1.0
        J_ana[k + 6, j] = th_anchor[j] * (180.0 / math.pi) * d_lnG.imag / 10.0

# Fit Empirical Weighted Ridge Jacobian with J_ana prior:
# min_J || diag(sqrt(W)) (X J^T - Y) ||_F^2 + alpha * || J^T - gamma * J_ana^T ||_F^2
Sw = np.diag(np.sqrt(W))
Xw = Sw @ X
Yw = Sw @ Y

# Pure empirical Ridge Jacobian (shrunk toward 0)
alpha_emp = 1.5
J_emp_T = np.linalg.solve(Xw.T @ Xw + alpha_emp * np.eye(12), Xw.T @ Yw)
J_emp = J_emp_T.T

# Hybrid Space-Mapped + Empirical Broyden/Ridge Jacobian (shrunk toward 0.5 * J_ana)
alpha_hyb = 1.5
J_hyb_T = np.linalg.solve(Xw.T @ Xw + alpha_hyb * np.eye(12), Xw.T @ Yw + alpha_hyb * 0.35 * J_ana.T)
J_hyb = J_hyb_T.T

print("\nComparison of Empirical OpenSim vs Analytical Space-Mapped Sensitivities at W4_P24G3:")
for k, (f, _, _) in enumerate(TARGETS):
    print(f"  Freq {f:.3f} Hz Gain dB/d(ln th):")
    print("    Emp:", " ".join(f"{PARAM_NAMES[j]}:{J_emp[k,j]:+5.2f}" for j in [0,2,3,4,5,7,10,11]))
    print("    Ana:", " ".join(f"{PARAM_NAMES[j]}:{J_ana[k,j]:+5.2f}" for j in [0,2,3,4,5,7,10,11]))
    print(f"  Freq {f:.3f} Hz Phase (10deg)/d(ln th):")
    print("    Emp:", " ".join(f"{PARAM_NAMES[j]}:{J_emp[k+6,j]:+5.2f}" for j in [0,2,3,4,5,7,10,11]))
    print("    Ana:", " ".join(f"{PARAM_NAMES[j]}:{J_ana[k+6,j]:+5.2f}" for j in [0,2,3,4,5,7,10,11]))

