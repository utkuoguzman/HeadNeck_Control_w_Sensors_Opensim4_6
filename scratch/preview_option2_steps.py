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

def get_residual_vec(w):
    res = np.zeros(12)
    for k, (f, tg, tp) in enumerate(TARGETS):
        d = by_worker[w]["freqs"][f]
        gdb = d["gain_db_diff"]
        sp = d["sim_phase"]
        pdiff = (sp - tp + 180.0) % 360.0 - 180.0
        res[k] = gdb / 1.0
        res[k + 6] = pdiff / 10.0
    return res

clean_workers = []
for w, v in by_worker.items():
    if "params" in v and len(v.get("freqs", {})) == 6:
        tot = sum(x["score"] for x in v["freqs"].values())
        max_g = max(x["sim_gain"] for x in v["freqs"].values())
        if tot < 45000 and max_g < 40000:
            clean_workers.append((tot, w))
clean_workers.sort()
print(f"Using {len(clean_workers)} clean OpenSim evaluations (< 45,000 pts)")

w_anchor = "W4_P24G3"
th_anchor = np.array(by_worker[w_anchor]["params"], dtype=float)
r_anchor = get_residual_vec(w_anchor)

X_list, Y_list, W_list = [], [], []
for tot, w in clean_workers:
    if w == w_anchor:
        continue
    th = np.array(by_worker[w]["params"], dtype=float)
    dx = (th - th_anchor) / th_anchor
    dist = np.linalg.norm(dx)
    r = get_residual_vec(w)
    dr = r - r_anchor
    for k in range(6):
        while dr[k + 6] > 18.0: dr[k + 6] -= 36.0
        while dr[k + 6] < -18.0: dr[k + 6] += 36.0
    X_list.append(dx)
    Y_list.append(dr)
    # Weight strongly by proximity to W4_P24G3 and low score
    w_val = math.exp(-0.5 * (dist / 0.30)**2) * (15000.0 / max(12000.0, tot))**1.5
    W_list.append(w_val)

X = np.array(X_list)
Y = np.array(Y_list)
W = np.array(W_list)
Sw = np.diag(np.sqrt(W))
Xw = Sw @ X
Yw = Sw @ Y

# Fit local Jacobian J (12 residuals x 12 normalized step dx)
alpha_reg = 0.8
J_T = np.linalg.solve(Xw.T @ Xw + alpha_reg * np.eye(12), Xw.T @ Yw)
J = J_T.T

# Test Trust-Region Gauss-Newton steps with different trust-region damping lambda_tr
# and enforce safety constraints: kp_prop >= 0.310, K_phas <= 5.20
for lam_tr in [0.5, 1.5, 3.0, 6.0]:
    dx = -np.linalg.solve(J.T @ J + lam_tr * np.eye(12), J.T @ r_anchor)
    # Clip max relative change per parameter to 15%
    dx = np.clip(dx, -0.15, 0.15)
    th_new = th_anchor * (1.0 + dx)
    # Safety bounds from abort analysis
    th_new[7] = max(0.310, th_new[7]) # kp_prop >= 0.310
    th_new[5] = min(5.15,  th_new[5]) # K_phas <= 5.15
    dx_safe = (th_new - th_anchor) / th_anchor
    r_pred = r_anchor + J @ dx_safe
    pred_score = 100.0 * np.sum(r_pred**2)
    print(f"\nTrust-Region lambda={lam_tr:4.1f} | ||dx||={np.linalg.norm(dx_safe):.3f} | Predicted Score: {pred_score:8.2f} pts (from 12415.56)")
    for j in range(12):
        print(f"  {PARAM_NAMES[j]:12s}: {th_anchor[j]:8.4f} -> {th_new[j]:8.4f} ({dx_safe[j]*100:+5.1f}%)")
    for k, (f, _, _) in enumerate(TARGETS):
        print(f"  {f:.3f} Hz: gain {r_anchor[k]:+5.2f} -> {r_pred[k]:+5.2f} dB | phase {r_anchor[k+6]*10:+6.2f} -> {r_pred[k+6]*10:+6.2f} deg")

