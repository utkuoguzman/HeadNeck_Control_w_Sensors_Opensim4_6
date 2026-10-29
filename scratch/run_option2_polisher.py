import os
import sys
import glob
import json
import math
import time
import cmath
import subprocess
from concurrent.futures import ThreadPoolExecutor
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

# Physiological & 0.324 Hz Deadband-Stable Bounds
THETA_MIN = np.array([0.35, 50.0, 0.08, 11.0, 60.0, 3.50, 12.0, 0.308,  6.0, 130.0, 0.50, 0.78])
THETA_MAX = np.array([0.85, 95.0, 0.22, 26.0, 115.0, 5.15, 22.0, 0.420, 15.0, 220.0, 0.85, 1.35])

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
    r_neck = 0.035 # Effective saturated rotational coupling in OpenSim
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

def compute_analytical_jacobian(th_anchor):
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
    return J_ana

def load_all_clean_candidates():
    files = glob.glob("logs/candidates/W*_P24G*.json") + glob.glob("logs/candidates/W_OPT2_*.json") + glob.glob("logs/candidates/W_NEW_KP0*.json")
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

    dataset = []
    for w, v in by_worker.items():
        if "params" in v and len(v.get("freqs", {})) == 6:
            tot = sum(x["score"] for x in v["freqs"].values())
            max_g = max(x["sim_gain"] for x in v["freqs"].values())
            if tot < 45000 and max_g < 40000:
                res = np.zeros(12)
                for k, (f, tg, tp) in enumerate(TARGETS):
                    d = v["freqs"][f]
                    gdb = d["gain_db_diff"]
                    sp = d["sim_phase"]
                    pdiff = (sp - tp + 180.0) % 360.0 - 180.0
                    res[k] = gdb / 1.0
                    res[k + 6] = pdiff / 10.0
                dataset.append({
                    "worker": w,
                    "score": float(tot),
                    "params": np.array(v["params"], dtype=float),
                    "res": res,
                    "freqs": v["freqs"]
                })
    dataset.sort(key=lambda item: item["score"])
    return dataset

def fit_space_mapped_broyden_jacobian(anchor_item, dataset):
    th_anchor = anchor_item["params"]
    r_anchor = anchor_item["res"]
    J_ana = compute_analytical_jacobian(th_anchor)

    X_list, Y_list, W_list = [], [], []
    for item in dataset:
        if item["worker"] == anchor_item["worker"]:
            continue
        dx = (item["params"] - th_anchor) / th_anchor
        dist = np.linalg.norm(dx)
        if dist < 1e-6:
            continue
        dr = item["res"] - r_anchor
        for k in range(6):
            while dr[k + 6] > 18.0: dr[k + 6] -= 36.0
            while dr[k + 6] < -18.0: dr[k + 6] += 36.0
        X_list.append(dx)
        Y_list.append(dr)
        # Stronger weight for nearby points and recent Option 2 secant probes
        is_opt2 = 2.5 if item["worker"].startswith("W_OPT2_") else 1.0
        w_val = is_opt2 * math.exp(-0.5 * (dist / 0.25)**2) * (14000.0 / max(10000.0, item["score"]))**2
        W_list.append(w_val)

    X = np.array(X_list)
    Y = np.array(Y_list)
    W = np.array(W_list)
    Sw = np.diag(np.sqrt(W))
    Xw = Sw @ X
    Yw = Sw @ Y

    # Tikhonov-regularized Space-Mapped + Broyden/Empirical Jacobian:
    # Solves min_J || Sw (X J^T - Y) ||_F^2 + alpha * || J^T - 0.25 * J_ana^T ||_F^2
    alpha = 0.65
    J_T = np.linalg.solve(Xw.T @ Xw + alpha * np.eye(12), Xw.T @ Yw + alpha * 0.25 * J_ana.T)
    return J_T.T, J_ana

def project_safe_params(th):
    th_proj = np.clip(th, THETA_MIN, THETA_MAX)
    # Enforce lead ratio bounds r_lead_sc = z_sc / p_sc in [0.165, 0.235] so 0.324 Hz and 2.446 Hz stay stable
    r_sc = th_proj[3] / th_proj[4]
    if r_sc < 0.165:
        th_proj[3] = 0.165 * th_proj[4]
    elif r_sc > 0.235:
        th_proj[3] = 0.235 * th_proj[4]
    return th_proj

def run_single_freq_job(job):
    worker_name, th, freq, tgt_gain, tgt_phase = job
    K_ton, p_ton, K_sc, z_sc, p_sc, K_phas, p_phas, kp_prop, z_ccr, p_ccr, k_gdyn, k_gstat = th
    r_lead_sc = z_sc / p_sc
    r_lead_ccr = z_ccr / p_ccr

    res_file = os.path.join("logs", "candidates", f"{worker_name}_{freq:.2f}Hz.json")
    if os.path.exists(res_file):
        os.remove(res_file)

    cmd = [
        sys.executable, "cmaes_worker_p22.py",
        "0.0", "1.0", "40.2786",
        f"{K_ton:.6f}", f"{p_ton:.6f}", f"{K_sc:.6f}", f"{r_lead_sc:.6f}", f"{p_sc:.6f}",
        f"{K_phas:.6f}", f"{p_phas:.6f}", f"{kp_prop:.6f}", f"{r_lead_ccr:.6f}", f"{p_ccr:.6f}",
        f"{k_gdyn:.6f}", f"{k_gstat:.6f}",
        f"{freq}", f"{tgt_gain}", f"{tgt_phase}",
        worker_name
    ]
    env = os.environ.copy()
    env["KP_TASK_PITCH"] = "0.0"
    env["KD_TASK_PITCH"] = "0.0"
    subprocess.run(cmd, capture_output=True, text=True, env=env)
    with open(res_file, "r", encoding="utf-8") as f:
        return worker_name, freq, json.load(f)

def evaluate_candidates_parallel(candidates_dict):
    # candidates_dict: {worker_name: th_array}
    # Saves candidate JSON and runs all (len(candidates) * 6) frequencies in parallel across 12 workers
    os.makedirs(os.path.join("logs", "candidates"), exist_ok=True)
    jobs = []
    for w_name, th in candidates_dict.items():
        cand_file = os.path.join("logs", "candidates", f"{w_name}.json")
        with open(cand_file, "w", encoding="utf-8") as f:
            json.dump({"worker": w_name, "params": [float(x) for x in th]}, f)
        for f_hz, tg, tp in TARGETS:
            jobs.append((w_name, th, f_hz, tg, tp))

    t0 = time.time()
    with ThreadPoolExecutor(max_workers=12) as ex:
        raw_results = list(ex.map(run_single_freq_job, jobs))
    elapsed = time.time() - t0

    eval_out = {}
    for w_name, th in candidates_dict.items():
        f_map = {f_hz: d for (w, f_hz, d) in raw_results if w == w_name}
        tot = sum(d["score"] for d in f_map.values())
        res = np.zeros(12)
        for k, (f_hz, tg, tp) in enumerate(TARGETS):
            d = f_map[f_hz]
            gdb = d["gain_db_diff"]
            sp = d["sim_phase"]
            pdiff = (sp - tp + 180.0) % 360.0 - 180.0
            res[k] = gdb / 1.0
            res[k + 6] = pdiff / 10.0
        eval_out[w_name] = {
            "worker": w_name,
            "score": float(tot),
            "params": th.copy(),
            "res": res,
            "freqs": f_map,
            "wall_sec": elapsed
        }
    return eval_out, elapsed

def print_candidate_table(item):
    print(f"  >>> {item['worker']} Total Score: {item['score']:,.2f} pts <<<", flush=True)
    for f_hz, tg, tp in TARGETS:
        d = item["freqs"][f_hz]
        pdiff = (d["sim_phase"] - tp + 180.0) % 360.0 - 180.0
        print(f"      {f_hz:5.3f} Hz | Gain: {d['sim_gain']:7.2f} (Tgt {tg:6.2f}, {d['gain_db_diff']:+6.2f} dB) | Phase: {d['sim_phase']:7.2f}° (Tgt {tp:7.2f}°, {pdiff:+6.2f}°) | Loss: {d['score']:7.1f}", flush=True)

def main():
    print("=" * 92, flush=True)
    print("OPTION 2: MULTIPLICATIVE SPACE-MAPPED + BROYDEN TRUST-REGION GAUSS-NEWTON POLISHER", flush=True)
    print("Starting Directly from All-Time Best W4_P24G3 (12,415.56 pts)", flush=True)
    print("=" * 92, flush=True)

    dataset = load_all_clean_candidates()
    best_item = next(item for item in dataset if item["worker"] == "W4_P24G3")
    second_item = next(item for item in dataset if item["worker"] == "W1_P24G5")

    print(f"[INITIAL ANCHOR] {best_item['worker']} = {best_item['score']:,.2f} pts", flush=True)
    print_candidate_table(best_item)

    history_log = [{
        "iter": 0,
        "best_worker": best_item["worker"],
        "best_score": best_item["score"],
        "best_params": [float(x) for x in best_item["params"]]
    }]

    n_iters = 3
    for it in range(1, n_iters + 1):
        print("\n" + "=" * 92, flush=True)
        print(f"ITERATION {it}/{n_iters} | Current Champion: {best_item['worker']} ({best_item['score']:,.2f} pts)", flush=True)
        print("=" * 92, flush=True)

        J, J_ana = fit_space_mapped_broyden_jacobian(best_item, dataset)
        th_curr = best_item["params"]
        r_curr = best_item["res"]

        # Weight high-loss frequencies (0.324 Hz phase, 5.583 Hz phase, 7.927 Hz phase) and keep mid-band tight
        W_res = np.diag([1.2, 1.0, 1.0, 1.2, 1.0, 1.0,  1.2, 1.0, 1.0, 1.1, 1.3, 1.3])
        Jw = W_res @ J
        rw = W_res @ r_curr

        if it == 1:
            # Candidate A: Conservative Trust-Region Gauss-Newton step (max 7% per param)
            lam_A = 4.0
            dx_A = -np.linalg.solve(Jw.T @ Jw + lam_A * np.eye(12), Jw.T @ rw)
            dx_A = np.clip(dx_A, -0.07, 0.07)
            th_A = project_safe_params(th_curr * (1.0 + dx_A))

            # Candidate B: Moderate Trust-Region Gauss-Newton + Elite Secant Extrapolation from W1_P24G5 -> W4_P24G3
            lam_B = 2.0
            dx_gn = -np.linalg.solve(Jw.T @ Jw + lam_B * np.eye(12), Jw.T @ rw)
            dx_sec = (best_item["params"] - second_item["params"]) / best_item["params"]
            dx_B = np.clip(0.65 * dx_gn + 0.35 * np.clip(dx_sec, -0.10, 0.10), -0.11, 0.11)
            th_B = project_safe_params(th_curr * (1.0 + dx_B))
        elif it == 2:
            # Refine around updated Jacobian with two trust-region radii (5% and 9%)
            lam_A = 5.0
            dx_A = -np.linalg.solve(Jw.T @ Jw + lam_A * np.eye(12), Jw.T @ rw)
            dx_A = np.clip(dx_A, -0.05, 0.05)
            th_A = project_safe_params(th_curr * (1.0 + dx_A))

            lam_B = 1.8
            dx_B = -np.linalg.solve(Jw.T @ Jw + lam_B * np.eye(12), Jw.T @ rw)
            dx_B = np.clip(dx_B, -0.09, 0.09)
            th_B = project_safe_params(th_curr * (1.0 + dx_B))
        else:
            # Iteration 3: Fine local polish (3.5% and 6.5%)
            lam_A = 6.5
            dx_A = -np.linalg.solve(Jw.T @ Jw + lam_A * np.eye(12), Jw.T @ rw)
            dx_A = np.clip(dx_A, -0.035, 0.035)
            th_A = project_safe_params(th_curr * (1.0 + dx_A))

            # Pure Analytical Space-Mapped + Broyden step
            J_mix = 0.65 * J + 0.35 * J_ana
            Jw_mix = W_res @ J_mix
            dx_B = -np.linalg.solve(Jw_mix.T @ Jw_mix + 3.0 * np.eye(12), Jw_mix.T @ rw)
            dx_B = np.clip(dx_B, -0.065, 0.065)
            th_B = project_safe_params(th_curr * (1.0 + dx_B))

        w_name_A = f"W_OPT2_I{it}A"
        w_name_B = f"W_OPT2_I{it}B"

        dx_eff_A = (th_A - th_curr) / th_curr
        dx_eff_B = (th_B - th_curr) / th_curr
        pred_A = 100.0 * np.sum((r_curr + J @ dx_eff_A)**2)
        pred_B = 100.0 * np.sum((r_curr + J @ dx_eff_B)**2)

        print(f"[PROPOSAL A: {w_name_A}] ||dx||={np.linalg.norm(dx_eff_A):.3f}, Linear Pred={pred_A:,.1f} pts", flush=True)
        print("  Params A:", " ".join(f"{PARAM_NAMES[j]}={th_A[j]:.4f}({dx_eff_A[j]*100:+.1f}%)" for j in range(12)), flush=True)
        print(f"[PROPOSAL B: {w_name_B}] ||dx||={np.linalg.norm(dx_eff_B):.3f}, Linear Pred={pred_B:,.1f} pts", flush=True)
        print("  Params B:", " ".join(f"{PARAM_NAMES[j]}={th_B[j]:.4f}({dx_eff_B[j]*100:+.1f}%)" for j in range(12)), flush=True)

        print(f"\n[RUNNING 12 PARALLEL OPENSIM WORKERS FOR {w_name_A} & {w_name_B}...]", flush=True)
        eval_out, wall_s = evaluate_candidates_parallel({w_name_A: th_A, w_name_B: th_B})
        print(f"[COMPLETED IN {wall_s:.1f}s]", flush=True)

        for w_name in [w_name_A, w_name_B]:
            item = eval_out[w_name]
            print_candidate_table(item)
            max_g = max(x["sim_gain"] for x in item["freqs"].values())
            if item["score"] < 45000 and max_g < 40000:
                dataset.append(item)
            if item["score"] < best_item["score"]:
                print(f"  *** NEW ALL-TIME CHAMPION! {w_name} improved score from {best_item['score']:,.2f} -> {item['score']:,.2f} pts! ***", flush=True)
                second_item = best_item
                best_item = item

        dataset.sort(key=lambda x: x["score"])
        history_log.append({
            "iter": it,
            "cand_A": {"worker": w_name_A, "score": eval_out[w_name_A]["score"], "params": [float(x) for x in th_A]},
            "cand_B": {"worker": w_name_B, "score": eval_out[w_name_B]["score"], "params": [float(x) for x in th_B]},
            "best_worker": best_item["worker"],
            "best_score": best_item["score"],
            "best_params": [float(x) for x in best_item["params"]]
        })

        with open("logs/option2_polisher_summary.json", "w", encoding="utf-8") as f:
            json.dump({
                "initial_score": 12415.56,
                "final_best_worker": best_item["worker"],
                "final_best_score": best_item["score"],
                "final_best_params": {PARAM_NAMES[j]: float(best_item["params"][j]) for j in range(12)},
                "history": history_log
            }, f, indent=2)

    print("\n" + "=" * 92, flush=True)
    print(f"OPTION 2 POLISHING COMPLETE! Best Champion: {best_item['worker']} ({best_item['score']:,.2f} pts)", flush=True)
    print("=" * 92, flush=True)

if __name__ == "__main__":
    main()

