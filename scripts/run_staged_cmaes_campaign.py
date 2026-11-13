"""
5-Stage Coordinate-Descent / Block-Staged CMA-ES Optimization Campaign
for the 6-DOF Head-Neck Neuromotor Controller (OpenSim 4.6).

Pipeline Architecture (Starting from HF_TEST_03):
-------------------------------------------------
Master 10-Parameter Vector:
  [0] K_ton          (Tonic Otolith VCR Gain)           init = 0.950000
  [1] K_sc           (Semicircular Canal VCR Gain)      init = 0.250000
  [2] K_phas         (Phasic Otolith VCR Gain)          init = 4.200000
  [3] kp_prop        (Spinal CCR Gain Kp_proprioception)init = 0.180000
  [4] Kp_task_pitch  (Voluntary Task Proportional Gain) init = 0.969350
  [5] Kd_task_pitch  (Voluntary Task Derivative Gain)   init = 0.363497
  [6] p_ton          (Tonic Otolith VCR Pole [rad/s])   init = 15.000000
  [7] p_sc           (Canal VCR Pole [rad/s])           init = 50.000000
  [8] p_phas         (Phasic Otolith VCR Pole [rad/s])  init = 10.000000
  [9] p_ccr          (Spinal CCR Pole [rad/s])          init = 100.000000

Sequential Stages (20 parallel workers = 10 orthogonal mirrored pairs/gen):
  * Step 1 (8 gens, N=5): Tune Gains [K_ton, K_sc, K_phas, kp_prop, Kp_task_pitch];
                          Freeze [Kd_task_pitch, p_ton, p_sc, p_phas, p_ccr].
  * Step 2 (8 gens, N=2): Tune Task Controller [Kp_task_pitch, Kd_task_pitch];
                          Freeze VCR and CCR at Step 1 best.
  * Step 3 (8 gens, N=6): Disable CCR (kp_prop = 0.0), freeze Task at Step 2 best,
                          and tune VCR Gains & Poles [K_ton, K_sc, K_phas, p_ton, p_sc, p_phas].
  * Step 4 (8 gens, N=2): Freeze VCR (at Step 3 best) & Task (at Step 2 best),
                          re-enable CCR at Step 1 best kp_prop, and tune CCR [kp_prop, p_ccr].
  * Step 5 (15 gens, N=10): Joint fine-tuning of all 10 parameters with smaller sigmas (0.4x)
                            initialized from the combined best of Steps 1-4.
"""

import os
import sys
import time
import json
import math
import shutil
import argparse
import subprocess
from datetime import datetime
from concurrent.futures import ThreadPoolExecutor, as_completed
import numpy as np
import cma

REPO_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PYTHON_EXE = r"C:\Users\User\miniforge3\envs\opensim_env\python.exe"
WORKER_SCRIPT = os.path.join(REPO_DIR, "util", "cmaes_worker_multisine.py")
CAND_DIR = os.path.join(REPO_DIR, "logs", "candidates_multisine")
LEDGER_PATH = os.path.join(REPO_DIR, "logs", "cmaes_staged_10param_ledger.md")
CHAMPION_JSON = os.path.join(REPO_DIR, "logs", "cmaes_staged_10param_champion.json")
STATE_JSON = os.path.join(REPO_DIR, "logs", "cmaes_staged_10param_state.json")
ART_DIR = r"C:\Users\User\.gemini\antigravity\brain\89fb8d56-9d12-4051-a256-02497ef8d6ae"

POPSIZE = 20  # Full-on 20 parallel OpenSim workers per generation
MAX_PARALLEL_WORKERS = 24  # Full CPU concurrency for daytime full-on runs

PARAM_NAMES = [
    "K_ton",          # 0
    "K_sc",           # 1
    "K_phas",         # 2
    "kp_prop",        # 3
    "Kp_task_pitch",  # 4
    "Kd_task_pitch",  # 5
    "p_ton",          # 6
    "p_sc",           # 7
    "p_phas",         # 8
    "p_ccr",          # 9
    "Ki_task_pitch",  # 10
]

# Initial 11-Parameter Vector (Global Champion W16_S5_G01 = 2366.83 pts):
X0_HF_TEST_03 = [
    6.43975069,  # 0: K_ton
    3.91221090,  # 1: K_sc
    13.48130263, # 2: K_phas
    0.54237935,  # 3: kp_prop (lower bound = 0.45 per Happee 2017 Table 1)
    3.58522863,  # 4: Kp_task_pitch
    0.46308195,  # 5: Kd_task_pitch
    15.7472650,  # 6: p_ton
    50.4415050,  # 7: p_sc
    4.37376700,  # 8: p_phas
    98.8278360,  # 9: p_ccr
    3.48108591,  # 10: Ki_task_pitch (bounds [1.50, 10.00])
]

LOWER_BOUNDS = [
    0.50,   # 0: K_ton
    0.20,   # 1: K_sc
    2.00,   # 2: K_phas
    0.45,   # 3: kp_prop (physiological minimum from Happee 2017 Table 1)
    0.20,   # 4: Kp_task_pitch
    0.05,   # 5: Kd_task_pitch
    4.00,   # 6: p_ton
    15.00,  # 7: p_sc
    1.50,   # 8: p_phas
    30.00,  # 9: p_ccr
    1.50,   # 10: Ki_task_pitch (lower bound = 1.50)
]

UPPER_BOUNDS = [
    12.00,   # 0: K_ton
    8.00,    # 1: K_sc
    18.00,   # 2: K_phas
    0.75,    # 3: kp_prop
    10.00,   # 4: Kp_task_pitch
    3.00,    # 5: Kd_task_pitch
    50.00,   # 6: p_ton
    120.00,  # 7: p_sc
    20.00,   # 8: p_phas
    140.00,  # 9: p_ccr
    10.00,   # 10: Ki_task_pitch (upper bound = 10.00)
]

# Standard coordinate-stage exploration sigmas:
STAGE_STD_SIGMAS = [
    0.35,   # 0: K_ton
    0.25,   # 1: K_sc
    0.60,   # 2: K_phas
    0.015,  # 3: kp_prop
    0.28,   # 4: Kp_task_pitch
    0.06,   # 5: Kd_task_pitch
    1.20,   # 6: p_ton
    3.00,   # 7: p_sc
    0.35,   # 8: p_phas
    2.00,   # 9: p_ccr
    0.60,   # 10: Ki_task_pitch
]

# Smaller joint fine-tuning sigmas for Stage 5 (0.5x of exploration sigmas):
STAGE_FINE_SIGMAS = [
    0.18,   # 0: K_ton
    0.12,   # 1: K_sc
    0.30,   # 2: K_phas
    0.008,  # 3: kp_prop
    0.14,   # 4: Kp_task_pitch
    0.03,   # 5: Kd_task_pitch
    0.60,   # 6: p_ton
    1.50,   # 7: p_sc
    0.18,   # 8: p_phas
    1.00,   # 9: p_ccr
    0.30,   # 10: Ki_task_pitch
]

STAGES = [
    {
        "step": 1,
        "code": "S1_GAINS",
        "title": "Step 1: Tune VCR + CCR + Task Kp Gains (Freeze Kd & Poles)",
        "gens": 8,
        "active_indices": [0, 1, 2, 3, 4],
        "sigma_scale": 1.0,
        "disable_ccr": False,
    },
    {
        "step": 2,
        "code": "S2_TASK_PD",
        "title": "Step 2: Tune Task Controller [Kp_task_pitch, Kd_task_pitch] (Freeze VCR & CCR)",
        "gens": 8,
        "active_indices": [4, 5],
        "sigma_scale": 1.0,
        "disable_ccr": False,
    },
    {
        "step": 3,
        "code": "S3_VCR_TASK_FREEZE_CCR",
        "title": "Part 1 (Step 3): Freeze CCR (kp_prop=0.542, p_ccr=98.8) & Train VCR + Task [K_ton, K_sc, K_phas, Kp_t, Kd_t, p_ton, p_sc, p_phas, Ki_t] (9D)",
        "gens": 6,
        "active_indices": [0, 1, 2, 4, 5, 6, 7, 8, 10],  # 9D VCR + Task (including Ki_task_pitch in [1.5, 10.0])
        "popsize": 20,  # 2 TPA + 18 Ortho Mirrored (9 antipodal pairs = 1 exact 9x9 Haar block)
        "use_tpa": True,
        "skip_sweep": False,
        "sigma_scale": 1.0,
        "disable_ccr": False,
        "plateau_patience": 3,
        "plateau_tol": 10.0,
    },
    {
        "step": 4,
        "code": "S4_CCR_FREEZE_VCR_TASK",
        "title": "Part 2 (Step 4): Freeze VCR + Task & Train CCR [kp_prop >= 0.45, p_ccr] (2D)",
        "gens": 5,
        "active_indices": [3, 9],  # 2D CCR Only [kp_prop, p_ccr]
        "popsize": 16,  # 2 TPA + 14 Ortho Mirrored
        "use_tpa": True,
        "skip_sweep": False,
        "sigma_scale": 1.0,
        "disable_ccr": False,
        "plateau_patience": 3,
        "plateau_tol": 5.0,
    },
    {
        "step": 5,
        "code": "S5_JOINT_11D_ALL",
        "title": "Part 3 (Step 5): Joint 11-Parameter VCR + CCR + Task Fine-Tuning (Full 11D, 22 Ortho Mirrored + 2 TPA = 24 Pop)",
        "gens": 8,
        "active_indices": [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10],  # Full 11D
        "popsize": 24,  # 2 TPA + 22 Ortho Mirrored (11 antipodal pairs = 1 exact 11x11 Haar block)
        "use_tpa": True,
        "skip_sweep": True,
        "sigma_scale": 0.50,
        "disable_ccr": False,
        "plateau_patience": 3,
        "plateau_tol": 5.0,
    },
]


def ask_orthogonal_mirrored_nd(es, popsize, lb_sub, ub_sub, use_tpa=False):
    """
    Dimension-Adaptive Orthogonal Mirrored Sampling for arbitrary active dimension N.
    - When use_tpa=True (e.g. N=8, popsize=18):
      * Slots 0 and 1 are preserved from pycma's `es.ask()` TPA mirrored pair along the
        previous mean shift direction (+Delta_m and -Delta_m) for Two-Point Step-Size Adaptation.
      * Slots 2..17 (16 points = 8 antipodal mirrored pairs) are generated from a single
        8x8 Haar-distributed random orthogonal matrix Q, spanning 100% of R^8!
    - When use_tpa=False:
      * Generates ceil((popsize/2) / N) independent Haar-distributed random orthogonal
        N x N blocks via QR decomposition (with R-diagonal sign stabilization).
    """
    n_dim = len(lb_sub)
    # Initialize pycma's internal generation state and archive (and get TPA pair in raw_ask[0:2] if TPA is active)
    raw_ask = es.ask()

    candidates = []
    if use_tpa and popsize >= 4:
        # Preserve pycma's exact TPA mirrored pair in Slots 0 and 1
        candidates.append(np.clip(np.asarray(raw_ask[0], dtype=float), lb_sub, ub_sub))
        candidates.append(np.clip(np.asarray(raw_ask[1], dtype=float), lb_sub, ub_sub))
        half = (popsize - 2) // 2  # For popsize=18, N=8: half = 8 orthogonal directions -> 16 mirrored points
    else:
        half = popsize // 2  # 10 mirrored pairs for popsize = 20

    n_blocks = int(math.ceil(float(half) / float(n_dim)))
    eigvals, B = np.linalg.eigh(es.C)
    D = np.sqrt(np.maximum(eigvals, 1e-16))
    mean_int = es.mean
    sigma = es.sigma

    z_dirs = []
    for _ in range(n_blocks):
        if n_dim == 1:
            q_mat = np.array([[1.0 if np.random.randn() >= 0.0 else -1.0]])
        else:
            raw_g = np.random.randn(n_dim, n_dim)
            q_mat, r_mat = np.linalg.qr(raw_g)
            d_signs = np.sign(np.diag(r_mat))
            d_signs[d_signs == 0.0] = 1.0
            q_mat = q_mat * d_signs

        chi_lengths = np.linalg.norm(np.random.randn(n_dim, n_dim), axis=0)
        for col_idx in range(n_dim):
            z_dirs.append(q_mat[:, col_idx] * chi_lengths[col_idx])
            if len(z_dirs) >= half:
                break
        if len(z_dirs) >= half:
            break

    coord_scale = np.asarray(es.sigma_vec.scaling, dtype=float) if hasattr(es, "sigma_vec") else np.ones(n_dim)
    for k in range(half):
        z_vec = z_dirs[k]
        step_int = coord_scale * np.dot(B, D * z_vec)
        for sgn in (+1.0, -1.0):
            x_int = mean_int + sgn * sigma * step_int
            x_ext = es.gp.pheno(x_int, into_bounds=es.boundary_handler.repair, archive=es.sent_solutions)
            x_ext = np.clip(x_ext, lb_sub, ub_sub)
            candidates.append(x_ext)

    return candidates


def generate_pre_stage_sweep_candidates(s_num, x0_sub, lb_sub, ub_sub, stds_sub, popsize=20):
    """
    Generates 1 parallel batch (20 candidates) for the Pre-Stage Laser Sweep before Steps 1-4:
    - For 2D stages (Step 2 Task PD and Step 4 CCR Only): builds a structured 5x4 2D grid around x0_sub
      (with slot 0 guaranteed to be the exact incumbent x0_sub).
    - For 5D/6D stages (Step 1 Gains and Step 3 VCR Gains+Poles): combines slot 0 (exact x0_sub),
      +/- 1.15*sigma single-axis coordinate probes along each active dimension, and orthogonal
      multi-axis diagonal probes to laser-focus the starting center before CMA-ES begins.
    """
    n_dim = len(x0_sub)
    x0_arr = np.clip(np.array(x0_sub, dtype=float), lb_sub, ub_sub)
    lb_arr = np.array(lb_sub, dtype=float)
    ub_arr = np.array(ub_sub, dtype=float)
    std_arr = np.array(stds_sub, dtype=float)

    cands = [x0_arr.copy()]  # Slot 0 is always the exact incumbent center

    if n_dim == 9 and s_num == 3:
        # Part 1 (Step 3, 9D VCR+Task with CCR frozen at W16_S5_G01's kp_prop=0.54237935, p_ccr=98.827836):
        # Active sub-vector indices: [0:K_ton, 1:K_sc, 2:K_phas, 4:Kp_t, 5:Kd_t, 6:p_ton, 7:p_sc, 8:p_phas, 10:Ki_t]
        cands = [x0_arr.copy()]  # Slot 0: Exact W16_S5_G01 (2366.83 pts, Kd=0.4631, Ki=3.4811)
        # Slots 1..6: Probe Kd_task_pitch (to recover 5.583 & 7.927 Hz phase lead) and Ki_task_pitch:
        for kd_try, ki_try in [
            (0.500, 3.481),
            (0.535, 3.481),
            (0.565, 3.100),
            (0.520, 3.000),
            (0.490, 4.000),
            (0.463, 4.500),
        ]:
            pt = x0_arr.copy()
            pt[4] = float(np.clip(kd_try, lb_arr[4], ub_arr[4]))
            pt[8] = float(np.clip(ki_try, lb_arr[8], ub_arr[8]))
            if not any(np.allclose(pt, ex) for ex in cands) and len(cands) < popsize:
                cands.append(pt)
        # Slots 7..19: Coordinate probes across VCR & Task active axes around W16_S5_G01:
        for d_idx, sgn in [
            (0, +0.7), (0, -0.7),  # K_ton
            (1, +0.7), (1, -0.7),  # K_sc
            (2, +0.7), (2, -0.7),  # K_phas
            (3, +0.7), (3, -0.7),  # Kp_task_pitch
            (5, +0.8), (5, -0.8),  # p_ton
            (6, +0.8), (6, -0.8),  # p_sc
            (7, +0.8),             # p_phas
        ]:
            if len(cands) >= popsize:
                break
            pt = x0_arr.copy()
            pt[d_idx] = float(np.clip(pt[d_idx] + sgn * std_arr[d_idx], lb_arr[d_idx], ub_arr[d_idx]))
            cands.append(pt)
        return cands[:popsize]

    if n_dim == 2:
        if s_num == 4:
            # Part 2 (Step 4, 2D CCR [kp_prop >= 0.45, p_ccr] with VCR+Task frozen at Part 1 best):
            cands = [x0_arr.copy()]
            kp_grid = [0.46, 0.50, 0.5424, 0.58]
            p_grid = [92.0, 96.0, 98.828, 104.0]
            for kp_v in kp_grid:
                for p_v in p_grid:
                    pt = np.clip(np.array([kp_v, p_v], dtype=float), lb_arr, ub_arr)
                    if not any(np.allclose(pt, ex) for ex in cands) and len(cands) < popsize:
                        cands.append(pt)
            return cands[:popsize]
        else:
            # Step 2 (Task [Kp_task_pitch, Kd_task_pitch]): 5x4 grid around current (Kp, Kd)
            offsets_0 = [-1.5, -0.75, 0.0, +0.75, +1.5]
            offsets_1 = [-1.25, -0.45, +0.45, +1.25]
            for o0 in offsets_0:
                for o1 in offsets_1:
                    pt = np.clip(x0_arr + np.array([o0 * std_arr[0], o1 * std_arr[1]]), lb_arr, ub_arr)
                    if not np.allclose(pt, x0_arr) and len(cands) < popsize:
                        cands.append(pt)

    # Coordinate probes (+/- 1.15 sigma along each active axis)
    for d in range(n_dim):
        for sgn in (+1.15, -1.15):
            if len(cands) >= popsize:
                break
            pt = x0_arr.copy()
            pt[d] = np.clip(pt[d] + sgn * std_arr[d], lb_arr[d], ub_arr[d])
            cands.append(pt)

    # Fill remaining slots with reproducible orthogonal mirrored probes
    rng = np.random.RandomState(9000 + s_num * 100)
    while len(cands) < popsize:
        raw_g = rng.randn(n_dim, n_dim)
        q_mat, r_mat = np.linalg.qr(raw_g)
        d_signs = np.sign(np.diag(r_mat))
        d_signs[d_signs == 0.0] = 1.0
        q_mat = q_mat * d_signs
        for col_idx in range(n_dim):
            for sgn in (+1.0, -1.0):
                if len(cands) >= popsize:
                    break
                pt = np.clip(x0_arr + sgn * 1.10 * std_arr * q_mat[:, col_idx], lb_arr, ub_arr)
                cands.append(pt)

    return cands[:popsize]


def assemble_full_10params(base_10, active_indices, x_sub, disable_ccr=False):
    full_p = list(base_10)
    while len(full_p) < len(LOWER_BOUNDS):
        full_p.append(X0_HF_TEST_03[len(full_p)])
    for idx_local, idx_global in enumerate(active_indices):
        full_p[idx_global] = float(np.clip(x_sub[idx_local], LOWER_BOUNDS[idx_global], UPPER_BOUNDS[idx_global]))
    if disable_ccr:
        full_p[3] = 0.0
    return full_p


def evaluate_worker(worker_idx, step_num, gen_num, full_params_10, save_ts=False, custom_id=None):
    wid = custom_id if custom_id else f"W{worker_idx + 1:02d}_S{step_num}_G{gen_num:02d}"
    out_json = os.path.join(CAND_DIR, f"{wid}.json")
    out_npz = os.path.join(CAND_DIR, f"{wid}_ts.npz")

    full_p11 = list(full_params_10)
    if len(full_p11) < 11:
        full_p11.append(0.60)

    # Fast cache reuse when resuming completed generations:
    if os.path.exists(out_json) and (not save_ts or os.path.exists(out_npz)):
        try:
            with open(out_json, "r", encoding="utf-8") as f:
                cached = json.load(f)
            cached_p11 = cached.get("params_11", cached.get("params_10"))
            if (
                cached_p11 is not None
                and len(cached_p11) == len(full_p11)
                and np.allclose(cached_p11, full_p11, atol=1e-4, rtol=1e-4)
                and float(cached.get("total_score", 6000000.0)) < 5999999.0
            ):
                cached["idx"] = worker_idx
                cached["wall_s"] = 0.0
                cached["params_10"] = list(full_p11)
                return cached
        except Exception:
            pass

    if os.path.exists(out_json):
        try:
            os.remove(out_json)
        except OSError:
            pass

    cmd = [
        PYTHON_EXE,
        WORKER_SCRIPT,
        f"{full_p11[0]:.8f}",   # K_ton
        f"{full_p11[1]:.8f}",   # K_sc
        f"{full_p11[2]:.8f}",   # K_phas
        f"{full_p11[3]:.8f}",   # kp_prop
        f"{full_p11[4]:.8f}",   # Kp_task_pitch
        f"{full_p11[5]:.8f}",   # Kd_task_pitch
        f"{full_p11[6]:.6f}",   # p_ton
        f"{full_p11[7]:.6f}",   # p_sc
        f"{full_p11[8]:.6f}",   # p_phas
        f"{full_p11[9]:.6f}",   # p_ccr
        f"{full_p11[10]:.8f}",  # Ki_task_pitch
        wid,
    ]
    env = os.environ.copy()
    env["SIMB_NUM_THREADS"] = "1"
    env["OMP_NUM_THREADS"] = "1"
    env["MKL_NUM_THREADS"] = "1"
    env["OPENBLAS_NUM_THREADS"] = "1"
    env["CCR_DRG_NORM_K0"] = "1"
    env["TAU_MUSCLE_ACT"] = "0.0100"
    env["KI_TASK_PITCH"] = f"{full_p11[10]:.8f}"
    env["PCORE_ONLY"] = os.environ.get("PCORE_ONLY", "0")
    env["PCORE_SLOT"] = str(worker_idx % 6)
    env["SAVE_TS"] = "1"

    t0 = time.time()
    try:
        subprocess.run(
            cmd,
            cwd=REPO_DIR,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=2400,
            check=False,
        )
    except subprocess.TimeoutExpired:
        return {
            "worker": wid,
            "idx": worker_idx,
            "total_score": 6000000.0,
            "params_10": list(full_p11),
            "wall_s": round(time.time() - t0, 1),
            "telemetry": {"aborted": True, "abort_reason": "TIMEOUT"},
        }

    wall_s = round(time.time() - t0, 1)
    if os.path.exists(out_json):
        try:
            with open(out_json, "r", encoding="utf-8") as f:
                data = json.load(f)
            data["idx"] = worker_idx
            data["wall_s"] = wall_s
            data["params_10"] = list(full_p11)
            return data
        except Exception:
            pass

    return {
        "worker": wid,
        "idx": worker_idx,
        "total_score": 6000000.0,
        "params_10": list(full_p11),
        "wall_s": wall_s,
        "telemetry": {"aborted": True, "abort_reason": "NO_JSON"},
    }


def trigger_champion_plot(wid):
    """Re-run champion with SAVE_TS=1 if needed and generate 3-panel Bode/time-series artifact plot."""
    try:
        cand_json = os.path.join(CAND_DIR, f"{wid}.json")
        ts_npz = os.path.join(CAND_DIR, f"{wid}_ts.npz")
        if not os.path.exists(cand_json):
            return None
        with open(cand_json, "r", encoding="utf-8") as f:
            d = json.load(f)
        p10 = d.get("params_11", d.get("params_10"))
        if not os.path.exists(ts_npz) and p10:
            evaluate_worker(0, 0, 0, p10, save_ts=True, custom_id=wid)
        if os.path.exists(ts_npz):
            scratch_dir = os.path.join(ART_DIR, "scratch")
            if scratch_dir not in sys.path:
                sys.path.insert(0, scratch_dir)
            from plot_3panel_multisine import generate_plot
            generate_plot(wid)
            out_png = os.path.join(ART_DIR, f"{wid.lower()}_tracking_and_bode.png")
            return out_png
    except Exception as e:
        print(f"  [Plot warning for {wid}]: {e}")
    return None


def format_p10_short(p10):
    ki_val = p10[10] if len(p10) > 10 else 0.0
    return (
        f"K_ton={p10[0]:.3f}, K_sc={p10[1]:.3f}, K_phas={p10[2]:.3f}, kp_prop={p10[3]:.3f}, "
        f"Kp_t={p10[4]:.3f}, Kd_t={p10[5]:.3f}, Ki_t={ki_val:.3f} | "
        f"p_ton={p10[6]:.1f}, p_sc={p10[7]:.1f}, p_phas={p10[8]:.2f}, p_ccr={p10[9]:.1f}"
    )


def init_ledger(baseline_res):
    os.makedirs(os.path.dirname(LEDGER_PATH), exist_ok=True)
    b_score = baseline_res["total_score"]
    b_tel = baseline_res.get("telemetry", {})
    with open(LEDGER_PATH, "w", encoding="utf-8") as f:
        f.write("# 5-Stage 10-Parameter Coordinate-Descent CMA-ES Campaign Ledger\n\n")
        f.write(f"- **Started**: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"- **Baseline (`HF_TEST_03`)**: `{b_score:.2f}` ")
        f.write(
            f"(Bode={b_tel.get('bode_score_without_hf', 0):.1f}, "
            f"HF_Pen={b_tel.get('hf_penalty', 0):.1f}, "
            f"DC_Pen={b_tel.get('dc_drift_penalty', 0):.1f}, "
            f"MaxRoll={b_tel.get('max_abs_roll_deg', 0):.3f} deg)\n"
        )
        f.write(f"- **Initial Vector**: `{format_p10_short(X0_HF_TEST_03)}`\n\n")
        f.write(
            "| Step | Gen | Dim $N$ | Best Worker | Gen Best | Stage Best | Global Best | Bode | HF Pen | DC Pen | Active Subset & Parameters |\n"
        )
        f.write("|---|---|---|---|---|---|---|---|---|---|---|\n")


def append_ledger_row(step_num, gen_num, n_dim, best_res, stage_best_score, global_best_score, active_names):
    tel = best_res.get("telemetry", {})
    p10 = best_res["params_10"]
    with open(LEDGER_PATH, "a", encoding="utf-8") as f:
        f.write(
            f"| S{step_num} | {gen_num:02d} | {n_dim}D | `{best_res['worker']}` | "
            f"**{best_res['total_score']:.1f}** | {stage_best_score:.1f} | **{global_best_score:.1f}** | "
            f"{tel.get('bode_score_without_hf', 0):.1f} | {tel.get('hf_penalty', 0):.1f} | "
            f"{tel.get('dc_drift_penalty', 0):.1f} | "
            f"`{','.join(active_names)}` -> {format_p10_short(p10)} |\n"
        )


def run_campaign(start_step=1, max_step=5):
    os.makedirs(CAND_DIR, exist_ok=True)
    print("=" * 100)
    print("5-STAGE 10-PARAMETER BLOCK-COORDINATE CMA-ES CAMPAIGN (STARTING FROM HF_TEST_03)")
    print("=" * 100)

    # Load or evaluate baseline HF_TEST_03
    if start_step == 1 or not os.path.exists(STATE_JSON):
        init_json_path = os.path.join(CAND_DIR, "HF_TEST_03_STAGE0_INIT.json")
        init_npz_path = os.path.join(CAND_DIR, "HF_TEST_03_STAGE0_INIT_ts.npz")
        base_res = None
        if os.path.exists(init_json_path) and os.path.exists(init_npz_path):
            try:
                with open(init_json_path, "r", encoding="utf-8") as f:
                    cached = json.load(f)
                if float(cached.get("total_score", 999999.0)) < 100000.0:
                    cached["idx"] = 0
                    cached["params_10"] = list(X0_HF_TEST_03)
                    base_res = cached
                    print("[Baseline] Loaded cached HF_TEST_03_STAGE0_INIT from disk.")
            except Exception:
                base_res = None
        if base_res is None:
            print("[Baseline] Evaluating HF_TEST_03_INIT with 10-parameter worker...")
            base_res = evaluate_worker(0, 0, 0, X0_HF_TEST_03, save_ts=True, custom_id="HF_TEST_03_STAGE0_INIT")
        theta_current = list(X0_HF_TEST_03)
        kp_prop_step1_best = float(X0_HF_TEST_03[3])
        global_best_score = float(base_res["total_score"])
        global_best_params = list(X0_HF_TEST_03)
        global_best_worker = base_res["worker"]
        init_ledger(base_res)
        with open(CHAMPION_JSON, "w", encoding="utf-8") as f:
            json.dump(base_res, f, indent=2)
        trigger_champion_plot(global_best_worker)
        print(
            f"[Baseline] HF_TEST_03_STAGE0_INIT Score = {global_best_score:.2f} "
            f"| {format_p10_short(theta_current)}"
        )
    else:
        with open(STATE_JSON, "r", encoding="utf-8") as f:
            st = json.load(f)
        theta_current = list(st["theta_current"])
        while len(theta_current) < 11:
            theta_current.append(0.60)
        kp_prop_step1_best = float(st.get("kp_prop_step1_best", theta_current[3]))
        global_best_score = float(st["global_best_score"])
        global_best_params = list(st["global_best_params"])
        while len(global_best_params) < 11:
            global_best_params.append(0.60)
        global_best_worker = st["global_best_worker"]
        print(f"[Resume] Loaded state at Step {start_step}: Global Best = {global_best_score:.2f} ({global_best_worker})")

    for stage in STAGES:
        s_num = stage["step"]
        if s_num < start_step or s_num > max_step:
            continue

        active_idx = stage["active_indices"]
        active_names = [PARAM_NAMES[i] for i in active_idx]
        n_dim = len(active_idx)
        n_gens = stage["gens"]
        disable_ccr = stage["disable_ccr"]
        stage_popsize = int(stage.get("popsize", POPSIZE))
        use_tpa = bool(stage.get("use_tpa", False))
        skip_sweep = bool(stage.get("skip_sweep", False))
        plateau_patience = int(stage.get("plateau_patience", 2))
        plateau_tol = float(stage.get("plateau_tol", 5.0))

        # Stage-specific initialization handoff:
        if s_num == 3:
            # Part 1: Initialize from W05_S5_G01 with kp_prop = 0.53 (lower bound 0.45) and Ki_task_pitch = 0.60:
            theta_current = list(X0_HF_TEST_03)

        stage_best_sub = [theta_current[i] for i in active_idx]
        x0_sub = list(stage_best_sub)
        lb_sub = [LOWER_BOUNDS[i] for i in active_idx]
        ub_sub = [UPPER_BOUNDS[i] for i in active_idx]
        base_sigmas = STAGE_FINE_SIGMAS if stage["sigma_scale"] < 0.99 else STAGE_STD_SIGMAS
        stds_sub = [base_sigmas[i] for i in active_idx]

        print("\n" + "=" * 100)
        print(f">>> {stage['title']} ({n_gens} Max Gens, Active Dim N={n_dim}, Pop={stage_popsize}, TPA={use_tpa})")
        print(f"    Active Variables : {active_names}")
        print(f"    Initial Sub-Vec  : {[round(v, 4) for v in x0_sub]}")
        print(f"    Initial Sigmas   : {[round(s, 4) for s in stds_sub]}")
        print(f"    Full 11D Start   : {format_p10_short(theta_current)}")
        print("=" * 100)

        stage_best_score = float("inf")
        stage_best_params10 = list(theta_current)
        stage_best_worker = f"S{s_num}_INIT"

        # Pre-Stage 1-Batch Laser Sweep before Steps 1-5 (unless skip_sweep=True):
        if s_num in (1, 2, 3, 4, 5) and not skip_sweep:
            t_sw0 = time.time()
            sweep_sub_cands = generate_pre_stage_sweep_candidates(
                s_num, x0_sub, lb_sub, ub_sub, stds_sub, popsize=stage_popsize
            )
            sweep_full_cands = [
                assemble_full_10params(theta_current, active_idx, sc, disable_ccr=disable_ccr)
                for sc in sweep_sub_cands
            ]
            sweep_results = [None] * stage_popsize
            with ThreadPoolExecutor(max_workers=min(stage_popsize, MAX_PARALLEL_WORKERS)) as pool:
                fut_map = {
                    pool.submit(
                        evaluate_worker,
                        w_idx,
                        s_num,
                        0,
                        sweep_full_cands[w_idx],
                        False,
                        f"W{w_idx + 1:02d}_S{s_num}_SWEEP",
                    ): w_idx
                    for w_idx in range(stage_popsize)
                }
                for fut in as_completed(fut_map):
                    res = fut.result()
                    sweep_results[res["idx"]] = res

            sw_scores = [float(r["total_score"]) for r in sweep_results]
            sw_best_idx = int(np.argmin(sw_scores))
            sw_best = sweep_results[sw_best_idx]
            sw_best_score = float(sw_best["total_score"])

            if sw_best_score <= stage_best_score:
                stage_best_score = sw_best_score
                stage_best_params10 = list(sw_best["params_10"])
                stage_best_sub = [stage_best_params10[i] for i in active_idx]
                stage_best_worker = sw_best["worker"]
                theta_current = list(stage_best_params10)
                x0_sub = list(stage_best_sub)
                if s_num == 1:
                    kp_prop_step1_best = float(theta_current[3])

            sw_flag = ""
            if sw_best_score < global_best_score:
                global_best_score = sw_best_score
                global_best_params = list(sw_best["params_10"])
                global_best_worker = sw_best["worker"]
                sw_flag = "★ NEW GLOBAL CHAMPION!"
                with open(CHAMPION_JSON, "w", encoding="utf-8") as f:
                    json.dump(sw_best, f, indent=2)
                trigger_champion_plot(global_best_worker)

            append_ledger_row(
                s_num, 0, n_dim, sw_best, stage_best_score, global_best_score, active_names
            )
            dt_sw = time.time() - t_sw0
            sw_tel = sw_best.get("telemetry", {})
            print(
                f"[S{s_num} SWEEP  | {n_dim}D | {dt_sw:4.1f}s] "
                f"SweepBest={sw_best_score:8.1f} ({sw_best['worker']}) | "
                f"Centering CMA-ES on {[round(v, 4) for v in x0_sub]} | "
                f"Bode={sw_tel.get('bode_score_without_hf', 0):6.1f} HF={sw_tel.get('hf_penalty', 0):5.1f} "
                f"DC_deg={sw_tel.get('dc_offset_rel_deg', 0):+5.2f} {sw_flag}"
            )

        cma_opts = {
            "popsize": stage_popsize,
            "bounds": [lb_sub, ub_sub],
            "CMA_stds": stds_sub,
            "CMA_active": True,
            "CMA_elitist": not use_tpa,
            "verbose": -9,
            "seed": 4200 + s_num * 100,
        }
        if use_tpa:
            cma_opts["AdaptSigma"] = cma.sigma_adaptation.CMAAdaptSigmaTPA

        es = cma.CMAEvolutionStrategy(x0_sub, 1.0, cma_opts)
        no_improve_streak = 0

        for gen in range(1, n_gens + 1):
            t_gen0 = time.time()
            # Dimension-adaptive orthogonal mirrored pairs in R^N (+ 2 TPA slots when use_tpa=True):
            sub_candidates = ask_orthogonal_mirrored_nd(es, stage_popsize, lb_sub, ub_sub, use_tpa=use_tpa)

            # Slot 0 Elitist Injection when not using TPA (or in Gen 1 before TPA has a mean shift):
            if not use_tpa or gen == 1:
                sub_candidates[0] = np.clip(np.array(stage_best_sub, dtype=float), lb_sub, ub_sub)

            # Check if this generation already completed on disk (e.g. G01 and G02 before an IDE restart):
            cached_gen_results = []
            for w_idx in range(stage_popsize):
                wid = f"W{w_idx + 1:02d}_S{s_num}_G{gen:02d}"
                out_json = os.path.join(CAND_DIR, f"{wid}.json")
                if os.path.exists(out_json):
                    try:
                        with open(out_json, "r", encoding="utf-8") as f:
                            cd = json.load(f)
                        p_vec = cd.get("params_11", cd.get("params_10"))
                        if float(cd.get("total_score", 6000000.0)) < 5999999.0 and p_vec is not None and len(p_vec) == 11:
                            cd["idx"] = w_idx
                            cd["wall_s"] = 0.0
                            cd["params_10"] = list(p_vec)
                            cached_gen_results.append(cd)
                    except Exception:
                        pass

            if len(cached_gen_results) == stage_popsize:
                results = cached_gen_results
                sub_candidates = [
                    np.array([r["params_10"][i] for i in active_idx], dtype=float)
                    for r in results
                ]
                replayed_from_cache = True
            else:
                replayed_from_cache = False
                full_candidates = [
                    assemble_full_10params(theta_current, active_idx, sc, disable_ccr=disable_ccr)
                    for sc in sub_candidates
                ]

                results = [None] * stage_popsize
                with ThreadPoolExecutor(max_workers=min(stage_popsize, MAX_PARALLEL_WORKERS)) as pool:
                    fut_map = {
                        pool.submit(evaluate_worker, w_idx, s_num, gen, full_candidates[w_idx]): w_idx
                        for w_idx in range(stage_popsize)
                    }
                    for fut in as_completed(fut_map):
                        res = fut.result()
                        results[res["idx"]] = res

            scores = [float(r["total_score"]) for r in results]
            es.tell(sub_candidates, scores)

            best_idx = int(np.argmin(scores))
            gen_best = results[best_idx]
            gen_best_score = float(gen_best["total_score"])

            prev_stage_best = stage_best_score
            improved_stage = False
            if gen_best_score < stage_best_score:
                stage_best_score = gen_best_score
                stage_best_params10 = list(gen_best["params_10"])
                stage_best_sub = [stage_best_params10[i] for i in active_idx]
                stage_best_worker = gen_best["worker"]
                theta_current = list(stage_best_params10)
                if s_num == 1:
                    kp_prop_step1_best = float(theta_current[3])
                improved_stage = True

            if (prev_stage_best - stage_best_score) >= plateau_tol:
                no_improve_streak = 0
            elif gen >= 2:
                no_improve_streak += 1

            improved_global = False
            if gen_best_score < global_best_score:
                global_best_score = gen_best_score
                global_best_params = list(gen_best["params_10"])
                global_best_worker = gen_best["worker"]
                improved_global = True
                with open(CHAMPION_JSON, "w", encoding="utf-8") as f:
                    json.dump(gen_best, f, indent=2)
                trigger_champion_plot(global_best_worker)

            if not replayed_from_cache:
                append_ledger_row(
                    s_num, gen, n_dim, gen_best, stage_best_score, global_best_score, active_names
                )

            # Save persistent state after every generation
            with open(STATE_JSON, "w", encoding="utf-8") as f:
                json.dump(
                    {
                        "last_completed_step": s_num,
                        "last_completed_gen": gen,
                        "theta_current": theta_current,
                        "kp_prop_step1_best": kp_prop_step1_best,
                        "stage_best_score": stage_best_score,
                        "stage_best_worker": stage_best_worker,
                        "global_best_score": global_best_score,
                        "global_best_params": global_best_params,
                        "global_best_worker": global_best_worker,
                    },
                    f,
                    indent=2,
                )

            dt_gen = time.time() - t_gen0
            tel = gen_best.get("telemetry", {})
            flag = "★ NEW GLOBAL CHAMPION!" if improved_global else ("+ Stage Best" if improved_stage else "")
            print(
                f"[S{s_num} G{gen:02d}/{n_gens:02d} | {n_dim}D | {dt_gen:4.1f}s] "
                f"GenBest={gen_best_score:8.1f} ({gen_best['worker']}) | "
                f"StageBest={stage_best_score:8.1f} | GlobalBest={global_best_score:8.1f} | "
                f"Bode={tel.get('bode_score_without_hf', 0):6.1f} HF={tel.get('hf_penalty', 0):5.1f} "
                f"DC_deg={tel.get('dc_offset_rel_deg', 0):+5.2f} {flag}"
            )
            print(f"    Best 11D: {format_p10_short(stage_best_params10)}")

            if gen >= 4 and no_improve_streak >= plateau_patience and gen < n_gens:
                print(
                    f"    [PLATEAU DETECTED] Stage {s_num} improved by < {plateau_tol:.1f} pts for "
                    f"{no_improve_streak} consecutive generations -> advancing to next stage!"
                )
                break

        # Handoff best parameter vector of this stage to initialize the next stage:
        theta_current = list(stage_best_params10)
        print(
            f"\n---> Completed Step {s_num} ({stage['code']}): Stage Best = {stage_best_score:.2f} ({stage_best_worker})"
        )
        print(f"---> Handoff to Next Step: {format_p10_short(theta_current)}")
        trigger_champion_plot(stage_best_worker)

    print("\n" + "=" * 100)
    print(f"5-STAGE CMA-ES CAMPAIGN COMPLETE! Global Champion: {global_best_worker} ({global_best_score:.2f})")
    print(f"Champion 11D Parameters: {format_p10_short(global_best_params)}")
    print("=" * 100)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--start-step", type=int, default=1, help="Stage number to start from (1..5)")
    parser.add_argument("--max-step", type=int, default=5, help="Stage number to stop after (1..5)")
    args = parser.parse_args()
    run_campaign(start_step=args.start_step, max_step=args.max_step)
