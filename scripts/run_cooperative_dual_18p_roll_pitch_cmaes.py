#!/usr/bin/env python3
"""Cooperative Tri-Axis (18D Roll + 18D Pitch + 15D Yaw) CSA CMA-ES (2 Fusimotor params frozen).

Supports four execution modes sharing atomic cooperative state in logs/:
  - dual  : advances ES_Roll (LatEC) + ES_Pitch (APEO) + 3-axis verification in one wave
  - roll  : advances ES_Roll (LatEC) + 3-axis verification, syncing partner best_pitch18/yaw15
  - pitch : advances ES_Pitch (APEO) + verification, syncing partner best_roll18/yaw15
  - yaw   : advances ES_Yaw (YawMA) + verification, syncing partner best_roll18/pitch18
"""

import argparse
import json
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from datetime import datetime

for _env_k in (
    "OMP_NUM_THREADS",
    "OPENBLAS_NUM_THREADS",
    "MKL_NUM_THREADS",
    "NUMEXPR_NUM_THREADS",
    "SIMBODY_NUM_THREADS",
):
    os.environ.setdefault(_env_k, "1")

import cma
import numpy as np

try:
    import fcntl
except ImportError:
    fcntl = None

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_SCRIPT_DIR)
for _p in (_ROOT, _SCRIPT_DIR, os.path.join(_ROOT, "util")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from scripts.generate_standard_plot import run_axis_worker
from scripts.run_3axis_42param_cmaes import LOG_DIR, ask_orthogonal_mirrored_nd

G35_CHAMP_JSON = os.path.join(LOG_DIR, "cmaes_3axis_18p_roll_champion.json")
G2_CLEAN_CHAMP_JSON = os.path.join(LOG_DIR, "cmaes_latec_only_18p_clean_champion.json")

BEST_ROLL18_JSON = os.path.join(LOG_DIR, "coop_best_roll18.json")
BEST_PITCH18_JSON = os.path.join(LOG_DIR, "coop_best_pitch18.json")
BEST_YAW15_JSON = os.path.join(LOG_DIR, "coop_best_yaw15.json")
CHAMPION_3AXIS_JSON = os.path.join(LOG_DIR, "coop_3axis_champion.json")
LOCK_PATH = os.path.join(LOG_DIR, "coop_shared.lock")


class _SharedLock:
    """Best-effort inter-process file lock on POSIX nodes sharing GPFS/Lustre."""

    def __init__(self, lock_path):
        self.lock_path = lock_path
        self.fp = None

    def __enter__(self):
        if fcntl is not None:
            os.makedirs(os.path.dirname(self.lock_path), exist_ok=True)
            self.fp = open(self.lock_path, "a+", encoding="utf-8")
            fcntl.flock(self.fp.fileno(), fcntl.LOCK_EX)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.fp is not None:
            try:
                fcntl.flock(self.fp.fileno(), fcntl.LOCK_UN)
            finally:
                self.fp.close()
                self.fp = None


def atomic_write_json(path, payload):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = f"{path}.tmp.{os.getpid()}"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)
    os.replace(tmp, path)


def safe_load_json(path, default=None):
    for attempt in range(4):
        if not os.path.exists(path):
            return default
        try:
            with open(path, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            if attempt == 3:
                return default
            time.sleep(0.05 * (attempt + 1))
    return default


def _sim_tagged_job(job):
    tag, cand_idx, mode, overrides = job
    res = run_axis_worker((mode, overrides))
    fr = res.get("freq_results") or {}
    db_5p583 = float(fr["5.583"]["gain_db_diff"]) if "5.583" in fr else None
    return {
        "tag": tag,
        "cand_idx": cand_idx,
        "mode": mode,
        "aborted": bool(res["aborted"]),
        "abort_time_s": float(res["abort_time_s"]),
        "classic_bode_score": float(res["classic_bode_score"]),
        "pitch_crosstalk_penalty": float(res["pitch_crosstalk_penalty"]),
        "hf_penalty": float(res["hf_penalty"]),
        "total_score": float(res["total_score"]),
        "hf_noise_rms": float(res["hf_noise_rms"]),
        "db_5p583": db_5p583,
    }


def compute_roll_rank(bode_l, xtalk_l, hf_l, hf_rms_l, aborted=False):
    latec_canon = float(bode_l + xtalk_l + hf_l)
    if aborted:
        return latec_canon + 100000.0
    wall = 0.0 if hf_rms_l <= 3.5 else (20000.0 + 3000.0 * (hf_rms_l - 3.5) ** 2)
    return float(latec_canon + wall)


def compute_pitch_rank(bode_a, xtalk_a, hf_a, hf_rms_a, aborted=False, db_5p583=None, pitch_max_5p5_db=0.0):
    apeo_canon = float(bode_a + xtalk_a + hf_a)
    if aborted:
        return apeo_canon + 100000.0
    wall = 20.0 * max(0.0, hf_rms_a - 3.5) ** 2 + (
        0.0 if hf_rms_a <= 6.8 else (15000.0 + 2000.0 * (hf_rms_a - 6.8) ** 2)
    )
    if pitch_max_5p5_db > 0.0 and db_5p583 is not None:
        excess_5p5 = max(0.0, abs(float(db_5p583)) - float(pitch_max_5p5_db))
        wall += 1500.0 * (excess_5p5 ** 2) + 80.0 * (float(db_5p583) ** 2)
    return float(apeo_canon + wall)


def compute_yaw_rank(bode_y, xtalk_y, hf_y, hf_rms_y, aborted=False):
    yawma_canon = float(bode_y + xtalk_y + hf_y)
    if aborted:
        return yawma_canon + 100000.0
    wall = 0.0 if hf_rms_y <= 3.0 else (15000.0 + 2500.0 * (hf_rms_y - 3.0) ** 2)
    return float(yawma_canon + wall)


def build_roll_specs_stiff_lowphas(roll_specs):
    """Build stiff low-phasic Roll specs keeping identical bounds and CMA_stds."""
    ov_x0 = {
        "K_phas_roll": 7.5,
        "p_phas_roll": 6.5,
        "K_ton_roll": 6.5,
        "p_ton_roll": 8.0,
        "Ki_task_roll": 0.15,
        "Kp_prop_roll": 4.2,
        "Kd_task_roll": 1.8,
        "Kp_task_roll": 1.8,
        "tau_lead_pole_roll": 0.016,
        "Kp_proprioception_roll": 0.025,
        "Kp_buckle_roll": 0.65,
        "Kd_buckle_roll": 0.22,
    }
    return [
        (n, float(np.clip(ov_x0.get(n, x0), lb, ub)), lb, ub, sd)
        for n, x0, lb, ub, sd in roll_specs
    ]


def build_specs():
    g35 = {k: float(v) for k, v in json.load(open(G35_CHAMP_JSON, "r", encoding="utf-8"))["params"].items()}
    g2c = {k: float(v) for k, v in json.load(open(G2_CLEAN_CHAMP_JSON, "r", encoding="utf-8"))["params"].items()}

    roll_specs = [
        ("K_sc_roll",               g2c["K_sc_roll"],               1.00,  10.00, 0.80),
        ("z_sc_roll",               g2c["z_sc_roll"],               0.002, 0.055, 0.0050),
        ("p_sc_roll",               g2c["p_sc_roll"],               12.0,  160.0, 16.0),
        ("K_phas_roll",             g2c["K_phas_roll"],             1.00,  28.00, 3.00),
        ("z_phas_roll",             g2c["z_phas_roll"],             0.0,   0.040, 0.0045),
        ("p_phas_roll",             g2c["p_phas_roll"],             0.80,  16.00, 1.60),
        ("K_ton_roll",              3.20,                           0.20,  14.00, 1.80),
        ("p_ton_roll",              g2c["p_ton_roll"],              0.40,  25.00, 2.50),
        ("Kp_prop_roll",            g2c["Kp_prop_roll"],            0.20,  10.00, 0.90),
        ("z_ccr_roll",              g2c["z_ccr_roll"],              0.0,   0.045, 0.0045),
        ("p_ccr_roll",              g2c["p_ccr_roll"],              20.0,  160.0, 16.0),
        ("Kp_proprioception_roll",  g2c["Kp_proprioception_roll"],  0.003, 0.120, 0.012),
        ("Kp_task_roll",            2.40,                           0.20,  10.00, 1.10),
        ("Kd_task_roll",            g2c["Kd_task_roll"],            0.10,  4.00,  0.40),
        ("Ki_task_roll",            g2c["Ki_task_roll"],            0.02,  4.00,  0.35),
        ("tau_lead_pole_roll",      0.012,                          0.004, 0.045, 0.0045),
        ("Kp_buckle_roll",          g2c["Kp_buckle_roll"],          0.20,  4.00,  0.40),
        ("Kd_buckle_roll",          g2c["Kd_buckle_roll"],          0.02,  0.55,  0.045),
    ]

    pitch_specs = [
        ("tau_lead_pole",      g35["tau_lead_pole"],      0.004, 0.035, 0.0025),
        ("K_sc",               g35["K_sc"],               2.00,  8.50,  0.45),
        ("z_sc",               g35["z_sc"],               0.003, 0.040, 0.0030),
        ("p_sc",               g35["p_sc"],               20.0,  110.0, 7.0),
        ("K_phas",             g35["K_phas"],             4.00,  20.00, 1.10),
        ("z_phas",             g35["z_phas"],             0.001, 0.035, 0.0030),
        ("p_phas",             g35["p_phas"],             1.50,  14.00, 1.10),
        ("K_ton",              g35["K_ton"],              2.00,  14.00, 1.10),
        ("p_ton",              g35["p_ton"],              4.00,  35.00, 3.00),
        ("Kp_prop_pitch",      g35["Kp_prop_pitch"],      0.15,  2.50,  0.15),
        ("z_ccr",              g35["z_ccr"],              0.003, 0.045, 0.0035),
        ("p_ccr",              g35["p_ccr"],              30.0,  160.0, 14.0),
        ("Kp_proprioception",  g35["Kp_proprioception"],  0.005, 0.120, 0.010),
        ("Kp_task_pitch",      g35["Kp_task_pitch"],      1.50,  10.00, 0.75),
        ("Kd_task_pitch",      g35["Kd_task_pitch"],      0.15,  2.00,  0.15),
        ("Ki_task_pitch",      g35["Ki_task_pitch"],      0.30,  7.50,  0.55),
        ("Kp_buckle",          g35["Kp_buckle"],          0.30,  4.00,  0.30),
        ("Kd_buckle",          g35["Kd_buckle"],          0.02,  0.50,  0.035),
    ]

    yaw_specs = [
        ("tau_lead_pole_yaw",      g35["tau_lead_pole_yaw"],      0.004, 0.040, 0.0025),
        ("K_sc_yaw",               g35["K_sc_yaw"],               0.20,  6.00,  0.30),
        ("z_sc_yaw",               g35["z_sc_yaw"],               0.001, 0.060, 0.0040),
        ("p_sc_yaw",               g35["p_sc_yaw"],               0.20,  25.0,  0.40),
        ("K_ton_yaw",              g35["K_ton_yaw"],              1.00,  20.0,  1.20),
        ("p_ton_yaw",              g35["p_ton_yaw"],              2.00,  40.0,  2.50),
        ("Kp_prop_yaw",            g35["Kp_prop_yaw"],            1.00,  20.0,  1.20),
        ("z_ccr_yaw",              g35["z_ccr_yaw"],              0.001, 0.055, 0.0035),
        ("p_ccr_yaw",              g35["p_ccr_yaw"],              20.0,  180.0, 12.0),
        ("Kp_proprioception_yaw",  g35["Kp_proprioception_yaw"],  0.005, 0.120, 0.008),
        ("Kp_task_yaw",            g35["Kp_task_yaw"],            0.05,  3.00,  0.12),
        ("Kd_task_yaw",            g35["Kd_task_yaw"],            0.01,  1.00,  0.030),
        ("Ki_task_yaw",            g35["Ki_task_yaw"],            0.005, 1.00,  0.025),
        ("Kp_buckle_yaw",          g35["Kp_buckle_yaw"],          0.20,  5.00,  0.30),
        ("Kd_buckle_yaw",          g35["Kd_buckle_yaw"],          0.02,  0.80,  0.040),
    ]

    roll_names = [s[0] for s in roll_specs]
    pitch_names = [s[0] for s in pitch_specs]
    yaw_names = [s[0] for s in yaw_specs]
    roll_set = set(roll_names)
    pitch_set = set(pitch_names)
    yaw_set = set(yaw_names)
    frozen_17 = {k: float(v) for k, v in g35.items() if k not in roll_set and k not in pitch_set}
    frozen_2 = {k: float(v) for k, v in g35.items() if k not in roll_set and k not in pitch_set and k not in yaw_set}

    assert len(roll_names) == 18, f"Expected 18 roll params, got {len(roll_names)}"
    assert len(pitch_names) == 18, f"Expected 18 pitch params, got {len(pitch_names)}"
    assert len(yaw_names) == 15, f"Expected 15 yaw params, got {len(yaw_names)}"
    assert len(frozen_2) == 2, f"Expected 2 frozen params, got {len(frozen_2)}"
    assert len(roll_set & pitch_set) == 0, "Overlap between roll and pitch"
    assert len(roll_set & yaw_set) == 0, "Overlap between roll and yaw"
    assert len(pitch_set & yaw_set) == 0, "Overlap between pitch and yaw"
    assert len(roll_set | pitch_set | yaw_set | set(frozen_2)) == 53, "Total params != 53"

    roll_specs_stiff_lowphas = build_roll_specs_stiff_lowphas(roll_specs)

    seed_roll_g2c = {k: float(np.clip(g2c[k], lb, ub)) for k, _x0, lb, ub, _sd in roll_specs}
    seed_roll_stiff = dict(seed_roll_g2c)
    for k_ov, v_ov in (("K_ton_roll", 5.5), ("p_ton_roll", 8.0), ("Kp_task_roll", 3.2), ("tau_lead_pole_roll", 0.014)):
        lb_k, ub_k = next((lb, ub) for n, _x0, lb, ub, _sd in roll_specs if n == k_ov)
        seed_roll_stiff[k_ov] = float(np.clip(v_ov, lb_k, ub_k))

    seed_roll_stiff_lowphas = {k: float(x0) for k, x0, _lb, _ub, _sd in roll_specs_stiff_lowphas}
    seed_roll_ultrastiff = dict(seed_roll_stiff_lowphas)
    for k_ov, v_ov in (
        ("K_phas_roll", 5.0),
        ("p_phas_roll", 9.0),
        ("K_ton_roll", 9.0),
        ("p_ton_roll", 11.0),
        ("Ki_task_roll", 0.08),
        ("Kp_proprioception_roll", 0.012),
    ):
        lb_k, ub_k = next((lb, ub) for n, _x0, lb, ub, _sd in roll_specs if n == k_ov)
        seed_roll_ultrastiff[k_ov] = float(np.clip(v_ov, lb_k, ub_k))

    seed_pitch_g35 = {k: float(np.clip(g35[k], lb, ub)) for k, _x0, lb, ub, _sd in pitch_specs}
    seed_pitch_stiff = dict(seed_pitch_g35)
    for k_ov, v_ov in (("Kp_task_pitch", 5.8), ("Ki_task_pitch", 2.8), ("K_ton", 7.5)):
        lb_k, ub_k = next((lb, ub) for n, _x0, lb, ub, _sd in pitch_specs if n == k_ov)
        seed_pitch_stiff[k_ov] = float(np.clip(v_ov, lb_k, ub_k))

    seed_yaw_g35 = {k: float(np.clip(g35[k], lb, ub)) for k, _x0, lb, ub, _sd in yaw_specs}

    return {
        "roll_specs": roll_specs,
        "roll_specs_stiff_lowphas": roll_specs_stiff_lowphas,
        "pitch_specs": pitch_specs,
        "yaw_specs": yaw_specs,
        "roll_names": roll_names,
        "pitch_names": pitch_names,
        "yaw_names": yaw_names,
        "frozen_17": frozen_17,
        "frozen_2": frozen_2,
        "seed_roll_g2c": seed_roll_g2c,
        "seed_roll_stiff": seed_roll_stiff,
        "seed_roll_stiff_lowphas": seed_roll_stiff_lowphas,
        "seed_roll_ultrastiff": seed_roll_ultrastiff,
        "seed_pitch_g35": seed_pitch_g35,
        "seed_pitch_stiff": seed_pitch_stiff,
        "seed_yaw_g35": seed_yaw_g35,
    }


def ensure_shared_partners(cfg):
    with _SharedLock(LOCK_PATH):
        if not os.path.exists(BEST_ROLL18_JSON):
            atomic_write_json(BEST_ROLL18_JSON, {
                "roll_id": "init_g0",
                "rank_score": 1e9,
                "initial_rank_score": 1e9,
                "effective_rank": 1e9,
                "verify_scores": [],
                "latec_canon": 1e9,
                "bode_latec": 37873.35,
                "hf_rms_latec": 0.773,
                "aborted": False,
                "gen": 0,
                "params_roll18": cfg["seed_roll_g2c"],
            })
        if not os.path.exists(BEST_PITCH18_JSON):
            atomic_write_json(BEST_PITCH18_JSON, {
                "rank_score": 1e9,
                "apeo_canon": 1e9,
                "bode_apeo": 4791.85,
                "hf_rms_apeo": 6.652,
                "aborted": False,
                "gen": 0,
                "params_pitch18": cfg["seed_pitch_g35"],
            })
        if not os.path.exists(BEST_YAW15_JSON):
            atomic_write_json(BEST_YAW15_JSON, {
                "rank_score": 1e9,
                "yawma_canon": 1e9,
                "bode_yawma": 4021.2,
                "hf_rms_yawma": 1.49,
                "aborted": False,
                "gen": 0,
                "params_yaw15": cfg["seed_yaw_g35"],
            })


def update_shared_roll(rec_roll18):
    with _SharedLock(LOCK_PATH):
        cur = safe_load_json(BEST_ROLL18_JSON, None)
        cur_clean = bool(
            cur
            and (not cur.get("verified_aborted", cur.get("aborted", False)))
            and float(cur.get("verified_hf_rms_latec", cur.get("hf_rms_latec", 99.0))) <= 3.5
        )
        new_clean = bool((not rec_roll18["aborted"]) and float(rec_roll18["hf_rms_latec"]) <= 3.5)
        cur_eff = float(cur.get("effective_rank", cur.get("rank_score", 1e12))) if cur else 1e12
        better = False
        if cur is None:
            better = True
        elif new_clean and not cur_clean:
            better = True
        elif new_clean == cur_clean and float(rec_roll18["rank_score"]) < cur_eff:
            better = True
        if better:
            payload = dict(rec_roll18)
            payload["initial_rank_score"] = round(float(rec_roll18["rank_score"]), 2)
            payload["effective_rank"] = round(float(rec_roll18["rank_score"]), 2)
            payload["verify_scores"] = []
            atomic_write_json(BEST_ROLL18_JSON, payload)
        return better


def record_roll_verification(partner_roll_id, v_rank, v_bode, v_hf, v_aborted):
    if not partner_roll_id:
        return None
    with _SharedLock(LOCK_PATH):
        cur = safe_load_json(BEST_ROLL18_JSON, None)
        if not cur or cur.get("roll_id") != partner_roll_id:
            return None
        scores = [float(s) for s in cur.get("verify_scores", [])]
        scores.append(round(float(v_rank), 2))
        if len(scores) > 5:
            scores = scores[-5:]
        cur["verify_scores"] = scores
        cur["effective_rank"] = round(float(np.median(scores)), 2)
        cur["verified_bode_latec"] = round(float(v_bode), 2)
        cur["verified_hf_rms_latec"] = round(float(v_hf), 3)
        cur["verified_aborted"] = bool(v_aborted)
        atomic_write_json(BEST_ROLL18_JSON, cur)
        return cur["effective_rank"]


def update_shared_pitch(rec_pitch18):
    with _SharedLock(LOCK_PATH):
        cur = safe_load_json(BEST_PITCH18_JSON, None)
        better = (cur is None) or (float(rec_pitch18["rank_score"]) < float(cur.get("rank_score", 1e12)))
        if better:
            atomic_write_json(BEST_PITCH18_JSON, rec_pitch18)
        return better


def update_shared_yaw(rec_yaw15):
    with _SharedLock(LOCK_PATH):
        cur = safe_load_json(BEST_YAW15_JSON, None)
        cur_clean = bool(cur and (not cur.get("aborted", False)) and float(cur.get("hf_rms_yawma", 99.0)) <= 3.0)
        new_clean = bool((not rec_yaw15["aborted"]) and float(rec_yaw15["hf_rms_yawma"]) <= 3.0)
        better = False
        if cur is None:
            better = True
        elif new_clean and not cur_clean:
            better = True
        elif new_clean == cur_clean and float(rec_yaw15["rank_score"]) < float(cur.get("rank_score", 1e12)):
            better = True
        if better:
            atomic_write_json(BEST_YAW15_JSON, rec_yaw15)
        return better


def update_shared_3axis_champion(rec_3ax):
    with _SharedLock(LOCK_PATH):
        cur = safe_load_json(CHAMPION_3AXIS_JSON, None)
        cur_clean = bool(cur and (not cur.get("aborted", False)) and float(cur.get("hf_rms_latec", 99.0)) <= 3.5)
        new_clean = bool((not rec_3ax["aborted"]) and float(rec_3ax["hf_rms_latec"]) <= 3.5)
        better = False
        if cur is None:
            better = True
        elif new_clean and not cur_clean:
            better = True
        elif new_clean == cur_clean and float(rec_3ax["total_score"]) < float(cur.get("total_score", 1e12)):
            better = True
        if better:
            atomic_write_json(CHAMPION_3AXIS_JSON, rec_3ax)
        return better


def _make_cma(specs, popsize, seed):
    x0 = [float(s[1]) for s in specs]
    lb = [float(s[2]) for s in specs]
    ub = [float(s[3]) for s in specs]
    stds = [float(s[4]) for s in specs]
    es = cma.CMAEvolutionStrategy(x0, 0.50, {
        "popsize": popsize,
        "bounds": [lb, ub],
        "CMA_stds": stds,
        "CMA_active": True,
        "CMA_elitist": False,
        "verbose": -9,
        "seed": seed,
    })
    return es, lb, ub


def _make_rec_roll(gen, slot, roll_id, rl, roll_names, vec):
    bl, xl, hl = float(rl["classic_bode_score"]), float(rl["pitch_crosstalk_penalty"]), float(rl["hf_penalty"])
    hfl, abl = float(rl["hf_noise_rms"]), bool(rl["aborted"])
    rk = compute_roll_rank(bl, xl, hl, hfl, abl)
    return rk, {
        "gen": gen,
        "slot": slot,
        "roll_id": roll_id,
        "rank_score": round(rk, 2),
        "latec_canon": round(bl + xl + hl, 2),
        "bode_latec": round(bl, 2),
        "xtalk_latec": round(xl, 2),
        "hf_latec": round(hl, 2),
        "hf_rms_latec": round(hfl, 3),
        "aborted": abl,
        "params_roll18": {k: float(v) for k, v in zip(roll_names, vec)},
    }


def _make_rec_pitch(gen, slot, ra, pitch_names, vec, pitch_max_5p5_db=0.0):
    ba, xa, ha = float(ra["classic_bode_score"]), float(ra["pitch_crosstalk_penalty"]), float(ra["hf_penalty"])
    hfa, aba = float(ra["hf_noise_rms"]), bool(ra["aborted"])
    db5 = ra.get("db_5p583")
    rk = compute_pitch_rank(ba, xa, ha, hfa, aba, db_5p583=db5, pitch_max_5p5_db=pitch_max_5p5_db)
    return rk, {
        "gen": gen,
        "slot": slot,
        "rank_score": round(rk, 2),
        "apeo_canon": round(ba + xa + ha, 2),
        "bode_apeo": round(ba, 2),
        "xtalk_apeo": round(xa, 2),
        "hf_apeo": round(ha, 2),
        "hf_rms_apeo": round(hfa, 3),
        "db_5p583": round(float(db5), 3) if db5 is not None else None,
        "aborted": aba,
        "params_pitch18": {k: float(v) for k, v in zip(pitch_names, vec)},
    }


def _make_rec_yaw(gen, slot, ry, yaw_names, vec):
    by, xy, hy = float(ry["classic_bode_score"]), float(ry["pitch_crosstalk_penalty"]), float(ry["hf_penalty"])
    hfy, aby = float(ry["hf_noise_rms"]), bool(ry["aborted"])
    rk = compute_yaw_rank(by, xy, hy, hfy, aby)
    return rk, {
        "gen": gen,
        "slot": slot,
        "rank_score": round(rk, 2),
        "yawma_canon": round(by + xy + hy, 2),
        "bode_yawma": round(by, 2),
        "xtalk_yawma": round(xy, 2),
        "hf_yawma": round(hy, 2),
        "hf_rms_yawma": round(hfy, 3),
        "aborted": aby,
        "params_yaw15": {k: float(v) for k, v in zip(yaw_names, vec)},
    }


def main():
    parser = argparse.ArgumentParser(description="Cooperative Tri-Axis 18D Roll + 18D Pitch + 15D Yaw CSA CMA-ES")
    parser.add_argument("--mode", choices=("dual", "roll", "pitch", "yaw"), default="dual")
    parser.add_argument("--pop-roll", type=int, default=72)
    parser.add_argument("--pop-pitch", type=int, default=36)
    parser.add_argument("--pop-yaw", type=int, default=16)
    parser.add_argument("--workers", type=int, default=112)
    parser.add_argument("--gens", type=int, default=160)
    parser.add_argument("--seed", type=int, default=4718)
    parser.add_argument("--run-tag", type=str, default="")
    parser.add_argument("--init-roll-preset", choices=("default", "stiff_lowphas"), default="default")
    parser.add_argument("--pitch-max-5p5-db", type=float, default=0.0, help="Target |dB| ceiling at 5.583 Hz for Pitch (0 = disabled)")
    parser.add_argument("--init-pitch-from-best", action="store_true", help="Seed Pitch CMA-ES x0 from coop_best_pitch18.json")
    parser.add_argument("--skip-latec-verify", action="store_true")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--dry-check", action="store_true")
    args = parser.parse_args()

    os.makedirs(LOG_DIR, exist_ok=True)
    np.random.seed(args.seed)

    cfg = build_specs()
    roll_names = cfg["roll_names"]
    pitch_names = cfg["pitch_names"]
    yaw_names = cfg["yaw_names"]
    frozen_2 = cfg["frozen_2"]

    run_roll = args.mode in ("dual", "roll")
    run_pitch = args.mode in ("dual", "pitch")
    run_yaw = args.mode == "yaw"
    run_tag_clean = args.run_tag.strip()
    roll_tag = run_tag_clean or args.mode
    hist_suffix = f"{args.mode}_{run_tag_clean}" if run_tag_clean else args.mode

    if args.dry_check:
        test_bg = dict(frozen_2)
        test_bg.update(cfg["seed_pitch_g35"])
        test_bg.update(cfg["seed_yaw_g35"])
        test_bg.update(
            cfg["seed_roll_stiff_lowphas"] if args.init_roll_preset == "stiff_lowphas" else cfg["seed_roll_g2c"]
        )
        assert len(test_bg) == 53, f"Expected 53 assembled params, got {len(test_bg)}"
        print(
            f"[DRY-CHECK] mode={args.mode} tag={hist_suffix} init_roll_preset={args.init_roll_preset} "
            f"pop_roll={args.pop_roll} pop_pitch={args.pop_pitch} pop_yaw={args.pop_yaw} "
            f"workers={args.workers} skip_latec_verify={args.skip_latec_verify} | "
            f"Roll(18)+Pitch(18)+Yaw(15)+Frozen(2)=53 OK"
        )
        print("Roll(18):", roll_names)
        print("Pitch(18):", pitch_names)
        print("Yaw(15):", yaw_names)
        print("Frozen(2):", sorted(frozen_2.keys()))
        return

    ensure_shared_partners(cfg)

    roll_specs_use = (
        cfg["roll_specs_stiff_lowphas"] if args.init_roll_preset == "stiff_lowphas" else cfg["roll_specs"]
    )
    pitch_specs_use = list(cfg["pitch_specs"])
    if args.init_pitch_from_best:
        bp_doc = safe_load_json(BEST_PITCH18_JSON, None) or {}
        bp_p = bp_doc.get("params_pitch18") or {}
        if bp_p:
            pitch_specs_use = [
                (n, float(np.clip(bp_p.get(n, x0), lb, ub)), lb, ub, sd)
                for n, x0, lb, ub, sd in pitch_specs_use
            ]
    es_roll, lb_r, ub_r = _make_cma(roll_specs_use, args.pop_roll, args.seed + 11) if run_roll else (None, None, None)
    es_pitch, lb_p, ub_p = _make_cma(pitch_specs_use, args.pop_pitch, args.seed + 29) if run_pitch else (None, None, None)
    es_yaw, lb_y, ub_y = _make_cma(cfg["yaw_specs"], args.pop_yaw, args.seed + 47) if run_yaw else (None, None, None)

    hist_path = os.path.join(LOG_DIR, f"coop_history_{hist_suffix}.json")
    ledger_path = os.path.join(LOG_DIR, f"coop_ledger_{hist_suffix}.md")

    history = []
    start_gen = 1
    if args.resume and os.path.exists(hist_path):
        history = safe_load_json(hist_path, []) or []
        for g_rec in history:
            if run_roll and es_roll is not None:
                rcands = g_rec.get("roll_candidates", [])
                if len(rcands) == args.pop_roll:
                    Xr = [np.asarray([c["params_roll18"][k] for k in roll_names], dtype=float) for c in rcands]
                    fr = [float(c["rank_score"]) for c in rcands]
                    es_roll.ask()
                    es_roll.tell(Xr, fr)
            if run_pitch and es_pitch is not None:
                pcands = g_rec.get("pitch_candidates", [])
                if len(pcands) == args.pop_pitch:
                    Xp = [np.asarray([c["params_pitch18"][k] for k in pitch_names], dtype=float) for c in pcands]
                    fp = [float(c["rank_score"]) for c in pcands]
                    es_pitch.ask()
                    es_pitch.tell(Xp, fp)
            if run_yaw and es_yaw is not None:
                ycands = g_rec.get("yaw_candidates", [])
                if len(ycands) == args.pop_yaw:
                    Xy = [np.asarray([c["params_yaw15"][k] for k in yaw_names], dtype=float) for c in ycands]
                    fy = [float(c["rank_score"]) for c in ycands]
                    es_yaw.ask()
                    es_yaw.tell(Xy, fy)
        if history:
            start_gen = int(history[-1]["gen"]) + 1
            print(f"=== Resumed mode={args.mode} (tag={hist_suffix}) from Gen {start_gen - 1} ===", flush=True)

    if not os.path.exists(ledger_path) or start_gen == 1:
        with open(ledger_path, "w", encoding="utf-8") as f:
            f.write(f"# Cooperative Tri-Axis Ledger (mode={args.mode}, tag={hist_suffix})\n\n")
            f.write(
                f"Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | "
                f"pop_roll={args.pop_roll}, pop_pitch={args.pop_pitch}, pop_yaw={args.pop_yaw}, "
                f"workers={args.workers}, roll_preset={args.init_roll_preset}, skip_latec={args.skip_latec_verify}\n\n"
            )
            f.write("| Gen | R_Clean | Best_LatEC_Bode | Best_Roll_HF | Best_APEO_Bode | Best_Pitch_HF | Best_YawMA_Bode | Best_Yaw_HF | Mean_Bode | 3Axis_Total | V_APEO | V_LatEC (HF) | V_YawMA | Elapsed | Sigmas |\n")
            f.write("|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|\n")

    for gen in range(start_gen, args.gens + 1):
        t0 = time.time()
        partner_roll_doc = safe_load_json(BEST_ROLL18_JSON, None) or {}
        partner_roll = partner_roll_doc.get("params_roll18", cfg["seed_roll_g2c"])
        partner_roll_id = partner_roll_doc.get("roll_id")
        partner_pitch_doc = safe_load_json(BEST_PITCH18_JSON, None) or {}
        partner_pitch = partner_pitch_doc.get("params_pitch18", cfg["seed_pitch_g35"])
        partner_yaw_doc = safe_load_json(BEST_YAW15_JSON, None) or {}
        partner_yaw = partner_yaw_doc.get("params_yaw15", cfg["seed_yaw_g35"])

        jobs = []
        cand_r = []
        mean_r_vec = None
        if run_roll:
            cand_r = ask_orthogonal_mirrored_nd(es_roll, args.pop_roll, lb_r, ub_r, use_tpa=False)
            if gen == 1 and start_gen == 1:
                if args.init_roll_preset == "stiff_lowphas":
                    cand_r[0] = np.asarray([cfg["seed_roll_stiff_lowphas"][k] for k in roll_names], dtype=float)
                    if len(cand_r) > 1:
                        cand_r[1] = np.asarray([cfg["seed_roll_ultrastiff"][k] for k in roll_names], dtype=float)
                else:
                    cand_r[0] = np.asarray([cfg["seed_roll_g2c"][k] for k in roll_names], dtype=float)
                    if len(cand_r) > 1:
                        cand_r[1] = np.asarray([cfg["seed_roll_stiff"][k] for k in roll_names], dtype=float)
            mean_r_vec = np.clip(np.asarray(es_roll.mean, dtype=float), lb_r, ub_r)
            bg_r = dict(frozen_2)
            bg_r.update(partner_pitch)
            bg_r.update(partner_yaw)
            bg_r_items = [(k, f"{float(v):.8g}") for k, v in bg_r.items()]
            for idx, vec in enumerate(cand_r):
                ovr = bg_r_items + [(k, f"{float(v):.8g}") for k, v in zip(roll_names, vec)]
                jobs.append(("R", idx, "LatEC", ovr))
            jobs.append(("RM", 0, "LatEC", bg_r_items + [(k, f"{float(v):.8g}") for k, v in zip(roll_names, mean_r_vec)]))

        cand_p = []
        mean_p_vec = None
        if run_pitch:
            cand_p = ask_orthogonal_mirrored_nd(es_pitch, args.pop_pitch, lb_p, ub_p, use_tpa=False)
            if gen == 1 and start_gen == 1:
                cand_p[0] = np.asarray([cfg["seed_pitch_g35"][k] for k in pitch_names], dtype=float)
                if len(cand_p) > 1:
                    cand_p[1] = np.asarray([cfg["seed_pitch_stiff"][k] for k in pitch_names], dtype=float)
            mean_p_vec = np.clip(np.asarray(es_pitch.mean, dtype=float), lb_p, ub_p)
            bg_p = dict(frozen_2)
            bg_p.update(partner_roll)
            bg_p.update(partner_yaw)
            bg_p_items = [(k, f"{float(v):.8g}") for k, v in bg_p.items()]
            for idx, vec in enumerate(cand_p):
                ovr = bg_p_items + [(k, f"{float(v):.8g}") for k, v in zip(pitch_names, vec)]
                jobs.append(("P", idx, "APEO", ovr))
            jobs.append(("PM", 0, "APEO", bg_p_items + [(k, f"{float(v):.8g}") for k, v in zip(pitch_names, mean_p_vec)]))

        cand_y = []
        mean_y_vec = None
        if run_yaw:
            cand_y = ask_orthogonal_mirrored_nd(es_yaw, args.pop_yaw, lb_y, ub_y, use_tpa=False)
            if gen == 1 and start_gen == 1:
                cand_y[0] = np.asarray([cfg["seed_yaw_g35"][k] for k in yaw_names], dtype=float)
            mean_y_vec = np.clip(np.asarray(es_yaw.mean, dtype=float), lb_y, ub_y)
            bg_y = dict(frozen_2)
            bg_y.update(partner_roll)
            bg_y.update(partner_pitch)
            bg_y_items = [(k, f"{float(v):.8g}") for k, v in bg_y.items()]
            for idx, vec in enumerate(cand_y):
                ovr = bg_y_items + [(k, f"{float(v):.8g}") for k, v in zip(yaw_names, vec)]
                jobs.append(("Y", idx, "YawMA", ovr))
            jobs.append(("YM", 0, "YawMA", bg_y_items + [(k, f"{float(v):.8g}") for k, v in zip(yaw_names, mean_y_vec)]))

        verify_params = dict(frozen_2)
        verify_params.update(partner_roll)
        verify_params.update(partner_pitch)
        verify_params.update(partner_yaw)
        verify_ovr = [(k, f"{float(v):.8g}") for k, v in verify_params.items()]
        jobs.append(("V", 0, "APEO", verify_ovr))
        if not args.skip_latec_verify:
            jobs.append(("V", 1, "LatEC", verify_ovr))
        jobs.append(("V", 2, "YawMA", verify_ovr))

        results = {}
        with ProcessPoolExecutor(max_workers=args.workers) as ex:
            futs = [ex.submit(_sim_tagged_job, job) for job in jobs]
            for fut in as_completed(futs):
                res = fut.result()
                results[(res["tag"], res["cand_idx"], res["mode"])] = res

        va = results[("V", 0, "APEO")]
        vl = results.get(("V", 1, "LatEC"))
        vy = results[("V", 2, "YawMA")]
        if vl is not None:
            bl_v, xl_v, hl_v = float(vl["classic_bode_score"]), float(vl["pitch_crosstalk_penalty"]), float(vl["hf_penalty"])
            hfl_v, abl_v = float(vl["hf_noise_rms"]), bool(vl["aborted"])
            v_r_rank = compute_roll_rank(bl_v, xl_v, hl_v, hfl_v, abl_v)
            new_eff = record_roll_verification(partner_roll_id, v_r_rank, bl_v, hfl_v, abl_v)
            if new_eff is not None:
                print(f"  [ROLL VERIFY] partner={partner_roll_id} V_LatEC={bl_v:,.1f}(HF={hfl_v:.2f}) -> eff_rank={new_eff:,.1f}", flush=True)

            v_tot = float(va["total_score"] + vl["total_score"] + vy["total_score"])
            v_aborted = bool(va["aborted"] or vl["aborted"] or vy["aborted"])
            rec_3ax = {
                "gen": gen,
                "source_mode": args.mode,
                "source_tag": hist_suffix,
                "updated_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "total_score": round(v_tot, 2),
                "bode_apeo": round(float(va["classic_bode_score"]), 2),
                "bode_latec": round(float(vl["classic_bode_score"]), 2),
                "bode_yawma": round(float(vy["classic_bode_score"]), 2),
                "hf_rms_apeo": round(float(va["hf_noise_rms"]), 3),
                "hf_rms_latec": round(float(vl["hf_noise_rms"]), 3),
                "hf_rms_yawma": round(float(vy["hf_noise_rms"]), 3),
                "xtalk_apeo": round(float(va["pitch_crosstalk_penalty"]), 2),
                "xtalk_latec": round(float(vl["pitch_crosstalk_penalty"]), 2),
                "xtalk_yawma": round(float(vy["pitch_crosstalk_penalty"]), 2),
                "aborted": v_aborted,
                "params": verify_params,
            }
            if update_shared_3axis_champion(rec_3ax):
                print(
                    f"  [NEW 3-AXIS CHAMP] Total={rec_3ax['total_score']:,.2f} | "
                    f"APEO={rec_3ax['bode_apeo']:,.1f}(HF={rec_3ax['hf_rms_apeo']:.2f}) "
                    f"LatEC={rec_3ax['bode_latec']:,.1f}(HF={rec_3ax['hf_rms_latec']:.2f}) "
                    f"YawMA={rec_3ax['bode_yawma']:,.1f}(HF={rec_3ax['hf_rms_yawma']:.2f})",
                    flush=True,
                )
        else:
            rec_3ax = {
                "gen": gen,
                "source_mode": args.mode,
                "source_tag": hist_suffix,
                "total_score": None,
                "bode_apeo": round(float(va["classic_bode_score"]), 2),
                "bode_latec": None,
                "bode_yawma": round(float(vy["classic_bode_score"]), 2),
                "hf_rms_apeo": round(float(va["hf_noise_rms"]), 3),
                "hf_rms_latec": None,
                "hf_rms_yawma": round(float(vy["hf_noise_rms"]), 3),
                "aborted": bool(va["aborted"] or vy["aborted"]),
            }

        mean_parts = []
        r_records = []
        r_mean_rec = None
        r_clean_count = 0
        if run_roll:
            f_roll = []
            for idx, vec in enumerate(cand_r):
                rk, rec = _make_rec_roll(gen, idx, f"{roll_tag}_g{gen}_s{idx}", results[("R", idx, "LatEC")], roll_names, vec)
                if (not rec["aborted"]) and rec["hf_rms_latec"] <= 3.5:
                    r_clean_count += 1
                f_roll.append(rk)
                r_records.append(rec)
            es_roll.tell(cand_r, f_roll)
            _, r_mean_rec = _make_rec_roll(gen, -1, f"{roll_tag}_g{gen}_mean", results[("RM", 0, "LatEC")], roll_names, mean_r_vec)
            mean_parts.append(f"R={r_mean_rec['bode_latec']:,.0f}({r_mean_rec['hf_rms_latec']:.2f})")
            best_r_gen = min(
                r_records + [r_mean_rec],
                key=lambda c: (0 if (not c["aborted"] and c["hf_rms_latec"] <= 3.5) else 1, c["rank_score"]),
            )
            if update_shared_roll(best_r_gen):
                print(
                    f"  [SHARED ROLL UPDATE] Gen {gen} ({best_r_gen['roll_id']}) "
                    f"BodeL={best_r_gen['bode_latec']:,.2f} RollHF={best_r_gen['hf_rms_latec']:.3f}",
                    flush=True,
                )

        p_records = []
        p_mean_rec = None
        if run_pitch:
            f_pitch = []
            for idx, vec in enumerate(cand_p):
                rk, rec = _make_rec_pitch(gen, idx, results[("P", idx, "APEO")], pitch_names, vec, pitch_max_5p5_db=args.pitch_max_5p5_db)
                f_pitch.append(rk)
                p_records.append(rec)
            es_pitch.tell(cand_p, f_pitch)
            _, p_mean_rec = _make_rec_pitch(gen, -1, results[("PM", 0, "APEO")], pitch_names, mean_p_vec, pitch_max_5p5_db=args.pitch_max_5p5_db)
            mean_parts.append(f"P={p_mean_rec['bode_apeo']:,.0f}({p_mean_rec['hf_rms_apeo']:.2f})")
            best_p_gen = min(p_records + [p_mean_rec], key=lambda c: c["rank_score"])
            if update_shared_pitch(best_p_gen):
                print(
                    f"  [SHARED PITCH UPDATE] Gen {gen} slot={best_p_gen['slot']} "
                    f"BodeA={best_p_gen['bode_apeo']:,.2f} PitchHF={best_p_gen['hf_rms_apeo']:.3f} db5p5={best_p_gen.get('db_5p583')}",
                    flush=True,
                )

        y_records = []
        y_mean_rec = None
        if run_yaw:
            f_yaw = []
            for idx, vec in enumerate(cand_y):
                rk, rec = _make_rec_yaw(gen, idx, results[("Y", idx, "YawMA")], yaw_names, vec)
                f_yaw.append(rk)
                y_records.append(rec)
            es_yaw.tell(cand_y, f_yaw)
            _, y_mean_rec = _make_rec_yaw(gen, -1, results[("YM", 0, "YawMA")], yaw_names, mean_y_vec)
            mean_parts.append(f"Y={y_mean_rec['bode_yawma']:,.0f}({y_mean_rec['hf_rms_yawma']:.2f})")
            best_y_gen = min(
                y_records + [y_mean_rec],
                key=lambda c: (0 if (not c["aborted"] and c["hf_rms_yawma"] <= 3.0) else 1, c["rank_score"]),
            )
            if update_shared_yaw(best_y_gen):
                print(
                    f"  [SHARED YAW UPDATE] Gen {gen} slot={best_y_gen['slot']} "
                    f"BodeY={best_y_gen['bode_yawma']:,.2f} YawHF={best_y_gen['hf_rms_yawma']:.3f}",
                    flush=True,
                )

        dt = time.time() - t0
        cur_best_r = safe_load_json(BEST_ROLL18_JSON) or {}
        cur_best_p = safe_load_json(BEST_PITCH18_JSON) or {}
        cur_best_y = safe_load_json(BEST_YAW15_JSON) or {}
        sig_parts = []
        if run_roll:
            sig_parts.append(f"R={es_roll.sigma:.3f}")
        if run_pitch:
            sig_parts.append(f"P={es_pitch.sigma:.3f}")
        if run_yaw:
            sig_parts.append(f"Y={es_yaw.sigma:.3f}")
        sig_str = " ".join(sig_parts)
        mean_str = " ".join(mean_parts) if mean_parts else "-"

        v_tot_str = f"{rec_3ax['total_score']:,.1f}" if rec_3ax["total_score"] is not None else "-"
        v_lat_str = f"{rec_3ax['bode_latec']:,.1f} ({rec_3ax['hf_rms_latec']:.2f})" if rec_3ax["bode_latec"] is not None else "skipped"
        v_lat_short = f"{rec_3ax['bode_latec']:,.0f}[{rec_3ax['hf_rms_latec']:.2f}]" if rec_3ax["bode_latec"] is not None else "skip"

        history.append({
            "gen": gen,
            "mode": args.mode,
            "run_tag": run_tag_clean,
            "elapsed_s": round(dt, 1),
            "roll_sigma": float(es_roll.sigma) if run_roll else None,
            "pitch_sigma": float(es_pitch.sigma) if run_pitch else None,
            "yaw_sigma": float(es_yaw.sigma) if run_yaw else None,
            "roll_candidates": r_records,
            "pitch_candidates": p_records,
            "yaw_candidates": y_records,
            "roll_mean": {k: r_mean_rec[k] for k in ("bode_latec", "hf_rms_latec", "rank_score", "aborted")} if r_mean_rec else None,
            "pitch_mean": {k: p_mean_rec[k] for k in ("bode_apeo", "hf_rms_apeo", "rank_score", "aborted")} if p_mean_rec else None,
            "yaw_mean": {k: y_mean_rec[k] for k in ("bode_yawma", "hf_rms_yawma", "rank_score", "aborted")} if y_mean_rec else None,
            "verify_3axis": {k: rec_3ax[k] for k in ("total_score", "bode_apeo", "bode_latec", "bode_yawma", "hf_rms_apeo", "hf_rms_latec", "hf_rms_yawma", "aborted")},
        })
        atomic_write_json(hist_path, history)

        with open(ledger_path, "a", encoding="utf-8") as f:
            f.write(
                f"| {gen} | {r_clean_count}/{args.pop_roll if run_roll else 0} | "
                f"{cur_best_r.get('bode_latec', 0):,.1f} | {cur_best_r.get('hf_rms_latec', 0):.3f} | "
                f"{cur_best_p.get('bode_apeo', 0):,.1f} | {cur_best_p.get('hf_rms_apeo', 0):.3f} | "
                f"{cur_best_y.get('bode_yawma', 0):,.1f} | {cur_best_y.get('hf_rms_yawma', 0):.3f} | "
                f"{mean_str} | {v_tot_str} | {rec_3ax['bode_apeo']:,.1f} | "
                f"{v_lat_str} | {rec_3ax['bode_yawma']:,.1f} | {dt:.1f}s | {sig_str} |\n"
            )

        print(
            f"Gen {gen:03d} [{hist_suffix}] ({dt:.1f}s) | R_Clean={r_clean_count}/{args.pop_roll if run_roll else 0} "
            f"BestR={cur_best_r.get('bode_latec', 0):,.1f}(eff={cur_best_r.get('effective_rank', cur_best_r.get('rank_score', 0)):,.0f}) "
            f"BestP={cur_best_p.get('bode_apeo', 0):,.1f} BestY={cur_best_y.get('bode_yawma', 0):,.1f} | "
            f"Mean[{mean_str}] | V3 Tot={v_tot_str} (A={rec_3ax['bode_apeo']:,.0f}, L={v_lat_short}, Y={rec_3ax['bode_yawma']:,.0f}) | {sig_str}",
            flush=True,
        )


if __name__ == "__main__":
    main()
