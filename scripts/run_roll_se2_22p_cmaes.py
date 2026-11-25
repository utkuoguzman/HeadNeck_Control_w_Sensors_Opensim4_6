#!/usr/bin/env python3
"""Cooperative Multi-Basin 22D Roll+SE2(3) CMA-ES targeting LatEC 0.1209 Hz phase (< 15 deg).

Locks 35 non-roll parameters (18 Pitch + 15 Yaw + 2 Fusimotor) from logs/coop_3axis_champion.json
and optimizes 18 Roll parameters + 4 SE2(3) lateral-translation voluntary roll parameters
(Kp_trans_roll, Kv_trans_roll, tau_vel_leak, tau_pos_leak) using compute_roll_0p12_rank()
with 3-axis verification every generation.
"""

import argparse
import json
import os
import pickle
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
from scripts.run_roll_0p12_phase_cmaes import (
    CHAMP_53_JSON,
    ROLL_NAMES,
    ROLL_SPECS,
    atomic_write_json,
    compute_roll_0p12_rank,
    safe_load_json,
)

SE2_SPECS = [
    ("Kp_trans_roll", -25.0, 25.0, 3.5),
    ("Kv_trans_roll", -15.0, 15.0, 2.0),
    ("tau_vel_leak",    0.8, 15.0, 2.0),
    ("tau_pos_leak",    0.5, 10.0, 1.2),
]
ROLL22_SPECS = list(ROLL_SPECS) + SE2_SPECS
ROLL22_NAMES = [s[0] for s in ROLL22_SPECS]

SE2_DEFAULTS = {
    "Kp_trans_roll": 0.0,
    "Kv_trans_roll": 0.0,
    "tau_vel_leak": 15.0,
    "tau_pos_leak": 5.0,
}

LOCK_PATH = os.path.join(LOG_DIR, "roll_se2_shared.lock")
BEST_ROLL22_JSON = os.path.join(LOG_DIR, "roll_se2_best_roll22.json")
CHAMPION_3AXIS_JSON = os.path.join(LOG_DIR, "roll_se2_3axis_champion.json")


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


def load_locked_base_and_roll22_seed(seed_preset="se2_pos", custom_seed_json=None):
    champ_doc = json.load(open(CHAMP_53_JSON, "r", encoding="utf-8"))
    base53 = {k: float(v) for k, v in champ_doc["params"].items()}
    roll18_set = set(ROLL_NAMES)
    locked_35 = {k: v for k, v in base53.items() if k not in roll18_set and k not in SE2_DEFAULTS}
    assert len(locked_35) == 35, f"Expected 35 locked params, got {len(locked_35)}"

    base_roll22 = {k: base53[k] for k in ROLL_NAMES}
    base_roll22.update(SE2_DEFAULTS)

    if custom_seed_json:
        cdoc = json.load(open(custom_seed_json, "r", encoding="utf-8"))
        csrc = cdoc.get("params_roll22") or cdoc.get("params") or cdoc
        seed_raw = {k: float(csrc.get(k, base_roll22[k])) for k in ROLL22_NAMES}
    elif seed_preset == "se2_pos":
        seed_raw = dict(base_roll22)
        seed_raw.update({"Kp_trans_roll": 5.0, "Kv_trans_roll": 1.0, "tau_vel_leak": 4.0, "tau_pos_leak": 2.5})
    elif seed_preset == "se2_neg":
        seed_raw = dict(base_roll22)
        seed_raw.update({"Kp_trans_roll": -5.0, "Kv_trans_roll": -1.0, "tau_vel_leak": 4.0, "tau_pos_leak": 2.5})
    elif seed_preset == "se2_emp":
        seed_raw = dict(base_roll22)
        seed_raw.update({"Kp_trans_roll": -2.26, "Kv_trans_roll": -0.02, "tau_vel_leak": 3.0, "tau_pos_leak": 2.0})
    else:
        seed_raw = dict(base_roll22)
        seed_raw.update({"Kp_trans_roll": 0.0, "Kv_trans_roll": 0.0, "tau_vel_leak": 5.0, "tau_pos_leak": 3.0})

    specs = [(name, float(np.clip(seed_raw[name], lb, ub)), lb, ub, sd) for name, lb, ub, sd in ROLL22_SPECS]
    return locked_35, base_roll22, specs


def _sim_tagged_job(job):
    tag, cand_idx, mode, overrides = job
    res = run_axis_worker((mode, overrides))
    if res.get("false_booleans"):
        raise RuntimeError(f"Disabled booleans in {tag}/{cand_idx}/{mode}: {res['false_booleans']}")
    fr = res.get("freq_results") or {}
    f0 = fr.get("0.1209") or {}
    db_5p583 = float(fr["5.583"]["gain_db_diff"]) if "5.583" in fr else None
    return {
        "tag": tag,
        "cand_idx": cand_idx,
        "mode": mode,
        "aborted": bool(res["aborted"]),
        "abort_time_s": float(res["abort_time_s"]),
        "quiet_main_max_deg": float(res.get("quiet_main_max_deg", 0.0)),
        "classic_bode_score": float(res["classic_bode_score"]),
        "pitch_crosstalk_penalty": float(res["pitch_crosstalk_penalty"]),
        "hf_penalty": float(res["hf_penalty"]),
        "total_score": float(res["total_score"]),
        "hf_noise_rms": float(res["hf_noise_rms"]),
        "corr_vel": float(res["corr_vel"]),
        "ph_0p12": float(f0.get("sim_phase", 0.0)),
        "ph_err_0p12": float(f0.get("phase_err", 999.0)),
        "ph_loss_0p12": float(f0.get("phase_loss_pts_raw", 998001.0)),
        "g_0p12": float(f0.get("sim_gain", 0.0)),
        "db_0p12": float(f0.get("gain_db_diff", 0.0)),
        "db_5p583": db_5p583,
    }


def _make_rec_roll22(gen, slot, roll_id, rl, vec):
    b5f = float(rl["classic_bode_score"])
    ph_loss = float(rl["ph_loss_0p12"])
    xl = float(rl["pitch_crosstalk_penalty"])
    hl = float(rl["hf_penalty"])
    hfl = float(rl["hf_noise_rms"])
    cv = float(rl["corr_vel"])
    abl = bool(rl["aborted"])
    qm = float(rl["quiet_main_max_deg"])
    rk = compute_roll_0p12_rank(b5f, ph_loss, xl, hl, hfl, cv, abl)
    if qm > 0.05:
        rk += 50000.0 + 10000.0 * ((qm - 0.05) ** 2)
    return rk, {
        "gen": gen,
        "slot": slot,
        "roll_id": roll_id,
        "rank_score": round(rk, 2),
        "bode_5f": round(b5f, 2),
        "bode_all6": round(b5f + ph_loss, 2),
        "ph_0p12": round(float(rl["ph_0p12"]), 2),
        "ph_err_0p12": round(float(rl["ph_err_0p12"]), 2),
        "ph_loss_0p12": round(ph_loss, 2),
        "g_0p12": round(float(rl["g_0p12"]), 3),
        "db_0p12": round(float(rl["db_0p12"]), 3),
        "xtalk_latec": round(xl, 2),
        "hf_latec": round(hl, 2),
        "hf_rms_latec": round(hfl, 3),
        "quiet_main_max_deg": round(qm, 4),
        "corr_vel": round(cv, 4),
        "aborted": abl,
        "params_roll22": {k: float(v) for k, v in zip(ROLL22_NAMES, vec)},
    }


def ensure_shared_best(base_roll22):
    """Initialize shared best deterministically from the clean champ+SE2_DEFAULTS baseline."""
    with _SharedLock(LOCK_PATH):
        if not os.path.exists(BEST_ROLL22_JSON):
            atomic_write_json(BEST_ROLL22_JSON, {
                "roll_id": "init_champ_se2_zero",
                "rank_score": 11731.1,
                "bode_5f": 2207.7,
                "bode_all6": 11347.2,
                "ph_0p12": -208.8,
                "ph_err_0p12": 95.6,
                "hf_rms_latec": 1.90,
                "quiet_main_max_deg": 0.0,
                "corr_vel": 0.9254,
                "aborted": False,
                "gen": 0,
                "params_roll22": {k: float(base_roll22[k]) for k in ROLL22_NAMES},
            })


def update_shared_roll22(rec_roll22):
    with _SharedLock(LOCK_PATH):
        cur = safe_load_json(BEST_ROLL22_JSON, None)
        cur_clean = bool(
            cur and (not cur.get("aborted", False))
            and float(cur.get("hf_rms_latec", 99.0)) <= 3.5
            and float(cur.get("corr_vel", 0.0)) >= 0.88
            and float(cur.get("quiet_main_max_deg", 0.0)) <= 0.05
        )
        new_clean = bool(
            (not rec_roll22["aborted"])
            and float(rec_roll22["hf_rms_latec"]) <= 3.5
            and float(rec_roll22["corr_vel"]) >= 0.88
            and float(rec_roll22["quiet_main_max_deg"]) <= 0.05
        )
        better = False
        if cur is None:
            better = True
        elif new_clean and not cur_clean:
            better = True
        elif new_clean == cur_clean and float(rec_roll22["rank_score"]) < float(cur.get("rank_score", 1e12)):
            better = True
        if better:
            atomic_write_json(BEST_ROLL22_JSON, rec_roll22)
        return better


def update_shared_3axis_champion(rec_3ax):
    with _SharedLock(LOCK_PATH):
        cur = safe_load_json(CHAMPION_3AXIS_JSON, None)
        db5 = rec_3ax.get("db_5p583_apeo")
        qmax = float(rec_3ax.get("quiet_max_deg_all3", 0.0))
        eligible = bool(
            (not rec_3ax["aborted"])
            and float(rec_3ax["hf_rms_latec"]) <= 3.5
            and float(rec_3ax["corr_vel_latec"]) >= 0.88
            and float(rec_3ax["bode_apeo"]) <= 2250.0
            and (db5 is None or abs(float(db5)) <= 1.40)
            and float(rec_3ax["bode_yawma"]) <= 780.0
            and qmax <= 0.05
        )
        if not eligible:
            return False
        better = (cur is None) or (float(rec_3ax["rank_roll_0p12"]) < float(cur.get("rank_roll_0p12", 1e12)))
        if better:
            atomic_write_json(CHAMPION_3AXIS_JSON, rec_3ax)
        return better


def main():
    parser = argparse.ArgumentParser(description="Cooperative 22D Roll+SE2(3) CMA-ES for LatEC 0.1209 Hz phase")
    parser.add_argument("--run-tag", type=str, required=True)
    parser.add_argument("--seed-preset", choices=("se2_pos", "se2_neg", "se2_zero", "se2_emp"), default="se2_pos")
    parser.add_argument("--custom-seed-json", type=str, default=None)
    parser.add_argument("--pop-roll", type=int, default=36)
    parser.add_argument("--gens", type=int, default=150)
    parser.add_argument("--workers", type=int, default=40)
    parser.add_argument("--seed", type=int, default=7101)
    parser.add_argument("--sigma0", type=float, default=0.50)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--dry-check", action="store_true")
    args = parser.parse_args()

    os.makedirs(LOG_DIR, exist_ok=True)
    np.random.seed(args.seed)
    tag = args.run_tag.strip()

    locked_35, base_roll22, specs = load_locked_base_and_roll22_seed(args.seed_preset, args.custom_seed_json)
    x0 = [float(s[1]) for s in specs]
    lb = [float(s[2]) for s in specs]
    ub = [float(s[3]) for s in specs]
    stds = [float(s[4]) for s in specs]

    if args.dry_check:
        assert len(locked_35) + len(specs) == 57, f"Expected 57, got {len(locked_35) + len(specs)}"
        print(f"[DRY-CHECK] tag={tag} preset={args.seed_preset} pop={args.pop_roll} workers={args.workers} 35+22=57 OK")
        for n, v, l, u, s in specs[-4:]:
            print(f"  {n:24s}: x0={v:+8.4f} bounds=[{l}, {u}] sd={s}")
        return

    ensure_shared_best(base_roll22)
    locked_items = [(k, f"{float(v):.15g}") for k, v in locked_35.items()]

    pkl_path = os.path.join(LOG_DIR, f"roll_se2_cma_state_{tag}.pkl")
    hist_path = os.path.join(LOG_DIR, f"roll_se2_history_{tag}.json")
    ledger_path = os.path.join(LOG_DIR, f"roll_se2_ledger_{tag}.md")

    es_roll = cma.CMAEvolutionStrategy(x0, args.sigma0, {
        "popsize": args.pop_roll,
        "bounds": [lb, ub],
        "CMA_stds": stds,
        "CMA_active": True,
        "CMA_elitist": False,
        "verbose": -9,
        "seed": args.seed,
    })

    history = []
    start_gen = 1
    if args.resume and os.path.exists(pkl_path) and os.path.exists(hist_path):
        with open(pkl_path, "rb") as f:
            es_roll = pickle.load(f)
        history = safe_load_json(hist_path, []) or []
        if history:
            start_gen = int(history[-1]["gen"]) + 1
            print(f"=== Resumed tag={tag} from Gen {start_gen - 1} (sigma={es_roll.sigma:.3f}) ===", flush=True)

    if not os.path.exists(ledger_path) or start_gen == 1:
        with open(ledger_path, "w", encoding="utf-8") as f:
            f.write(f"# Roll SE2(3) 22D Optimization Ledger (tag={tag}, preset={args.seed_preset})\n\n")
            f.write(f"Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | pop={args.pop_roll} workers={args.workers}\n\n")
            f.write("| Gen | Clean | BestRank | Best5f | BestPh0.12 (Err) | BestAll6 | BestHF | Corr | BestSE2 (Kp,Kv) | Mean5f/Ph0.12 | V3_APEO | V3_LatEC (5f/Ph0.12/HF) | V3_YawMA | Time | Sigma |\n")
            f.write("|---:|---:|---:|---:|---:|---:|---:|---:|---|---|---:|---|---:|---:|---:|\n")

    for gen in range(start_gen, args.gens + 1):
        t0 = time.time()
        shared_doc = safe_load_json(BEST_ROLL22_JSON, None) or {}
        shared_roll = shared_doc.get("params_roll22", base_roll22)
        shared_roll_id = shared_doc.get("roll_id", "init_champ_se2_zero")

        cand_r = ask_orthogonal_mirrored_nd(es_roll, args.pop_roll, lb, ub, use_tpa=False)
        if gen == 1 and start_gen == 1:
            cand_r[0] = np.asarray(x0, dtype=float)
        mean_r_vec = np.clip(np.asarray(es_roll.mean, dtype=float), lb, ub)

        jobs = []
        for idx, vec in enumerate(cand_r):
            ovr = locked_items + [(k, f"{float(v):.15g}") for k, v in zip(ROLL22_NAMES, vec)]
            jobs.append(("R", idx, "LatEC", ovr))
        jobs.append(("RM", 0, "LatEC", locked_items + [(k, f"{float(v):.15g}") for k, v in zip(ROLL22_NAMES, mean_r_vec)]))

        verify_params = dict(locked_35)
        verify_params.update({k: float(shared_roll[k]) for k in ROLL22_NAMES})
        verify_ovr = [(k, f"{float(v):.15g}") for k, v in verify_params.items()]
        jobs.append(("V", 0, "APEO", verify_ovr))
        jobs.append(("V", 1, "LatEC", verify_ovr))
        jobs.append(("V", 2, "YawMA", verify_ovr))

        results = {}
        with ProcessPoolExecutor(max_workers=args.workers) as ex:
            futs = [ex.submit(_sim_tagged_job, job) for job in jobs]
            for fut in as_completed(futs):
                res = fut.result()
                results[(res["tag"], res["cand_idx"], res["mode"])] = res

        va = results[("V", 0, "APEO")]
        vl = results[("V", 1, "LatEC")]
        vy = results[("V", 2, "YawMA")]
        v_r_rank = compute_roll_0p12_rank(
            vl["classic_bode_score"], vl["ph_loss_0p12"], vl["pitch_crosstalk_penalty"],
            vl["hf_penalty"], vl["hf_noise_rms"], vl["corr_vel"], vl["aborted"]
        )
        v_qmax = max(va["quiet_main_max_deg"], vl["quiet_main_max_deg"], vy["quiet_main_max_deg"])
        v_tot_5f = float(va["total_score"] + vl["total_score"] + vy["total_score"])
        rec_3ax = {
            "gen": gen,
            "source_tag": tag,
            "partner_roll_id": shared_roll_id,
            "updated_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "rank_roll_0p12": round(v_r_rank, 2),
            "total_score_5f": round(v_tot_5f, 2),
            "total_score_all6": round(v_tot_5f + float(vl["ph_loss_0p12"]), 2),
            "bode_apeo": round(float(va["classic_bode_score"]), 2),
            "db_5p583_apeo": round(float(va["db_5p583"]), 3) if va["db_5p583"] is not None else None,
            "bode_latec_5f": round(float(vl["classic_bode_score"]), 2),
            "bode_latec_all6": round(float(vl["classic_bode_score"] + vl["ph_loss_0p12"]), 2),
            "ph_0p12": round(float(vl["ph_0p12"]), 2),
            "ph_err_0p12": round(float(vl["ph_err_0p12"]), 2),
            "ph_loss_0p12": round(float(vl["ph_loss_0p12"]), 2),
            "db_0p12": round(float(vl["db_0p12"]), 3),
            "bode_yawma": round(float(vy["classic_bode_score"]), 2),
            "hf_rms_apeo": round(float(va["hf_noise_rms"]), 3),
            "hf_rms_latec": round(float(vl["hf_noise_rms"]), 3),
            "hf_rms_yawma": round(float(vy["hf_noise_rms"]), 3),
            "corr_vel_latec": round(float(vl["corr_vel"]), 4),
            "quiet_max_deg_all3": round(v_qmax, 4),
            "xtalk_apeo": round(float(va["pitch_crosstalk_penalty"]), 2),
            "xtalk_latec": round(float(vl["pitch_crosstalk_penalty"]), 2),
            "xtalk_yawma": round(float(vy["pitch_crosstalk_penalty"]), 2),
            "aborted": bool(va["aborted"] or vl["aborted"] or vy["aborted"]),
            "params": verify_params,
        }
        if update_shared_3axis_champion(rec_3ax):
            print(
                f"  [NEW 3AX CHAMP] Ph0.12={rec_3ax['ph_0p12']:.1f}({rec_3ax['ph_err_0p12']:+.1f}°) "
                f"L_5f={rec_3ax['bode_latec_5f']:,.1f} L_all6={rec_3ax['bode_latec_all6']:,.1f} HF={rec_3ax['hf_rms_latec']:.2f} "
                f"Q={rec_3ax['quiet_max_deg_all3']:.3f}° A={rec_3ax['bode_apeo']:,.1f}(db5p5={rec_3ax['db_5p583_apeo']}) Y={rec_3ax['bode_yawma']:,.1f}",
                flush=True,
            )

        f_roll, r_records = [], []
        r_clean_count = 0
        for idx, vec in enumerate(cand_r):
            rk, rec = _make_rec_roll22(gen, idx, f"{tag}_g{gen}_s{idx}", results[("R", idx, "LatEC")], vec)
            if (not rec["aborted"]) and rec["hf_rms_latec"] <= 3.5 and rec["corr_vel"] >= 0.88 and rec["quiet_main_max_deg"] <= 0.05:
                r_clean_count += 1
            f_roll.append(rk)
            r_records.append(rec)
        es_roll.tell(cand_r, f_roll)

        _, r_mean_rec = _make_rec_roll22(gen, -1, f"{tag}_g{gen}_mean", results[("RM", 0, "LatEC")], mean_r_vec)
        best_r_gen = min(
            r_records + [r_mean_rec],
            key=lambda c: (0 if (not c["aborted"] and c["hf_rms_latec"] <= 3.5 and c["corr_vel"] >= 0.88 and c["quiet_main_max_deg"] <= 0.05) else 1, c["rank_score"]),
        )
        if update_shared_roll22(best_r_gen):
            bp = best_r_gen["params_roll22"]
            print(
                f"  [SHARED ROLL22 UPDATE] Gen {gen} ({best_r_gen['roll_id']}) "
                f"Ph0.12={best_r_gen['ph_0p12']:.1f}({best_r_gen['ph_err_0p12']:+.1f}°) "
                f"5f={best_r_gen['bode_5f']:,.1f} All6={best_r_gen['bode_all6']:,.1f} HF={best_r_gen['hf_rms_latec']:.2f} "
                f"SE2=({bp['Kp_trans_roll']:+.2f},{bp['Kv_trans_roll']:+.2f},{bp['tau_vel_leak']:.1f},{bp['tau_pos_leak']:.1f})",
                flush=True,
            )

        dt = time.time() - t0
        cur_best_r = safe_load_json(BEST_ROLL22_JSON) or best_r_gen
        cbp = cur_best_r.get("params_roll22", base_roll22)

        with open(pkl_path, "wb") as f:
            pickle.dump(es_roll, f)
        history.append({
            "gen": gen,
            "run_tag": tag,
            "elapsed_s": round(dt, 1),
            "sigma": float(es_roll.sigma),
            "r_clean_count": r_clean_count,
            "best_gen": best_r_gen,
            "mean": {k: r_mean_rec[k] for k in ("rank_score", "bode_5f", "bode_all6", "ph_0p12", "ph_err_0p12", "hf_rms_latec", "corr_vel", "quiet_main_max_deg", "aborted")},
            "verify_3axis": {k: rec_3ax[k] for k in ("rank_roll_0p12", "bode_apeo", "db_5p583_apeo", "bode_latec_5f", "bode_latec_all6", "ph_0p12", "ph_err_0p12", "hf_rms_latec", "corr_vel_latec", "quiet_max_deg_all3", "bode_yawma", "aborted")},
        })
        atomic_write_json(hist_path, history)

        with open(ledger_path, "a", encoding="utf-8") as f:
            f.write(
                f"| {gen} | {r_clean_count}/{args.pop_roll} | {cur_best_r['rank_score']:,.1f} | "
                f"{cur_best_r['bode_5f']:,.1f} | {cur_best_r['ph_0p12']:.1f} ({cur_best_r['ph_err_0p12']:+.1f}°) | "
                f"{cur_best_r['bode_all6']:,.1f} | {cur_best_r['hf_rms_latec']:.2f} | {cur_best_r['corr_vel']:+.3f} | "
                f"({cbp['Kp_trans_roll']:+.2f},{cbp['Kv_trans_roll']:+.2f}) | "
                f"{r_mean_rec['bode_5f']:,.0f}/{r_mean_rec['ph_err_0p12']:+.1f}° | {rec_3ax['bode_apeo']:,.1f} | "
                f"{rec_3ax['bode_latec_5f']:,.0f}/{rec_3ax['ph_err_0p12']:+.1f}°/{rec_3ax['hf_rms_latec']:.2f} | "
                f"{rec_3ax['bode_yawma']:,.1f} | {dt:.1f}s | {es_roll.sigma:.3f} |\n"
            )

        print(
            f"Gen {gen:03d} [{tag}] ({dt:.1f}s) | Clean={r_clean_count}/{args.pop_roll} "
            f"SharedBest: 5f={cur_best_r['bode_5f']:,.1f} Ph={cur_best_r['ph_0p12']:.1f}({cur_best_r['ph_err_0p12']:+.1f}°) "
            f"All6={cur_best_r['bode_all6']:,.1f} HF={cur_best_r['hf_rms_latec']:.2f} SE2=({cbp['Kp_trans_roll']:+.2f},{cbp['Kv_trans_roll']:+.2f}) | "
            f"Mean={r_mean_rec['bode_5f']:,.0f}({r_mean_rec['ph_err_0p12']:+.1f}°) | "
            f"V3: A={rec_3ax['bode_apeo']:,.0f} L={rec_3ax['bode_latec_5f']:,.0f}({rec_3ax['ph_err_0p12']:+.1f}°) Y={rec_3ax['bode_yawma']:,.0f} | sig={es_roll.sigma:.3f}",
            flush=True,
        )


if __name__ == "__main__":
    main()
