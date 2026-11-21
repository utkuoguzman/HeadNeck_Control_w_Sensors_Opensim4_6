#!/usr/bin/env python3
"""Cooperative Multi-Basin 18D Roll CMA-ES targeting LatEC 0.1209 Hz phase (< 15 deg).

Freezes the 18 Pitch, 15 Yaw, and 2 Fusimotor parameters from logs/coop_3axis_champion.json
so APEO and YawMA remain protected while optimizing the 18 axis-decoupled Roll parameters
using compute_roll_0p12_rank() and 3-axis verification every generation.
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

CHAMP_53_JSON = os.path.join(LOG_DIR, "coop_3axis_champion.json")
G35_CHAMP_JSON = os.path.join(LOG_DIR, "cmaes_3axis_18p_roll_champion.json")

LOCK_PATH = os.path.join(LOG_DIR, "roll_0p12_shared.lock")
BEST_ROLL18_JSON = os.path.join(LOG_DIR, "roll_0p12_best_roll18.json")
CHAMPION_3AXIS_JSON = os.path.join(LOG_DIR, "roll_0p12_3axis_champion.json")

ROLL_SPECS = [
    ("K_sc_roll",              1.00,  10.00, 0.70),
    ("z_sc_roll",              0.002, 0.055, 0.0045),
    ("p_sc_roll",              12.0,  140.0, 14.0),
    ("K_phas_roll",            0.50,  28.00, 3.00),
    ("z_phas_roll",            0.0,   0.040, 0.0045),
    ("p_phas_roll",            0.25,  16.00, 1.80),
    ("K_ton_roll",             0.02,  16.00, 1.60),
    ("p_ton_roll",             0.50,  25.00, 2.50),
    ("Kp_prop_roll",           0.20,  10.00, 0.80),
    ("z_ccr_roll",             0.0,   0.045, 0.0045),
    ("p_ccr_roll",             20.0,  160.0, 16.0),
    ("Kp_proprioception_roll", 0.003, 0.120, 0.012),
    ("Kp_task_roll",           0.02,  12.00, 1.20),
    ("Kd_task_roll",           0.05,  6.00,  0.55),
    ("Ki_task_roll",           0.00,  8.00,  0.55),
    ("tau_lead_pole_roll",     0.004, 0.045, 0.0045),
    ("Kp_buckle_roll",         0.20,  4.00,  0.40),
    ("Kd_buckle_roll",         0.02,  0.55,  0.045),
]
ROLL_NAMES = [s[0] for s in ROLL_SPECS]


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


def compute_roll_0p12_rank(bode_5f, ph_loss_0p12, xtalk_l, hf_l, hf_rms_l, corr_v, aborted=False):
    bode_all6 = float(bode_5f + ph_loss_0p12)
    canon_all6 = float(bode_all6 + xtalk_l + hf_l)
    if aborted:
        return canon_all6 + 200000.0
    rank = canon_all6
    # Guardrail 1: Protect the other 5 LatEC frequencies and all 6 gains (do not trade them away)
    if bode_5f > 2400.0:
        ex5 = bode_5f - 2400.0
        rank += 3.0 * ex5 + 0.002 * (ex5 ** 2)
    # Guardrail 2: Keep HF jitter strictly clean (no chaos basin)
    if hf_rms_l > 2.5:
        rank += 40.0 * ((hf_rms_l - 2.5) ** 2)
    if hf_rms_l > 3.5:
        rank += 25000.0 + 4000.0 * ((hf_rms_l - 3.5) ** 2)
    # Guardrail 3: Keep velocity waveform correlation high
    if corr_v < 0.88:
        rank += 15000.0 * ((0.88 - corr_v) ** 2)
    # Guardrail 4: Keep crosstalk bounded
    if xtalk_l > 400.0:
        rank += 2.0 * (xtalk_l - 400.0)
    return float(rank)


def load_locked_base_and_roll_seed(seed_preset="champ", custom_seed_json=None):
    champ_doc = json.load(open(CHAMP_53_JSON, "r", encoding="utf-8"))
    base53 = {k: float(v) for k, v in champ_doc["params"].items()}
    roll_set = set(ROLL_NAMES)
    locked_35 = {k: v for k, v in base53.items() if k not in roll_set}
    assert len(locked_35) == 35, f"Expected 35 locked params, got {len(locked_35)}"

    if custom_seed_json:
        cdoc = json.load(open(custom_seed_json, "r", encoding="utf-8"))
        csrc = cdoc.get("params_roll18") or cdoc.get("params") or cdoc
        seed_raw = {k: float(csrc[k]) for k in ROLL_NAMES}
    elif seed_preset == "g35clean":
        g35_doc = json.load(open(G35_CHAMP_JSON, "r", encoding="utf-8"))
        seed_raw = {k: float(g35_doc["params"][k]) for k in ROLL_NAMES}
        seed_raw.update({"p_sc_roll": 45.0, "z_sc_roll": 0.015, "K_sc_roll": 3.5})
    elif seed_preset == "voloto":
        seed_raw = {k: base53[k] for k in ROLL_NAMES}
        seed_raw.update({
            "Kp_task_roll": 2.0,
            "Kd_task_roll": 1.6,
            "Ki_task_roll": 0.05,
            "K_ton_roll": 1.5,
            "p_ton_roll": 1.5,
            "K_phas_roll": 12.0,
            "p_phas_roll": 6.5,
        })
    else:
        seed_raw = {k: base53[k] for k in ROLL_NAMES}

    specs = []
    for name, lb, ub, sd in ROLL_SPECS:
        x0 = float(np.clip(seed_raw[name], lb, ub))
        specs.append((name, x0, lb, ub, sd))
    return locked_35, specs


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


def _make_rec_roll(gen, slot, roll_id, rl, vec):
    b5f = float(rl["classic_bode_score"])
    ph_loss = float(rl["ph_loss_0p12"])
    xl = float(rl["pitch_crosstalk_penalty"])
    hl = float(rl["hf_penalty"])
    hfl = float(rl["hf_noise_rms"])
    cv = float(rl["corr_vel"])
    abl = bool(rl["aborted"])
    rk = compute_roll_0p12_rank(b5f, ph_loss, xl, hl, hfl, cv, abl)
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
        "corr_vel": round(cv, 4),
        "aborted": abl,
        "params_roll18": {k: float(v) for k, v in zip(ROLL_NAMES, vec)},
    }


def ensure_shared_best(seed_specs):
    with _SharedLock(LOCK_PATH):
        if not os.path.exists(BEST_ROLL18_JSON):
            atomic_write_json(BEST_ROLL18_JSON, {
                "roll_id": "init_seed",
                "rank_score": 1e9,
                "bode_5f": 99999.0,
                "bode_all6": 99999.0,
                "ph_0p12": -208.8,
                "ph_err_0p12": 95.6,
                "hf_rms_latec": 1.90,
                "corr_vel": 0.925,
                "aborted": False,
                "gen": 0,
                "params_roll18": {n: float(x0) for n, x0, _lb, _ub, _sd in seed_specs},
            })


def update_shared_roll(rec_roll18):
    with _SharedLock(LOCK_PATH):
        cur = safe_load_json(BEST_ROLL18_JSON, None)
        cur_clean = bool(
            cur and (not cur.get("aborted", False))
            and float(cur.get("hf_rms_latec", 99.0)) <= 3.5
            and float(cur.get("corr_vel", 0.0)) >= 0.88
        )
        new_clean = bool(
            (not rec_roll18["aborted"])
            and float(rec_roll18["hf_rms_latec"]) <= 3.5
            and float(rec_roll18["corr_vel"]) >= 0.88
        )
        better = False
        if cur is None:
            better = True
        elif new_clean and not cur_clean:
            better = True
        elif new_clean == cur_clean and float(rec_roll18["rank_score"]) < float(cur.get("rank_score", 1e12)):
            better = True
        if better:
            atomic_write_json(BEST_ROLL18_JSON, rec_roll18)
        return better


def update_shared_3axis_champion(rec_3ax):
    with _SharedLock(LOCK_PATH):
        cur = safe_load_json(CHAMPION_3AXIS_JSON, None)
        db5 = rec_3ax.get("db_5p583_apeo")
        eligible = bool(
            (not rec_3ax["aborted"])
            and float(rec_3ax["hf_rms_latec"]) <= 3.5
            and float(rec_3ax["corr_vel_latec"]) >= 0.88
            and float(rec_3ax["bode_apeo"]) <= 2250.0
            and (db5 is None or abs(float(db5)) <= 1.40)
            and float(rec_3ax["bode_yawma"]) <= 780.0
        )
        if not eligible:
            return False
        better = (cur is None) or (float(rec_3ax["rank_roll_0p12"]) < float(cur.get("rank_roll_0p12", 1e12)))
        if better:
            atomic_write_json(CHAMPION_3AXIS_JSON, rec_3ax)
        return better


def main():
    parser = argparse.ArgumentParser(description="Cooperative 18D Roll CMA-ES for LatEC 0.1209 Hz phase")
    parser.add_argument("--run-tag", type=str, required=True)
    parser.add_argument("--seed-preset", choices=("champ", "g35clean", "voloto"), default="champ")
    parser.add_argument("--custom-seed-json", type=str, default=None)
    parser.add_argument("--pop-roll", type=int, default=36)
    parser.add_argument("--gens", type=int, default=150)
    parser.add_argument("--workers", type=int, default=40)
    parser.add_argument("--seed", type=int, default=6101)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--dry-check", action="store_true")
    args = parser.parse_args()

    os.makedirs(LOG_DIR, exist_ok=True)
    np.random.seed(args.seed)
    tag = args.run_tag.strip()

    locked_35, specs = load_locked_base_and_roll_seed(args.seed_preset, args.custom_seed_json)
    x0 = [float(s[1]) for s in specs]
    lb = [float(s[2]) for s in specs]
    ub = [float(s[3]) for s in specs]
    stds = [float(s[4]) for s in specs]

    if args.dry_check:
        assert len(locked_35) + len(specs) == 53
        print(f"[DRY-CHECK] tag={tag} preset={args.seed_preset} pop={args.pop_roll} workers={args.workers} 18+35=53 OK")
        for n, v, l, u, s in specs[:6]:
            print(f"  {n:24s}: x0={v:8.4f} bounds=[{l}, {u}] sd={s}")
        return

    ensure_shared_best(specs)
    locked_items = [(k, f"{float(v):.15g}") for k, v in locked_35.items()]

    pkl_path = os.path.join(LOG_DIR, f"roll_0p12_cma_state_{tag}.pkl")
    hist_path = os.path.join(LOG_DIR, f"roll_0p12_history_{tag}.json")
    ledger_path = os.path.join(LOG_DIR, f"roll_0p12_ledger_{tag}.md")

    es_roll = cma.CMAEvolutionStrategy(x0, 0.50, {
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
            f.write(f"# Roll 0.1209 Hz Phase Optimization Ledger (tag={tag}, preset={args.seed_preset})\n\n")
            f.write(f"Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | pop={args.pop_roll} workers={args.workers}\n\n")
            f.write("| Gen | Clean | BestRank | Best5f | BestPh0.12 (Err) | BestAll6 | BestHF | Corr | Mean5f/Ph0.12 | V3_APEO | V3_LatEC (5f/Ph0.12/HF) | V3_YawMA | Time | Sigma |\n")
            f.write("|---:|---:|---:|---:|---:|---:|---:|---:|---|---:|---|---:|---:|---:|\n")

    for gen in range(start_gen, args.gens + 1):
        t0 = time.time()
        shared_doc = safe_load_json(BEST_ROLL18_JSON, None) or {}
        shared_roll = shared_doc.get("params_roll18", {n: float(v) for n, v in zip(ROLL_NAMES, x0)})
        shared_roll_id = shared_doc.get("roll_id", "init")

        cand_r = ask_orthogonal_mirrored_nd(es_roll, args.pop_roll, lb, ub, use_tpa=False)
        if gen == 1 and start_gen == 1:
            cand_r[0] = np.asarray(x0, dtype=float)
        mean_r_vec = np.clip(np.asarray(es_roll.mean, dtype=float), lb, ub)

        jobs = []
        for idx, vec in enumerate(cand_r):
            ovr = locked_items + [(k, f"{float(v):.15g}") for k, v in zip(ROLL_NAMES, vec)]
            jobs.append(("R", idx, "LatEC", ovr))
        jobs.append(("RM", 0, "LatEC", locked_items + [(k, f"{float(v):.15g}") for k, v in zip(ROLL_NAMES, mean_r_vec)]))

        verify_params = dict(locked_35)
        verify_params.update({k: float(shared_roll[k]) for k in ROLL_NAMES})
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
                f"A={rec_3ax['bode_apeo']:,.1f}(db5p5={rec_3ax['db_5p583_apeo']}) Y={rec_3ax['bode_yawma']:,.1f}",
                flush=True,
            )

        f_roll, r_records = [], []
        r_clean_count = 0
        for idx, vec in enumerate(cand_r):
            rk, rec = _make_rec_roll(gen, idx, f"{tag}_g{gen}_s{idx}", results[("R", idx, "LatEC")], vec)
            if (not rec["aborted"]) and rec["hf_rms_latec"] <= 3.5 and rec["corr_vel"] >= 0.88:
                r_clean_count += 1
            f_roll.append(rk)
            r_records.append(rec)
        es_roll.tell(cand_r, f_roll)

        _, r_mean_rec = _make_rec_roll(gen, -1, f"{tag}_g{gen}_mean", results[("RM", 0, "LatEC")], mean_r_vec)
        best_r_gen = min(
            r_records + [r_mean_rec],
            key=lambda c: (0 if (not c["aborted"] and c["hf_rms_latec"] <= 3.5 and c["corr_vel"] >= 0.88) else 1, c["rank_score"]),
        )
        if update_shared_roll(best_r_gen):
            print(
                f"  [SHARED ROLL UPDATE] Gen {gen} ({best_r_gen['roll_id']}) "
                f"Ph0.12={best_r_gen['ph_0p12']:.1f}({best_r_gen['ph_err_0p12']:+.1f}°) "
                f"5f={best_r_gen['bode_5f']:,.1f} All6={best_r_gen['bode_all6']:,.1f} HF={best_r_gen['hf_rms_latec']:.2f}",
                flush=True,
            )

        dt = time.time() - t0
        cur_best_r = safe_load_json(BEST_ROLL18_JSON) or best_r_gen

        with open(pkl_path, "wb") as f:
            pickle.dump(es_roll, f)
        history.append({
            "gen": gen,
            "run_tag": tag,
            "elapsed_s": round(dt, 1),
            "sigma": float(es_roll.sigma),
            "r_clean_count": r_clean_count,
            "best_gen": best_r_gen,
            "mean": {k: r_mean_rec[k] for k in ("rank_score", "bode_5f", "bode_all6", "ph_0p12", "ph_err_0p12", "hf_rms_latec", "corr_vel", "aborted")},
            "verify_3axis": {k: rec_3ax[k] for k in ("rank_roll_0p12", "bode_apeo", "db_5p583_apeo", "bode_latec_5f", "bode_latec_all6", "ph_0p12", "ph_err_0p12", "hf_rms_latec", "corr_vel_latec", "bode_yawma", "aborted")},
        })
        atomic_write_json(hist_path, history)

        with open(ledger_path, "a", encoding="utf-8") as f:
            f.write(
                f"| {gen} | {r_clean_count}/{args.pop_roll} | {cur_best_r['rank_score']:,.1f} | "
                f"{cur_best_r['bode_5f']:,.1f} | {cur_best_r['ph_0p12']:.1f} ({cur_best_r['ph_err_0p12']:+.1f}°) | "
                f"{cur_best_r['bode_all6']:,.1f} | {cur_best_r['hf_rms_latec']:.2f} | {cur_best_r['corr_vel']:+.3f} | "
                f"{r_mean_rec['bode_5f']:,.0f}/{r_mean_rec['ph_err_0p12']:+.1f}° | {rec_3ax['bode_apeo']:,.1f} | "
                f"{rec_3ax['bode_latec_5f']:,.0f}/{rec_3ax['ph_err_0p12']:+.1f}°/{rec_3ax['hf_rms_latec']:.2f} | "
                f"{rec_3ax['bode_yawma']:,.1f} | {dt:.1f}s | {es_roll.sigma:.3f} |\n"
            )

        print(
            f"Gen {gen:03d} [{tag}] ({dt:.1f}s) | Clean={r_clean_count}/{args.pop_roll} "
            f"SharedBest: 5f={cur_best_r['bode_5f']:,.1f} Ph={cur_best_r['ph_0p12']:.1f}({cur_best_r['ph_err_0p12']:+.1f}°) "
            f"All6={cur_best_r['bode_all6']:,.1f} HF={cur_best_r['hf_rms_latec']:.2f} | "
            f"Mean={r_mean_rec['bode_5f']:,.0f}({r_mean_rec['ph_err_0p12']:+.1f}°) | "
            f"V3: A={rec_3ax['bode_apeo']:,.0f} L={rec_3ax['bode_latec_5f']:,.0f}({rec_3ax['ph_err_0p12']:+.1f}°) Y={rec_3ax['bode_yawma']:,.0f} | sig={es_roll.sigma:.3f}",
            flush=True,
        )


if __name__ == "__main__":
    main()
