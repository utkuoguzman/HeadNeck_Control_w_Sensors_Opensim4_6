import json
import glob
import sys
import os

def update_phase2_4_ledger(gen_num=None):
    if gen_num is None:
        summaries = sorted(
            glob.glob("logs/phase2_4_gen_*_summary.json"),
            key=lambda x: int(x.split("_gen_")[1].split("_")[0])
        )
        if not summaries:
            print("No Phase 2.4 summaries found.")
            return
        latest_file = summaries[-1]
        gen_num = int(latest_file.split("_gen_")[1].split("_")[0])
    else:
        latest_file = f"logs/phase2_4_gen_{gen_num}_summary.json"

    with open(latest_file, "r", encoding="utf-8") as fp:
        sdata = json.load(fp)

    cand_idx = sdata["best_candidate_idx"]
    cand = f"W{cand_idx}_P24G{gen_num}"

    bode_files = sorted(glob.glob(f"logs/candidates/{cand}_*Hz.json"))
    bode = {}
    telemetry_by_freq = {}
    for bf in bode_files:
        with open(bf, "r", encoding="utf-8") as bfp:
            d = json.load(bfp)
            f_str = f"{d['freq']:.3f}"
            bode[f_str] = {
                "sim_gain": round(d.get("sim_gain", 0.0), 2),
                "target_gain": round(d.get("target_gain", 0.0), 2),
                "gain_db": round(d["gain_db_diff"], 2),
                "sim_phase": round(d.get("sim_phase", 0.0), 2),
                "target_phase": round(d.get("target_phase", 0.0), 2),
                "phase_err": round(d["phase_err_deg"], 2),
                "score": round(d["score"], 2)
            }
            if "telemetry" in d:
                telemetry_by_freq[f_str] = d["telemetry"]

    # Also compute population-wide per-frequency pass rate across all 28 workers in this generation
    pop_freq_pass = {}
    for f_target in ["2.45", "5.58", "7.93", "1.10", "0.78", "0.32"]:
        f_files = glob.glob(f"logs/candidates/W*_P24G{gen_num}_{f_target}Hz.json")
        n_eval = len(f_files)
        n_ok = 0
        for ff in f_files:
            try:
                with open(ff, "r", encoding="utf-8") as ffp:
                    fd = json.load(ffp)
                if fd.get("score", 999999.0) < 150000.0 and not fd.get("telemetry", {}).get("aborted", False):
                    n_ok += 1
            except Exception:
                pass
        pop_freq_pass[f_target] = f"{n_ok}/{n_eval}"

    ledger_path = "logs/phase2_4_master_ledger.json"
    if not os.path.exists(ledger_path):
        ledger = {
            "phase": 2.4,
            "description": "15D CMA-ES + Local GP Campaign with 72-Muscle Literature-Centered 6-DOF Equilibrium Co-Contraction, Incremental Active-Set QP, Resting Spindle Coactivation, and 0.1–1.0 ms Step Bounds",
            "benchmark_score": 2803.88,
            "integrator": {
                "method": "SemiExplicitEuler2 (5)",
                "accuracy": 1e-4,
                "min_step_s": 1e-4,
                "max_step_s": 0.001,
                "fastest_sensor_pole_s": 0.0006
            },
            "generations": []
        }
    else:
        with open(ledger_path, "r", encoding="utf-8") as fp:
            ledger = json.load(fp)

    existing = [i for i, g in enumerate(ledger["generations"]) if g["generation"] == gen_num]
    record = {
        "generation": gen_num,
        "candidate": cand,
        "score": round(sdata["best_score"], 2),
        "duration_min": round(sdata.get("duration_min", 0.0), 1),
        "params": {k: round(v, 4) for k, v in sdata.get("best_params", {}).items()},
        "cmaes_state": sdata.get("cmaes_state", {}),
        "pop_freq_pass_rates": pop_freq_pass,
        "bode": bode,
        "telemetry": telemetry_by_freq
    }
    if existing:
        ledger["generations"][existing[0]] = record
    else:
        ledger["generations"].append(record)

    with open(ledger_path, "w", encoding="utf-8") as out:
        json.dump(ledger, out, indent=2)

    export_matrix()
    export_detailed_markdown_ledger(ledger)
    print(f"Gen {gen_num} ({cand}) successfully updated in Phase 2.4 master ledger and matrices!")


def export_matrix():
    ledger_path = "logs/phase2_4_master_ledger.json"
    if not os.path.exists(ledger_path):
        return
    with open(ledger_path, "r", encoding="utf-8") as fp:
        ledger = json.load(fp)

    md = []
    md.append("| Campaign Milestone | 0.324 Hz | 0.783 Hz | 1.099 Hz | 2.446 Hz | 5.583 Hz | 7.927 Hz | Total Score |")
    md.append("| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |")
    md.append("| **Happee (2017) Benchmark** | -2.5 dB<br>/ 39.1° | +0.0 dB<br>/ 3.0° | +0.7 dB<br>/ 9.1° | -0.9 dB<br>/ 0.1° | -0.7 dB<br>/ 6.7° | +1.0 dB<br>/ 14.8° | **`2803.88`** |")

    freq_keys = ["0.324", "0.783", "1.099", "2.446", "5.583", "7.927"]
    for g in ledger.get("generations", []):
        g_num = g["generation"]
        cand = g["candidate"]
        score_str = f"**`{g['score']:.2f}`**"
        label = f"**Phase 2.4 Baseline** (`{cand}`)" if g_num == 0 else f"**Phase 2.4 Gen {g_num}** (`{cand}`)"
        row = [label]
        for f in freq_keys:
            if f in g.get("bode", {}):
                db = g["bode"][f]["gain_db"]
                db_str = f"+{db:.1f} dB" if db >= 0 else f"{db:.1f} dB"
                ph = g["bode"][f]["phase_err"]
                row.append(f"{db_str}<br>/ {ph:.1f}°")
            else:
                row.append("—")
        row.append(score_str)
        md.append("| " + " | ".join(row) + " |")

    with open("logs/phase2_4_bode_progress_matrix.md", "w", encoding="utf-8") as f:
        f.write("\n".join(md) + "\n")
    print("Exported logs/phase2_4_bode_progress_matrix.md successfully!")


def export_detailed_markdown_ledger(ledger):
    md = []
    md.append("# Phase 2.4 Master Optimization & Verification Ledger")
    md.append("")
    md.append("## 1. Architectural & Numerical Configuration Verified")
    md.append(r"- **72-Muscle Literature-Centered 6-DOF Equilibrium Co-Contraction (`u_min,i`):** Superficial Flexors `1.0–2.5% MVC`, Deep/Upper Flexors `2.0–4.0% MVC`, Superficial Extensors `2.0–5.0% MVC`, Deep Cervical Extensors `3.0–7.0% MVC` ($\ddot{\mathbf{q}}_{6\text{DOF}}(0) = \mathbf{0}$, `init_pitch_integral = 0.0`, `grav_trim_torque = 0.0`).")
    md.append(r"- **Incremental Active-Set Cholesky/LU QP Allocator:** $\Delta u_i = u_{\alpha,i} - u_{\min,i} \in [0, 1 - u_{\min,i}]$.")
    md.append(r"- **Resting Extrafusal & Intrafusal Spindle Initialization:** $a_i(0) = u_{\min,i}$ and $f_{\text{dyn},0,i}, f_{\text{stat},0,i}, f_{\text{chain},0,i}$ initialized prior to `model.equilibrateMuscles(state)` with incremental fusimotor efference subtraction $\Delta f_j = \max(0, f_j - f_{j,0})$.")
    md.append(r"- **Physiological Sensory & Synaptic Recruitment Thresholds:** Spindle $\Delta\text{Ia}_{\min} = 0.5\,\text{imp/s}$, Canal $\omega_{\min,\text{scc}} = 0.005\,\text{rad/s}$, Otolith $a_{\min,\text{oto}} = 0.020\,\text{m/s}^2$, Tilt $\theta_{\min,\text{tilt}} = 0.002\,\text{rad}$.")
    md.append(r"- **Integrator Stability Guarantee (`SemiExplicitEuler2`):** `min_step = 1e-4 s` (`0.1 ms`, $|h\lambda_{\max}| = 0.167 \ll 1.0$), `max_step = 0.001 s` (`1.0 ms` < $2\tau_{2,\text{irr}} = 1.2\,\text{ms}$ stability boundary).")
    md.append("")

    freq_keys = ["2.446", "5.583", "7.927", "1.099", "0.783", "0.324"]
    for g in ledger.get("generations", []):
        g_num = g["generation"]
        cand = g["candidate"]
        cs = g.get("cmaes_state", {})
        md.append(f"---")
        md.append(f"## Generation {g_num} — Champion `{cand}` (Total Score: `{g['score']:.2f} pts`, Duration: `{g['duration_min']:.1f} min`)")
        md.append("")
        if cs:
            md.append("### A. CMA-ES Search Distribution & Population Health")
            md.append(f"- **Global Step Size (`sigma`):** `{cs.get('sigma_global', 1.0)}` | **Covariance Condition Number (`cond(C)`):** `{cs.get('condition_number', 1.0)}`")
            md.append(f"- **Candidates Passing All 6 Frequencies:** `{cs.get('n_passed_all_6', 0)} / {cs.get('popsize', 28)}` | **Median Score:** `{cs.get('pop_median_score', 0.0)}` | **Mean Pass Score:** `{cs.get('pop_mean_pass_score', 'N/A')}`")
            pfp = g.get("pop_freq_pass_rates", {})
            if pfp:
                pfp_str = " | ".join([f"`{k}Hz`: {v}" for k, v in pfp.items()])
                md.append(f"- **Per-Frequency Pass Rates:** {pfp_str}")
            md.append("")

        md.append("### B. Per-Frequency Bode, 6-DOF Kinematics (`pitch`, `pitch1`, `pitch2`, `roll`, `yaw`) & Equilibrium Drift")
        md.append("| Freq (`Hz`) | Sim / Target Gain (`dB`) | Sim / Target Phase (`err`) | Score | Pre-Drift (`0–1.5s`) | Post `pitch` (`[min, max]`) | `pitch1` (`[-12,+18°]`) | `pitch2` (`[-24.8,+36°]`) | Max `|roll|` | Max `|yaw|` |")
        md.append("| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |")
        bode = g.get("bode", {})
        telem = g.get("telemetry", {})
        for fk in freq_keys:
            if fk in bode:
                b = bode[fk]
                t = telem.get(fk, {})
                md.append(
                    f"| **`{fk}`** | `{b.get('sim_gain',0):.1f}` / `{b.get('target_gain',0):.1f}` (`{b['gain_db']:+.1f} dB`) | "
                    f"`{b.get('sim_phase',0):+.1f}°` / `{b.get('target_phase',0):+.1f}°` (`{b['phase_err']:.1f}°`) | **`{b['score']:.1f}`** | "
                    f"`{t.get('pre_pitch_drift_deg',0.0):.4f}°` | `[{t.get('post_pitch_min_deg',0):+.2f}°, {t.get('post_pitch_max_deg',0):+.2f}°]` | "
                    f"`[{t.get('p1_min_deg',0):+.2f}°, {t.get('p1_max_deg',0):+.2f}°]` | `[{t.get('p2_min_deg',0):+.2f}°, {t.get('p2_max_deg',0):+.2f}°]` | "
                    f"`{t.get('max_abs_roll_deg',0):.3f}°` | `{t.get('max_abs_yaw_deg',0):.3f}°` |"
                )
        md.append("")

        md.append("### C. Proprioception, Vestibular Luenberger & Mahony `SE_2(3)` Observer Tracking")
        md.append("| Freq (`Hz`) | Mahony Pitch RMSE (`deg`) | Mahony Pitch Corr (`r`) | Canal Luenberger RMSE (`deg/s`) | Canal Luenberger Corr (`r`) | Max Spindle `dL` (`mm`) | Max `|I_pitch|` (`rad·s`) | Max `|τ_vol,pitch|` (`N·m`) |")
        md.append("| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |")
        for fk in freq_keys:
            if fk in telem:
                t = telem[fk]
                md.append(
                    f"| **`{fk}`** | `{t.get('mahony_pitch_rmse_deg',0):.3f}°` | `{t.get('mahony_pitch_corr',0):+.4f}` | "
                    f"`{t.get('canal_luen_rmse_deg_s',0):.2f}` | `{t.get('canal_luen_corr',0):+.4f}` | "
                    f"`{t.get('spindle_dL_max_mm',0):.3f}` | `{t.get('pitch_integral_max',0):.4f}` | `{t.get('vol_pitch_torque_max_nm',0):.3f}` |"
                )
        md.append("")

        md.append("### D. Champion 15D Parameters vs. CMA-ES Mean & Effective Sigma")
        md.append("| Parameter | Champion Value | CMA-ES Mean (`μ`) | Effective Step (`σ · std`) |")
        md.append("| :--- | :---: | :---: | :---: |")
        p_dict = g.get("params", {})
        m_dict = cs.get("mean_vector", {})
        s_dict = cs.get("effective_stds", {})
        for k, v in p_dict.items():
            m_val = m_dict.get(k, "—")
            s_val = s_dict.get(k, "—")
            md.append(f"| **`{k}`** | `{v}` | `{m_val}` | `{s_val}` |")
        md.append("")

    with open("logs/phase2_4_master_ledger.md", "w", encoding="utf-8") as f:
        f.write("\n".join(md) + "\n")


if __name__ == "__main__":
    g = int(sys.argv[1]) if len(sys.argv) > 1 else None
    update_phase2_4_ledger(g)
