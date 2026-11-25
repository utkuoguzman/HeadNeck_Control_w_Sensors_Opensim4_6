"""Generate Phase A4 publication-quality verification figures from phase_a4_full_verification.json."""

import json
import os
import shutil
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ART_DIR = r"C:\Users\User\.gemini\antigravity\brain\89fb8d56-9d12-4051-a256-02497ef8d6ae"
A4_JSON = os.path.join(_ROOT, "logs", "phase_a3", "phase_a4_full_verification.json")
OUT_BODE = os.path.join(_ROOT, "results", "phase_a4_3axis_bode_verification.png")
OUT_SLED = os.path.join(_ROOT, "results", "phase_a4_sled_2g_3g_urgency.png")
OUT_VOL = os.path.join(_ROOT, "results", "phase_a4_voluntary_rpy_and_shift.png")


def plot_bode(results):
    bode_rows = [r for r in results if r.get("suite") == "bode" and not r.get("aborted")]
    fig, axes = plt.subplots(2, 3, figsize=(15.5, 7.6), sharex="col")
    modes = [("APEO", "Pitch Gain (deg/m)", "APEO Pitch (z_phas=0)"),
             ("LatEC", "Roll Gain (deg/m)", "LatEC Roll (z_phas_roll=0)"),
             ("YawMA", "Yaw Gain (deg/deg)", "YawMA Global Yaw")]
    for col, (mode, ylabel, title) in enumerate(modes):
        off = next((r for r in bode_rows if r["mode"] == mode and not r["noise"]), None)
        ons = [r for r in bode_rows if r["mode"] == mode and r["noise"]]
        if off is None:
            continue
        fk_list = sorted(off["frf"].keys(), key=lambda x: float(x))
        freqs = np.array([off["frf"][k]["f_hz"] for k in fk_list])
        tgt_g = np.array([off["frf"][k]["tgt_gain"] for k in fk_list])
        sim_g = np.array([off["frf"][k]["sim_gain"] for k in fk_list])
        tgt_p = np.array([off["frf"][k]["tgt_phase"] for k in fk_list])
        sim_p = np.array([off["frf"][k]["sim_phase"] for k in fk_list])

        ax_g, ax_p = axes[0, col], axes[1, col]
        ax_g.loglog(freqs, tgt_g, "ko--", lw=1.8, ms=5, label="Human Target")
        ax_g.loglog(freqs, sim_g, "b-o", lw=2.2, ms=5, label=f"Noise-OFF (tot={off['total_score']:.0f}, rv={off['corr_vel']:.3f})")
        ax_p.semilogx(freqs, tgt_p, "ko--", lw=1.8, ms=5, label="Human Target")
        ax_p.semilogx(freqs, sim_p, "b-o", lw=2.2, ms=5, label="Noise-OFF")
        if ons:
            g_on = np.array([[r["frf"][k]["sim_gain"] for k in fk_list] for r in ons])
            p_on = np.array([[r["frf"][k]["sim_phase"] for k in fk_list] for r in ons])
            ax_g.fill_between(freqs, np.min(g_on, axis=0), np.max(g_on, axis=0), color="tab:orange", alpha=0.25, label="3-Seed Noise-ON range")
            ax_g.loglog(freqs, np.mean(g_on, axis=0), color="tab:orange", lw=1.5, ls="-.", label=f"Noise-ON mean (tot={np.mean([r['total_score'] for r in ons]):.0f})")
            ax_p.fill_between(freqs, np.min(p_on, axis=0), np.max(p_on, axis=0), color="tab:orange", alpha=0.25)
            ax_p.semilogx(freqs, np.mean(p_on, axis=0), color="tab:orange", lw=1.5, ls="-.")
        ax_g.set_title(title, fontsize=11, fontweight="bold")
        ax_g.set_ylabel(ylabel)
        ax_p.set_ylabel("Phase (deg)")
        ax_p.set_xlabel("Frequency (Hz)")
        ax_g.grid(True, which="both", alpha=0.3)
        ax_p.grid(True, which="both", alpha=0.3)
        ax_g.legend(fontsize=8, loc="best")
    fig.suptitle("Phase A4 3-Axis Passive Bode Verification (Tier A Cleanup + Tier B SRS/GTO/Renshaw/Capsule + Kuo67 B_k50_c30)", fontsize=12, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    os.makedirs(os.path.dirname(OUT_BODE), exist_ok=True)
    fig.savefig(OUT_BODE, dpi=180)
    shutil.copy2(OUT_BODE, os.path.join(ART_DIR, os.path.basename(OUT_BODE)))
    plt.close(fig)


def plot_sled(results):
    sled_off = sorted([r for r in results if r.get("suite") == "sled" and not r.get("noise") and not r.get("aborted")], key=lambda r: (r["g"], r["u"]))
    fig, axes = plt.subplots(2, 2, figsize=(14, 8))
    cmap = plt.cm.viridis(np.linspace(0.05, 0.95, 8))
    for col, g in enumerate((2.0, 3.0)):
        rows = [r for r in sled_off if abs(r["g"] - g) < 0.1]
        ax_f, ax_e = axes[0, col], axes[1, col]
        for idx, r in enumerate(rows):
            tr = r.get("trace")
            if not tr:
                continue
            lbl = f"u={r['u']:+.2f} (pk={r['peak_flex_deg']:+.1f}°, nPk={r['n_pk']})"
            ax_f.plot(tr["t_ms"], tr["flex_deg"], color=cmap[idx], lw=1.8, label=lbl)
            ax_e.plot(tr["t_ms"], tr["splen"], color=cmap[idx], lw=1.8, label=f"u={r['u']:+.2f} (nPk={r['n_pk']})")
        ax_f.set_title(f"{g:.0f}g Frontal Sled — Head Flexion (Relative to Settle)", fontweight="bold")
        ax_f.set_ylabel("Flexion (deg)")
        ax_f.grid(True, alpha=0.3)
        ax_f.legend(fontsize=7.5, loc="upper right")
        ax_e.set_title(f"{g:.0f}g Frontal Sled — Splenius Capitis Activation", fontweight="bold")
        ax_e.set_ylabel("Activation (0-1)")
        ax_e.set_xlabel("Time post-impact (ms)")
        ax_e.grid(True, alpha=0.3)
        ax_e.legend(fontsize=7.5, loc="upper right")
    fig.suptitle("Phase A4 Zheng 2g/3g 16-Case Urgency Sled Sweep (Noise-OFF, All Tier A+B & Kuo67 Active)", fontsize=12, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(OUT_SLED, dpi=180)
    shutil.copy2(OUT_SLED, os.path.join(ART_DIR, os.path.basename(OUT_SLED)))
    plt.close(fig)


def plot_voluntary(results):
    rpy_off = next((r for r in results if r.get("suite") == "rpy_18s" and not r.get("noise") and not r.get("aborted")), None)
    shifts = [r for r in results if r.get("suite") == "shift_5dof" and not r.get("aborted")]
    fig, axes = plt.subplots(2, 1, figsize=(13.5, 7.2), gridspec_kw={"height_ratios": [2.2, 1.0]})
    if rpy_off and "trace" in rpy_off:
        tr = rpy_off["trace"]
        ax = axes[0]
        ax.plot(tr["t"], tr["pitch"], label="Pitch (+60/-50°)", lw=2.0)
        ax.plot(tr["t"], tr["roll"], label="Roll (±40°)", lw=2.0)
        ax.plot(tr["t"], tr["yaw"], label="Yaw (±50°)", lw=2.0)
        ax.set_title("Phase A4 — 18.0 s Voluntary 50° RPY 10-Phase Sweep (Noise-OFF, Equilibrated SRS + GTO Layer 2 + Capsule + Kuo67)", fontweight="bold")
        ax.set_ylabel("Head Angle (deg)")
        ax.set_xlabel("Time (s)")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right")
    ax2 = axes[1]
    if shifts:
        names = [r["name"].replace("Case", "C").replace("_", " ") for r in shifts]
        ap = [r["ap_cm"] for r in shifts]
        lat = [r["lat_cm"] for r in shifts]
        x = np.arange(len(names))
        ax2.bar(x - 0.18, ap, width=0.35, label="Steady C1 AP Shift (cm)")
        ax2.bar(x + 0.18, lat, width=0.35, label="Steady C1 Lat Shift (cm)")
        ax2.set_xticks(x)
        ax2.set_xticklabels(names, fontsize=9)
        ax2.set_ylabel("Shift (cm)")
        ax2.set_title("5-DOF Voluntary AP/Lat Counter-Translation Cases", fontweight="bold")
        ax2.axhline(0, color="k", lw=0.8)
        ax2.grid(True, axis="y", alpha=0.3)
        ax2.legend()
    fig.tight_layout()
    fig.savefig(OUT_VOL, dpi=180)
    shutil.copy2(OUT_VOL, os.path.join(ART_DIR, os.path.basename(OUT_VOL)))
    plt.close(fig)


def main():
    doc = json.load(open(A4_JSON, "r", encoding="utf-8"))
    results = doc["results"]
    plot_bode(results)
    plot_sled(results)
    plot_voluntary(results)
    print(f"Generated plots:\n  {OUT_BODE}\n  {OUT_SLED}\n  {OUT_VOL}")


if __name__ == "__main__":
    main()
