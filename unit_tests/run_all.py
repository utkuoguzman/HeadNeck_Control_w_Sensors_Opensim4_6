#!/usr/bin/env python3
"""Runner for the 21-Block Head-Neck Neurobiomechanical Functional Unit Test Suite.

Examples:
  python unit_tests/run_all.py                    # Run all 21 blocks in parallel + generate scorecard, Mahony & waveform plots + JSON
  python unit_tests/run_all.py --layer brainstem  # Run only Layer 4 (B14-B15)
  python unit_tests/run_all.py --block B14        # Run single block B14 (VCR all pathways + phasic sign check)
  python unit_tests/run_all.py --block B03 B04 B10 B19 B20
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from typing import Callable, Dict, List

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_THIS_DIR)
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from unit_tests.common import BlockReport
from unit_tests.test_layer1_plant import LAYER1_TESTS
from unit_tests.test_layer2_vestibular import LAYER2_TESTS
from unit_tests.test_layer3_observers import LAYER3_TESTS
from unit_tests.test_layer4_brainstem import LAYER4_TESTS
from unit_tests.test_layer5_cortex import LAYER5_TESTS
from unit_tests.test_layer6_spinal import LAYER6_TESTS

LAYER_MAP: Dict[str, List[Callable[[], BlockReport]]] = {
    "plant": LAYER1_TESTS,
    "vestibular": LAYER2_TESTS,
    "observers": LAYER3_TESTS,
    "brainstem": LAYER4_TESTS,
    "cortex": LAYER5_TESTS,
    "spinal": LAYER6_TESTS,
}

ALL_TESTS: List[Callable[[], BlockReport]] = [fn for tests in LAYER_MAP.values() for fn in tests]
BLOCK_MAP: Dict[str, Callable[[], BlockReport]] = {fn.__name__.split("_")[1]: fn for fn in ALL_TESTS}


def _run_block_by_id(block_id: str) -> BlockReport:
    return BLOCK_MAP[block_id]()


def save_summary_plot(reports: List[BlockReport], out_png: str) -> None:
    n = len(reports)
    fig, ax = plt.subplots(figsize=(14, max(5.5, 0.42 * n + 1.8)), dpi=150)
    fig.patch.set_facecolor("#ffffff")

    y_pos = list(range(n))[::-1]
    for y, r in zip(y_pos, reports):
        n_tot = len(r.checks)
        n_ok = sum(1 for c in r.checks if c.passed)
        color = "#16a34a" if r.passed else "#dc2626"
        ax.barh(y, 1.0, height=0.68, color=color, alpha=0.18, edgecolor=color, lw=1.4)
        status = "PASS" if r.passed else ("ERR" if r.error else "FAIL")
        ax.text(0.015, y, f"[{status:4s}] {r.block_id} ({r.layer:10s})  {r.title}",
                va="center", ha="left", fontsize=9.2, fontweight="bold", color="#111827")
        detail = f"{n_ok}/{n_tot} checks | {r.wall_s:.1f}s" if not r.error else f"ERROR: {r.error[:60]}"
        ax.text(0.985, y, detail, va="center", ha="right", fontsize=8.8, family="monospace",
                color="#14532d" if r.passed else "#991b1b", fontweight="bold")

    ax.set_xlim(0.0, 1.0)
    ax.set_ylim(-0.7, n - 0.2)
    ax.axis("off")
    n_pass = sum(1 for r in reports if r.passed)
    tot_checks = sum(len(r.checks) for r in reports)
    pass_checks = sum(sum(1 for c in r.checks if c.passed) for r in reports)
    ax.set_title(
        f"21-Block Neurobiomechanical Functional Unit Test Scorecard:  {n_pass}/{n} Blocks PASS  ({pass_checks}/{tot_checks} Assertions)",
        fontsize=12.5, fontweight="bold", pad=12, color="#0f172a",
    )
    plt.tight_layout()
    fig.savefig(out_png, bbox_inches="tight")
    plt.close(fig)


def save_mahony_detailed_plot(reports: List[BlockReport], out_png: str) -> bool:
    b12 = next((r for r in reports if r.block_id == "B12" and r.plot_data), None)
    if b12 is None or "tilt" not in b12.plot_data:
        return False
    tilt = b12.plot_data["tilt"]
    accel = b12.plot_data["accel"]

    fig, axes = plt.subplots(2, 3, figsize=(15.5, 8.2), dpi=150)
    fig.patch.set_facecolor("#ffffff")
    colors = ["#2563eb", "#16a34a", "#ea580c"]
    ax_names = ["Pitch (deg)", "Roll (deg)", "Yaw (deg)"]

    # Row 0: Coupled 3D Tilt (oto_heavy vs prop_only) + Attitude Error
    for col, (tag, title) in enumerate([("oto_heavy", "3D Coupled Tilt (w_oto = 5.0, Otolith-Heavy)"),
                                        ("prop_only", "3D Coupled Tilt (w_oto = 0.0, Proprioception-Only)")]):
        ax = axes[0, col]
        d = tilt[tag]
        t = np.asarray(d["t"])
        tr = np.asarray(d["true"])
        mh = np.asarray(d["mah"])
        for i, (lbl, col_c) in enumerate(zip(["Pitch", "Roll", "Yaw"], colors)):
            ax.plot(t, tr[:, i], "--", color=col_c, lw=1.8, alpha=0.75, label=f"True {lbl}")
            ax.plot(t, mh[:, i], "-", color=col_c, lw=2.0, label=f"Mahony {lbl}")
        ax.set_title(title, fontsize=10.5, fontweight="bold")
        ax.set_xlabel("Time (s)"); ax.set_ylabel("Angle (deg)")
        ax.grid(True, alpha=0.3); ax.legend(ncol=2, fontsize=7.5, loc="best")

    ax_err = axes[0, 2]
    for tag, ls in [("oto_heavy", "-"), ("prop_only", "--")]:
        d = tilt[tag]
        t = np.asarray(d["t"])
        err = np.asarray(d["mah"]) - np.asarray(d["true"])
        for i, (lbl, col_c) in enumerate(zip(["P", "R", "Y"], colors)):
            ax_err.plot(t, err[:, i], ls, color=col_c, lw=1.6, label=f"{tag[:4]} {lbl}")
    ax_err.axhline(1.5, color="#dc2626", ls=":", lw=1.0); ax_err.axhline(-1.5, color="#dc2626", ls=":", lw=1.0)
    ax_err.set_title("3D Estimation Error (Mahony - True)", fontsize=10.5, fontweight="bold")
    ax_err.set_xlabel("Time (s)"); ax_err.set_ylabel("Error (deg)")
    ax_err.grid(True, alpha=0.3); ax_err.legend(ncol=2, fontsize=7.5, loc="best")

    # Row 1: Pure +20 m/s^2 Linear Accel Rejection (X, Z) + Error Summary Bar
    for col, (tag, title) in enumerate([("X_20mps2", "Pure +20 m/s² AP (X) Pulse: False Tilt Rejection"),
                                        ("Z_20mps2", "Pure +20 m/s² Lat (Z) Pulse: False Tilt Rejection")]):
        ax = axes[1, col]
        d = accel[tag]
        t = np.asarray(d["t"])
        mh = np.asarray(d["mah"])
        for i, (lbl, col_c) in enumerate(zip(["Pitch", "Roll", "Yaw"], colors)):
            ax.plot(t, mh[:, i], "-", color=col_c, lw=1.8, label=lbl)
        ax.axvspan(0.04, 0.32, color="#fef3c7", alpha=0.45, label="20 m/s² pulse")
        ax.set_title(title, fontsize=10.5, fontweight="bold")
        ax.set_xlabel("Time (s)"); ax.set_ylabel("False Attitude (deg)")
        ax.grid(True, alpha=0.3); ax.legend(fontsize=8.0, loc="best")

    ax_sum = axes[1, 2]
    labels = ["Tilt Oto\nMaxErr", "Tilt Prop\nMaxErr", "20m/s² X\nFalseTilt", "20m/s² Z\nFalseTilt"]
    vals = [
        float(np.max(np.abs(np.asarray(tilt["oto_heavy"]["mah"])[-1] - np.asarray(tilt["oto_heavy"]["true"])[-1]))),
        float(np.max(np.abs(np.asarray(tilt["prop_only"]["mah"])[-1] - np.asarray(tilt["prop_only"]["true"])[-1]))),
        float(np.max(np.abs(np.asarray(accel["X_20mps2"]["mah"])[:, :2]))),
        float(np.max(np.abs(np.asarray(accel["Z_20mps2"]["mah"])[:, :2]))),
    ]
    bars = ax_sum.bar(labels, vals, color=["#2563eb", "#0891b2", "#16a34a", "#16a34a"], alpha=0.85)
    for b, v in zip(bars, vals):
        ax_sum.text(b.get_x() + b.get_width() / 2.0, v + 0.015, f"{v:.4f}°", ha="center", va="bottom", fontsize=8.5, fontweight="bold")
    ax_sum.set_title("B12 Mahony Verification Summary", fontsize=10.5, fontweight="bold")
    ax_sum.set_ylabel("Error / False Tilt (deg)")
    ax_sum.set_ylim(0.0, max(0.35, max(vals) * 1.35))
    ax_sum.grid(True, axis="y", alpha=0.3)

    fig.suptitle("Block B12 — Cerebellar SE₂(3) Mahony Spatial Attitude Observer & Somatogravic Rejection", fontsize=13.0, fontweight="bold")
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(out_png, bbox_inches="tight")
    plt.close(fig)
    return True


def save_key_waveforms_plot(reports: List[BlockReport], out_png: str) -> bool:
    rmap = {r.block_id: r for r in reports if r.plot_data}
    if not any(k in rmap for k in ("B03", "B05", "B07", "B14")):
        return False
    fig, axes = plt.subplots(2, 2, figsize=(14.5, 8.6), dpi=150)
    fig.patch.set_facecolor("#ffffff")

    # (0,0) B03 SRS Thixotropy
    ax = axes[0, 0]
    if "B03" in rmap:
        d = rmap["B03"].plot_data
        ax.plot(d["t"], d["thix"], color="#7c3aed", lw=2.2, label="srs_thix state")
        ax.axvspan(0.15, 0.75, color="#fee2e2", alpha=0.45, label="±14° 3Hz oscillation")
        ax.axvspan(0.75, 1.35, color="#dcfce7", alpha=0.45, label="Quiet rest recovery")
        ax.set_title("B03 — Cross-Bridge SRS Thixotropy Breakdown & Rest Recovery", fontsize=10.5, fontweight="bold")
        ax.set_xlabel("Time (s)"); ax.set_ylabel("Thixotropy Factor s_thix"); ax.set_ylim(0.0, 1.1)
        ax.grid(True, alpha=0.3); ax.legend(fontsize=8.0)

    # (0,1) B05 Lin02 GTO exact ODE match
    ax = axes[0, 1]
    if "B05" in rmap:
        d = rmap["B05"].plot_data
        ax.plot(d["t"], d["ib_sim"], "-", color="#2563eb", lw=2.2, label="OpenSim tendon_Ib")
        ax.plot(d["t"], d["ib_ana"], "--", color="#dc2626", lw=1.8, label="Exact Lin02 2-Pole ODE")
        ax.set_title("B05 — Lin & Crago (2002) GTO Ib vs Analytical 2-Pole ODE", fontsize=10.5, fontweight="bold")
        ax.set_xlabel("Time (s)"); ax.set_ylabel("Firing Rate (imp/s)")
        ax.grid(True, alpha=0.3); ax.legend(fontsize=8.5)

    # (1,0) B07 Cupula Washout & Velocity Storage
    ax = axes[1, 0]
    if "B07" in rmap:
        d = rmap["B07"].plot_data
        ax.plot(d["t"], d["luen"], "--", color="#ea580c", lw=2.0, label="Cupula Luenberger (T_c=5.7s)")
        ax.plot(d["t"], d["tot"], "-", color="#16a34a", lw=2.2, label="Velocity-Storage Restored (luen + vs)")
        ax.axhline(30.0, color="#111827", ls=":", lw=1.2, label="True Yaw Spin (+30°/s)")
        ax.set_title("B07 — Canal Cupula Washout & Velocity-Storage Restoration", fontsize=10.5, fontweight="bold")
        ax.set_xlabel("Time (s)"); ax.set_ylabel("Angular Velocity (deg/s)")
        ax.grid(True, alpha=0.3); ax.legend(fontsize=8.0)

    # (1,1) B14 Phasic VCR Sign Audit (Pitch & Roll)
    ax = axes[1, 1]
    if "B14" in rmap and "phasic" in rmap["B14"].plot_data:
        ph = rmap["B14"].plot_data["phasic"]
        ax2 = ax.twinx()
        for lbl, col_c in [("Pitch_AP", "#2563eb"), ("Roll_Lat", "#dc2626")]:
            if lbl in ph:
                ax.plot(ph[lbl]["t"], ph[lbl]["dq_pass"], "--", color=col_c, lw=1.8, label=f"{lbl} passive dq (deg)")
                ax2.plot(ph[lbl]["t"], ph[lbl]["tau_vcr"], "-", color=col_c, lw=2.2, label=f"{lbl} phasic tau (Nm)")
        ax.axhline(0.0, color="#64748b", lw=0.8)
        ax.set_title("B14.4 — Phasic Otolith VCR Sign Audit (Opposes Passive Whiplash!)", fontsize=10.5, fontweight="bold")
        ax.set_xlabel("Time (s)"); ax.set_ylabel("Passive Deflection dq (deg)"); ax2.set_ylabel("Phasic VCR Torque (Nm)")
        ax.grid(True, alpha=0.3)
        h1, l1 = ax.get_legend_handles_labels(); h2, l2 = ax2.get_legend_handles_labels()
        ax.legend(h1 + h2, l1 + l2, fontsize=7.5, loc="best")

    fig.suptitle("Key Functional Block Internal Waveforms (Plant, Vestibular & Brainstem)", fontsize=13.0, fontweight="bold")
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(out_png, bbox_inches="tight")
    plt.close(fig)
    return True


def main() -> int:
    ap = argparse.ArgumentParser(description="Run 21-Block Head-Neck Neurobiomechanical Functional Unit Tests")
    ap.add_argument("--layer", nargs="*", choices=list(LAYER_MAP.keys()), help="Run specific layer(s)")
    ap.add_argument("--block", nargs="*", help="Run specific block ID(s), e.g. B08 B14")
    ap.add_argument("--workers", type=int, default=6, help="Parallel worker processes (default: 6)")
    ap.add_argument("--out-dir", default=os.path.join(_THIS_DIR, "results"), help="Directory for JSON report and PNG plots")
    args = ap.parse_args()

    selected_ids: List[str] = []
    if args.block:
        for b in args.block:
            key = b.upper()
            if key not in BLOCK_MAP:
                print(f"Unknown block '{b}'. Available: {sorted(BLOCK_MAP.keys())}", file=sys.stderr)
                return 2
            selected_ids.append(key)
    elif args.layer:
        for lay in args.layer:
            for fn in LAYER_MAP[lay]:
                selected_ids.append(fn.__name__.split("_")[1])
    else:
        selected_ids = sorted(BLOCK_MAP.keys())

    t0 = time.time()
    n_workers = min(max(1, args.workers), len(selected_ids))
    print(f"=== Running {len(selected_ids)} functional block unit tests on {n_workers} worker(s) ===", flush=True)
    reports: List[BlockReport] = []
    if n_workers == 1:
        for bid in selected_ids:
            r = _run_block_by_id(bid)
            reports.append(r)
            st = "PASS" if r.passed else ("ERR " if r.error else "FAIL")
            print(f"  done [{st}] {r.block_id} ({r.wall_s:.1f}s)", flush=True)
    else:
        with ProcessPoolExecutor(max_workers=n_workers) as pool:
            futs = {pool.submit(_run_block_by_id, bid): bid for bid in selected_ids}
            for fut in as_completed(futs):
                r = fut.result()
                reports.append(r)
                st = "PASS" if r.passed else ("ERR " if r.error else "FAIL")
                print(f"  done [{st}] {r.block_id} ({r.wall_s:.1f}s) [{len(reports)}/{len(selected_ids)}]", flush=True)

    reports.sort(key=lambda r: r.block_id)
    os.makedirs(args.out_dir, exist_ok=True)
    out_json = os.path.join(args.out_dir, "unit_test_report.json")
    out_png = os.path.join(args.out_dir, "unit_test_scorecard.png")
    out_mah = os.path.join(args.out_dir, "B12_mahony_detailed.png")
    out_wf = os.path.join(args.out_dir, "key_blocks_waveforms.png")

    payload = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "wall_s": round(time.time() - t0, 2),
        "blocks_passed": sum(1 for r in reports if r.passed),
        "blocks_total": len(reports),
        "assertions_passed": sum(sum(1 for c in r.checks if c.passed) for r in reports),
        "assertions_total": sum(len(r.checks) for r in reports),
        "reports": [r.to_dict() for r in reports],
    }
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)
    save_summary_plot(reports, out_png)
    has_mah = save_mahony_detailed_plot(reports, out_mah)
    has_wf = save_key_waveforms_plot(reports, out_wf)

    print("\n" + "=" * 92)
    for r in reports:
        status = "PASS" if r.passed else ("ERR " if r.error else "FAIL")
        print(f"[{status}] {r.block_id} ({r.layer:10s}) | {r.title} ({r.wall_s:.1f}s)")
        if r.error:
            print(f"       ERROR: {r.error}")
        for c in r.checks:
            cs = "OK  " if c.passed else "FAIL"
            print(f"       [{cs}] {c.check_id:26s} | exp: {c.expected}")
            print(f"       {'':33s}| act: {c.actual}")
    print("=" * 92)
    print(f"SUMMARY: {payload['blocks_passed']}/{payload['blocks_total']} blocks PASS | "
          f"{payload['assertions_passed']}/{payload['assertions_total']} assertions PASS | "
          f"wall={payload['wall_s']:.1f}s")
    print(f"JSON:      {out_json}")
    print(f"SCORECARD: {out_png}")
    if has_mah:
        print(f"MAHONY:    {out_mah}")
    if has_wf:
        print(f"WAVEFORMS: {out_wf}")
    return 0 if payload["blocks_passed"] == payload["blocks_total"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
