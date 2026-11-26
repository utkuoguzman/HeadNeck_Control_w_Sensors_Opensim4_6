"""Layer 6 Functional Unit Tests: Spinal CCR, Anti-Buckling, 6-DOF QP & Tier B Spinal Reflexes.

Blocks:
  B19 - Spinal 72-Muscle Ia CCR, Task-Decoded CCR & Voluntary Sherrington Reciprocal Inhibition (incl. Static Holds!)
  B20 - Direct Passive 6-DOF Intersegmental Anti-Buckling (tau_buckle) & Semismooth-Newton Bounded QP Allocation
  B21 - Tier B Spinal Loops: Group Ib GTO Inhibition, Renshaw Recurrent Inhibition & Articular Ligament Reflex
"""
from __future__ import annotations

import math
import time
import numpy as np
import opensim as osim

from unit_tests.common import (
    BlockReport,
    build_isolated_model,
    integrate_and_sample,
    make_spline,
    read_ctrl_sv,
    read_out_double,
    read_out_vec3,
    smooth_step_ramp,
)


def test_B19_ccr_and_reciprocal_inhibition() -> BlockReport:
    """B19: Passive +10 deg deflection -> task CCR opposes on all 3 axes & spinal CCR excites stretched antagonists; voluntary hold sustains vol_gate -> 1."""
    t0 = time.time()
    rep = BlockReport(
        block_id="B19",
        layer="spinal",
        title="Spinal Ia CCR, Task-Decoded CCR & Postural Reciprocal Inhibition",
        scenario="(a) Passive u=0 +10 deg Pitch/Roll/Yaw -> dbg_ccr_torque < 0; (b) Voluntary +30 deg Yaw hold at t=1.6s -> dbg_vol_gate > 0.95 (holds reciprocal inhibition ON!).",
    )
    try:
        # (a) Passive u=0 external loads on untouched free-neck topology -> covariant CCR opposes on all 3 axes!
        for label, ext_t, ax_i in [
            ("Pitch", {"pitch1": (0.02, 0.10, 1.5), "pitch2": (0.02, 0.10, 1.5)}, 2),
            ("Roll",  {"roll1":  (0.02, 0.10, 2.0), "roll2":  (0.02, 0.10, 2.0)}, 0),
            ("Yaw",   {"yaw1":   (0.02, 0.10, 0.8), "yaw2":   (0.02, 0.10, 0.8)}, 1),
        ]:
            m, c, st = build_isolated_model(
                ctrl_overrides={
                    "enable_vcr": False, "enable_task_controller": False, "enable_antibuckling": False, "enable_ccr": True,
                    "c_disc_ccr": 5.0, "gamma_musc_ccr": 0.0,
                },
                external_torques=ext_t,
            )
            tr = integrate_and_sample(m, st, t_end=0.50, dt=0.05, probe_fn=lambda t, s: {"ccr": read_out_vec3(c, "dbg_ccr_torque", s)[ax_i]})
            tau_end = tr[-1]["ccr"]
            rep.add(f"B19.1_passive_ccr_{label}", f"Passive external {label} load -> restoring ccr_torque[{ax_i}] < -0.25 Nm", f"{tau_end:+.3f} Nm", tau_end < -0.25)

        # (b) Static voluntary hold at +30 deg Yaw: check vol_gate stays > 0.95 after velocity decays to 0!
        mv, cv, sv = build_isolated_model(
            ctrl_overrides={"urgency": 1.0, "desired_rotation_vec": (0.0, math.radians(30.0), 0.0), "omega_task": 9.0, "zeta_task": 1.1, "ki_task": 2.5}
        )
        trv = integrate_and_sample(
            mv, sv, t_end=1.40, dt=0.05,
            probe_fn=lambda t, s: {
                "vg": read_out_double(cv, "dbg_vol_gate", s),
                "tvel": abs(math.degrees(read_ctrl_sv(mv, s, "traj_vel_y"))),
            },
        )
        vg_end, tvel_end = trv[-1]["vg"], trv[-1]["tvel"]
        rep.add("B19.2_postural_recip_gate", "During static +30 deg hold (traj_vel < 0.5 d/s), vol_gate > 0.95", f"vol_gate={vg_end:.4f} (traj_vel={tvel_end:.3f} d/s)", vg_end > 0.95 and tvel_end < 0.5)
    except Exception as e:
        rep.error = str(e)
    rep.wall_s = time.time() - t0
    return rep


def test_B20_antibuckling_and_qp() -> BlockReport:
    """B20: S-buckling differential deflection (p1>0, p2<0) -> tau_buckle[2] > 0; QP tracks known voluntary torque accurately (< 5%)."""
    t0 = time.time()
    rep = BlockReport(
        block_id="B20",
        layer="spinal",
        title="Chain Nullspace Anti-Buckling & Bounded QP Synergy Allocation",
        scenario="(a) S-buckling couple (p1=+2Nm, p2=-2Nm) -> tau_buckle[2] matches nullspace K_disc law; (b) Pure voluntary pitch torque command -> QP delivered matches requested within 5%.",
    )
    try:
        # (a) S-buckling on untouched free-neck topology (external opposing couple: pitch1=+2.0 Nm, pitch2=-2.0 Nm)
        kap_b = 1.00
        mb, cb, sb = build_isolated_model(
            ctrl_overrides={"enable_vcr": False, "enable_ccr": False, "enable_task_controller": False, "enable_antibuckling": True, "kappa_buckle": kap_b, "beta_buckle": 0.08},
            external_torques={"pitch1": (0.02, 0.10, 2.0), "pitch2": (0.02, 0.10, -2.0)},
        )
        cs_b = mb.getCoordinateSet()
        trb = integrate_and_sample(
            mb, sb, t_end=0.55, dt=0.05,
            probe_fn=lambda t, s: {
                "p1": math.degrees(cs_b.get("pitch1").getValue(s)),
                "p2": math.degrees(cs_b.get("pitch2").getValue(s)),
                "bk": read_out_vec3(cb, "dbg_buckle_torque", s)[2],
                "kd_zz": read_out_vec3(cb, "dbg_tensor_kdisc_diag", s)[2],
            },
        )
        bk_end = trb[-1]["bk"]
        p1_d, p2_d = trb[-1]["p1"], trb[-1]["p2"]
        diff_deg = p1_d - p2_d
        # Nullspace restoring dipole for symmetric series discs ~ Kd_zz * kap_b * (p1 - p2)
        exp_bk = trb[-1]["kd_zz"] * kap_b * math.radians(diff_deg)
        rep.add(
            "B20.1_antibuckling_sign_mag",
            f"S-buckling couple (p1={p1_d:+.1f}, p2={p2_d:+.1f}, diff={diff_deg:+.1f}d) -> tau_buckle[2] ~ {exp_bk:+.3f} Nm (±18%)",
            f"{bk_end:+.3f} Nm (err={100*abs(bk_end-exp_bk)/max(0.05,abs(exp_bk)):.1f}%)",
            diff_deg > 5.0 and bk_end > 0.0 and abs(bk_end - exp_bk) / max(0.05, abs(exp_bk)) < 0.18,
        )

        # (b) QP torque tracking accuracy under known pure voluntary pitch torque (neck locked at 0 -> inside capacity!)
        mq, cq, sq = build_isolated_model(
            ctrl_overrides={
                "enable_vcr": False, "enable_ccr": False, "enable_antibuckling": False,
                "enable_task_controller": True, "urgency": 1.0,
                "desired_rotation_vec": (0.0, 0.0, math.radians(15.0)),
                "omega_task": 8.0, "zeta_task": 1.0, "ki_task": 0.0,
            },
            lock_neck_except=(),
        )
        trq = integrate_and_sample(
            mq, sq, t_end=0.45, dt=0.02,
            probe_fn=lambda t, s: {
                "req": read_out_vec3(cq, "dbg_vol_torque", s)[2],
                "del": read_out_vec3(cq, "dbg_qp_delivered_3dof", s)[2],
            },
        )
        req_e, del_e = trq[-1]["req"], trq[-1]["del"]
        rel_err = abs(del_e - req_e) / max(0.1, abs(req_e))
        rep.add("B20.2_qp_torque_tracking", "QP delivered sagittal torque matches requested within 5%", f"req={req_e:+.3f} Nm, del={del_e:+.3f} Nm (err={100*rel_err:.2f}%)", abs(req_e) > 3.0 and rel_err < 0.05)
    except Exception as e:
        rep.error = str(e)
    rep.wall_s = time.time() - t0
    return rep


def test_B21_tier_b_spinal_loops() -> BlockReport:
    """B21: Verify GTO Ib autogenic inhibition, Renshaw recurrent inhibition, and Articular Ligament protective reflex."""
    t0 = time.time()
    rep = BlockReport(
        block_id="B21",
        layer="spinal",
        title="Tier B Spinal Loops (GTO Ib, Renshaw Recurrent & Articular Ligament)",
        scenario="(a) GTO Ib ON vs OFF under active torque; (b) Renshaw ON vs OFF under active torque; (c) Articular lig_ref_0 silent at 0 deg and active at -45 deg extreme flexion.",
    )
    try:
        # (a) GTO Ib & (b) Renshaw under active load (isometric, VCR+CCR OFF, strong voluntary torque)
        def run_tierb(gto_on: bool, ren_on: bool):
            m, c, st = build_isolated_model(
                ctrl_overrides={
                    "enable_vcr": False, "enable_ccr": False, "enable_antibuckling": False,
                    "enable_gto_reflex": gto_on, "K_gto_ib": 0.0020, "K_gto_vol_suppress": 0.0,
                    "enable_renshaw": ren_on, "K_renshaw": 0.25, "tau_renshaw": 0.022,
                    "enable_ligament_reflex": False,
                    "urgency": 1.0, "desired_rotation_vec": (0.0, 0.0, math.radians(20.0)),
                    "omega_task": 8.0, "ki_task": 0.0,
                },
                lock_neck_except=(),
                enable_gto=True,
            )
            mus = m.getMuscles().get("splen_cap_sklc6")
            tr = integrate_and_sample(
                m, st, t_end=0.35, dt=0.02,
                probe_fn=lambda t, s: {
                    "act": float(mus.getActivation(s)),
                    "ib": read_ctrl_sv(m, s, "ib_state_splen_cap_sklc6") if gto_on else 0.0,
                    "ren": read_ctrl_sv(m, s, "ren_state_splen_cap_sklc6") if ren_on else 0.0,
                },
            )
            return tr[-1]

        base = run_tierb(False, False)
        gto = run_tierb(True, False)
        ren = run_tierb(False, True)
        rep.add("B21.1_gto_inhibition", "GTO ON builds ib_state > 5 and reduces activation vs OFF", f"act: OFF={base['act']:.4f} -> ON={gto['act']:.4f} (ib_st={gto['ib']:.1f})", gto["ib"] > 5.0 and gto["act"] < base["act"] - 0.005)
        rep.add("B21.2_renshaw_inhibition", "Renshaw ON builds ren_state > 0.05 and reduces activation vs OFF", f"act: OFF={base['act']:.4f} -> ON={ren['act']:.4f} (ren_st={ren['ren']:.3f})", ren["ren"] > 0.05 and ren["act"] < base["act"] - 0.005)

        # (c) Articular Ligament protective reflex: silent at 0 deg / 5 deg, fires strongly at -45 deg extreme flexion
        ts = np.linspace(0.0, 0.50, 101)
        p1 = smooth_step_ramp(ts, 0.05, 0.20, math.radians(-15.0))
        p2 = smooth_step_ramp(ts, 0.05, 0.20, math.radians(-30.0))
        ml, _, sl = build_isolated_model(
            ctrl_overrides={"enable_vcr": False, "enable_ccr": False, "enable_task_controller": False, "enable_ligament_reflex": True},
            prescribed_coords={"pitch1": make_spline(ts, p1), "pitch2": make_spline(ts, p2)},
            lock_neck_except=("pitch1", "pitch2"),
        )
        trl = integrate_and_sample(ml, sl, t_end=0.45, dt=0.02, probe_fn=lambda t, s: {"t": t, "lig0": read_ctrl_sv(ml, s, "lig_ref_0")})
        lig_start = trl[2]["lig0"]
        lig_end = trl[-1]["lig0"]
        rep.add("B21.3_ligament_reflex", "lig_ref_0 silent at neutral (< 1e-4) and active at -45 deg flexion (> 0.05)", f"neutral={lig_start:.4f} -> flex_-45={lig_end:.4f}", lig_start < 1e-4 and lig_end > 0.05)
    except Exception as e:
        rep.error = str(e)
    rep.wall_s = time.time() - t0
    return rep


LAYER6_TESTS = [
    test_B19_ccr_and_reciprocal_inhibition,
    test_B20_antibuckling_and_qp,
    test_B21_tier_b_spinal_loops,
]
