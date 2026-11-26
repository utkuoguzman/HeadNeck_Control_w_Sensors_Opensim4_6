"""Layer 5 Functional Unit Tests: Cortical Trajectory Planner, Continuous Urgency Gate & 2x2 Shift IK.

Blocks:
  B16 - 3rd-Order Jerk-Limited Trajectory Planner & Anisotropic Urgency Compression
  B17 - Continuous Urgency Voluntary Engagement e(u) & Strict Zero Cortical Leak at u=0
  B18 - 2x2 Weighted Cervical Shift IK & Horizontal Translation Controller (AP Protraction & Lat Glide)
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
    set_ctrl_vec3,
    smooth_accel_pulse_pos,
)


def test_B16_trajectory_planner() -> BlockReport:
    """B16: 3rd-order critically damped SE(3) planner has 0.00% overshoot, exact steady state, and analytical T95(u)."""
    t0 = time.time()
    rep = BlockReport(
        block_id="B16",
        layer="cortex",
        title="3rd-Order Jerk-Limited SE(3) Planner & Urgency Compression",
        scenario="Neck locked at 0; step desired_rotation_vec=(0, 0, +30 deg) at urgency=0.0, 0.5, 1.0 (tau0=0.28s); probe traj_rot_z & traj_vel_z.",
    )
    try:
        t95_map = {}
        max_ovs = 0.0
        max_ss_err = 0.0
        for u in (0.0, 0.5, 1.0):
            m, c, st = build_isolated_model(
                ctrl_overrides={"enable_vcr": False, "enable_ccr": False, "enable_task_controller": False, "trajectory_tau": 0.28, "urgency": u},
                lock_neck_except=(),
            )
            set_ctrl_vec3(c, "desired_rotation_vec", (0.0, 0.0, math.radians(30.0)))
            tr = integrate_and_sample(
                m, st, t_end=2.2, dt=0.005,
                probe_fn=lambda t, s: {"t": t, "pos": math.degrees(read_ctrl_sv(m, s, "traj_rot_z")), "vel": math.degrees(read_ctrl_sv(m, s, "traj_vel_z"))},
            )
            ps = np.array([x["pos"] for x in tr])
            ts = np.array([x["t"] for x in tr])
            max_ovs = max(max_ovs, float(np.max(ps) - 30.0))
            max_ss_err = max(max_ss_err, abs(float(ps[-1]) - 30.0))
            idx95 = next((i for i, v in enumerate(ps) if v >= 0.95 * 30.0), len(ps) - 1)
            t95_map[u] = float(ts[idx95])

        # Analytical T95(u) = 6.296 * 0.7535 * 0.28 / (1 + 1.8*u)
        exp_t95 = {u: 6.296 * 0.7535 * 0.28 / (1.0 + 1.8 * u) for u in (0.0, 0.5, 1.0)}
        rep.add("B16.1_zero_overshoot", "Planner overshoot <= 0.01 deg across all urgencies", f"max_ovs={max_ovs:+.4f} deg", max_ovs <= 0.01)
        rep.add("B16.2_exact_ss", "Steady-state error < 0.15 deg at t=2.2s", f"max_ss_err={max_ss_err:.4f} deg", max_ss_err < 0.15)
        ok_t95 = all(abs(t95_map[u] - exp_t95[u]) < 0.025 for u in (0.0, 0.5, 1.0))
        rep.add(
            "B16.3_urgency_T95",
            f"T95 matches analytical {[round(exp_t95[u], 3) for u in (0.0, 0.5, 1.0)]}s (±0.025s)",
            f"actual={[round(t95_map[u], 3) for u in (0.0, 0.5, 1.0)]}s",
            ok_t95 and t95_map[1.0] < t95_map[0.5] < t95_map[0.0],
        )
    except Exception as e:
        rep.error = str(e)
    rep.wall_s = time.time() - t0
    return rep


def test_B17_urgency_gate_and_zero_u0_leak() -> BlockReport:
    """B17: At u=0 (even with strong perturbations & shift gains), cortical voluntary & shift wrenches MUST be identically 0.0."""
    t0 = time.time()
    rep = BlockReport(
        block_id="B17",
        layer="cortex",
        title="Continuous Urgency Engagement e(u) & Zero Cortical Leakage at u=0",
        scenario="(a) u=0 with 3 m/s^2 AP+Lat sled & nonzero k_shift_obs/k_trans_obs -> dbg_vol_torque & dbg_vol_shift_wrench == 0; (b) u sweep [0, 0.05, 0.2, 0.5, 1.0] -> smooth monotonic torque.",
    )
    try:
        # (a) Strict zero cortical leak at u=0 even if k_shift_obs_pos & w_translation are nonzero!
        ts = np.linspace(0.0, 0.35, 176)
        acc_p = smooth_accel_pulse_pos(ts, t0=0.03, t_rise=0.03, t_hold=0.20, a_peak=3.0)
        m0, c0, s0 = build_isolated_model(
            ctrl_overrides={
                "urgency": 0.0,
                "desired_rotation_vec": (0.0, 0.0, 0.0),
                "desired_translation_vec": (0.0, 0.0, 0.0),
                "w_translation": 1.0,
                "k_shift_obs_pos": -0.05,
                "k_trans_obs_pos": -0.05,
                "omega_task": 10.0,
            },
            prescribed_coords={"spine_tx": make_spline(ts, acc_p), "spine_tz": make_spline(ts, acc_p)},
        )
        tr0 = integrate_and_sample(
            m0, s0, t_end=0.30, dt=0.02,
            probe_fn=lambda t, s: {
                "vol": float(np.max(np.abs(read_out_vec3(c0, "dbg_vol_torque", s)))),
                "vsh": float(np.max(np.abs(read_out_vec3(c0, "dbg_vol_shift_wrench", s)))),
                "vgate": abs(read_out_double(c0, "dbg_vol_gate", s)),
            },
        )
        max_vol0 = max(x["vol"] for x in tr0)
        max_vsh0 = max(x["vsh"] for x in tr0)
        max_vg0 = max(x["vgate"] for x in tr0)
        rep.add("B17.1_zero_vol_at_u0", "At u=0: max |vol_torque| < 1e-9 Nm", f"{max_vol0:.3e} Nm", max_vol0 < 1e-9)
        rep.add("B17.2_zero_shift_leak_u0", "At u=0 (even with k_shift_obs=-0.05): max |vol_shift_wrench| < 1e-9", f"{max_vsh0:.3e} (vol_gate={max_vg0:.3e})", max_vsh0 < 1e-9 and max_vg0 < 1e-9)

        # (b) Smooth monotonic ramp across u in [0.0, 0.05, 0.20, 0.50, 1.0]
        taus = []
        for u in (0.0, 0.05, 0.20, 0.50, 1.0):
            mu, cu, su = build_isolated_model(
                ctrl_overrides={"enable_vcr": False, "enable_ccr": False, "urgency": u, "desired_rotation_vec": (0.0, 0.0, math.radians(20.0))},
                lock_neck_except=(),
            )
            tru = integrate_and_sample(mu, su, t_end=0.35, dt=0.05, probe_fn=lambda t, s: {"v": read_out_vec3(cu, "dbg_vol_torque", s)[2]})
            taus.append(tru[-1]["v"])
        mono = all(taus[i] < taus[i + 1] for i in range(len(taus) - 1)) and abs(taus[0]) < 1e-9
        rep.add("B17.3_smooth_monotonic_u", "vol_torque(u) strictly monotonic increasing from 0 across u=[0,0.05,0.2,0.5,1.0]", f"{[round(v, 2) for v in taus]} Nm", mono)
    except Exception as e:
        rep.error = str(e)
    rep.wall_s = time.time() - t0
    return rep


def test_B18_shift_ik_controller() -> BlockReport:
    """B18: Nullspace Cervical Shift IK achieves AP protraction (+2.0 cm) and Lateral glide (+0.5 cm) while keeping skull level (< 1.5 deg)."""
    t0 = time.time()
    rep = BlockReport(
        block_id="B18",
        layer="cortex",
        title="Nullspace Cervical Shift IK (AP Protraction & Lateral Glide)",
        scenario="Step desired_translation_vec=(+0.020, 0, 0) and (0, 0, +0.005) (w_translation=1, u=0.5) for 1.8s; measure C1 shift & attitude levelness.",
    )
    try:
        def run_shift(tgt_vec):
            m, c, st = build_isolated_model(
                ctrl_overrides={"w_translation": 1.0, "urgency": 0.0}
            )
            c1 = m.getBodySet().get("cerv1")
            sp = m.getBodySet().get("spine")
            m.realizePosition(st)
            p0 = c1.getTransformInGround(st).p()
            s0 = sp.getTransformInGround(st).p()
            dx0, dz0 = float(p0.get(0) - s0.get(0)), float(p0.get(2) - s0.get(2))
            p_u = c.updPropertyByName("urgency")
            done = {"v": False}
            def hook(t, s):
                if not done["v"] and t >= 0.15:
                    set_ctrl_vec3(c, "desired_translation_vec", tgt_vec)
                    osim.PropertyHelper.setValueDouble(0.5, p_u)
                    done["v"] = True
            cs = m.getCoordinateSet()
            tr = integrate_and_sample(
                m, st, t_end=1.80, dt=0.05, step_hook=hook,
                probe_fn=lambda t, s: {
                    "ap_cm": 100.0 * (float(c1.getTransformInGround(s).p().get(0) - sp.getTransformInGround(s).p().get(0)) - dx0),
                    "lat_cm": 100.0 * (float(c1.getTransformInGround(s).p().get(2) - sp.getTransformInGround(s).p().get(2)) - dz0),
                    "pitch": math.degrees(cs.get("pitch1").getValue(s) + cs.get("pitch2").getValue(s)),
                    "roll": math.degrees(cs.get("roll1").getValue(s) + cs.get("roll2").getValue(s)),
                    "p1": math.degrees(cs.get("pitch1").getValue(s)),
                    "p2": math.degrees(cs.get("pitch2").getValue(s)),
                },
            )
            return tr[-1]

        ap = run_shift((0.020, 0.0, 0.0))
        lat = run_shift((0.0, 0.0, 0.005))
        rep.add("B18.1_ap_protraction", "AP shift in [1.5, 2.3] cm, |pitch| < 1.5 deg, opposite p1/p2", f"ap={ap['ap_cm']:+.2f}cm, pitch={ap['pitch']:+.2f}d (p1={ap['p1']:+.1f}, p2={ap['p2']:+.1f})", 1.5 <= ap["ap_cm"] <= 2.3 and abs(ap["pitch"]) < 1.5 and (ap["p1"] * ap["p2"] < 0.0))
        rep.add("B18.2_lat_glide", "Lat shift in [0.32, 0.65] cm, |roll| < 1.5 deg", f"lat={lat['lat_cm']:+.3f}cm, roll={lat['roll']:+.2f}d, ap={lat['ap_cm']:+.2f}cm", 0.32 <= lat["lat_cm"] <= 0.65 and abs(lat["roll"]) < 1.5)
    except Exception as e:
        rep.error = str(e)
    rep.wall_s = time.time() - t0
    return rep


LAYER5_TESTS = [
    test_B16_trajectory_planner,
    test_B17_urgency_gate_and_zero_u0_leak,
    test_B18_shift_ik_controller,
]
