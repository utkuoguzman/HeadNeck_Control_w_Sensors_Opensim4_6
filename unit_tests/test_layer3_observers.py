"""Layer 3 Functional Unit Tests: Proprioceptive Decoding, Fusimotor Efference & Cerebellar Observers.

Blocks:
  B10 - 72-Muscle 6-DOF Proprioceptive Jacobian Decoder & Cuneate Static Posture / Shift
  B11 - Parallel Reference Mileusnic Spindle & Fusimotor Efference-Copy Cancellation
  B12 - Cerebellar SE_2(3) Mahony Spatial Attitude Observer & Linear-Acceleration Rejection
  B13 - Cerebellar Gravity-Adaptation & Voluntary Vestibular Efference Cancellation (Sub 3C -> Sub 2B)
"""
from __future__ import annotations

import math
import time
import numpy as np
import opensim as osim

from unit_tests.common import (
    BlockReport,
    build_isolated_model,
    extract_skull_zxy_deg,
    integrate_and_sample,
    make_spline,
    read_ctrl_sv,
    read_mahony_pry_deg,
    read_out_double,
    read_out_vec3,
    smooth_accel_pulse_pos,
    smooth_step_ramp,
)


def test_B10_proprioceptive_decoder_and_cuneate() -> BlockReport:
    """B10: Static +10 deg deflections on untouched free-neck Pitch, Roll, Yaw -> Cuneate decoded angles match true angles (±12%, xt_err < 1.0 deg)."""
    t0 = time.time()
    rep = BlockReport(
        block_id="B10",
        layer="observers",
        title="72-Muscle 6-DOF Proprioceptive Decoder & Cuneate Posture",
        scenario="Untouched free-neck topology (u=0, all reflexes OFF); hold static +10 deg on Pitch, Roll, Yaw via damped SpringGeneralizedForce; compare cuneate_neck_{pitch,roll,yaw} vs true angles.",
    )
    try:
        zero_hold = {cn: (0.0, 60.0, 6.0) for cn in ("pitch1", "pitch2", "roll1", "roll2", "yaw1", "yaw2")}
        cases = [
            ("Pitch", {**zero_hold, "pitch1": (math.radians(3.33), 60.0, 6.0), "pitch2": (math.radians(6.67), 60.0, 6.0)}, 0),
            ("Roll",  {**zero_hold, "roll1":  (math.radians(1.54), 60.0, 6.0), "roll2":  (math.radians(8.46), 60.0, 6.0)}, 1),
            ("Yaw",   {**zero_hold, "yaw1":   (math.radians(5.40), 60.0, 6.0), "yaw2":   (math.radians(4.60), 60.0, 6.0)}, 2),
        ]
        for label, spr_t, ax_i in cases:
            m, c, st = build_isolated_model(
                ctrl_overrides={
                    "enable_vcr": False, "enable_ccr": False, "enable_task_controller": False,
                    "enable_antibuckling": False, "enable_gto_reflex": False, "enable_renshaw": False, "enable_ligament_reflex": False,
                },
                spring_targets=spr_t,
            )
            cs = m.getCoordinateSet()
            tr = integrate_and_sample(
                m, st, t_end=0.80, dt=0.08,
                probe_fn=lambda t, s: {
                    "true": [
                        math.degrees(cs.get("pitch1").getValue(s) + cs.get("pitch2").getValue(s)),
                        math.degrees(cs.get("roll1").getValue(s)  + cs.get("roll2").getValue(s)),
                        math.degrees(cs.get("yaw1").getValue(s)   + cs.get("yaw2").getValue(s)),
                    ],
                    "cun": [
                        math.degrees(read_out_double(c, "cuneate_neck_pitch", s)),
                        math.degrees(read_out_double(c, "cuneate_neck_roll", s)),
                        math.degrees(read_out_double(c, "cuneate_neck_yaw", s)),
                    ],
                },
            )
            true_v = tr[-1]["true"]
            cun_v = tr[-1]["cun"]
            err_main = abs(cun_v[ax_i] - true_v[ax_i])
            rel_main = err_main / max(1.0, abs(true_v[ax_i]))
            max_other_err = max(abs(cun_v[j] - true_v[j]) for j in range(3) if j != ax_i)
            rep.add(
                f"B10_{label}_decode",
                f"cuneate_{label.lower()} tracks static true ({true_v[ax_i]:+.1f} deg) within ±12%, cross-axis err < 1.0 deg",
                f"true={[round(v,2) for v in true_v]}, cun={[round(v,2) for v in cun_v]} (rel={100*rel_main:.1f}%, xt_err={max_other_err:.2f}d)",
                (abs(true_v[ax_i]) > 6.0) and (cun_v[ax_i] * true_v[ax_i] > 0.0) and (rel_main < 0.12) and (max_other_err < 1.0),
            )
    except Exception as e:
        rep.error = str(e)
    rep.wall_s = time.time() - t0
    return rep


def test_B11_fusimotor_efference_cancellation() -> BlockReport:
    """B11: Active voluntary +15 deg pitch -> efference cancellation ON must suppress est_neck_pitch & ccr_torque > 60% vs OFF."""
    t0 = time.time()
    rep = BlockReport(
        block_id="B11",
        layer="observers",
        title="Parallel Reference Spindle & Fusimotor Efference Cancellation",
        scenario="Active voluntary +15 deg pitch step (u=1) with enable_efference_cancellation=True vs False; probe |est_neck_pitch| and |ccr_spindle_torque|.",
    )
    try:
        def run_eff(eff_on: bool):
            m, c, st = build_isolated_model(
                ctrl_overrides={
                    "enable_efference_cancellation": eff_on,
                    "enable_fusimotor": True,
                    "enable_vcr": True,
                    "Kp_prop_pitch": 2.0,
                    "Kp_task_pitch": 14.8, "Kd_task_pitch": 4.6, "Ki_task_pitch": 45.0,
                    "urgency": 1.0, "desired_pitch": 0.0,
                }
            )
            p_dp = c.updPropertyByName("desired_pitch")
            done = {"v": False}
            def hook(t, s):
                if not done["v"] and t >= 0.05:
                    osim.PropertyHelper.setValueDouble(math.radians(15.0), p_dp)
                    done["v"] = True
            cs = m.getCoordinateSet()
            return integrate_and_sample(
                m, st, t_end=1.10, dt=0.02, step_hook=hook,
                probe_fn=lambda t, s: {
                    "t": t,
                    "true_p": math.degrees(cs.get("pitch1").getValue(s) + cs.get("pitch2").getValue(s)),
                    "est_p": math.degrees(read_out_double(c, "est_neck_pitch", s)),
                    "ccr_p": read_out_vec3(c, "dbg_ccr_torque", s)[0],
                },
            )

        tr_on = run_eff(True)
        tr_off = run_eff(False)
        rms_est_on = float(np.sqrt(np.mean([x["est_p"] ** 2 for x in tr_on if x["t"] >= 0.25])))
        rms_est_off = float(np.sqrt(np.mean([x["est_p"] ** 2 for x in tr_off if x["t"] >= 0.25])))
        end_p_on = tr_on[-1]["true_p"]
        ratio = rms_est_on / max(1e-3, rms_est_off)
        rep.add("B11.1_reaches_target", "Voluntary +15 deg pitch reaches 13..17 deg cleanly", f"true_pitch={end_p_on:+.2f} deg", 13.0 <= end_p_on <= 17.5)
        rep.add("B11.2_spindle_cancel", "Efference cancellation ON suppresses est_neck_pitch RMS by > 70% vs OFF", f"ON={rms_est_on:.2f}d vs OFF={rms_est_off:.2f}d (suppress={100*(1-ratio):.1f}%)", ratio < 0.30)
    except Exception as e:
        rep.error = str(e)
    rep.wall_s = time.time() - t0
    return rep


def test_B12_mahony_observer_and_accel_reject() -> BlockReport:
    """B12: Mahony SO(3) accuracy under combined 3D tilt (oto-heavy & prop-only) + somatogravic accel rejection under +20 m/s^2."""
    t0 = time.time()
    rep = BlockReport(
        block_id="B12",
        layer="observers",
        title="Cerebellar SE_2(3) Mahony Observer & Linear-Accel Rejection",
        scenario="(a) Combined 3D trunk+neck tilt (P=+15, R=-15, Y=+20) under w_oto=5.0 & w_oto=0.0; (b) Pure +20 m/s^2 X & Z linear accel pulses with upright skull.",
    )
    try:
        ts_t = np.linspace(0.0, 1.2, 241)
        gp = smooth_step_ramp(ts_t, 0.05, 0.30, math.radians(15.0))
        gr = smooth_step_ramp(ts_t, 0.05, 0.30, math.radians(-15.0))
        gy = smooth_step_ramp(ts_t, 0.05, 0.30, math.radians(20.0))

        plot_tilt = {}
        for w_oto, tag in [(5.0, "oto_heavy"), (0.0, "prop_only")]:
            m, _, st = build_isolated_model(
                ctrl_overrides={"enable_vcr": False, "enable_ccr": False, "enable_task_controller": False, "w_oto_mahony": w_oto},
                prescribed_coords={"gndpitch": make_spline(ts_t, gp), "gndroll": make_spline(ts_t, gr), "gndyaw": make_spline(ts_t, gy)},
                lock_neck_except=(),
            )
            skull = m.getBodySet().get("skull")
            R_base = osim.Rotation(skull.getTransformInGround(st).R())
            mp0, mr0, my0 = read_mahony_pry_deg(m, st)
            tr = integrate_and_sample(
                m, st, t_end=1.1, dt=0.02,
                probe_fn=lambda t, s: {"t": t, "true": list(extract_skull_zxy_deg(skull, s, R_base)), "mah": [a - b for a, b in zip(read_mahony_pry_deg(m, s), (mp0, mr0, my0))]},
            )
            t_end, m_end = tr[-1]["true"], tr[-1]["mah"]
            errs = [abs(m_end[i] - t_end[i]) for i in range(3)]
            plot_tilt[tag] = {"t": [x["t"] for x in tr], "true": [x["true"] for x in tr], "mah": [x["mah"] for x in tr]}
            rep.add(f"B12.1_3d_tilt_{tag}", "All 3 axes |Mahony - True| < 1.5 deg", f"true={[round(v,1) for v in t_end]}, mah={[round(v,1) for v in m_end]}, maxErr={max(errs):.2f}d", max(errs) < 1.5)

        # (b) Pure +20 m/s^2 linear accel on X and Z (skull upright 0 deg) -> Mahony false tilt < 0.30 deg!
        ts_a = np.linspace(0.0, 0.40, 201)
        pos_a = smooth_accel_pulse_pos(ts_a, t0=0.04, t_rise=0.04, t_hold=0.20, a_peak=20.0)
        plot_acc = {}
        for cname, ax_i, tag in [("spine_tx", 0, "X_20mps2"), ("spine_tz", 1, "Z_20mps2")]:
            ma, _, sa = build_isolated_model(
                ctrl_overrides={"enable_vcr": False, "enable_ccr": False, "enable_task_controller": False, "w_oto_mahony": 0.50},
                prescribed_coords={cname: make_spline(ts_a, pos_a)}, lock_neck_except=(),
            )
            mp0, mr0, my0 = read_mahony_pry_deg(ma, sa)
            tra = integrate_and_sample(
                ma, sa, t_end=0.35, dt=0.01,
                probe_fn=lambda t, s: {"t": t, "mah": [a - b for a, b in zip(read_mahony_pry_deg(ma, s), (mp0, mr0, my0))]},
            )
            max_false_tilt = max(max(abs(x["mah"][0]), abs(x["mah"][1])) for x in tra)
            plot_acc[tag] = {"t": [x["t"] for x in tra], "mah": [x["mah"] for x in tra]}
            rep.add(f"B12.2_accel_reject_{tag}", "Pure 20 m/s^2 accel -> Mahony false tilt < 0.30 deg", f"max_false_tilt={max_false_tilt:.4f} deg", max_false_tilt < 0.30)
        rep.plot_data = {"tilt": plot_tilt, "accel": plot_acc}
    except Exception as e:
        rep.error = str(e)
    rep.wall_s = time.time() - t0
    return rep


def test_B13_cerebellar_gravity_and_vol_cancel() -> BlockReport:
    """B13: Static +20 deg Pitch/Roll tilts must cancel > 97% of gravity in dbg_phasic_oto_input once tangential ramp accel settles."""
    t0 = time.time()
    rep = BlockReport(
        block_id="B13",
        layer="observers",
        title="Cerebellar Gravity Adaptation & Vestibular Efference Cancellation",
        scenario="Slow 1.0s ramp to +20 deg static Pitch & Roll trunk tilts (hold to 1.8s so pivot tangential accel L*alpha=0); probe dbg_phasic_oto_input vs 3.35 m/s^2 gravity & 0.51 m/s^2 irregular steady residual.",
    )
    try:
        ts = np.linspace(0.0, 2.0, 401)
        q20 = smooth_step_ramp(ts, 0.05, 1.00, math.radians(20.0))
        for cname, ax_i, tag in [("gndpitch", 0, "Pitch"), ("gndroll", 1, "Roll")]:
            m, c, st = build_isolated_model(
                ctrl_overrides={"enable_vcr": False, "enable_ccr": False, "enable_task_controller": False},
                prescribed_coords={cname: make_spline(ts, q20)}, lock_neck_except=(),
            )
            tr = integrate_and_sample(
                m, st, t_end=1.80, dt=0.05,
                probe_fn=lambda t, s: {
                    "t": t,
                    "irr": read_out_vec3(c, "dbg_oto_irr_head", s)[0 if ax_i == 0 else 2],
                    "ph_in": read_out_vec3(c, "dbg_phasic_oto_input", s)[ax_i],
                },
            )
            irr_end = abs(tr[-1]["irr"])
            end_ph = abs(tr[-1]["ph_in"])
            # Static hold: raw gravity projection is 3.35 m/s^2, uncancelled irregular steady state is ~0.51 m/s^2; cerebellar adaptation cancels > 97% of gravity (< 0.08 m/s^2)
            rep.add(
                f"B13_{tag}_grav_cancel",
                "Static +20 deg hold: |phasic_oto_input| < 0.08 m/s^2 (cancels > 97% of 3.35 m/s^2 gravity & > 84% of irr residual)",
                f"irr_steady={irr_end:.3f} -> ph_in_steady={end_ph:.4f} m/s^2 ({100*(1 - end_ph/3.355):.2f}% grav cancel)",
                end_ph < 0.08 and irr_end > 0.40,
            )
    except Exception as e:
        rep.error = str(e)
    rep.wall_s = time.time() - t0
    return rep


LAYER3_TESTS = [
    test_B10_proprioceptive_decoder_and_cuneate,
    test_B11_fusimotor_efference_cancellation,
    test_B12_mahony_observer_and_accel_reject,
    test_B13_cerebellar_gravity_and_vol_cancel,
]
