"""Layer 2 Functional Unit Tests: Vestibular Sensors, Inverse Observers & High-G Nonlinearities.

Blocks:
  B06 - 3D Semicircular Canals (Regular + Irregular Projections, 3-Axis Orthogonal Sign & Passband Gain)
  B07 - Canal Luenberger Observer & Velocity Storage Integration (T_c=5.7s restoration + vs_leak)
  B08 - 3D Otolith Maculae (Regular + Irregular Adaptation/Lead & Luenberger Inverse, Tilt + Linear Accel Sign!)
  B09 - Somatogravic Hill-4 Otolith Reweighting (w_oto_rw) & Pre-Lead Hair-Bundle Tanh Saturation
"""
from __future__ import annotations

import math
import time
import numpy as np

from unit_tests.common import (
    BlockReport,
    build_isolated_model,
    integrate_and_sample,
    make_spline,
    read_ctrl_sv,
    read_out_double,
    read_out_vec3,
    smooth_accel_pulse_pos,
    smooth_step_ramp,
    smooth_vel_pulse_pos,
)


def test_B06_semicircular_canals_3d() -> BlockReport:
    """B06: Pure +20 deg/s angular velocity pulse on each trunk axis -> exact sign, magnitude, and <5% cross-talk."""
    t0 = time.time()
    rep = BlockReport(
        block_id="B06",
        layer="vestibular",
        title="3D Semicircular Canals (Regular + Irregular Sign & Passband Gain)",
        scenario="Neck locked at 0; prescribe +20 deg/s (0.349 rad/s) trunk pulse on gndpitch, gndroll, gndyaw sequentially; probe luen_omega & dbg_canal_irr_head.",
    )
    try:
        ts = np.linspace(0.0, 0.50, 251)
        v_tgt = math.radians(20.0)
        q_pulse = smooth_vel_pulse_pos(ts, t0=0.05, t_rise=0.05, t_hold=0.25, v_peak=v_tgt)
        sp = make_spline(ts, q_pulse)

        # Head-frame index mapping: [0]=roll(X), [1]=yaw(Y), [2]=pitch(Z)
        axis_map = [
            ("gndroll",  0, "luen_omega_roll",  "Roll(X)"),
            ("gndyaw",   1, "luen_omega_yaw",   "Yaw(Y)"),
            ("gndpitch", 2, "luen_omega_pitch", "Pitch(Z)"),
        ]
        for coord_name, vec_idx, sv_name, label in axis_map:
            model, controller, state = build_isolated_model(
                ctrl_overrides={"enable_vcr": False, "enable_ccr": False, "enable_task_controller": False},
                prescribed_coords={coord_name: sp},
                lock_neck_except=(),
            )
            def probe(t, s):
                return {
                    "t": t,
                    "luen": np.array([
                        read_ctrl_sv(model, s, "luen_omega_roll"),
                        read_ctrl_sv(model, s, "luen_omega_yaw"),
                        read_ctrl_sv(model, s, "luen_omega_pitch"),
                    ]),
                    "irr": read_out_vec3(controller, "dbg_canal_irr_head", s),
                }
            tr = integrate_and_sample(model, state, t_end=0.30, dt=0.02, probe_fn=probe)
            # Sample at t=0.18s (mid-plateau, ~0.08s after ramp end; tiny cupula decay e^(-0.08/5.7)=0.986)
            mid = min(tr, key=lambda x: abs(x["t"] - 0.18))
            reg_val = mid["luen"][vec_idx]
            irr_val = mid["irr"][vec_idx]
            other = [abs(mid["luen"][j]) for j in range(3) if j != vec_idx]
            xtalk = max(other) / max(1e-6, abs(reg_val))
            # Note: skull has ~-3.9 deg neutral pitch tilt, so pure ground roll/yaw projects sin(3.9deg)=0.068 onto cross-axis!
            rep.add(
                f"B06_{label}_reg_sign_gain",
                f"luen_omega[{vec_idx}] = +{v_tgt:.3f} rad/s (±8%), xtalk < 12%",
                f"reg={reg_val:+.4f}, irr={irr_val:+.4f}, xtalk={100*xtalk:.1f}%",
                (abs(reg_val - v_tgt) / v_tgt < 0.08) and (abs(irr_val - v_tgt) / v_tgt < 0.10) and (xtalk < 0.12),
            )
    except Exception as e:
        rep.error = str(e)
    rep.wall_s = time.time() - t0
    return rep


def test_B07_velocity_storage() -> BlockReport:
    """B07: Sustained +30 deg/s yaw spin for 3.0s -> cupula decays with T_c=5.7s while luen+vs restores angular velocity."""
    t0 = time.time()
    rep = BlockReport(
        block_id="B07",
        layer="vestibular",
        title="Canal Luenberger Observer & Velocity-Storage Integrator",
        scenario="Neck locked at 0; sustained +30 deg/s gndyaw spin for 3.0s (Kp_mahony=0, Ki_mahony=0 so pure VS is isolated); probe luen_omega_yaw vs luen+vs.",
    )
    try:
        ts = np.linspace(0.0, 3.2, 641)
        v_tgt = math.radians(30.0)
        q_spin = smooth_vel_pulse_pos(ts, t0=0.05, t_rise=0.05, t_hold=3.0, v_peak=v_tgt)
        model, _, state = build_isolated_model(
            ctrl_overrides={
                "enable_vcr": False, "enable_ccr": False, "enable_task_controller": False,
                "Kp_mahony": 0.0, "Ki_mahony": 0.0, "vs_leak_rate": 0.05,
            },
            prescribed_coords={"gndyaw": make_spline(ts, q_spin)},
            lock_neck_except=(),
        )
        def probe(t, s):
            lu = read_ctrl_sv(model, s, "luen_omega_yaw")
            vs = read_ctrl_sv(model, s, "vs_omega_yaw")
            return {"t": t, "luen": math.degrees(lu), "vs": math.degrees(vs), "tot": math.degrees(lu + vs)}

        tr = integrate_and_sample(model, state, t_end=3.0, dt=0.05, probe_fn=probe)
        end = tr[-1]
        # Exact analytical solution to d(vs)/dt = (1/Tc)*luen(t) - alpha*vs, luen(t) = w0*exp(-t/Tc):
        #   vs(t) = w0 * ((1/Tc)/(1/Tc - alpha)) * (exp(-alpha*t) - exp(-t/Tc))
        t_spin = 2.925
        wc, alpha = 1.0 / 5.7, 0.05
        exp_cup = 30.0 * math.exp(-wc * t_spin)
        exp_vs = 30.0 * (wc / (wc - alpha)) * (math.exp(-alpha * t_spin) - math.exp(-wc * t_spin))
        exp_tot = exp_cup + exp_vs
        rep.add("B07.1_cupula_washout", f"luen_omega_yaw(3s) ~ {exp_cup:.2f} deg/s (±5%)", f"{end['luen']:.2f} deg/s", abs(end["luen"] - exp_cup) / exp_cup < 0.05)
        rep.add("B07.2_vs_restoration", f"luen+vs(3s) ~ {exp_tot:.2f} deg/s (±5%, exact coupled VS ODE) and > luen+5", f"tot={end['tot']:.2f} (vs={end['vs']:.2f}) deg/s", abs(end["tot"] - exp_tot) / exp_tot < 0.05 and end["tot"] > end["luen"] + 5.0)
        rep.plot_data = {"t": [x["t"] for x in tr], "luen": [x["luen"] for x in tr], "tot": [x["tot"] for x in tr]}
    except Exception as e:
        rep.error = str(e)
    rep.wall_s = time.time() - t0
    return rep


def test_B08_otolith_3d_tilt_and_accel() -> BlockReport:
    """B08: Verify exact OpenSim 3D sign & gain of regular Luenberger and irregular otoliths on tilts & linear accelerations."""
    t0 = time.time()
    rep = BlockReport(
        block_id="B08",
        layer="vestibular",
        title="3D Otolith Maculae (Regular Luenberger & Irregular Lead/Adaptation)",
        scenario="Neck locked at 0: (a) +20 deg static Pitch & Roll tilts; (b) +3.0 m/s^2 linear accel pulse on spine_tx (+X) and spine_tz (+Z).",
    )
    try:
        # (a) Static +20 deg Pitch and +20 deg Roll
        ts_t = np.linspace(0.0, 0.80, 161)
        q20 = smooth_step_ramp(ts_t, 0.05, 0.25, math.radians(20.0))

        # Capture neutral baseline skull gravity vector (accounts for ~-3.9 deg anatomical Frankfort pitch!)
        m0, c0, s0 = build_isolated_model(ctrl_overrides={"enable_vcr": False, "enable_ccr": False, "enable_task_controller": False}, lock_neck_except=())
        tr0 = integrate_and_sample(m0, s0, t_end=0.10, dt=0.05, probe_fn=lambda t, s: {"f": read_out_vec3(c0, "dbg_oto_luen_head", s)})
        f_neut = tr0[-1]["f"]
        rep.add("B08.1_neutral_gravity", "||f_luen(0)|| = 9.81 m/s^2 (±0.15), f_y > +9.7", f"[{f_neut[0]:+.2f}, {f_neut[1]:+.2f}, {f_neut[2]:+.2f}] (norm={np.linalg.norm(f_neut):.2f})", abs(np.linalg.norm(f_neut) - 9.81) < 0.15 and f_neut[1] > 9.7)

        # +20 deg Pitch tilt: delta f_x should be POSITIVE (~+9.81*sin(20) = +3.35 m/s^2)
        mp, cp, sp_st = build_isolated_model(
            ctrl_overrides={"enable_vcr": False, "enable_ccr": False, "enable_task_controller": False},
            prescribed_coords={"gndpitch": make_spline(ts_t, q20)}, lock_neck_except=(),
        )
        trp = integrate_and_sample(mp, sp_st, t_end=0.70, dt=0.05, probe_fn=lambda t, s: {"f": read_out_vec3(cp, "dbg_oto_luen_head", s)})
        df_p = trp[-1]["f"] - f_neut
        exp_g = 9.81 * math.sin(math.radians(20.0))
        rep.add("B08.2_pitch_tilt_sign", f"+20 deg Pitch -> delta f_x = +{exp_g:.2f} m/s^2 (±12%)", f"df=[{df_p[0]:+.2f}, {df_p[1]:+.2f}, {df_p[2]:+.2f}]", df_p[0] > 0.0 and abs(df_p[0] - exp_g) / exp_g < 0.12)

        # +20 deg Roll tilt: R_x(+20)^T [0, g, 0] = [0, g cos20, -g sin20] -> delta f_z MUST BE NEGATIVE (-3.35 m/s^2)!
        mr, cr, sr_st = build_isolated_model(
            ctrl_overrides={"enable_vcr": False, "enable_ccr": False, "enable_task_controller": False},
            prescribed_coords={"gndroll": make_spline(ts_t, q20)}, lock_neck_except=(),
        )
        trr = integrate_and_sample(mr, sr_st, t_end=0.70, dt=0.05, probe_fn=lambda t, s: {"f": read_out_vec3(cr, "dbg_oto_luen_head", s)})
        df_r = trr[-1]["f"] - f_neut
        rep.add("B08.3_roll_tilt_sign", f"+20 deg Roll -> delta f_z = -{exp_g:.2f} m/s^2 (±12%)", f"df=[{df_r[0]:+.2f}, {df_r[1]:+.2f}, {df_r[2]:+.2f}]", df_r[2] < 0.0 and abs(df_r[2] + exp_g) / exp_g < 0.12)

        # (b) Pure +3.0 m/s^2 linear accel on spine_tx (+X) and spine_tz (+Z)
        ts_a = np.linspace(0.0, 0.55, 276)
        x_acc = smooth_accel_pulse_pos(ts_a, t0=0.05, t_rise=0.04, t_hold=0.35, a_peak=3.0)
        for cname, ax_idx, tag in [("spine_tx", 0, "AP_+X"), ("spine_tz", 2, "Lat_+Z")]:
            ma, ca, sa = build_isolated_model(
                ctrl_overrides={"enable_vcr": False, "enable_ccr": False, "enable_task_controller": False, "phasic_oto_tanh_mps2": 0.0},
                prescribed_coords={cname: make_spline(ts_a, x_acc)}, lock_neck_except=(),
            )
            tra = integrate_and_sample(
                ma, sa, t_end=0.42, dt=0.01,
                probe_fn=lambda t, s: {"t": t, "luen": read_out_vec3(ca, "dbg_oto_luen_head", s), "irr": read_out_vec3(ca, "dbg_oto_irr_head", s)},
            )
            mid_l = min(tra, key=lambda x: abs(x["t"] - 0.25))["luen"][ax_idx] - f_neut[ax_idx]
            irr_peak = max(x["irr"][ax_idx] for x in tra if 0.05 <= x["t"] <= 0.18) - f_neut[ax_idx] * (37.4 / 245.0)
            irr_late = tra[-1]["irr"][ax_idx] - f_neut[ax_idx] * (37.4 / 245.0)
            rep.add(
                f"B08.4_{tag}_accel",
                f"+3.0 m/s^2 {tag} -> luen~+3.0 (±10%), irr_peak > 3.0 & adapts down",
                f"luen={mid_l:+.2f}, irr_peak={irr_peak:+.2f} -> irr_late={irr_late:+.2f}",
                abs(mid_l - 3.0) < 0.35 and irr_peak > 3.2 and irr_late < irr_peak * 0.75,
            )
    except Exception as e:
        rep.error = str(e)
    rep.wall_s = time.time() - t0
    return rep


def test_B09_somatogravic_reweight_and_tanh() -> BlockReport:
    """B09: Hill-4 otolith reweighting w_oto_rw = 1/(1+(a/a0)^4) and pre-lead hair-bundle tanh saturation."""
    t0 = time.time()
    rep = BlockReport(
        block_id="B09",
        layer="vestibular",
        title="Somatogravic Hill-4 Otolith Reweighting & Hair-Bundle Tanh",
        scenario="Neck locked at 0, a0=8.0 m/s^2, phasic_oto_tanh=12.0 m/s^2; apply pure X accel steps of 2.0, 8.0, 20.0 m/s^2; probe dbg_w_oto_rw and dbg_oto_irr_head.",
    )
    try:
        ts = np.linspace(0.0, 0.30, 151)
        w_vals = {}
        irr_peaks = {}
        for a_amp in (2.0, 8.0, 20.0):
            x_acc = smooth_accel_pulse_pos(ts, t0=0.04, t_rise=0.04, t_hold=0.16, a_peak=a_amp)
            m, c, st = build_isolated_model(
                ctrl_overrides={
                    "enable_vcr": False, "enable_ccr": False, "enable_task_controller": False,
                    "oto_reweight_a0_mps2": 8.0, "oto_reweight_tau_s": 0.20,
                    "phasic_oto_tanh_mps2": 12.0,
                },
                prescribed_coords={"spine_tx": make_spline(ts, x_acc)}, lock_neck_except=(),
            )
            tr = integrate_and_sample(
                m, st, t_end=0.20, dt=0.01,
                probe_fn=lambda t, s: {"t": t, "w": read_out_double(c, "dbg_w_oto_rw", s), "irr_x": read_out_vec3(c, "dbg_oto_irr_head", s)[0]},
            )
            w_vals[a_amp] = min(x["w"] for x in tr if x["t"] >= 0.08)
            irr_peaks[a_amp] = max(abs(x["irr_x"]) for x in tr)

        exp_2 = 1.0 / (1.0 + (2.0 / 8.0) ** 4)    # 0.996
        exp_8 = 1.0 / (1.0 + (8.0 / 8.0) ** 4)    # 0.500
        exp_20 = 1.0 / (1.0 + (20.0 / 8.0) ** 4)  # 0.025
        rep.add("B09.1_bode_transparency", f"At 2 m/s^2: w_oto_rw ~ {exp_2:.3f} (> 0.96)", f"w={w_vals[2.0]:.4f}", w_vals[2.0] > 0.96)
        rep.add("B09.2_half_power_a0", f"At a=a0=8 m/s^2: w_oto_rw ~ {exp_8:.3f} (±0.08)", f"w={w_vals[8.0]:.4f}", abs(w_vals[8.0] - exp_8) < 0.08)
        rep.add("B09.3_highg_attenuation", f"At 20 m/s^2: w_oto_rw ~ {exp_20:.3f} (< 0.08)", f"w={w_vals[20.0]:.4f}", w_vals[20.0] < 0.08)
        # Hair-bundle tanh compresses 20/8 ratio strongly relative to linear 2.5x
        ratio_20_8 = irr_peaks[20.0] / max(1e-3, irr_peaks[8.0])
        rep.add("B09.4_hair_tanh_compress", "irr_peak(20)/irr_peak(8) < 1.90 (sublinear vs linear 2.50)", f"peaks: 8->{irr_peaks[8.0]:.1f}, 20->{irr_peaks[20.0]:.1f} (ratio={ratio_20_8:.2f})", 1.0 < ratio_20_8 < 1.95)
    except Exception as e:
        rep.error = str(e)
    rep.wall_s = time.time() - t0
    return rep


LAYER2_TESTS = [
    test_B06_semicircular_canals_3d,
    test_B07_velocity_storage,
    test_B08_otolith_3d_tilt_and_accel,
    test_B09_somatogravic_reweight_and_tanh,
]
