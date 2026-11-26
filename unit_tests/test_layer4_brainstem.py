"""Layer 4 Functional Unit Tests: Brainstem VCR Pathways & Delay Lead Compensators.

Blocks:
  B14 - Brainstem VCR Individual Pathways (Tonic, Ki_ton, Canal, Phasic Otolith) — EXACT 3-AXIS SIGN & GAIN AUDIT!
  B15 - Causal Biproper 1st-Order Branch Lead Compensators (Phase Restoration & Zero-Kick Start)
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
    read_out_vec3,
    smooth_accel_pulse_pos,
    smooth_step_ramp,
    smooth_vel_pulse_pos,
)


def test_B14_brainstem_vcr_all_pathways() -> BlockReport:
    """B14: Open-loop isolation of Tonic, Ki_ton, Canal, and Phasic Otolith VCR on all 3 axes.

    CRITICAL SIGN VERIFICATION FOR PHASIC OTOLITH:
      First runs a passive unactuated head (VCR=OFF, CCR=OFF) under +5 m/s^2 spine_tx and +5 m/s^2 spine_tz
      to measure the TRUE OpenSim inertial head deflection sign (delta_pitch_passive, delta_roll_passive).
      Then enables ONLY phasic VCR with neck held at 0 and verifies whether dbg_vcr_torque OPPOSES the
      passive inertial deflection!
    """
    t0 = time.time()
    rep = BlockReport(
        block_id="B14",
        layer="brainstem",
        title="Brainstem VCR Pathways (Tonic, Ki_ton, Canal, Phasic — Exact Sign & Gain)",
        scenario="Isolate each VCR sub-branch (Tonic, Ki_ton, Canal, Phasic) with open-loop prescribed tilt, angular vel, and linear accel; compare Phasic sign against passive inertial deflection!",
    )
    try:
        # 1. Tonic VCR Covariant Tensor Pencil (K_ton = alpha_grav*Kg3 + omega_ton^2*I3 + c_disc*Kdisc3):
        #    +10 deg static tilt on Pitch(Z=2), Roll(X=0), Yaw(Y=1) -> vcr_torque[ax_i] MUST match -K_ii * rad(10)
        ts_t = np.linspace(0.0, 0.70, 141)
        q10 = smooth_step_ramp(ts_t, 0.05, 0.20, math.radians(10.0))
        a_g, w_t, c_d = 0.60, 6.0, 1.00
        for cname, ax_i, label in [
            ("gndpitch", 2, "Pitch"),
            ("gndroll",  0, "Roll"),
            ("gndyaw",   1, "Yaw"),
        ]:
            m, c, st = build_isolated_model(
                ctrl_overrides={
                    "enable_vcr": True, "enable_vcr_tonic": True, "enable_vcr_canal": False, "enable_vcr_phasic": False,
                    "enable_ccr": False, "enable_task_controller": False,
                    "alpha_grav_ton": a_g, "omega_ton": w_t, "c_disc_ton": c_d, "gamma_musc_ton": 0.0,
                    "ki_grav_ton": 0.0, "p_ton": 25.0,
                },
                prescribed_coords={cname: make_spline(ts_t, q10)}, lock_neck_except=(),
            )
            tr = integrate_and_sample(
                m, st, t_end=0.60, dt=0.02,
                probe_fn=lambda t, s: {
                    "vcr": read_out_vec3(c, "dbg_vcr_torque", s)[ax_i],
                    "I": read_out_vec3(c, "dbg_tensor_inertia_diag", s)[ax_i],
                    "Kg": read_out_vec3(c, "dbg_tensor_kgrav_diag", s)[ax_i],
                    "Kd": read_out_vec3(c, "dbg_tensor_kdisc_diag", s)[ax_i],
                },
            )
            tau_end = tr[-1]["vcr"]
            k_ii = a_g * tr[-1]["Kg"] + (w_t * w_t) * tr[-1]["I"] + c_d * tr[-1]["Kd"]
            exp_tau = -k_ii * math.radians(10.0)
            rep.add(f"B14.1_tonic_{label}", f"vcr_torque[{ax_i}] ~ {exp_tau:+.3f} Nm (±15%, strictly negative)", f"{tau_end:+.3f} Nm (K_ii={k_ii:.2f})", tau_end < 0.0 and abs(tau_end - exp_tau) / abs(exp_tau) < 0.15)

        # 2. Brainstem Leaky Tonic Verticality Integrator (ki_grav_ton * Kg3): hold +10 deg Pitch for 1.8s
        ts_2 = np.linspace(0.0, 2.0, 201)
        ki_g_val = 0.60
        m_ti, c_ti, s_ti = build_isolated_model(
            ctrl_overrides={
                "enable_vcr": True, "enable_vcr_tonic": True, "enable_vcr_canal": False, "enable_vcr_phasic": False,
                "enable_ccr": False, "enable_task_controller": False,
                "alpha_grav_ton": 0.0, "omega_ton": 0.0, "c_disc_ton": 0.0, "gamma_musc_ton": 0.0,
                "ki_grav_ton": ki_g_val, "tau_ton_int": 4.0, "p_ton": 25.0,
            },
            prescribed_coords={"gndpitch": make_spline(ts_2, smooth_step_ramp(ts_2, 0.02, 0.10, math.radians(10.0)))},
            lock_neck_except=(),
        )
        tr_ti = integrate_and_sample(
            m_ti, s_ti, t_end=1.80, dt=0.05,
            probe_fn=lambda t, s: {
                "ti": read_ctrl_sv(m_ti, s, "ton_int_z"),
                "vcr": read_out_vec3(c_ti, "dbg_vcr_torque", s)[2],
                "Kg_zz": read_out_vec3(c_ti, "dbg_tensor_kgrav_diag", s)[2],
            },
        )
        ki_eff = ki_g_val * tr_ti[-1]["Kg_zz"]
        exp_ti = -3.5 * math.tanh((ki_eff * math.radians(10.0) * 4.0 * (1.0 - math.exp(-1.70 / 4.0))) / 3.5)
        tau_ti = tr_ti[-1]["vcr"]
        rep.add("B14.2_tonic_integrator", f"ki_grav_ton={ki_g_val} (Ki_zz={ki_eff:.2f}) -> restoring torque ~ {exp_ti:+.3f} Nm (±15%)", f"vcr={tau_ti:+.3f} Nm (x_ti={tr_ti[-1]['ti']:+.3f})", tau_ti < 0.0 and abs(tau_ti - exp_ti) / abs(exp_ti) < 0.15)

        # 3. Canal VCR Covariant Tensor Pencil (D_sc = omega_sc*I3 + c_disc_sc*Kdisc3): +20 deg/s pulse on Pitch(2), Roll(0), Yaw(1)
        ts_v = np.linspace(0.0, 0.45, 226)
        v20 = smooth_vel_pulse_pos(ts_v, t0=0.04, t_rise=0.04, t_hold=0.25, v_peak=math.radians(20.0))
        w_sc_v, cd_sc_v = 25.0, 0.30
        for cname, ax_i, label in [
            ("gndpitch", 2, "Pitch"),
            ("gndroll",  0, "Roll"),
            ("gndyaw",   1, "Yaw"),
        ]:
            m, c, st = build_isolated_model(
                ctrl_overrides={
                    "enable_vcr": True, "enable_vcr_tonic": False, "enable_vcr_canal": True, "enable_vcr_phasic": False,
                    "enable_ccr": False, "enable_task_controller": False,
                    "omega_sc": w_sc_v, "c_disc_sc": cd_sc_v, "beta_musc_sc": 0.0, "c_grav_sc": 0.0,
                    "z_sc": 0.0, "p_sc": 45.0,
                },
                prescribed_coords={cname: make_spline(ts_v, v20)}, lock_neck_except=(),
            )
            tr = integrate_and_sample(
                m, st, t_end=0.28, dt=0.01,
                probe_fn=lambda t, s: {
                    "t": t,
                    "vcr": read_out_vec3(c, "dbg_vcr_torque", s)[ax_i],
                    "I": read_out_vec3(c, "dbg_tensor_inertia_diag", s)[ax_i],
                    "Kd": read_out_vec3(c, "dbg_tensor_kdisc_diag", s)[ax_i],
                },
            )
            mid_s = min(tr, key=lambda x: abs(x["t"] - 0.22))
            mid_tau = mid_s["vcr"]
            d_ii = w_sc_v * mid_s["I"] + cd_sc_v * mid_s["Kd"]
            exp_tau = -d_ii * math.radians(20.0)
            rep.add(f"B14.3_canal_{label}", f"vcr_torque[{ax_i}] ~ {exp_tau:+.3f} Nm (±15%, opposes +vel)", f"{mid_tau:+.3f} Nm (D_ii={d_ii:.2f})", mid_tau < 0.0 and abs(mid_tau - exp_tau) / abs(exp_tau) < 0.15)

        # 4. Phasic Otolith VCR (df_A x u_up_A) — EMPIRICAL GROUND-TRUTH SIGN CHECK AGAINST PASSIVE INERTIAL WHIPLASH!
        ts_a = np.linspace(0.0, 0.30, 151)
        pos_a = smooth_accel_pulse_pos(ts_a, t0=0.02, t_rise=0.03, t_hold=0.15, a_peak=5.0)
        phasic_plot = {}
        for cname, ax_i, c1, c2, label in [
            ("spine_tx", 2, "pitch1", "pitch2", "Pitch_AP"),
            ("spine_tz", 0, "roll1",  "roll2",  "Roll_Lat"),
        ]:
            # Step A: Measure TRUE passive unactuated head deflection under +5 m/s^2 T1 accel
            mp, _, sp_s = build_isolated_model(
                ctrl_overrides={"enable_vcr": False, "enable_ccr": False, "enable_task_controller": False, "enable_antibuckling": False},
                prescribed_coords={cname: make_spline(ts_a, pos_a)},
                enable_srs=False,
            )
            cs_p = mp.getCoordinateSet()
            tr_p = integrate_and_sample(mp, sp_s, t_end=0.14, dt=0.01, probe_fn=lambda t, s: {"t": t, "ang": math.degrees(cs_p.get(c1).getValue(s) + cs_p.get(c2).getValue(s))})
            dq_passive = tr_p[-1]["ang"] - tr_p[0]["ang"]

            # Step B: Measure Phasic VCR torque sign with neck held at 0 under identical +5 m/s^2 T1 accel
            mv, cv, sv_s = build_isolated_model(
                ctrl_overrides={
                    "enable_vcr": True, "enable_vcr_tonic": False, "enable_vcr_canal": False, "enable_vcr_phasic": True,
                    "enable_ccr": False, "enable_task_controller": False,
                    "alpha_inert_phas": 10.0, "c_disc_phas": 1.0, "gamma_musc_phas": 0.0,
                    "oto_reweight_a0_mps2": 0.0, "phasic_oto_tanh_mps2": 0.0,
                },
                prescribed_coords={cname: make_spline(ts_a, pos_a)},
                lock_neck_except=(),
            )
            tr_v = integrate_and_sample(mv, sv_s, t_end=0.14, dt=0.01, probe_fn=lambda t, s: {"t": t, "vcr": read_out_vec3(cv, "dbg_vcr_torque", s)[ax_i], "ph_in": read_out_vec3(cv, "dbg_phasic_oto_input", s)[ax_i]})
            idx_pk = max(range(len(tr_v)), key=lambda i: abs(tr_v[i]["vcr"]))
            tau_phas = tr_v[idx_pk]["vcr"]
            ph_in_pk = tr_v[idx_pk]["ph_in"]
            opposes_inertia = (dq_passive * tau_phas) < 0.0
            phasic_plot[label] = {
                "t": [x["t"] for x in tr_p],
                "dq_pass": [x["ang"] - tr_p[0]["ang"] for x in tr_p],
                "tau_vcr": [x["vcr"] for x in tr_v],
                "ph_in": [x["ph_in"] for x in tr_v],
            }
            rep.add(
                f"B14.4_phasic_sign_{label}",
                f"vcr_phasic opposes passive inertial deflection (dq_pass={dq_passive:+.2f} deg -> tau MUST be {'< 0' if dq_passive > 0 else '> 0'})",
                f"dq_passive={dq_passive:+.2f}d, ph_in={ph_in_pk:+.2f}, tau_phas={tau_phas:+.2f} Nm -> {'OPPOSES' if opposes_inertia else 'ASSISTS (INVERTED!)'}",
                opposes_inertia and abs(tau_phas) > 2.0,
            )
        rep.plot_data = {"phasic": phasic_plot}
    except Exception as e:
        rep.error = str(e)
    rep.wall_s = time.time() - t0
    return rep


def test_B15_branch_lead_compensators() -> BlockReport:
    """B15: Causal proper lead compensators restore phase across anatomical delays without t=0 transient kick."""
    t0 = time.time()
    rep = BlockReport(
        block_id="B15",
        layer="brainstem",
        title="Causal Proper 1st-Order Branch Lead Compensators",
        scenario="2.0 Hz sinusoidal gndpitch angular velocity with enable_lead_compensators=True vs False; measure phase of dbg_vcr_torque vs -omega(t).",
    )
    try:
        ts = np.linspace(0.0, 2.0, 1001)
        f_hz = 2.0
        amp_rad = math.radians(4.0)
        # Smoothly ramped 2 Hz sinusoid
        q_sin = np.array([amp_rad * min(1.0, max(0.0, (t - 0.1) / 0.3)) * math.sin(2.0 * math.pi * f_hz * t) for t in ts])

        def measure_phase(lead_on: bool):
            m, c, st = build_isolated_model(
                ctrl_overrides={
                    "enable_lead_compensators": lead_on,
                    "enable_vcr": True, "enable_vcr_tonic": False, "enable_vcr_canal": True, "enable_vcr_phasic": False,
                    "enable_ccr": False, "enable_task_controller": False,
                    "omega_sc": 25.0, "c_disc_sc": 0.20, "z_sc": 0.0, "p_sc": 80.0, "tau_lead_pole": 0.008,
                },
                prescribed_coords={"gndpitch": make_spline(ts, q_sin)}, lock_neck_except=(),
            )
            tr = integrate_and_sample(m, st, t_end=1.80, dt=0.005, probe_fn=lambda t, s: {"t": t, "vcr": read_out_vec3(c, "dbg_vcr_torque", s)[2]})
            t_arr = np.array([x["t"] for x in tr if x["t"] >= 0.8])
            y_arr = np.array([x["vcr"] for x in tr if x["t"] >= 0.8])
            # Input velocity is +omega*amp*cos(2*pi*f*t); ideal -K_sc*vel is -cos(2*pi*f*t) -> phase 0 relative to -cos
            ref_c = -np.cos(2.0 * np.pi * f_hz * t_arr)
            ref_s = -np.sin(2.0 * np.pi * f_hz * t_arr)
            a_c = float(np.mean(y_arr * ref_c))
            b_s = float(np.mean(y_arr * ref_s))
            # Negative phase = lag
            ph_deg = math.degrees(math.atan2(-b_s, a_c))
            kick0 = abs(tr[0]["vcr"])
            return ph_deg, kick0

        ph_on, k0_on = measure_phase(True)
        ph_off, _ = measure_phase(False)
        # Total delay on canal VCR path is 2.0ms (2A) + 2.5ms (Path 5 MVST) = 4.5 ms -> at 2 Hz = 3.24 deg phase saved
        rep.add("B15.1_zero_start_kick", "|vcr(0)| < 1e-4 Nm (zero initial transient)", f"{k0_on:.4e} Nm", k0_on < 1e-4)
        rep.add("B15.2_phase_lead_restored", "Lead ON reduces phase lag vs Lead OFF (ph_on > ph_off + 1.5 deg)", f"ON={ph_on:+.2f}d vs OFF={ph_off:+.2f}d (gain={ph_on-ph_off:+.2f}d)", ph_on > ph_off + 1.5)
    except Exception as e:
        rep.error = str(e)
    rep.wall_s = time.time() - t0
    return rep


LAYER4_TESTS = [
    test_B14_brainstem_vcr_all_pathways,
    test_B15_branch_lead_compensators,
]
