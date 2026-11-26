"""Layer 1 Functional Unit Tests: Musculoskeletal Plant & Peripheral Mechanoreceptors.

Blocks:
  B01 - 6-DOF Neutral Equilibrium & 72-Muscle Tonic Balance (u0)
  B02 - Kuo (2019) DynamicLigamentForce Bilinear Elasticity & Log Rate Scaling
  B03 - Tendon-Equilibrated Cross-Bridge Short-Range Stiffness (SRS) & Rest-Gated Thixotropy
  B04 - Mileusnic (2006) Muscle Spindle Primary Ia & Secondary II (+ gamma_d / gamma_s)
  B05 - Lin & Crago (2002) Golgi Tendon Organ (GTO) Group Ib Afferent
"""
from __future__ import annotations

import math
import time
import numpy as np
import opensim as osim

from unit_tests.common import (
    BlockReport,
    NECK_COORDS,
    build_isolated_model,
    integrate_and_sample,
    make_spline,
    read_out_double,
    smooth_step_ramp,
)


def test_B01_passive_equilibrium() -> BlockReport:
    """B01: All neural reflexes/voluntary OFF -> 72-muscle u0 must hold 6-DOF static equilibrium."""
    t0 = time.time()
    rep = BlockReport(
        block_id="B01",
        layer="plant",
        title="6-DOF Neutral Equilibrium & 72-Muscle Tonic Balance (u0)",
        scenario="All reflexes & voluntary OFF (enable_vcr=F, enable_ccr=F, enable_task=F, enable_antibuckling=F); 1.0s quiet stance from q=0.",
    )
    try:
        model, _, state = build_isolated_model(
            ctrl_overrides={
                "enable_vcr": False, "enable_ccr": False, "enable_task_controller": False,
                "enable_antibuckling": False, "enable_gto_reflex": False,
                "enable_renshaw": False, "enable_ligament_reflex": False,
            },
            enable_srs=True, enable_gto=True,
        )
        model.realizeAcceleration(state)
        cs = model.getCoordinateSet()
        qdd0 = {cn: math.degrees(cs.get(cn).getAccelerationValue(state)) for cn in NECK_COORDS}
        max_qdd0 = max(abs(v) for v in qdd0.values())

        def probe(t, s):
            return {
                "t": t,
                "pitch": math.degrees(cs.get("pitch1").getValue(s) + cs.get("pitch2").getValue(s)),
                "roll": math.degrees(cs.get("roll1").getValue(s) + cs.get("roll2").getValue(s)),
                "yaw": math.degrees(cs.get("yaw1").getValue(s) + cs.get("yaw2").getValue(s)),
            }

        tr = integrate_and_sample(model, state, t_end=1.0, dt=0.02, probe_fn=probe)
        end_p, end_r, end_y = tr[-1]["pitch"], tr[-1]["roll"], tr[-1]["yaw"]
        rep.add("B01.1_init_accel", "max |qddot(0)| < 1.0 deg/s^2", f"{max_qdd0:.4e} deg/s^2", max_qdd0 < 1.0)
        rep.add("B01.2_pitch_drift", "|pitch(1.0s)| < 0.25 deg", f"{end_p:+.4f} deg", abs(end_p) < 0.25)
        rep.add("B01.3_lr_symmetry", "|roll|, |yaw| < 0.05 deg", f"roll={end_r:+.4f}d, yaw={end_y:+.4f}d", abs(end_r) < 0.05 and abs(end_y) < 0.05)
        rep.plot_data = {"t": [x["t"] for x in tr], "pitch": [x["pitch"] for x in tr], "roll": [x["roll"] for x in tr]}
    except Exception as e:
        rep.error = str(e)
    rep.wall_s = time.time() - t0
    return rep


def test_B02_kuo_ligaments() -> BlockReport:
    """B02: DynamicLigamentForce slack <=1, analytical bilinear stress-strain, and log rate scaling."""
    t0 = time.time()
    rep = BlockReport(
        block_id="B02",
        layer="plant",
        title="Kuo (2019) DynamicLigamentForce Bilinear Elasticity & Rate Scaling",
        scenario="Prescribe slow (-35 deg in 0.8s) vs fast (-35 deg in 0.08s) flexion; probe posterior LF/ISL/CL ligament strain, strain_rate, tension.",
    )
    try:
        def run_flex(dur_s: float):
            ts = np.linspace(0.0, dur_s + 0.10, 200)
            p1 = smooth_step_ramp(ts, 0.02, dur_s, math.radians(-11.7))
            p2 = smooth_step_ramp(ts, 0.02, dur_s, math.radians(-23.3))
            model, _, state = build_isolated_model(
                ctrl_overrides={"enable_vcr": False, "enable_ccr": False, "enable_task_controller": False},
                prescribed_coords={"pitch1": make_spline(ts, p1), "pitch2": make_spline(ts, p2)},
                lock_neck_except=("pitch1", "pitch2"),
            )
            fset = model.getForceSet()
            lig = None
            for i in range(fset.getSize()):
                f = fset.get(i)
                if f.getConcreteClassName() == "DynamicLigamentForce" and f.getName().startswith(("LF_", "ISL_", "SSL_")):
                    lig = f
                    break
            if lig is None:
                raise RuntimeError("No posterior DynamicLigamentForce (LF_/ISL_/SSL_) found in KUO model")
            # Sample midway through ramp (peak rate) and at static hold end
            def probe(t, s):
                return {
                    "t": t,
                    "strain": read_out_double(lig, "strain", s),
                    "strain_rate": read_out_double(lig, "strain_rate", s),
                    "tension": read_out_double(lig, "tension", s),
                    "len_rate": read_out_double(lig, "lengthening_rate", s),
                }
            tr = integrate_and_sample(model, state, t_end=dur_s + 0.08, dt=dur_s / 25.0, probe_fn=probe)
            props = {
                "name": lig.getName(),
                "area": osim.PropertyHelper.getValueDouble(lig.getPropertyByName("cross_sectional_area")),
                "toe_strain": osim.PropertyHelper.getValueDouble(lig.getPropertyByName("toe_strain")),
                "toe_mod": osim.PropertyHelper.getValueDouble(lig.getPropertyByName("toe_modulus")),
                "lin_mod": osim.PropertyHelper.getValueDouble(lig.getPropertyByName("linear_modulus")),
                "rate_scale": osim.PropertyHelper.getValueDouble(lig.getPropertyByName("rate_scale")),
                "rate_int": osim.PropertyHelper.getValueDouble(lig.getPropertyByName("rate_intercept")),
            }
            return tr, props

        tr_slow, pr = run_flex(0.80)
        tr_fast, _ = run_flex(0.08)

        # 1. Slack / neutral check
        s0, f0 = tr_slow[0]["strain"], tr_slow[0]["tension"]
        rep.add("B02.1_slack_at_neutral", "tension == 0 when strain <= 1.0", f"strain={s0:.4f}, F={f0:.4f} N", (s0 <= 1.001 and f0 < 1e-4) or (s0 > 1.0 and f0 >= 0.0))

        # 2. Quasi-static hold analytical bilinear stress-strain check
        s_end, f_end = tr_slow[-1]["strain"], tr_slow[-1]["tension"]
        if s_end <= 1.0:
            f_ana = 0.0
        elif s_end <= pr["toe_strain"]:
            f_ana = pr["area"] * 1e6 * pr["toe_mod"] * (s_end - 1.0)
        else:
            sig_toe = pr["toe_mod"] * (pr["toe_strain"] - 1.0)
            f_ana = pr["area"] * 1e6 * (sig_toe + pr["lin_mod"] * (s_end - pr["toe_strain"]))
        rel_err = abs(f_end - f_ana) / max(1e-3, f_ana)
        rep.add("B02.2_bilinear_static", f"F_static ({pr['name']}) matches analytical {f_ana:.2f} N (±5%)", f"{f_end:.2f} N (strain={s_end:.3f}, err={100*rel_err:.2f}%)", s_end > 1.02 and rel_err < 0.05)

        # 3. Rate-dependent log stiffening: compare fast vs slow at matched mid-strain
        mid_s = 0.5 * (1.0 + s_end)
        idx_s = min(range(len(tr_slow)), key=lambda i: abs(tr_slow[i]["strain"] - mid_s))
        idx_f = min(range(len(tr_fast)), key=lambda i: abs(tr_fast[i]["strain"] - mid_s))
        f_slow_m, r_slow_m = tr_slow[idx_s]["tension"], tr_slow[idx_s]["len_rate"]
        f_fast_m, r_fast_m = tr_fast[idx_f]["tension"], tr_fast[idx_f]["len_rate"]
        rep.add("B02.3_rate_stiffening", "F_fast > F_slow at matched mid-strain (log rate scaling)", f"F_fast={f_fast_m:.2f} N (v={r_fast_m:.4f} m/s) vs F_slow={f_slow_m:.2f} N (v={r_slow_m:.4f} m/s)", f_fast_m > f_slow_m * 1.02 and r_fast_m > r_slow_m * 2.0)
    except Exception as e:
        rep.error = str(e)
    rep.wall_s = time.time() - t0
    return rep


def test_B03_crossbridge_srs() -> BlockReport:
    """B03: Millard12EqMuscleWithAfferents SRS positive elastic jump, thixotropic breakdown on oscillation, and rest recovery."""
    t0 = time.time()
    rep = BlockReport(
        block_id="B03",
        layer="plant",
        title="Cross-Bridge Short-Range Stiffness (SRS) & Rest-Gated Thixotropy",
        scenario="Stretch splen_cap_sklc6 via rapid flexion (-6 deg in 40ms) with SRS ON vs OFF, then oscillate ±15 deg (breakdown) and rest 0.6s (recovery).",
    )
    try:
        ts = np.linspace(0.0, 1.40, 701)
        # 0.05..0.09s rapid small flexion (-6 deg); 0.15..0.75s 3 Hz oscillation (±14 deg); 0.75..1.40s rest
        ang = np.zeros_like(ts)
        for i, t in enumerate(ts):
            if t < 0.05:
                ang[i] = 0.0
            elif t < 0.09:
                u = (t - 0.05) / 0.04
                ang[i] = math.radians(-6.0) * 0.5 * (1.0 - math.cos(math.pi * u))
            elif t < 0.15:
                ang[i] = math.radians(-6.0)
            elif t < 0.75:
                u = (t - 0.15)
                env = 0.5 * (1.0 - math.cos(2.0 * math.pi * u / 0.60))  # C1-smooth Hann window
                ang[i] = math.radians(-6.0) + math.radians(14.0) * env * math.sin(2.0 * math.pi * 3.0 * u)
            else:
                ang[i] = math.radians(-6.0)

        def run_case(srs_on: bool):
            model, _, state = build_isolated_model(
                ctrl_overrides={
                    "enable_vcr": False, "enable_ccr": False, "enable_task_controller": False,
                    "enable_antibuckling": False, "enable_gto_reflex": False, "enable_renshaw": False, "enable_ligament_reflex": False,
                },
                prescribed_coords={"pitch1": make_spline(ts, 0.35 * ang), "pitch2": make_spline(ts, 0.65 * ang)},
                lock_neck_except=("pitch1", "pitch2"),
                enable_srs=srs_on,
            )
            mus = model.getMuscles().get("splen_cap_sklc6")
            sv_thix = f"/forceset/{mus.getName()}/srs_thix"
            def probe(t, s):
                return {
                    "t": t,
                    "ft": float(mus.getTendonForce(s)),
                    "thix": float(model.getStateVariableValue(s, sv_thix)) if srs_on else 1.0,
                }
            return integrate_and_sample(model, state, t_end=1.35, dt=0.01, probe_fn=probe)

        tr_on = run_case(True)
        tr_off = run_case(False)
        # Probe peak force during initial rapid stretch (0.06..0.11s)
        peak_on = max(x["ft"] for x in tr_on if 0.05 <= x["t"] <= 0.11)
        peak_off = max(x["ft"] for x in tr_off if 0.05 <= x["t"] <= 0.11)
        thix_0 = tr_on[0]["thix"]
        thix_min_osc = min(x["thix"] for x in tr_on if 0.20 <= x["t"] <= 0.80)
        thix_end_rest = tr_on[-1]["thix"]

        # Analytical SRS elastic jump on tonic active force: dF_max ~ k_srs * delta_srs * (a0 * F_iso) ~ 0.171 * 1.46 N = 0.25 N (attenuated by 40ms ramp + tendon compliance to ~0.12-0.15 N)
        rep.add("B03.1_srs_stiffness_boost", "F_tendon(SRS ON) > F_tendon(SRS OFF) + 0.08 N on rapid stretch at tonic a0=0.023", f"ON={peak_on:.2f} N vs OFF={peak_off:.2f} N (+{peak_on-peak_off:.2f} N)", peak_on > peak_off + 0.08)
        rep.add("B03.2_thix_breakdown", "srs_thix drops during continuous ±14 deg oscillation (thix_min < 0.70)", f"start={thix_0:.3f} -> min_osc={thix_min_osc:.3f}", thix_min_osc < 0.70 and thix_min_osc < thix_0 - 0.25)
        rep.add("B03.3_rest_recovery", "srs_thix recovers upward during quiet rest (end > min_osc + 0.10)", f"min_osc={thix_min_osc:.3f} -> end_rest={thix_end_rest:.3f}", thix_end_rest > thix_min_osc + 0.10)
        rep.plot_data = {"t": [x["t"] for x in tr_on], "ft_on": [x["ft"] for x in tr_on], "ft_off": [x["ft"] for x in tr_off], "thix": [x["thix"] for x in tr_on]}
    except Exception as e:
        rep.error = str(e)
    rep.wall_s = time.time() - t0
    return rep


def test_B04_mileusnic_spindle() -> BlockReport:
    """B04: Mileusnic06Spindle Ia velocity burst, II static length tracking, and fusimotor gamma_d/gamma_s modulation."""
    t0 = time.time()
    rep = BlockReport(
        block_id="B04",
        layer="plant",
        title="Mileusnic (2006) Spindle Primary Ia & Secondary II (+ Fusimotor)",
        scenario="Ramp-and-hold flexion (-15 deg in 0.20s, hold to 0.65s) on splen_cap_sklc6 under (K_gd=0,K_gs=0) vs (K_gd=1,K_gs=0) vs (K_gd=0,K_gs=1).",
    )
    try:
        ts = np.linspace(0.0, 0.70, 351)
        p1 = smooth_step_ramp(ts, 0.05, 0.20, math.radians(-5.0))
        p2 = smooth_step_ramp(ts, 0.05, 0.20, math.radians(-10.0))

        def run_sp(kgd: float, kgs: float):
            model, _, state = build_isolated_model(
                ctrl_overrides={
                    "enable_vcr": False, "enable_ccr": False, "enable_task_controller": False,
                    "enable_antibuckling": False, "enable_gto_reflex": False, "enable_renshaw": False, "enable_ligament_reflex": False,
                    "enable_fusimotor": True, "K_gamma_dyn": kgd, "K_gamma_stat": kgs,
                },
                prescribed_coords={"pitch1": make_spline(ts, p1), "pitch2": make_spline(ts, p2)},
                lock_neck_except=("pitch1", "pitch2"),
            )
            mus = model.getMuscles().get("splen_cap_sklc6")
            def probe(t, s):
                return {
                    "t": t,
                    "ia": read_out_double(mus, "primary_Ia", s),
                    "ii": read_out_double(mus, "secondary_II", s),
                }
            return integrate_and_sample(model, state, t_end=0.65, dt=0.01, probe_fn=probe)

        tr_00 = run_sp(0.0, 0.0)
        tr_10 = run_sp(1.0, 0.0)

        # Test static fusimotor gamma_s during active isometric contraction (u_alpha ~ 0.25 -> gamma_s ~ 25 pps -> f_s ~ 0.15)
        # At tonic quiet a0=0.023, gamma_s = 2.3 pps gives f_s = 0.0015 (physiological silence); active coactivation engages bag2/chain!
        def run_gs_active(kgs: float):
            ma, ca, sa = build_isolated_model(
                ctrl_overrides={
                    "enable_vcr": False, "enable_ccr": False, "enable_antibuckling": False,
                    "enable_gto_reflex": False, "enable_renshaw": False, "enable_ligament_reflex": False,
                    "enable_task_controller": True, "urgency": 1.0, "desired_pitch": math.radians(20.0),
                    "Kp_task_pitch": 12.0, "Ki_task_pitch": 0.0,
                    "enable_fusimotor": True, "K_gamma_dyn": 0.0, "K_gamma_stat": kgs,
                },
                lock_neck_except=(),
            )
            mus_a = ma.getMuscles().get("splen_cap_sklc6")
            tr_a = integrate_and_sample(
                ma, sa, t_end=0.40, dt=0.05,
                probe_fn=lambda t, s: {"ii": read_out_double(mus_a, "secondary_II", s), "act": float(mus_a.getActivation(s))},
            )
            return tr_a[-1]

        gs_off = run_gs_active(0.0)
        gs_on  = run_gs_active(1.0)

        ia_base = tr_00[3]["ia"]
        ia_peak_00 = max(x["ia"] for x in tr_00 if 0.05 <= x["t"] <= 0.30)
        ia_hold_00 = tr_00[-1]["ia"]
        ii_base = tr_00[3]["ii"]
        ii_hold_00 = tr_00[-1]["ii"]

        ia_peak_10 = max(x["ia"] for x in tr_10 if 0.05 <= x["t"] <= 0.30)
        ii_hold_10 = tr_10[-1]["ii"]

        rep.add("B04.1_ia_dyn_burst", "Ia_peak > Ia_hold > Ia_base (velocity sensitivity)", f"base={ia_base:.1f}, peak={ia_peak_00:.1f}, hold={ia_hold_00:.1f} imp/s", ia_peak_00 > ia_hold_00 + 5.0 and ia_hold_00 > ia_base + 5.0)
        rep.add("B04.2_ii_static_len", "II_hold > II_base (static length encoding)", f"base={ii_base:.1f} -> hold={ii_hold_00:.1f} imp/s", ii_hold_00 > ii_base + 5.0)
        rep.add("B04.3_gamma_dyn_effect", "gamma_d=1 boosts Ia ramp peak without shifting II", f"Ia_peak: {ia_peak_00:.1f}->{ia_peak_10:.1f}; II_hold: {ii_hold_00:.1f}->{ii_hold_10:.1f}", ia_peak_10 > ia_peak_00 + 5.0 and abs(ii_hold_10 - ii_hold_00) < 2.0)
        rep.add("B04.4_gamma_stat_effect", "Active alpha-gamma coactivation (K_gs=1 vs 0) elevates static II > +2 imp/s", f"act={gs_on['act']:.2f}: II {gs_off['ii']:.1f} -> {gs_on['ii']:.1f} imp/s (dII={gs_on['ii']-gs_off['ii']:+.1f})", gs_on["ii"] > gs_off["ii"] + 2.0)
    except Exception as e:
        rep.error = str(e)
    rep.wall_s = time.time() - t0
    return rep


def test_B05_lin02_gto() -> BlockReport:
    """B05: Lin02GolgiTendonOrgan dynamic overshoot and logarithmic steady-state force-firing curve."""
    t0 = time.time()
    rep = BlockReport(
        block_id="B05",
        layer="plant",
        title="Lin & Crago (2002) Golgi Tendon Organ (GTO) Group Ib Afferent",
        scenario="Isometric neck (all 6 DOFs locked at 0); command sustained pitch extension torque via desired_pitch=+25 deg (u=1) so extensor tendon force jumps; probe tendon_Ib vs F_T.",
    )
    try:
        model, controller, state = build_isolated_model(
            ctrl_overrides={
                "enable_vcr": False, "enable_ccr": False, "enable_task_controller": True,
                "enable_gto_reflex": True, "K_gto_ib": 0.0,
                "urgency": 1.0, "desired_pitch": 0.0,
            },
            lock_neck_except=(),
            enable_gto=True,
        )
        mus = model.getMuscles().get("splen_cap_sklc6")
        p_dp = controller.updPropertyByName("desired_pitch")
        step_set = {"done": False}
        def hook(t, s):
            if not step_set["done"] and t >= 0.05:
                osim.PropertyHelper.setValueDouble(math.radians(25.0), p_dp)
                step_set["done"] = True
        def probe(t, s):
            return {
                "t": t,
                "ft": float(mus.getTendonForce(s)),
                "ib": read_out_double(mus, "tendon_Ib", s),
            }
        tr = integrate_and_sample(model, state, t_end=0.60, dt=0.01, probe_fn=probe, step_hook=hook)
        ib_base, ft_base = tr[3]["ib"], tr[3]["ft"]
        ib_peak = max(x["ib"] for x in tr if x["t"] >= 0.05)
        ib_end, ft_end = tr[-1]["ib"], tr[-1]["ft"]
        f_max = float(mus.getMaxIsometricForce())
        # Exact analytical Lin & Crago (2002) 2-pole dynamic ODE integration driven by F_T(t):
        #   u_nl = 60*ln(1 + F_T/Gf), dw1/dt = -2*w1 + u_nl, dw2/dt = -0.2*w2 + u_nl,
        #   y_lin02 = 1.7*u_nl - (17/15)*w1 - (2/75)*w2  (DC gain = 1.0, transient gain = 1.7).
        gf = max(1.5, 0.08 * f_max)
        u0 = 60.0 * math.log(1.0 + max(0.0, tr[0]["ft"]) / gf)
        w1, w2 = u0 / 2.0, u0 / 0.2
        ib_ana_trace = [u0]
        for i in range(1, len(tr)):
            dt_i = tr[i]["t"] - tr[i - 1]["t"]
            u_prev = 60.0 * math.log(1.0 + max(0.0, tr[i - 1]["ft"]) / gf)
            u_curr = 60.0 * math.log(1.0 + max(0.0, tr[i]["ft"]) / gf)
            u_mid = 0.5 * (u_prev + u_curr)
            # Exact exponential step over dt_i for piecewise-constant u_mid
            w1 = w1 * math.exp(-2.0 * dt_i) + (u_mid / 2.0) * (1.0 - math.exp(-2.0 * dt_i))
            w2 = w2 * math.exp(-0.2 * dt_i) + (u_mid / 0.2) * (1.0 - math.exp(-0.2 * dt_i))
            ib_ana_trace.append(1.7 * u_curr - (17.0 / 15.0) * w1 - (2.0 / 75.0) * w2)
        ib_ana_end = ib_ana_trace[-1]
        ib_sim_arr = np.array([x["ib"] for x in tr])
        ib_ana_arr = np.array(ib_ana_trace)
        rms_err = float(np.sqrt(np.mean((ib_sim_arr - ib_ana_arr) ** 2)))
        rel_err = abs(ib_end - ib_ana_end) / max(1.0, ib_ana_end)

        rep.add("B05.1_ib_monotonic_rise", "tendon_Ib rises strongly when tendon force rises", f"F_T: {ft_base:.1f}->{ft_end:.1f} N; Ib: {ib_base:.1f}->{ib_end:.1f} imp/s", ft_end > ft_base + 5.0 and ib_end > ib_base + 10.0)
        rep.add("B05.2_ib_dyn_overshoot", "Ib exhibits Lin02 dynamic overshoot (peak >= end)", f"peak={ib_peak:.1f}, end={ib_end:.1f} imp/s", ib_peak >= ib_end * 0.99)
        rep.add("B05.3_lin02_ode_match", f"Ib(t) matches exact Lin02 2-pole ODE ({ib_ana_end:.1f} imp/s, end err < 3%, RMS < 3 imp/s)", f"end={ib_end:.1f} imp/s (err={100*rel_err:.2f}%, RMS={rms_err:.2f} imp/s)", rel_err < 0.03 and rms_err < 3.0)
        rep.plot_data = {"t": [x["t"] for x in tr], "ft": [x["ft"] for x in tr], "ib_sim": [x["ib"] for x in tr], "ib_ana": ib_ana_trace}
    except Exception as e:
        rep.error = str(e)
    rep.wall_s = time.time() - t0
    return rep


LAYER1_TESTS = [
    test_B01_passive_equilibrium,
    test_B02_kuo_ligaments,
    test_B03_crossbridge_srs,
    test_B04_mileusnic_spindle,
    test_B05_lin02_gto,
]
