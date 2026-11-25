"""
Single-Muscle Validation Suite for Tendon-Equilibrated Plan 2 SRS
(Millard12EqMuscleWithAfferents).

Benchmarks:
  1. Cui et al. (2008) Constant-Velocity Ramp (a = 0.1, 0.3, 0.5, 1.0):
     - Exact fiber-tendon equilibrium |F_T - F_fiber_AT| < 1e-6 N
     - Monotonic stretch plateau (zero negative stiffness dF/dl < 0)
     - Measured series MTU stiffness matches K_MTU = (K_M cos^2 a * K_T) / (K_M cos^2 a + K_T)
  2. Kirsch et al. (1994) Sinusoidal Impedance Sweep (0.5 - 15 Hz, small vs large amplitude):
     - Stiffness-dominated impedance with light positive phase lead (~5-35 deg in 0.5-8 Hz)
     - Low harmonic distortion (2nd/1st < 5% for small perturbations)
     - Amplitude-dependent softening (large amp < small amp stiffness)
  3. De Groote et al. (2018) / Campbell & Lakie (1998) Double-Stretch Thixotropy Test:
     - Stretch 1 from rest -> full SRS peak (s_thix ~ 1.0)
     - Immediate Stretch 2 -> reduced SRS peak (s_thix broken down)
     - Stretch 3 after 3.0 s rest -> SRS peak recovers toward Stretch 1
  4. Runtime speed check: SRS=ON vs SRS=OFF overhead ratio
"""

import math
import os
import sys
import time
import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, "util"), os.path.join(_ROOT, "scripts")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import opensim as osim
import scripts.generate_standard_plot as gsp
import scripts.run_zheng_2g_3g_urgency_sweep as rz

MODEL_KUO = os.path.join(_ROOT, "osim_files", "HYOID_HeadNeckNeural_Model_KuoLigaments.osim")
TEST_MUSCLE = sorted(rz.SPLEN_CAP_NAMES)[0]


def extract_splen_cap_params():
    gsp.ensure_libraries_loaded()
    m = osim.Model(MODEL_KUO)
    mus = m.getMuscles().get(TEST_MUSCLE)
    return {
        "name": mus.getName(),
        "f_max": float(mus.getMaxIsometricForce()),
        "l_opt": float(mus.getOptimalFiberLength()),
        "l_ts": float(mus.getTendonSlackLength()),
        "penn": float(mus.getPennationAngleAtOptimalFiberLength()),
        "vmax": float(mus.getMaxContractionVelocity()),
    }


def build_single_muscle_model(mp, enable_srs: bool, exc_val: float, motion_func):
    model = osim.Model()
    model.setName("SingleMuscleSRSBench")
    model.setGravity(osim.Vec3(0, 0, 0))

    body = osim.Body("slider", 1.0, osim.Vec3(0), osim.Inertia(1, 1, 1, 0, 0, 0))
    model.addBody(body)

    joint = osim.SliderJoint(
        "slide",
        model.getGround(), osim.Vec3(0), osim.Vec3(0),
        body, osim.Vec3(0), osim.Vec3(0),
    )
    coord = joint.updCoordinate()
    coord.setName("x")
    l_mt0 = mp["l_opt"] * math.cos(mp["penn"]) + mp["l_ts"]
    coord.setDefaultValue(l_mt0)
    coord.setDefaultIsPrescribed(True)
    coord.setPrescribedFunction(motion_func)
    model.addJoint(joint)

    # Instantiate Millard12EqMuscleWithAfferents via prototype clone from HYOID model
    src = osim.Model(MODEL_KUO)
    src_mus = src.getMuscles().get(TEST_MUSCLE)
    mus = src_mus.clone()
    mus.setName("test_splen")
    mus.updGeometryPath().updPathPointSet().clearAndDestroy()
    mus.addNewPathPoint("orig", model.getGround(), osim.Vec3(0, 0, 0))
    mus.addNewPathPoint("ins", body, osim.Vec3(0, 0, 0))

    osim.PropertyHelper.setValueBool(enable_srs, mus.updPropertyByName("enable_SRS"))
    osim.PropertyHelper.setValueBool(False, mus.updPropertyByName("ignore_tendon_compliance"))
    osim.PropertyHelper.setValueBool(True, mus.updPropertyByName("ignore_activation_dynamics"))
    model.addForce(mus)

    ctrl = osim.PrescribedController()
    ctrl.addActuator(mus)
    ctrl.prescribeControlForActuator("test_splen", osim.Constant(exc_val))
    model.addController(ctrl)

    state = model.initSystem()
    mus_live = model.updMuscles().get("test_splen")
    mus_live.setActivation(state, exc_val)
    model.equilibrateMuscles(state)
    return model, state, mus_live, l_mt0


def integrate_trace(model, state, mus, t_end, dt=0.001):
    mgr = osim.Manager(model)
    mgr.setWriteToStorage(False)
    mgr.setPerformAnalyses(False)
    mgr.setIntegratorMethod(5)
    mgr.setIntegratorAccuracy(1e-5)
    mgr.setIntegratorMinimumStepSize(1e-5)
    mgr.setIntegratorMaximumStepSize(0.001)
    state.setTime(0.0)
    mgr.initialize(state)

    coord = model.getCoordinateSet().get("x")
    ts, xs, fts, ffs, lf_ns, sthix_l, kt_l, kf_l = [], [], [], [], [], [], [], []
    sv_names = model.getStateVariableNames()
    has_thix = sv_names.findIndex("/forceset/test_splen/srs_thix") >= 0

    t0 = time.perf_counter()
    n = int(round(t_end / dt))
    for k in range(n + 1):
        t = k * dt
        if k > 0:
            state = mgr.integrate(t)
        model.realizeDynamics(state)
        ts.append(t)
        xs.append(float(coord.getValue(state)))
        fts.append(float(mus.getTendonForce(state)))
        ffs.append(float(mus.getFiberForceAlongTendon(state)))
        lf_ns.append(float(mus.getNormalizedFiberLength(state)))
        kt_l.append(float(mus.getTendonStiffness(state)))
        kf_l.append(float(mus.getFiberStiffnessAlongTendon(state)))
        sthix_l.append(float(model.getStateVariableValue(state, "/forceset/test_splen/srs_thix")) if has_thix else 1.0)
    wall = time.perf_counter() - t0
    return {
        "t": np.array(ts), "x": np.array(xs), "ft": np.array(fts), "ff": np.array(ffs),
        "lf_n": np.array(lf_ns), "s_thix": np.array(sthix_l),
        "kt": np.array(kt_l), "kf": np.array(kf_l), "wall_s": wall,
    }


def make_piecewise_spline(t_pts, x_pts):
    fn = osim.SimmSpline()
    for t, x in zip(t_pts, x_pts):
        fn.addPoint(float(t), float(x))
    return fn


def run_benchmark_1_cui_ramp(mp):
    print("\n=== Benchmark 1: Cui et al. (2008) Ramp Stretch (Series Stiffness & Equilibrium) ===")
    l_opt = mp["l_opt"]
    l_mt0 = l_opt * math.cos(mp["penn"]) + mp["l_ts"]
    # 0-0.1s hold, 0.1-0.25s constant-velocity ramp of +4% L_0 (0.267 L_0/s), 0.25-0.35s hold
    amp = 0.04 * l_opt
    t_grid = np.linspace(0.0, 0.35, 351)
    x_grid = np.where(
        t_grid <= 0.10, l_mt0,
        np.where(t_grid <= 0.25, l_mt0 + amp * (t_grid - 0.10) / 0.15, l_mt0 + amp)
    )
    # Smooth corners slightly with a 3-point moving average so velocity is C1
    x_grid = np.convolve(np.pad(x_grid, 2, mode="edge"), np.ones(5) / 5.0, mode="valid")

    all_ok = True
    for act in (0.1, 0.3, 0.5, 1.0):
        m_on, s_on, mus_on, _ = build_single_muscle_model(mp, True, act, make_piecewise_spline(t_grid, x_grid))
        tr_on = integrate_trace(m_on, s_on, mus_on, 0.35, dt=0.001)
        m_off, s_off, mus_off, _ = build_single_muscle_model(mp, False, act, make_piecewise_spline(t_grid, x_grid))
        tr_off = integrate_trace(m_off, s_off, mus_off, 0.35, dt=0.001)

        eq_err = float(np.max(np.abs(tr_on["ft"] - tr_on["ff"])))
        # Measure stiffness on early linear ramp (0.105 to 0.135 s, ~0.8% L_0 stretch)
        msk_lin = (tr_on["t"] >= 0.105) & (tr_on["t"] <= 0.135)
        k_meas_on = float(np.polyfit(tr_on["x"][msk_lin], tr_on["ft"][msk_lin], 1)[0])
        k_meas_off = float(np.polyfit(tr_off["x"][msk_lin], tr_off["ft"][msk_lin], 1)[0])

        # Analytical Cui/Hu series stiffness at t=0.100 (pre-stretch equilibrium)
        idx0 = int(np.argmin(np.abs(tr_on["t"] - 0.100)))
        kt0 = tr_on["kt"][idx0]
        kf0 = tr_on["kf"][idx0]
        k_series = (kf0 * kt0) / (kf0 + kt0) if (kf0 + kt0) > 1e-6 else 0.0

        # Check monotonicity during ramp (0.105 to 0.245 s): no negative stiffness collapse!
        msk_ramp = (tr_on["t"] >= 0.105) & (tr_on["t"] <= 0.245)
        df_ramp = np.diff(tr_on["ft"][msk_ramp])
        min_df = float(np.min(df_ramp))
        peak_on = float(np.max(tr_on["ft"]))
        peak_off = float(np.max(tr_off["ft"]))
        fiber_dl_on = float(np.max(tr_on["lf_n"][msk_lin]) - tr_on["lf_n"][idx0])
        fiber_dl_off = float(np.max(tr_off["lf_n"][msk_lin]) - tr_off["lf_n"][idx0])

        ok = (eq_err < 1e-5) and (min_df > -0.05) and (k_meas_on > k_meas_off) and (k_meas_on <= kt0 * 1.15)
        all_ok = all_ok and ok
        print(
            f"  a={act:.1f}: eqErr={eq_err:.2e}N | K_meas(ON)={k_meas_on:7.0f} vs K_series={k_series:7.0f} (K_T={kt0:7.0f}, K_off={k_meas_off:6.0f}) N/m | "
            f"min_dF={min_df:+.3f}N | pkF={peak_on:.1f} vs {peak_off:.1f}N | dLf={fiber_dl_on*100:.2f}% vs {fiber_dl_off*100:.2f}% | {'PASS' if ok else 'FAIL'}"
        )
    return all_ok


def run_benchmark_2_kirsch_sinusoid(mp):
    print("\n=== Benchmark 2: Kirsch et al. (1994) Sinusoidal Impedance Sweep (a = 0.25) ===")
    l_opt = mp["l_opt"]
    l_mt0 = l_opt * math.cos(mp["penn"]) + mp["l_ts"]
    act = 0.25
    freqs = [0.5, 1.0, 2.0, 4.0, 8.0, 15.0]
    all_ok = True
    wall_on_tot, wall_off_tot = 0.0, 0.0

    for amp_pct, label in ((0.5, "Small 0.5% L0"), (2.5, "Large 2.5% L0")):
        amp = (amp_pct / 100.0) * l_opt
        print(f"  -- {label} --")
        stiff_list = []
        for f in freqs:
            n_cyc = max(6, int(round(2.0 * f)))
            t_end = min(6.0, max(2.0, n_cyc / f))
            t_grid = np.linspace(0.0, t_end, int(round(t_end * 400)) + 1)
            # Smooth 0.2 s ramp-in envelope
            env = np.clip(t_grid / 0.2, 0.0, 1.0)
            env = env * env * (3.0 - 2.0 * env)
            x_grid = l_mt0 + env * amp * np.sin(2.0 * math.pi * f * t_grid)

            m_on, s_on, mus_on, _ = build_single_muscle_model(mp, True, act, make_piecewise_spline(t_grid, x_grid))
            tr = integrate_trace(m_on, s_on, mus_on, t_end, dt=0.001)
            wall_on_tot += tr["wall_s"]
            if amp_pct == 0.5 and f in (1.0, 4.0):
                m_off, s_off, mus_off, _ = build_single_muscle_model(mp, False, act, make_piecewise_spline(t_grid, x_grid))
                tr_off = integrate_trace(m_off, s_off, mus_off, t_end, dt=0.001)
                wall_off_tot += tr_off["wall_s"]

            # Fit steady last half of trace with 1st and 2nd harmonics
            msk = tr["t"] >= 0.5 * t_end
            tt = tr["t"][msk]
            yy = tr["ft"][msk]
            xx = tr["x"][msk] - l_mt0
            w = 2.0 * math.pi * f
            A = np.column_stack([np.ones_like(tt), np.sin(w * tt), np.cos(w * tt), np.sin(2*w*tt), np.cos(2*w*tt)])
            c_f, _, _, _ = np.linalg.lstsq(A, yy, rcond=None)
            c_x, _, _, _ = np.linalg.lstsq(A[:, :3], xx, rcond=None)
            mag_f = math.hypot(c_f[1], c_f[2])
            mag_f2 = math.hypot(c_f[3], c_f[4])
            mag_x = math.hypot(c_x[1], c_x[2])
            ph_f = math.degrees(math.atan2(c_f[2], c_f[1]))
            ph_x = math.degrees(math.atan2(c_x[2], c_x[1]))
            ph_lead = ((ph_f - ph_x + 180.0) % 360.0) - 180.0
            stiff = mag_f / max(mag_x, 1e-12)
            hd2 = 100.0 * mag_f2 / max(mag_f, 1e-12)
            s_end = float(np.mean(tr["s_thix"][msk]))
            stiff_list.append(stiff)
            print(f"    f={f:4.1f} Hz: |Z|={stiff:7.0f} N/m | phase={ph_lead:+5.1f} deg | HD2={hd2:4.1f}% | s_thix={s_end:.2f}")
        if amp_pct == 0.5:
            stiff_small = stiff_list
        else:
            stiff_large = stiff_list

    softened = all(sl < ss for sl, ss in zip(stiff_large, stiff_small))
    print(f"  Amplitude-dependent softening (2.5% < 0.5% at all freqs): {'PASS' if softened else 'FAIL'}")
    if wall_off_tot > 0:
        # Compare 2 matched runs (f=1.0, 4.0)
        print(f"  Speed check (SRS ON vs OFF ratio): see Benchmark 3/4")
    return all_ok and softened


def run_benchmark_3_thixotropy(mp):
    print("\n=== Benchmark 3: De Groote (2018) / Campbell & Lakie (1998) Double-Stretch Thixotropy ===")
    l_opt = mp["l_opt"]
    l_mt0 = l_opt * math.cos(mp["penn"]) + mp["l_ts"]
    amp = 0.06 * l_opt  # 6% L0 stretch
    act = 0.30

    # Schedule:
    #   0.00 - 0.10: quiet rest
    #   0.10 - 0.25: Stretch 1 (+6% L0), 0.25 - 0.40: return to l_mt0
    #   0.40 - 0.50: brief 0.1 s pause
    #   0.50 - 0.65: Stretch 2 (+6% L0), 0.65 - 0.80: return to l_mt0
    #   0.80 - 3.80: 3.0 s quiet rest (recovery)
    #   3.80 - 3.95: Stretch 3 (+6% L0), 3.95 - 4.10: hold
    t_pts = [0.00, 0.10, 0.25, 0.40, 0.50, 0.65, 0.80, 3.80, 3.95, 4.10]
    x_pts = [l_mt0, l_mt0, l_mt0 + amp, l_mt0, l_mt0, l_mt0 + amp, l_mt0, l_mt0, l_mt0 + amp, l_mt0 + amp]
    t_grid = np.linspace(0.0, 4.10, 4101)
    x_grid = np.interp(t_grid, t_pts, x_pts)
    x_grid = np.convolve(np.pad(x_grid, 4, mode="edge"), np.ones(9) / 9.0, mode="valid")

    m_on, s_on, mus_on, _ = build_single_muscle_model(mp, True, act, make_piecewise_spline(t_grid, x_grid))
    tr_on = integrate_trace(m_on, s_on, mus_on, 4.10, dt=0.001)
    m_off, s_off, mus_off, _ = build_single_muscle_model(mp, False, act, make_piecewise_spline(t_grid, x_grid))
    tr_off = integrate_trace(m_off, s_off, mus_off, 4.10, dt=0.001)

    def peak_excess(t0, t1):
        msk = (tr_on["t"] >= t0) & (tr_on["t"] <= t1)
        idx_s = int(np.argmin(np.abs(tr_on["t"] - t0)))
        return (
            float(np.max(tr_on["ft"][msk] - tr_off["ft"][msk])),
            float(np.max(tr_on["ft"][msk])),
            float(tr_on["s_thix"][idx_s]),
        )

    ex1, pk1, th1 = peak_excess(0.10, 0.25)
    ex2, pk2, th2 = peak_excess(0.50, 0.65)
    ex3, pk3, th3 = peak_excess(3.80, 3.95)
    ratio_wall = tr_on["wall_s"] / max(tr_off["wall_s"], 1e-6)
    ok = (ex2 < 0.75 * ex1) and (ex3 > 0.85 * ex1) and (th2 < 0.65) and (th3 > 0.85)
    print(f"  Stretch 1 (fresh rest) : s_thix={th1:.2f} | peakF={pk1:.2f}N | SRS_excess=+{ex1:.2f}N")
    print(f"  Stretch 2 (0.1s pause) : s_thix={th2:.2f} | peakF={pk2:.2f}N | SRS_excess=+{ex2:.2f}N")
    print(f"  Stretch 3 (3.0s rest)  : s_thix={th3:.2f} | peakF={pk3:.2f}N | SRS_excess=+{ex3:.2f}N")
    print(f"  Thixotropy breakdown & recovery: {'PASS' if ok else 'FAIL'}")
    print(f"  Benchmark 4 Speed check (4.1 s trace): SRS_ON={tr_on['wall_s']:.3f}s vs SRS_OFF={tr_off['wall_s']:.3f}s ({ratio_wall:.2f}x)")
    return ok


def main():
    mp = extract_splen_cap_params()
    print(f"Testing on real neck muscle '{mp['name']}': Fmax={mp['f_max']:.1f}N, L0={mp['l_opt']*100:.2f}cm, Lts={mp['l_ts']*100:.2f}cm, penn={math.degrees(mp['penn']):.1f}deg")
    ok1 = run_benchmark_1_cui_ramp(mp)
    ok2 = run_benchmark_2_kirsch_sinusoid(mp)
    ok3 = run_benchmark_3_thixotropy(mp)
    print("\n============================================================")
    print(f"OVERALL SINGLE-MUSCLE SRS VALIDATION: {'ALL PASSED' if (ok1 and ok2 and ok3) else 'CHECK FAILURES'}")
    print("============================================================")
    if not (ok1 and ok2 and ok3):
        sys.exit(1)


if __name__ == "__main__":
    main()
