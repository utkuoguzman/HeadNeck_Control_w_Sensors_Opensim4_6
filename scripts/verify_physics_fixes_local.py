"""
Local Exact-15.8s Verification of the 3 Physics Fixes:
  1. Mixed-fiber human neck SRS (k_srs=10-12, tau_srs_reset=0.10-0.12s, s_thix_min=0.18-0.20, d_break=0.035)
  2. Post-integrator LVN/MVN smooth tanh ceiling (phasic_torque_tanh_nm = 8.5-11.0 Nm; ALL highg cliffs OFF)
  3. Free biological MVN/LVN lead zero (z_phas = 0.020-0.028 s)
Runs 4 exact 15.8s APEO + 2g/3g sled cases locally across 4 workers (~9 min).
"""
import json, math, os, sys, time
from concurrent.futures import ProcessPoolExecutor

for _k in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS", "SIMBODY_NUM_THREADS"):
    os.environ.setdefault(_k, "1")

import numpy as np
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, "util"), os.path.join(_ROOT, "scripts")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import opensim as osim
from cmaes_worker_p22 import CANONICAL_TARGETS, PER_MUSCLE_72_U0, ensure_libraries_loaded
import scripts.generate_standard_plot as gsp
from scripts.run_roll_yaw_baseline import build_c2_multisine_spline
from scripts.run_zheng_2g_3g_urgency_sweep import DT_LOG, SPLEN_CAP_NAMES, T_SETTLE, T_TOTAL, build_sled_spline

KUO_OSIM = os.path.join(_ROOT, "osim_files", "HYOID_HeadNeckNeural_Model_KuoLigaments.osim")
C1_JSON = os.path.join(_ROOT, "logs", "se2_c1pos_3axis_champion.json")
OUT_JSON = r"C:\Users\User\.gemini\antigravity\brain\89fb8d56-9d12-4051-a256-02497ef8d6ae\scratch\physics_fixes_local_check.json"


def _setup_model(ov: dict, srs_cfg: dict):
    ensure_libraries_loaded()
    model = osim.Model(KUO_OSIM)
    muscles = model.updMuscles()
    for i in range(muscles.getSize()):
        m = muscles.get(i)
        if m.getName().startswith(("r_", "l_")):
            continue
        osim.PropertyHelper.setValueBool(True, m.updPropertyByName("enable_SRS"))
        for prop, val in srs_cfg.items():
            if m.hasProperty(prop):
                osim.PropertyHelper.setValueDouble(float(val), m.updPropertyByName(prop))
        gto_sub = m.updComponent("GTO")
        osim.PropertyHelper.setValueBool(True, gto_sub.updPropertyByName("enable_GTO"))

    ctrl = None
    for i in range(model.getControllerSet().getSize()):
        c = model.updControllerSet().get(i)
        if c.getConcreteClassName() == "FixationController":
            c.set_enabled(False)
        elif c.getConcreteClassName() == "HeadNeckNeuralController":
            ctrl = c
    pairs = []
    for k, v in ov.items():
        if not ctrl.hasProperty(k):
            continue
        if isinstance(v, bool):
            pairs.append((k, "true" if v else "false"))
        elif isinstance(v, str):
            pairs.append((k, v))
        else:
            pairs.append((k, repr(float(v))))
    gsp.apply_cli_overrides_and_inspect(ctrl, pairs)

    for eye_c in ["r_eye_add_abd", "r_eye_sup_inf", "r_eye_inc_exc", "l_eye_add_abd", "l_eye_sup_inf", "l_eye_inc_exc"]:
        if model.getCoordinateSet().contains(eye_c):
            c = model.updCoordinateSet().get(eye_c)
            c.setDefaultValue(0.0)
            c.setDefaultLocked(True)
    zero_func = osim.Constant(0.0)
    for cn in ["gndpitch", "gndroll", "gndyaw", "spine_tx", "spine_ty", "spine_tz"]:
        c = model.updCoordinateSet().get(cn)
        c.set_locked(False)
        c.setDefaultIsPrescribed(True)
        c.setPrescribedFunction(zero_func)
    cy1 = model.updCoordinateSet().get("yaw1")
    cy1.set_locked(False)
    cy1.setDefaultIsPrescribed(False)
    return model, ctrl, cy1


def _init_state(model, state, ov: dict):
    model.realizePosition(state)
    skull = model.getBodySet().get("skull")
    q = osim.Rotation(skull.getTransformInGround(state).R()).convertRotationToQuaternion()
    for idx in range(4):
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/obs_q{idx}", q.get(idx))
    for sv in ("pitch_error_integral", "roll_error_integral", "yaw_error_integral",
               "vol_filt_pitch", "vol_filt_roll", "vol_filt_yaw"):
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/{sv}", 0.0)
    k_gd = float(ov.get("K_gamma_dyn", 0.857744))
    k_gs = float(ov.get("K_gamma_stat", 0.69312344))
    muscles = model.updMuscles()
    for i in range(muscles.getSize()):
        m = muscles.get(i)
        nm = m.getName()
        if nm.startswith(("r_", "l_")):
            m.setAppliesForce(state, False)
            continue
        a0 = PER_MUSCLE_72_U0.get(nm, 0.030)
        m.setActivation(state, a0)
        gd0 = 100.0 * k_gd * (0.25 + 0.75 * a0)
        gs0 = 100.0 * k_gs * a0
        fd0 = (gd0 * gd0) / (gd0 * gd0 + 3600.0)
        fs0 = (gs0 * gs0) / (gs0 * gs0 + 3600.0)
        m.setStateVariableValue(state, "spindle/dynamic_activation", fd0)
        m.setStateVariableValue(state, "spindle/static_activation", fs0)
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/sp_fdyn_{nm}", fd0)
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/sp_fstat_{nm}", fs0)
    model.equilibrateMuscles(state)


def run_apeo_exact(ov: dict, srs_cfg: dict):
    t0 = time.time()
    model, _, cy1 = _setup_model(ov, srs_cfg)
    delay, ramp, target_time, eval_start = 0.8, 1.5, 15.8, 5.0
    ordered = sorted(CANONICAL_TARGETS, key=lambda x: x[0])
    freqs = np.array([t[0] for t in ordered], dtype=float)
    tgt_g = np.array([t[1] for t in ordered], dtype=float)
    tgt_p = np.array([t[2] for t in ordered], dtype=float)
    K = len(freqs)
    phases = np.array([math.pi * k * (k - 1) / K for k in range(K)], dtype=float)
    amps = np.array([0.0215 * (0.835 / f) / math.sqrt(K) for f in freqs], dtype=float)
    sp, _ = build_c2_multisine_spline(freqs, amps, phases, delay, ramp, target_time)
    ctx = model.updCoordinateSet().get("spine_tx")
    ctx.setPrescribedFunction(sp)

    state = model.initSystem()
    cy1.setIsPrescribed(state, False)
    ctx.setSpeedValue(state, 0.0)
    _init_state(model, state, ov)

    mgr = osim.Manager(model)
    mgr.setWriteToStorage(False); mgr.setPerformAnalyses(False)
    mgr.setIntegratorMethod(5); mgr.setIntegratorAccuracy(1e-4)
    mgr.setIntegratorMinimumStepSize(1e-4); mgr.setIntegratorMaximumStepSize(0.001)
    state.setTime(0.0); mgr.initialize(state)

    cs = model.getCoordinateSet()
    cp1, cp2, cr1, cr2, cy_1, cy_2 = cs.get("pitch1"), cs.get("pitch2"), cs.get("roll1"), cs.get("roll2"), cs.get("yaw1"), cs.get("yaw2")
    ts, ps, rs, ys = [], [], [], []
    t_now = 0.0; aborted = False
    while t_now < target_time - 1e-9:
        t_now = min(t_now + 0.005, target_time)
        try:
            state = mgr.integrate(t_now)
        except Exception:
            aborted = True; break
        pd = math.degrees(cp1.getValue(state) + cp2.getValue(state))
        rd = math.degrees(cr1.getValue(state) + cr2.getValue(state))
        yd = math.degrees(cy_1.getValue(state) + cy_2.getValue(state))
        ts.append(t_now); ps.append(pd); rs.append(rd); ys.append(yd)
        if abs(pd) > 54.0 or abs(rd) > 30.0 or abs(yd) > 49.0:
            aborted = True; break

    t_arr, p_arr, r_arr, y_arr = np.asarray(ts), np.asarray(ps), np.asarray(rs), np.asarray(ys)
    m_post = t_arr >= eval_start
    if aborted or int(np.sum(m_post)) < 50:
        return {"aborted": True, "total": 1200000.0, "wall_s": round(time.time()-t0, 1)}

    t_fit = t_arr[m_post]; t_rel = t_fit - delay; y_fit = p_arr[m_post]
    cols = [np.ones_like(t_fit), t_fit - eval_start]
    for k in range(K):
        psi = 2.0 * math.pi * float(freqs[k]) * t_rel + float(phases[k])
        cols.append(np.sin(psi)); cols.append(np.cos(psi))
    coeffs, _, _, _ = np.linalg.lstsq(np.column_stack(cols), y_fit, rcond=None)
    ref_fit = np.zeros_like(t_fit); harm_fit = np.zeros_like(t_fit)
    g_loss = 0.0; p_loss = 0.0; bins = {}
    for k in range(K):
        fk = float(freqs[k]); psi = 2.0 * np.pi * fk * t_rel + float(phases[k])
        ref_fit += float(amps[k]) * float(tgt_g[k]) * np.sin(psi + math.radians(float(tgt_p[k])))
        ac, bc = float(coeffs[2 + 2*k]), float(coeffs[2 + 2*k + 1])
        harm_fit += ac * np.sin(psi) + bc * np.cos(psi)
        sg = math.hypot(ac, bc) / float(amps[k])
        rph = math.degrees(math.atan2(bc, ac))
        ph_err = ((rph - float(tgt_p[k]) + 180.0) % 360.0) - 180.0
        db_err = 20.0 * math.log10(max(sg, 1e-6) / float(tgt_g[k]))
        g_loss += 100.0 * db_err * db_err; p_loss += ph_err * ph_err
        bins[f"{fk:.3f}"] = {"db": round(db_err, 2), "ph": round(ph_err, 1)}
    va = np.gradient(y_fit, t_fit); vh = np.gradient(harm_fit, t_fit); vr = np.gradient(ref_fit, t_fit)
    hf_rms = float(np.sqrt(np.mean((va - vh)**2)))
    rv = float(np.corrcoef(va, vr)[0, 1]) if np.std(va) > 1e-6 else 0.0
    xt = 100.0 * (float(np.max(np.abs(r_arr[m_post])))**2 + float(np.max(np.abs(y_arr[m_post])))**2)
    cls = g_loss + p_loss
    return {"aborted": False, "total": round(cls + xt + 5.0 * hf_rms**2, 1), "gain_loss": round(g_loss, 1),
            "phase_loss": round(p_loss, 1), "hf_rms": round(hf_rms, 2), "rv": round(rv, 4),
            "bins": bins, "wall_s": round(time.time()-t0, 1)}


def run_sled(ov: dict, srs_cfg: dict, g_level: float, urgency: float):
    o = dict(ov)
    o.update({"w_pos_ap": 0.0, "w_pos_lat": 0.0, "Kp_shift_obs_px": 0.0, "Kv_shift_obs_vx": 0.0,
              "Kp_shift_obs_pz": 0.0, "Kv_shift_obs_vz": 0.0, "shift_station_blend": 0.0, "urgency": urgency})
    model, _, cy1 = _setup_model(o, srs_cfg)
    ctx = model.updCoordinateSet().get("spine_tx")
    ctx.setPrescribedFunction(build_sled_spline(g_level, sign_x=-1.0))
    state = model.initSystem()
    cy1.setIsPrescribed(state, False); ctx.setSpeedValue(state, 0.0)
    _init_state(model, state, o)

    mgr = osim.Manager(model)
    mgr.setWriteToStorage(False); mgr.setPerformAnalyses(False)
    mgr.setIntegratorMethod(5); mgr.setIntegratorAccuracy(1e-4)
    mgr.setIntegratorMinimumStepSize(1e-4); mgr.setIntegratorMaximumStepSize(0.001)
    state.setTime(0.0); mgr.initialize(state)
    muscles = model.updMuscles()
    sp_idx = [i for i in range(muscles.getSize()) if muscles.get(i).getName() in SPLEN_CAP_NAMES]
    cp1, cp2 = model.getCoordinateSet().get("pitch1"), model.getCoordinateSet().get("pitch2")
    ts, fl, sp = [], [], []
    t_now = 0.0
    while t_now < T_TOTAL - 1e-9:
        t_now = round(min(t_now + DT_LOG, T_TOTAL), 6)
        state = mgr.integrate(t_now)
        model.realizeDynamics(state)
        ts.append(t_now)
        fl.append(-math.degrees(cp1.getValue(state) + cp2.getValue(state)))
        sp.append(sum(float(muscles.get(i).getActivation(state)) for i in sp_idx) / len(sp_idx))
    ts, fl, sp = np.asarray(ts), np.asarray(fl), np.asarray(sp)
    imp = ts >= (T_SETTLE - 1e-9); idx0 = int(np.where(imp)[0][0])
    f_imp = fl[imp] - float(fl[idx0]); sp_imp = sp[imp]
    n_pk = 0
    for k in range(1, len(sp_imp) - 1):
        if sp_imp[k] > 0.08 and sp_imp[k] > sp_imp[k-1] and sp_imp[k] >= sp_imp[k+1]:
            lo, hi = max(0, k-25), min(len(sp_imp), k+26)
            if float(sp_imp[k] - max(np.min(sp_imp[lo:k+1]), np.min(sp_imp[k:hi]))) >= 0.015:
                n_pk += 1
    return {"pk_flex": round(float(np.max(f_imp)), 2), "n_pk": n_pk, "pk_splen": round(float(np.max(sp_imp)), 3)}


def _worker(c):
    ap = run_apeo_exact(c["ov"], c["srs"])
    s2 = run_sled(c["ov"], c["srs"], 2.0, 0.0)
    s2n = run_sled(c["ov"], c["srs"], 2.0, -0.75)
    s3 = run_sled(c["ov"], c["srs"], 3.0, 0.0)
    return {"name": c["name"], "apeo": ap, "s2_u0": s2, "s2_nb": s2n, "s3_u0": s3}


def main():
    t0 = time.time()
    doc = json.load(open(C1_JSON, "r", encoding="utf-8"))
    base = dict(doc["params"]); base.update(doc.get("se2_c1pos_params", {}))
    # Clean Tier A (zero ad-hoc highg knees/cliffs!) + ALL Tier B ON
    base.update({
        "phasic_oto_sat_mps2": 0.0, "highg_oto_knee_mps2": 0.0, "phasic_torque_sat_nm": 0.0,
        "phasic_lead_sat_nm": 0.0, "highg_p_sc_floor": 60.0, "highg_tau_lead_add_s": 0.0,
        "highg_sc_gain_boost": 0.0, "highg_phas_pole_boost": 0.0, "highg_phas_sust_floor": 0.0,
        "phasic_oto_tanh_mps2": 25.0,
        "enable_gto_reflex": True, "K_gto_ib": 0.0005, "K_gto_vol_suppress": 0.85,
        "enable_renshaw": True, "K_renshaw": 0.16, "tau_renshaw": 0.025,
        "enable_ligament_reflex": True, "K_lig_protective": 0.15, "p_lig_reflex": 40.0,
        "enable_sensor_noise": False, "enable_motor_noise": False,
    })

    def m(ov_add, srs_add, name):
        o = dict(base); o.update(ov_add)
        return {"name": name, "ov": o, "srs": srs_add}

    cases = [
        m({"phasic_torque_tanh_nm": 9.5},  {"k_srs": 10.0, "tau_srs_reset": 0.12, "srs_thix_min": 0.18, "delta_srs_thix_break": 0.035}, "F1_ksrs10_tau012_ttanh9p5"),
        m({"phasic_torque_tanh_nm": 8.0},  {"k_srs": 12.0, "tau_srs_reset": 0.10, "srs_thix_min": 0.18, "delta_srs_thix_break": 0.030}, "F2_ksrs12_tau010_ttanh8p0"),
        m({"phasic_torque_tanh_nm": 11.0}, {"k_srs": 8.0,  "tau_srs_reset": 0.10, "srs_thix_min": 0.15, "delta_srs_thix_break": 0.030}, "F3_ksrs08_tau010_ttanh11p0"),
        m({"phasic_torque_tanh_nm": 7.0, "K_phas": 14.5, "z_phas": 0.028}, {"k_srs": 12.0, "tau_srs_reset": 0.12, "srs_thix_min": 0.20, "delta_srs_thix_break": 0.035}, "F4_ksrs12_tau012_ttanh7p0_Kph14p5"),
    ]
    print(f"Running {len(cases)} exact 15.8s physics-fix verification cases...", flush=True)
    with ProcessPoolExecutor(max_workers=4) as ex:
        res = list(ex.map(_worker, cases))
    with open(OUT_JSON, "w", encoding="utf-8") as f:
        json.dump({"wall_s": round(time.time()-t0, 1), "results": res}, f, indent=2)
    print(f"\nDone in {time.time()-t0:.1f}s -> {OUT_JSON}")
    print(f"{'Case':34s} | {'APEO':>7s} | {'GainL':>6s} | {'PhasL':>6s} | {'rv':>6s} | {'2g_u0':>6s} | {'2g_nb':>6s} | {'3g_u0':>6s} | {'nPk2/3'}")
    print("-" * 105)
    for r in res:
        a = r["apeo"]
        print(f"{r['name']:34s} | {a['total']:7.1f} | {a.get('gain_loss',0):6.1f} | {a.get('phase_loss',0):6.1f} | {a.get('rv',0):6.4f} | {r['s2_u0']['pk_flex']:+6.2f} | {r['s2_nb']['pk_flex']:+6.2f} | {r['s3_u0']['pk_flex']:+6.2f} | {r['s2_u0']['n_pk']}/{r['s3_u0']['n_pk']}")

if __name__ == "__main__":
    main()
