"""
Local Exact-15.8s Verification of Actions 1, 2, 3, and 4:
  - Action 3: Passive plant APEO baseline (u=-1.0, B_k50_c30 vs 0.35x bushing damping)
  - Action 2: Pre-lead utricular hair-bundle tanh saturation (before Stage-2 3.89x lead)
  - Action 1: Single C^infty divisive somatogravic otolith reweighting w_oto = 1/(1+(a_env/a0)^2)
  - Action 4: MVST/LVST upper- vs lower-cervical VCR projection ratio (vcr_upper_share_pitch)
Runs 8 exact-15.8s cases across 8 workers locally (~10-12 min) on pulled TRUBA champions.
"""
import json, math, os, sys, time
from concurrent.futures import ProcessPoolExecutor, as_completed

for _k in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS", "SIMBODY_NUM_THREADS"):
    os.environ.setdefault(_k, "1")

import numpy as np
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, "util"), os.path.join(_ROOT, "scripts")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import opensim as osim
from cmaes_worker_p22 import CANONICAL_TARGETS, PER_MUSCLE_72_U0, ensure_libraries_loaded
import scripts.truba_phase_a3_exact_roi_cmaes as ex
from scripts.run_roll_yaw_baseline import build_c2_multisine_spline
from scripts.run_zheng_2g_3g_urgency_sweep import DT_LOG, SPLEN_CAP_NAMES, T_SETTLE, T_TOTAL, build_sled_spline

OUT_JSON = r"C:\Users\User\.gemini\antigravity\brain\89fb8d56-9d12-4051-a256-02497ef8d6ae\scratch\actions_1234_local_test.json"


def _setup_model(ov: dict, enable_srs: bool = True, bush_damp_scale: float = 1.0):
    ensure_libraries_loaded()
    model = osim.Model(ex.KUO_MODEL_PATH)
    if abs(bush_damp_scale - 1.0) > 1e-6:
        fset = model.updForceSet()
        for i in range(fset.getSize()):
            bf = osim.BushingForce.safeDownCast(fset.get(i))
            if bf is not None:
                v = bf.get_rotational_damping()
                bf.set_rotational_damping(
                    osim.Vec3(v.get(0)*bush_damp_scale, v.get(1)*bush_damp_scale, v.get(2)*bush_damp_scale)
                )
    controller, _ = ex._apply_srs_and_ctrl(model, ov)
    if not enable_srs:
        mus = model.updMuscles()
        for i in range(mus.getSize()):
            m = mus.get(i)
            if not m.getName().startswith(("r_", "l_")):
                osim.PropertyHelper.setValueBool(False, m.updPropertyByName("enable_SRS"))

    for eye_c in ["r_eye_add_abd", "r_eye_sup_inf", "r_eye_inc_exc", "l_eye_add_abd", "l_eye_sup_inf", "l_eye_inc_exc"]:
        if model.getCoordinateSet().contains(eye_c):
            c = model.updCoordinateSet().get(eye_c)
            c.setDefaultValue(0.0)
            c.setDefaultLocked(True)
    zf = osim.Constant(0.0)
    for cn in ["gndpitch", "gndroll", "gndyaw", "spine_tx", "spine_ty", "spine_tz"]:
        c = model.updCoordinateSet().get(cn)
        c.set_locked(False)
        c.setDefaultIsPrescribed(True)
        c.setPrescribedFunction(zf)
    cy1 = model.updCoordinateSet().get("yaw1")
    cy1.set_locked(False)
    cy1.setDefaultIsPrescribed(False)
    return model, controller, cy1


def run_apeo_detailed(ov: dict, enable_srs: bool = True, bush_damp_scale: float = 1.0):
    t0 = time.time()
    model, _, cy1 = _setup_model(ov, enable_srs=enable_srs, bush_damp_scale=bush_damp_scale)
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
    ex._init_neck_state(model, state, ov)

    mgr = osim.Manager(model)
    mgr.setWriteToStorage(False); mgr.setPerformAnalyses(False)
    mgr.setIntegratorMethod(5); mgr.setIntegratorAccuracy(1e-4)
    mgr.setIntegratorMinimumStepSize(1e-4); mgr.setIntegratorMaximumStepSize(0.001)
    state.setTime(0.0); mgr.initialize(state)

    cs = model.getCoordinateSet()
    cp1, cp2, cr1, cr2, cy_1, cy_2 = cs.get("pitch1"), cs.get("pitch2"), cs.get("roll1"), cs.get("roll2"), cs.get("yaw1"), cs.get("yaw2")
    ts, ps, rs, ys = [], [], [], []
    t_now = 0.0; aborted = False; abort_reason = ""
    while t_now < target_time - 1e-9:
        t_now = min(t_now + 0.005, target_time)
        try:
            state = mgr.integrate(t_now)
        except Exception as e:
            aborted = True; abort_reason = str(e).splitlines()[0]; break
        pd = math.degrees(cp1.getValue(state) + cp2.getValue(state))
        rd = math.degrees(cr1.getValue(state) + cr2.getValue(state))
        yd = math.degrees(cy_1.getValue(state) + cy_2.getValue(state))
        ts.append(t_now); ps.append(pd); rs.append(rd); ys.append(yd)
        if abs(pd) > 54.0 or abs(rd) > 30.0 or abs(yd) > 49.0:
            aborted = True; abort_reason = f"ROM@{t_now:.2f}s p={pd:+.1f}"; break

    t_arr, p_arr, r_arr, y_arr = np.asarray(ts), np.asarray(ps), np.asarray(rs), np.asarray(ys)
    m_post = t_arr >= eval_start
    if aborted or int(np.sum(m_post)) < 50:
        return {"aborted": True, "abort_reason": abort_reason, "total": 1200000.0, "wall_s": round(time.time()-t0, 1)}

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
        bins[f"{fk:.3f}"] = {"gain": round(sg, 1), "tgt": round(float(tgt_g[k]), 1), "db": round(db_err, 2), "ph": round(ph_err, 1)}
    va = np.gradient(y_fit, t_fit); vh = np.gradient(harm_fit, t_fit); vr = np.gradient(ref_fit, t_fit)
    hf_rms = float(np.sqrt(np.mean((va - vh)**2)))
    rv = float(np.corrcoef(va, vr)[0, 1]) if np.std(va) > 1e-6 else 0.0
    xt = 100.0 * (float(np.max(np.abs(r_arr[m_post])))**2 + float(np.max(np.abs(y_arr[m_post])))**2)
    cls = g_loss + p_loss
    return {"aborted": False, "total": round(cls + xt + 5.0 * hf_rms**2, 1), "gain_loss": round(g_loss, 1),
            "phase_loss": round(p_loss, 1), "hf_rms": round(hf_rms, 2), "rv": round(rv, 4),
            "bins": bins, "wall_s": round(time.time()-t0, 1)}


def _worker(c):
    ap = run_apeo_detailed(c["ov"], enable_srs=c.get("srs", True), bush_damp_scale=c.get("bush_damp", 1.0))
    out = {"name": c["name"], "apeo": ap}
    if not c.get("passive_only", False):
        out["s2_u0"] = ex.run_sled_exact(c["ov"], 2.0, 0.0)
        out["s2_nb"] = ex.run_sled_exact(c["ov"], 2.0, -0.75)
        out["s2_br"] = ex.run_sled_exact(c["ov"], 2.0, 0.5)
        out["s3_u0"] = ex.run_sled_exact(c["ov"], 3.0, 0.0)
        out["vol60"] = ex.run_vol60_exact(c["ov"])
    return out


def main():
    t0 = time.time()
    _, base_p = ex.load_base_params()
    champ = ex.merge_shared_params(base_p, exclude_axis="")
    # Clean defaults for new properties
    champ.update({"oto_reweight_a0_mps2": 0.0, "oto_reweight_tau_s": 0.050, "vcr_upper_share_pitch": 0.50})

    def m(**kw):
        d = dict(champ); d.update(kw); return d

    cases = [
        {"name": "A3_Passive_NoSRS_Bk50c30",   "ov": m(urgency=-1.0), "srs": False, "bush_damp": 1.0,  "passive_only": True},
        {"name": "A3_Passive_NoSRS_Damp0p35",  "ov": m(urgency=-1.0), "srs": False, "bush_damp": 0.35, "passive_only": True},
        {"name": "B0_TRUBA_G06_PreLeadHair",   "ov": m(),                                                                                                     "srs": True},
        {"name": "A12_RW_a0_6p0_hair12",       "ov": m(oto_reweight_a0_mps2=6.0, oto_reweight_tau_s=0.050, phasic_oto_tanh_mps2=12.0, phasic_torque_tanh_nm=0.0), "srs": True},
        {"name": "A12_RW_a0_5p0_hair10",       "ov": m(oto_reweight_a0_mps2=5.0, oto_reweight_tau_s=0.050, phasic_oto_tanh_mps2=10.0, phasic_torque_tanh_nm=0.0), "srs": True},
        {"name": "A12_RW_a0_4p2_hair10_t60",   "ov": m(oto_reweight_a0_mps2=4.2, oto_reweight_tau_s=0.060, phasic_oto_tanh_mps2=10.0, phasic_torque_tanh_nm=0.0), "srs": True},
        {"name": "A124_RW5p0_h10_uShare35",    "ov": m(oto_reweight_a0_mps2=5.0, oto_reweight_tau_s=0.050, phasic_oto_tanh_mps2=10.0, phasic_torque_tanh_nm=0.0, vcr_upper_share_pitch=0.35), "srs": True},
        {"name": "A124_RW5p0_h10_uShare65",    "ov": m(oto_reweight_a0_mps2=5.0, oto_reweight_tau_s=0.050, phasic_oto_tanh_mps2=10.0, phasic_torque_tanh_nm=0.0, vcr_upper_share_pitch=0.65), "srs": True},
    ]
    print(f"Running {len(cases)} exact-15.8s Actions 1-4 local tests across 8 workers...", flush=True)
    res_by_idx = {}
    with ProcessPoolExecutor(max_workers=8) as pool:
        futs = {pool.submit(_worker, c): i for i, c in enumerate(cases)}
        for fut in as_completed(futs):
            idx = futs[fut]
            try:
                r = fut.result()
            except Exception as e:
                r = {"name": cases[idx]["name"], "error": repr(e), "apeo": {"aborted": True, "total": 1200000.0}}
            res_by_idx[idx] = r
            a = r["apeo"]
            extra = f"2g={r['s2_u0']['pk_flex']:+.2f} 3g={r['s3_u0']['pk_flex']:+.2f} nPk={r['s2_u0']['n_pk']}/{r['s3_u0']['n_pk']}" if "s2_u0" in r else "(passive)"
            print(f"  [DONE {len(res_by_idx)}/{len(cases)}] {r['name']:28s} | APEO={a['total']:7.1f} rv={a.get('rv',0):.4f} | {extra}", flush=True)
            ordered_now = [res_by_idx[k] for k in sorted(res_by_idx)]
            with open(OUT_JSON, "w", encoding="utf-8") as f:
                json.dump({"wall_s": round(time.time()-t0, 1), "results": ordered_now}, f, indent=2)
    results = [res_by_idx[k] for k in sorted(res_by_idx)]

    print(f"\nCompleted in {time.time()-t0:.1f}s -> {OUT_JSON}")
    print(f"{'Case':28s} | {'APEO':>7s} | {'GainL':>6s} | {'PhasL':>6s} | {'rv':>6s} | {'2g_u0':>6s} | {'2g_br':>6s} | {'3g_u0':>6s} | {'nPk2/b/3'} | {'VOL60'}")
    print("-" * 116)
    for r in results:
        a = r["apeo"]
        if "s2_u0" in r:
            npk = f"{r['s2_u0']['n_pk']}/{r['s2_br']['n_pk']}/{r['s3_u0']['n_pk']}"
            v60 = f"{r['vol60']['end_pitch_deg']:.1f}({r['vol60']['ovs_deg']:.1f})"
            print(f"{r['name']:28s} | {a['total']:7.1f} | {a.get('gain_loss',0):6.1f} | {a.get('phase_loss',0):6.1f} | {a.get('rv',0):6.4f} | {r['s2_u0']['pk_flex']:+6.2f} | {r['s2_br']['pk_flex']:+6.2f} | {r['s3_u0']['pk_flex']:+6.2f} | {npk:8s} | {v60}")
        else:
            print(f"{r['name']:28s} | {a['total']:7.1f} | {a.get('gain_loss',0):6.1f} | {a.get('phase_loss',0):6.1f} | {a.get('rv',0):6.4f} |   (passive plant audit)")
        if not a.get("aborted") and "bins" in a:
            bs = " | ".join(f"{fk}Hz:{v['gain']:.0f}/{v['tgt']:.0f}({v['db']:+.1f}dB,{v['ph']:+.0f}°)" for fk, v in a["bins"].items())
            print(f"   bins: {bs}")

if __name__ == "__main__":
    main()
