"""
Local Exact-15.8s Proof of Combined Actions 1 (Hill-4 C^infty divisive otolith reweighting)
+ Action 2 (Pre-lead hair-bundle tanh) + Action 4 (MVST/LVST vcr_upper_share_pitch ~ 0.38-0.42).
Runs 6 cases across 6 workers locally (~13 min) checking APEO + 2g/3g sled + VOL60 simultaneously.
"""
import json, math, os, sys, time
from concurrent.futures import ProcessPoolExecutor, as_completed

for _k in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS", "SIMBODY_NUM_THREADS"):
    os.environ.setdefault(_k, "1")

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, "util"), os.path.join(_ROOT, "scripts")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import scripts.truba_phase_a3_exact_roi_cmaes as ex
from scripts.test_actions_1234_local import run_apeo_detailed

OUT_JSON = r"C:\Users\User\.gemini\antigravity\brain\89fb8d56-9d12-4051-a256-02497ef8d6ae\scratch\actions_124_hill4_local.json"


def _worker(c):
    ap = run_apeo_detailed(c["ov"], enable_srs=True, bush_damp_scale=1.0)
    return {
        "name": c["name"],
        "apeo": ap,
        "s2_u0": ex.run_sled_exact(c["ov"], 2.0, 0.0),
        "s2_nb": ex.run_sled_exact(c["ov"], 2.0, -0.75),
        "s2_br": ex.run_sled_exact(c["ov"], 2.0, 0.5),
        "s3_u0": ex.run_sled_exact(c["ov"], 3.0, 0.0),
        "vol60": ex.run_vol60_exact(c["ov"]),
    }


def main():
    t0 = time.time()
    _, base_p = ex.load_base_params()
    champ = ex.merge_shared_params(base_p, exclude_axis="")
    champ.update({"phasic_torque_tanh_nm": 0.0, "oto_reweight_tau_s": 0.035})

    def m(**kw):
        d = dict(champ); d.update(kw); return d

    cases = [
        {"name": "H1_uSh40_Kph9p5_a0_6p0_t35",  "ov": m(vcr_upper_share_pitch=0.40, K_phas=9.5,  z_phas=0.028, oto_reweight_a0_mps2=6.0, oto_reweight_tau_s=0.035, phasic_oto_tanh_mps2=12.0)},
        {"name": "H2_uSh40_Kph9p5_a0_5p0_t30",  "ov": m(vcr_upper_share_pitch=0.40, K_phas=9.5,  z_phas=0.028, oto_reweight_a0_mps2=5.0, oto_reweight_tau_s=0.030, phasic_oto_tanh_mps2=12.0)},
        {"name": "H3_uSh38_Kph8p5_a0_5p5_t30",  "ov": m(vcr_upper_share_pitch=0.38, K_phas=8.5,  z_phas=0.028, oto_reweight_a0_mps2=5.5, oto_reweight_tau_s=0.030, phasic_oto_tanh_mps2=12.0)},
        {"name": "H4_uSh38_Kph8p5_a0_4p5_t25",  "ov": m(vcr_upper_share_pitch=0.38, K_phas=8.5,  z_phas=0.028, oto_reweight_a0_mps2=4.5, oto_reweight_tau_s=0.025, phasic_oto_tanh_mps2=10.0)},
        {"name": "H5_uSh42_Kph10p5_a0_5p0_t25", "ov": m(vcr_upper_share_pitch=0.42, K_phas=10.5, z_phas=0.028, oto_reweight_a0_mps2=5.0, oto_reweight_tau_s=0.025, phasic_oto_tanh_mps2=12.0)},
        {"name": "H6_uSh38_Kph8p5_a0_5p0_Klig08","ov": m(vcr_upper_share_pitch=0.38, K_phas=8.5, z_phas=0.028, oto_reweight_a0_mps2=5.0, oto_reweight_tau_s=0.025, phasic_oto_tanh_mps2=10.0, K_lig_protective=0.08, srs_k_srs=8.5)},
    ]

    print(f"Running {len(cases)} exact-15.8s Hill-4 + PreLeadHair + LVST-Share local tests across 6 workers...", flush=True)
    res_by_idx = {}
    with ProcessPoolExecutor(max_workers=6) as pool:
        futs = {pool.submit(_worker, c): i for i, c in enumerate(cases)}
        for fut in as_completed(futs):
            idx = futs[fut]
            r = fut.result()
            res_by_idx[idx] = r
            a = r["apeo"]
            print(f"  [DONE {len(res_by_idx)}/{len(cases)}] {r['name']:30s} | APEO={a['total']:7.1f} (G={a.get('gain_loss',0):5.0f} P={a.get('phase_loss',0):4.0f} rv={a.get('rv',0):.4f}) | 2g={r['s2_u0']['pk_flex']:+5.2f}/{r['s2_nb']['pk_flex']:+5.1f} 3g={r['s3_u0']['pk_flex']:+5.2f} nPk={r['s2_u0']['n_pk']}/{r['s2_br']['n_pk']}/{r['s3_u0']['n_pk']} VOL60={r['vol60']['end_pitch_deg']:.1f}", flush=True)
            ordered = [res_by_idx[k] for k in sorted(res_by_idx)]
            with open(OUT_JSON, "w", encoding="utf-8") as f:
                json.dump({"wall_s": round(time.time()-t0, 1), "results": ordered}, f, indent=2)

    results = [res_by_idx[k] for k in sorted(res_by_idx)]
    print(f"\nCompleted in {time.time()-t0:.1f}s -> {OUT_JSON}")
    for r in results:
        a = r["apeo"]
        if not a.get("aborted") and "bins" in a:
            bs = " | ".join(f"{fk}Hz:{v['gain']:.0f}/{v['tgt']:.0f}({v['db']:+.1f}dB,{v['ph']:+.0f}°)" for fk, v in a["bins"].items())
            print(f"  {r['name']:30s}: {bs}")


if __name__ == "__main__":
    main()
