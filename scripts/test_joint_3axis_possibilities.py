import os
import sys
import json
import time
from concurrent.futures import ProcessPoolExecutor, as_completed

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, "util"), os.path.join(_ROOT, "scripts")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from run_roll_yaw_baseline import P15_RUN4
from cmaes_worker_multisine import run_multisine_simulation
from evaluate_and_plot_tuned_3axis import run_full_axis_sim, compute_latec_cost


R4_PARAMS = {
    "Kp_roll": 3.8,
    "Kd_roll": 1.40,
    "Ki_roll": 0.0,
    "K_ton_roll": 4.2,
    "K_sc_roll": 3.2,
    "z_sc_roll": 0.0,
    "K_phas_roll": 16.5,
    "p_phas_roll": 6.0,
    "z_phas_roll": 0.0,
    "deriv_comp_roll": 1.0,
}

Y4_PARAMS = {
    "Kp_yaw": 0.0,
    "Kd_yaw": 0.03,
    "Ki_yaw": 0.0,
    "K_ton_yaw": 5.5,
    "p_ton_yaw": 2.4,
    "K_sc_yaw": 1.2,
    "p_sc_yaw": 3.5,
    "Kp_prop_yaw": 8.5,
    "z_ccr_yaw": 0.06,
    "deriv_comp_yaw": 0.25,
}

Y3_PARAMS = {
    "Kp_yaw": 0.0,
    "Kd_yaw": 0.02,
    "Ki_yaw": 0.0,
    "K_ton_yaw": 3.5,
    "p_ton_yaw": 2.5,
    "K_sc_yaw": 0.6,
    "p_sc_yaw": 4.0,
    "Kp_prop_yaw": 6.0,
    "z_ccr_yaw": 0.06,
    "deriv_comp_yaw": 0.25,
}

# Intermediate softer Yaw candidate Y3b
Y3B_PARAMS = {
    "Kp_yaw": 0.0,
    "Kd_yaw": 0.025,
    "Ki_yaw": 0.0,
    "K_ton_yaw": 4.2,
    "p_ton_yaw": 2.4,
    "K_sc_yaw": 0.9,
    "p_sc_yaw": 3.8,
    "Kp_prop_yaw": 5.0,
    "z_ccr_yaw": 0.07,
    "deriv_comp_yaw": 0.25,
}


def compute_yawma_cost(sim_res):
    fr = sim_res["freq_results"]
    K = len(fr)
    gain_cost = sum((d["gain_db_diff"] ** 2) for d in fr.values()) / K
    phase_cost = sum(((d["phase_err"] / 12.0) ** 2) for d in fr.values()) / K
    noise_cost = 0.25 * (sim_res["hf_noise_rms"] ** 2)
    return gain_cost + phase_cost + noise_cost


def make_param_set(*dicts):
    p = dict(P15_RUN4)
    p["Ki_roll"] = 0.0
    p["Ki_yaw"] = 0.0
    for d in dicts:
        p.update(d)
    return p


def run_apeo_worker(args):
    name, params = args
    os.environ["SAVE_TS"] = "0"
    os.environ["MULTISINE_EXP"] = "1.0"
    os.environ["MULTISINE_MIN_AMP_M"] = "0.0"
    os.environ["NOISE_FLOOR_7927_DEG"] = "0.0"
    os.environ["MULTISINE_CYCLES"] = "6.0"
    score, freq_res, tel = run_multisine_simulation(
        params["Kp_pitch"], params["Ki_pitch"], params["Kd_pitch"],
        params["K_ton"], params["p_ton"], params["K_sc"], params["z_sc"], params["p_sc"],
        params["K_phas"], params["p_phas"], params["kp_prop"], params["z_ccr"], params["p_ccr"],
        params["k_gamma_dyn"], params["k_gamma_stat"],
        worker_name=f"diag_{name}", p_phas2=-1.0, z_phas=params["z_phas"],
        extra_props=params,
    )
    return {
        "axis": "APEO",
        "name": name,
        "score": float(score),
        "aborted": bool(tel.get("aborted", False)),
        "p0324": float(freq_res["0.324"]["phase_err_deg"]),
        "sp0324": float(freq_res["0.324"]["sim_phase"]),
        "g1099_db": float(freq_res["1.099"]["gain_db_diff"]),
        "p1099": float(freq_res["1.099"]["phase_err_deg"]),
        "g7927_db": float(freq_res["7.927"]["gain_db_diff"]),
        "p7927": float(freq_res["7.927"]["phase_err_deg"]),
        "max_roll": float(tel.get("max_abs_roll_deg", 0.0)),
        "max_yaw": float(tel.get("max_abs_yaw_deg", 0.0)),
        "dc_pen": float(tel.get("dc_drift_penalty", 0.0)),
        "hf_pen": float(tel.get("hf_penalty", 0.0)),
    }


def main():
    t0 = time.time()
    mode = sys.argv[1] if len(sys.argv) > 1 else "batch1"

    if mode == "batch1":
        # 1) Ablation + Roll/Yaw Joint Combinations on LatEC, YawMA, and APEO
        configs = {
            "R4_only": make_param_set(R4_PARAMS),
            "Y3_only": make_param_set(Y3_PARAMS),
            "Y4_only": make_param_set(Y4_PARAMS),
            "R4_Y4": make_param_set(R4_PARAMS, Y4_PARAMS),
            "R4_Y3": make_param_set(R4_PARAMS, Y3_PARAMS),
            "R4_Y3B": make_param_set(R4_PARAMS, Y3B_PARAMS),
            "R4_Y4_noKpPropYaw": make_param_set(R4_PARAMS, Y4_PARAMS, {"Kp_prop_yaw": -1000.0}),
            "R4_Y4_kpRoll28_kt32": make_param_set(R4_PARAMS, Y4_PARAMS, {"Kp_roll": 2.8, "K_ton_roll": 3.2}),
            "R4_Y3_kpRoll28_kt32": make_param_set(R4_PARAMS, Y3_PARAMS, {"Kp_roll": 2.8, "K_ton_roll": 3.2}),
            "R4_Y3_kpRoll24_kt28": make_param_set(R4_PARAMS, Y3_PARAMS, {"Kp_roll": 2.4, "K_ton_roll": 2.8}),
            "R4_Y3_kiRoll15": make_param_set(R4_PARAMS, Y3_PARAMS, {"Ki_roll": 1.5}),
            "R4_Y3_kpPropRoll05": make_param_set(R4_PARAMS, Y3_PARAMS, {"Kp_prop_roll": 0.5, "z_ccr_roll": 0.05}),
            "R4_Y3_kpPropRollNeg05": make_param_set(R4_PARAMS, Y3_PARAMS, {"Kp_prop_roll": -0.5, "z_ccr_roll": 0.0}),
            "R4_Y4_pTonRoll2": make_param_set(R4_PARAMS, Y4_PARAMS, {"p_ton_roll": 2.0, "Kp_roll": 3.0, "K_ton_roll": 3.5}),
        }

        tasks_lat = [(name, configs[name]) for name in configs]
        tasks_yaw = [(name, configs[name]) for name in ["R4_Y4", "R4_Y3", "R4_Y3B", "Y3_only"]]
        tasks_apeo = [(name, configs[name]) for name in ["R4_only", "Y3_only", "Y4_only", "R4_Y4", "R4_Y3", "R4_Y3B", "R4_Y4_noKpPropYaw"]]

        with ProcessPoolExecutor(max_workers=8) as ex:
            futs = {}
            for name, p in tasks_lat:
                futs[ex.submit(run_full_axis_sim, ("LatEC", name, p, 2.0))] = ("LatEC", name)
            for name, p in tasks_yaw:
                futs[ex.submit(run_full_axis_sim, ("YawMA", name, p, 3.0))] = ("YawMA", name)
            for name, p in tasks_apeo:
                futs[ex.submit(run_apeo_worker, (name, p))] = ("APEO", name)

            lat_res, yaw_res, apeo_res = {}, {}, {}
            for f in as_completed(futs):
                axis, name = futs[f]
                r = f.result()
                if axis == "LatEC":
                    c = compute_latec_cost(r)
                    fr = r["freq_results"]
                    lat_res[name] = {
                        "cost": c,
                        "ph0.12": fr["0.1209"]["phase_err"],
                        "ph0.20": fr["0.1986"]["phase_err"],
                        "ph0.32": fr["0.3214"]["phase_err"],
                        "g0.12_db": fr["0.1209"]["gain_db_diff"],
                        "g3.99_db": fr["3.9915"]["gain_db_diff"],
                        "noise": r["hf_noise_rms"],
                        "corr": r["corr_vel"],
                    }
                    print(f"[LatEC] {name:24s} | cost={c:5.2f} | ph0.12={lat_res[name]['ph0.12']:+6.1f} ph0.20={lat_res[name]['ph0.20']:+6.1f} ph0.32={lat_res[name]['ph0.32']:+6.1f} | g3.99={lat_res[name]['g3.99_db']:+5.2f} dB | noise={r['hf_noise_rms']:.2f} | corr={r['corr_vel']:.3f}", flush=True)
                elif axis == "YawMA":
                    c = compute_yawma_cost(r)
                    fr = r["freq_results"]
                    yaw_res[name] = {
                        "cost": c,
                        "noise": r["hf_noise_rms"],
                        "corr": r["corr_vel"],
                        "gains": [round(d["sim_gain"], 2) for d in fr.values()],
                        "ph_errs": [round(d["phase_err"], 1) for d in fr.values()],
                    }
                    print(f"[YawMA] {name:24s} | cost={c:5.2f} | noise={r['hf_noise_rms']:.2f} | corr={r['corr_vel']:.3f} | gains={yaw_res[name]['gains']} | ph={yaw_res[name]['ph_errs']}", flush=True)
                else:
                    apeo_res[name] = r
                    print(f"[APEO ] {name:24s} | score={r['score']:7.2f} | 0.324ph={r['sp0324']:+5.1f} | 1.099g={r['g1099_db']:+5.2f} dB | 7.927ph={r['p7927']:4.1f} | maxR={r['max_roll']:.2f} maxY={r['max_yaw']:.2f}", flush=True)

        out = {"LatEC": lat_res, "YawMA": yaw_res, "APEO": apeo_res, "elapsed_s": time.time() - t0}
        with open(os.path.join(_ROOT, "logs/roll_yaw_baseline/diag_batch1.json"), "w", encoding="utf-8") as f:
            json.dump(out, f, indent=2)
        print(f"[DONE batch1] in {time.time() - t0:.1f}s", flush=True)


if __name__ == "__main__":
    main()
