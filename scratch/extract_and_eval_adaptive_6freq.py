import os
import sys
import math
import time
import json
import subprocess
from concurrent.futures import ThreadPoolExecutor
import numpy as np
import opensim as osim

ADAPT_12_NAMES = [
    "adapt_K_ton", "adapt_p_ton", "adapt_K_sc", "adapt_z_sc", "adapt_p_sc",
    "adapt_K_phas", "adapt_p_phas", "adapt_kp_prop", "adapt_z_ccr", "adapt_p_ccr",
    "adapt_K_gamma_dyn", "adapt_K_gamma_stat"
]

TARGETS = [
    (0.324,  86.55,   -7.03),
    (0.783, 236.93,  -96.96),
    (1.099, 319.74, -109.23),
    (2.446, 423.34, -161.34),
    (5.583, 503.30, -201.38),
    (7.927, 572.69, -247.62)
]

def create_multisine_spline(targets, duration=30.0, delay=1.5, ramp=1.0):
    K = len(targets)
    freqs = np.array([t[0] for t in targets])
    phases = np.array([math.pi * k * (k - 1) / K for k in range(K)])
    base_amps = np.array([0.0215 * (0.835 / f)**2 for f in freqs])
    scale_factor = 0.35 / math.sqrt(K) # 0.142887
    amps = base_amps * scale_factor

    dt = 0.005 # 200 Hz grid for spline
    t_arr = np.arange(0.0, duration + 0.1, dt)
    spline = osim.SimmSpline()
    
    for t in t_arr:
        if t < delay:
            spline.addPoint(t, 0.0)
        else:
            t_rel = t - delay
            if t_rel < ramp:
                tau = t_rel / ramp
                w = 10.0 * (tau**3) - 15.0 * (tau**4) + 6.0 * (tau**5)
            else:
                w = 1.0
            
            y = 0.0
            for i in range(K):
                y += amps[i] * math.sin(2.0 * math.pi * freqs[i] * t_rel + phases[i])
            spline.addPoint(t, float(w * y))
            
    return spline, freqs, amps, phases, scale_factor

def run_single_sine_worker(args):
    freq, tgt_gain, tgt_phase, params_12 = args
    # params_12 has [K_ton, p_ton, K_sc, z_sc, p_sc, K_phas, p_phas, kp_prop, z_ccr, p_ccr, K_gamma_dyn, K_gamma_stat]
    # cmaes_worker_p22.py expects r_lead_sc = z_sc / p_sc and r_lead_ccr = z_ccr / p_ccr
    K_ton = params_12["adapt_K_ton"]
    p_ton = params_12["adapt_p_ton"]
    K_sc = params_12["adapt_K_sc"]
    z_sc = params_12["adapt_z_sc"]
    p_sc = params_12["adapt_p_sc"]
    r_lead_sc = z_sc / p_sc
    K_phas = params_12["adapt_K_phas"]
    p_phas = params_12["adapt_p_phas"]
    kp_prop = params_12["adapt_kp_prop"]
    z_ccr = params_12["adapt_z_ccr"]
    p_ccr = params_12["adapt_p_ccr"]
    r_lead_ccr = z_ccr / p_ccr
    k_gamma_dyn = params_12["adapt_K_gamma_dyn"]
    k_gamma_stat = params_12["adapt_K_gamma_stat"]

    worker_name = "W_ADAPT_LYAP"
    res_file = os.path.join("logs", "candidates", f"{worker_name}_{freq:.2f}Hz.json")
    if os.path.exists(res_file):
        os.remove(res_file)

    cmd = [
        sys.executable, "cmaes_worker_p22.py",
        "0.0", "1.0", "40.2786",
        f"{K_ton:.6f}", f"{p_ton:.6f}", f"{K_sc:.6f}", f"{r_lead_sc:.6f}", f"{p_sc:.6f}",
        f"{K_phas:.6f}", f"{p_phas:.6f}", f"{kp_prop:.6f}", f"{r_lead_ccr:.6f}", f"{p_ccr:.6f}",
        f"{k_gamma_dyn:.6f}", f"{k_gamma_stat:.6f}",
        f"{freq}", f"{tgt_gain}", f"{tgt_phase}",
        worker_name
    ]
    env = os.environ.copy()
    env["KP_TASK_PITCH"] = "0.0"
    env["KD_TASK_PITCH"] = "0.0"
    proc = subprocess.run(cmd, capture_output=True, text=True, env=env)
    with open(res_file, "r", encoding="utf-8") as f:
        return json.load(f)

def main():
    print("=" * 88, flush=True)
    print("STEP 1: INTEGRATE LYAPUNOV SPACE-MAPPED ADAPTIVE CONTROLLER TO t = 18.0s FREEZE", flush=True)
    print("=" * 88, flush=True)

    spline, freqs, amps, phases, scale_factor = create_multisine_spline(
        TARGETS, duration=30.0, delay=1.5, ramp=1.0
    )

    os.environ["SIMB_NUM_THREADS"] = "1"
    os.environ["OMP_NUM_THREADS"] = "1"
    osim.Logger.setLevelString("Warn")

    fix_dll = os.path.abspath("FixationController_plugin/build/Release/osimFixation.dll")
    vest_dll = os.path.abspath("vestibular_plugin/build/Release/osimVestibular.dll")
    prop_dll = os.path.abspath("proprioception_plugin/build/Release/osimMillard12EqWithAff.dll")
    adapt_dll = os.path.abspath("HeadNeckNeuralControllerAdaptive_plugin/build/Release/osimHeadNeckNeuralAdaptive.dll")

    osim.LoadOpenSimLibraryExact(fix_dll)
    osim.LoadOpenSimLibraryExact(vest_dll)
    osim.LoadOpenSimLibraryExact(prop_dll)
    osim.LoadOpenSimLibraryExact(adapt_dll)

    model_path = "osim_files/HYOID_HeadNeckNeural_Model_Adaptive.osim"
    model = osim.Model(model_path)

    for i in range(model.getControllerSet().getSize()):
        ctrl = model.getControllerSet().get(i)
        if ctrl.getConcreteClassName() == "FixationController":
            ctrl.set_enabled(False)

    zero_func = osim.Constant(0.0)
    for coord_name in ["gndpitch", "gndroll", "gndyaw", "spine_ty", "spine_tz"]:
        c = model.updCoordinateSet().get(coord_name)
        c.set_locked(False)
        c.setDefaultIsPrescribed(True)
        c.setPrescribedFunction(zero_func)

    c_yaw1 = model.updCoordinateSet().get("yaw1")
    c_yaw1.set_locked(False)
    c_yaw1.setDefaultIsPrescribed(False)

    coord_tx = model.updCoordinateSet().get("spine_tx")
    coord_tx.set_locked(False)
    coord_tx.setDefaultIsPrescribed(True)
    coord_tx.setPrescribedFunction(spline)

    state = model.initSystem()
    c_yaw1.setIsPrescribed(state, False)
    coord_tx.setSpeedValue(state, 0.0)

    skull = model.getBodySet().get("skull")
    R_head_to_world = skull.getTransformInGround(state).R()
    q = osim.Rotation(R_head_to_world).convertRotationToQuaternion()
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q0", q.get(0))
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q1", q.get(1))
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q2", q.get(2))
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q3", q.get(3))
    
    ki = 40.2786
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/pitch_error_integral", 0.10528)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/roll_error_integral", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/yaw_error_integral", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_pitch", ki * 0.10528)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_roll", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_yaw", 0.0)

    model.equilibrateMuscles(state)

    manager = osim.Manager(model)
    manager.setIntegratorMethod(5) # RungeKuttaMerson
    manager.setIntegratorAccuracy(1e-4)
    manager.setIntegratorMinimumStepSize(1e-4)
    manager.setIntegratorMaximumStepSize(0.01)

    state.setTime(0.0)
    manager.initialize(state)

    t0 = time.time()
    for t_step in range(1, 19):
        t_target = float(t_step)
        state = manager.integrate(t_target)
        elapsed = time.time() - t0
        k_sc_v = model.getStateVariableValue(state, "/controllerset/VCR_CCR_Controller/adapt_K_sc")
        z_sc_v = model.getStateVariableValue(state, "/controllerset/VCR_CCR_Controller/adapt_z_sc")
        p_sc_v = model.getStateVariableValue(state, "/controllerset/VCR_CCR_Controller/adapt_p_sc")
        k_ton_v = model.getStateVariableValue(state, "/controllerset/VCR_CCR_Controller/adapt_K_ton")
        kg_dyn_v = model.getStateVariableValue(state, "/controllerset/VCR_CCR_Controller/adapt_K_gamma_dyn")
        print(f"t = {t_target:5.1f}s | Wall = {elapsed:6.1f}s | K_sc={k_sc_v:.4f} z_sc={z_sc_v:.2f} p_sc={p_sc_v:.2f} K_ton={k_ton_v:.4f} K_gdyn={kg_dyn_v:.3f}", flush=True)

    final_params = {}
    for name in ADAPT_12_NAMES:
        final_params[name] = float(model.getStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/{name}"))

    print("\n" + "=" * 88, flush=True)
    print("CONVERGED 12 PARAMETERS AT t = 18.0s (COSINE TAPER FREEZE):", flush=True)
    print("=" * 88, flush=True)
    for k, v in final_params.items():
        print(f"  {k:20s}: {v:.6f}", flush=True)

    print("\n" + "=" * 88, flush=True)
    print("STEP 2: EVALUATING ALL 6 SINGLE-SINE FREQUENCIES IN PARALLEL (cmaes_worker_p22.py)", flush=True)
    print("=" * 88, flush=True)

    t_eval_start = time.time()
    tasks = [(f, g, p, final_params) for (f, g, p) in TARGETS]
    with ThreadPoolExecutor(max_workers=6) as ex:
        results = list(ex.map(run_single_sine_worker, tasks))
    t_eval_elapsed = time.time() - t_eval_start

    results.sort(key=lambda r: r["freq"])
    print(f"\n6-Frequency Parallel Evaluation Finished in {t_eval_elapsed:.1f}s!", flush=True)
    print("=" * 88, flush=True)
    print("SINGLE-SINE BENCHMARK SCORE OF CONVERGED ADAPTIVE CONTROLLER", flush=True)
    print("=" * 88, flush=True)
    print(f"{'Freq(Hz)':>8s} | {'SimGain(deg/m)':>14s} | {'TgtGain':>8s} | {'GainErr(dB)':>11s} | {'SimPhase':>8s} | {'TgtPhase':>8s} | {'PhaseErr':>8s} | {'Loss(pts)':>9s}", flush=True)
    print("-" * 88, flush=True)

    total_score = 0.0
    for r in results:
        f = r["freq"]
        sg = r["sim_gain"]
        tg = r["target_gain"]
        gdb = r["gain_db_diff"]
        sp = r["sim_phase"]
        tp = r["target_phase"]
        perr = (sp - tp + 180.0) % 360.0 - 180.0
        sc = r["score"]
        total_score += sc
        print(f"{f:8.3f} | {sg:14.2f} | {tg:8.2f} | {gdb:+10.2f}dB | {sp:8.2f}° | {tp:8.2f}° | {perr:+7.2f}° | {sc:9.1f}", flush=True)

    print("-" * 88, flush=True)
    print(f"TOTAL 6-FREQUENCY SINGLE-SINE BENCHMARK SCORE: {total_score:,.2f} pts", flush=True)
    print("=" * 88, flush=True)

    os.makedirs("logs", exist_ok=True)
    out_summary = "logs/adaptive_lyapunov_6freq_eval.json"
    with open(out_summary, "w", encoding="utf-8") as f:
        json.dump({
            "converged_parameters_at_18s": final_params,
            "cmaes_vector_representation": [
                final_params["adapt_K_ton"],
                final_params["adapt_p_ton"],
                final_params["adapt_K_sc"],
                final_params["adapt_z_sc"] / final_params["adapt_p_sc"],
                final_params["adapt_p_sc"],
                final_params["adapt_K_phas"],
                final_params["adapt_p_phas"],
                final_params["adapt_kp_prop"],
                final_params["adapt_z_ccr"] / final_params["adapt_p_ccr"],
                final_params["adapt_p_ccr"],
                final_params["adapt_K_gamma_dyn"],
                final_params["adapt_K_gamma_stat"]
            ],
            "total_single_sine_score": total_score,
            "per_frequency_results": results
        }, f, indent=2)
    print(f"[SAVED] {out_summary}", flush=True)

if __name__ == "__main__":
    main()

