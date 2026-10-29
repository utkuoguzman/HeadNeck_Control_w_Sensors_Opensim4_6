import os
import sys
import json
import math
import time
import shutil
import subprocess
from concurrent.futures import ThreadPoolExecutor

TARGETS = [
    (0.324,  86.55,   -7.03),
    (0.783, 236.93,  -96.96),
    (1.099, 319.74, -109.23),
    (2.446, 423.34, -161.34),
    (5.583, 503.30, -201.38),
    (7.927, 572.69, -247.62)
]

def apply_fixes_and_build():
    cpp_path = "HeadNeckNeuralController_plugin/HeadNeckNeuralController.cpp"
    bak_path = "HeadNeckNeuralController_plugin/HeadNeckNeuralController.cpp.pre_3fixes.bak"
    if not os.path.exists(bak_path):
        shutil.copyfile(cpp_path, bak_path)

    with open(bak_path, "r", encoding="utf-8") as f:
        txt = f.read()

    # Fix 1: Scale AC integral drift update by 0.05 (K_i,AC ~ 2.01 Nm/(rad*s), fc ~ 0.02 Hz << 0.324 Hz)
    # while keeping static gravity trim tau_grav,0 = Ki * p_int(0) = 4.24053 Nm intact at t=0
    old_int = """    double p_err = traj_pos_pitch - vol_rel_pitch;
    double p_int = sv_err_int[0]->getValue(s);
    if (p_int > 0.6 && p_err > 0) p_err = 0.0;
    if (p_int < -0.6 && p_err < 0) p_err = 0.0;
    sv_err_int[0]->setDerivative(s, p_err);"""

    new_int = """    double p_err = traj_pos_pitch - vol_rel_pitch;
    double p_int = sv_err_int[0]->getValue(s);
    if (p_int > 0.6 && p_err > 0) p_err = 0.0;
    if (p_int < -0.6 && p_err < 0) p_err = 0.0;
    // Scale AC integral drift update by 0.05 (K_i,AC ~ 2.01 Nm/(rad*s), fc ~ 0.02 Hz << 0.324 Hz)
    // while keeping static gravity trim tau_grav,0 = Ki * p_int(0) = 4.24053 Nm intact at t=0!
    sv_err_int[0]->setDerivative(s, 0.05 * p_err);"""

    assert old_int in txt, "old_int not found"
    txt = txt.replace(old_int, new_int, 1)

    # Fix 3: Use luen_omega instead of omega_recon in u_sc_pitch/roll/yaw
    old_sc = """        double u_sc_pitch = vcr_data.record.omega_recon[2] - traj_vel_pitch;
        double u_sc_roll  = vcr_data.record.omega_recon[0] - traj_vel_roll;
        double u_sc_yaw   = vcr_data.record.omega_recon[1] - traj_vel_yaw;"""

    new_sc = """        double u_sc_pitch = vcr_data.record.luen_omega[2] - traj_vel_pitch;
        double u_sc_roll  = vcr_data.record.luen_omega[0] - traj_vel_roll;
        double u_sc_yaw   = vcr_data.record.luen_omega[1] - traj_vel_yaw;"""

    old_sc_ctrl = """        double u_sc_pitch = vcr_record.omega_recon[2] - traj_vel_pitch;
        double u_sc_roll  = vcr_record.omega_recon[0] - traj_vel_roll;
        double u_sc_yaw   = vcr_record.omega_recon[1] - traj_vel_yaw;"""

    new_sc_ctrl = """        double u_sc_pitch = vcr_record.luen_omega[2] - traj_vel_pitch;
        double u_sc_roll  = vcr_record.luen_omega[0] - traj_vel_roll;
        double u_sc_yaw   = vcr_record.luen_omega[1] - traj_vel_yaw;"""

    assert old_sc in txt, "old_sc not found"
    assert old_sc_ctrl in txt, "old_sc_ctrl not found"
    txt = txt.replace(old_sc, new_sc, 1).replace(old_sc_ctrl, new_sc_ctrl, 1)

    # Fix 2: Phasic-Tonic Lead-Lag Otolith Filter H_phas(s) = K_phas * (alpha_hf * s + p_phas) / (s + p_phas)
    old_phas = """        // Strictly-Proper Phasic Otolith Filter: H_phas(s) = K_phas * p_phas / (s + p_phas)
        // (Integrates skull tangential acceleration -r_neck*ddtheta into viscous damping above p_phas)
        double y_phas_pitch = get_K_phas() * get_p_phas() * x_phas_pitch;
        double y_phas_roll  = get_K_phas() * get_p_phas() * x_phas_roll;
        double y_phas_yaw   = get_K_phas() * get_p_phas() * x_phas_yaw;"""

    new_phas = """        // Phasic-Tonic Lead-Lag Otolith Filter (Fernandez & Goldberg 1976; Happee et al. 2017):
        // H_phas(s) = K_phas * (alpha_hf * s + p_phas) / (s + p_phas) = K_phas * (alpha_hf * u_phas + (1 - alpha_hf) * p_phas * x_phas)
        // Preserves 100% DC/low-frequency gain K_phas at 0 deg while restoring high-frequency phase lead at 5-8 Hz!
        const double alpha_phas_hf = 0.35;
        double y_phas_pitch = get_K_phas() * (alpha_phas_hf * u_phas_pitch + (1.0 - alpha_phas_hf) * get_p_phas() * x_phas_pitch);
        double y_phas_roll  = get_K_phas() * (alpha_phas_hf * u_phas_roll  + (1.0 - alpha_phas_hf) * get_p_phas() * x_phas_roll);
        double y_phas_yaw   = get_K_phas() * (alpha_phas_hf * u_phas_yaw   + (1.0 - alpha_phas_hf) * get_p_phas() * x_phas_yaw);"""

    assert old_phas in txt, "old_phas not found"
    txt = txt.replace(old_phas, new_phas, 1)

    with open(cpp_path, "w", encoding="utf-8") as f:
        f.write(txt)
    print("[STEP 1] Applied all 3 fixes to HeadNeckNeuralController.cpp.", flush=True)

    print("[STEP 2] Building osimHeadNeckNeural.dll...", flush=True)
    res = subprocess.run(["cmake", "--build", "HeadNeckNeuralController_plugin/build", "--config", "Release"], capture_output=True, text=True)
    if res.returncode != 0:
        print(res.stdout, res.stderr, flush=True)
        sys.exit(1)
    print("[STEP 2] Build succeeded!", flush=True)

def run_job(job):
    w_name, th, freq, tg, tp = job
    K_ton, p_ton, K_sc, z_sc, p_sc, K_phas, p_phas, kp_prop, z_ccr, p_ccr, k_gdyn, k_gstat = th
    r_lead_sc = z_sc / p_sc
    r_lead_ccr = z_ccr / p_ccr

    res_file = os.path.join("logs", "candidates", f"{w_name}_{freq:.2f}Hz.json")
    if os.path.exists(res_file):
        os.remove(res_file)

    cmd = [
        sys.executable, "cmaes_worker_p22.py",
        "0.0", "1.0", "40.2786",
        f"{K_ton:.6f}", f"{p_ton:.6f}", f"{K_sc:.6f}", f"{r_lead_sc:.6f}", f"{p_sc:.6f}",
        f"{K_phas:.6f}", f"{p_phas:.6f}", f"{kp_prop:.6f}", f"{r_lead_ccr:.6f}", f"{p_ccr:.6f}",
        f"{k_gdyn:.6f}", f"{k_gstat:.6f}",
        f"{freq}", f"{tg}", f"{tp}",
        w_name
    ]
    env = os.environ.copy()
    env["KP_TASK_PITCH"] = "0.0"
    env["KD_TASK_PITCH"] = "0.0"
    subprocess.run(cmd, capture_output=True, text=True, env=env)
    with open(res_file, "r", encoding="utf-8") as f:
        return w_name, freq, json.load(f)

def main():
    apply_fixes_and_build()

    with open("logs/candidates/W4_P24G3.json", "r", encoding="utf-8") as f:
        w4_params = json.load(f)["params"]

    # Candidate 1: Exact W4_P24G3 parameters
    th_raw = list(w4_params)
    # Candidate 2: W4_P24G3 with K_sc scaled by 1.12x to compensate for removing the un-inverted (1 + 0.02442s) gain boost
    th_rescaled = list(w4_params)
    th_rescaled[2] = w4_params[2] * 1.12

    cands = {
        "W4_3FIX_RAW": th_raw,
        "W4_3FIX_KSC112": th_rescaled
    }

    jobs = []
    for w_name, th in cands.items():
        with open(f"logs/candidates/{w_name}.json", "w", encoding="utf-8") as f:
            json.dump({"worker": w_name, "params": th}, f)
        for freq, tg, tp in TARGETS:
            jobs.append((w_name, th, freq, tg, tp))

    print("\n[STEP 3] Evaluating W4_3FIX_RAW and W4_3FIX_KSC112 across all 6 frequencies (12 parallel workers)...", flush=True)
    t0 = time.time()
    with ThreadPoolExecutor(max_workers=12) as ex:
        results = list(ex.map(run_job, jobs))
    elapsed = time.time() - t0
    print(f"[COMPLETED in {elapsed:.1f}s]\n", flush=True)

    for w_name in ["W4_3FIX_RAW", "W4_3FIX_KSC112"]:
        f_map = {freq: d for (w, freq, d) in results if w == w_name}
        tot = sum(d["score"] for d in f_map.values())
        print("=" * 90, flush=True)
        print(f"CANDIDATE: {w_name} | TOTAL SCORE: {tot:,.2f} pts (vs old W4_P24G3 = 12,415.56 pts)", flush=True)
        print("=" * 90, flush=True)
        for freq, tg, tp in TARGETS:
            d = f_map[freq]
            pdiff = (d["sim_phase"] - tp + 180.0) % 360.0 - 180.0
            print(f"  {freq:5.3f} Hz | Gain: {d['sim_gain']:7.2f} (Tgt {tg:6.2f}, {d['gain_db_diff']:+6.2f} dB, {d['gain_loss']:6.1f} pts) | Phase: {d['sim_phase']:7.2f}° (Tgt {tp:7.2f}°, {pdiff:+6.2f}°, {d['phase_loss']:6.1f} pts) | Score: {d['score']:7.1f}", flush=True)
        print(flush=True)

if __name__ == "__main__":
    main()

