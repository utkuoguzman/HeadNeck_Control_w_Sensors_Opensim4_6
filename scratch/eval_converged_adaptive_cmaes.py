import math
import os
import sys

# Import run_simulation from cmaes_worker_p22
sys.path.append(os.getcwd())
from cmaes_worker_p22 import run_simulation

TARGETS = [
    (0.324,  86.55,   -7.03),
    (0.783, 236.93,  -96.96),
    (1.099, 319.74, -109.23),
    (2.446, 423.34, -161.34),
    (5.583, 503.30, -201.38),
    (7.927, 572.69, -247.62)
]

# Converged parameters from Path 1 adaptive simulation:
K_ton = 0.5799
p_ton = 78.0
K_sc = 0.4377
z_sc = 21.6
p_sc = 90.0
K_phas = 4.5088
p_phas = 15.0
kp_prop = 1.3967
z_ccr = 8.5
p_ccr = 170.0
k_gamma_dyn = 0.70
k_gamma_stat = 1.15

kp_task = 10.24
kd_task = 1.31
ki_task = 40.2786

print("=" * 85)
print("EVALUATING CONVERGED ADAPTIVE GAINS VIA CLEAN CMA-ES WORKER PIPELINE")
print("=" * 85)
print(f"{'Freq(Hz)':>8s} | {'SimGain(deg/m)':>14s} | {'TgtGain':>8s} | {'GainErr(dB)':>11s} | {'SimPhase':>8s} | {'TgtPhase':>8s} | {'PhaseErr':>8s} | {'Loss(pts)':>9s}")
print("-" * 85)

total_loss = 0.0

for freq, tgt_gain, tgt_phase in TARGETS:
    sim_gain, sim_phase = run_simulation(
        kp_task, ki_task, kd_task,
        K_ton, p_ton,
        K_sc, z_sc, p_sc,
        K_phas, p_phas,
        kp_prop, z_ccr, p_ccr,
        k_gamma_dyn, k_gamma_stat,
        freq, worker_name="W-EVAL"
    )
    
    sim_gain_safe = max(1e-6, sim_gain)
    gain_db_diff = 20.0 * math.log10(sim_gain_safe / tgt_gain)
    gain_loss = ((gain_db_diff / 1.0) ** 2) * 100.0

    phase_diff = (sim_phase - tgt_phase + 180.0) % 360.0 - 180.0
    phase_loss = ((phase_diff / 10.0) ** 2) * 100.0

    error = gain_loss + phase_loss
    total_loss += error

    print(f"{freq:8.3f} | {sim_gain:14.2f} | {tgt_gain:8.2f} | {gain_db_diff:+10.2f}dB | {sim_phase:8.2f}° | {tgt_phase:8.2f}° | {phase_diff:+7.2f}° | {error:9.1f}")

print("-" * 85)
print(f"OFFICIAL CMA-ES BENCHMARK SCORE: {total_loss:,.2f} pts")
print("=" * 85)

