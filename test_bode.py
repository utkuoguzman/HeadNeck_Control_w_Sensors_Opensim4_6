import cmaes_worker
import sys
targets = [
    (0.323929, 50.840742, -77.472714),
    (0.441951, 63.811802, -82.607424),
    (0.602951, 80.957500, -86.082725),
    (0.822615, 106.182412, -85.503639),
    (1.124195, 149.387995, -81.042217)
]
kp, ki, kd = 50.0, 40.0, 5.0
print(f"Testing Kp={kp}, Ki={ki}, Kd={kd}", flush=True)
sse = 0.0
for freq, target_gain, target_phase in targets:
    print(f"Starting Freq: {freq:.2f} Hz...", flush=True)
    sim_gain, sim_phase = cmaes_worker.run_simulation(kp, ki, kd, freq)
    gain_error = sim_gain - target_gain
    phase_error = sim_phase - target_phase
    err = (gain_error**2) + 0.5*(phase_error**2)
    sse += err
    print(f"Freq: {freq:.2f} Hz done. Sim Gain: {sim_gain:.1f}, Sim Phase: {sim_phase:.1f}", flush=True)
print(f"Total SSE: {sse:.1f}", flush=True)
