import glob
import json

HAPPEE_TARGETS = {
    0.31: {'gain': 0.893, 'phase': 4.5},
    0.78: {'gain': 0.840, 'phase': -98.9},
    1.11: {'gain': 0.814, 'phase': -112.6},
    2.48: {'gain': 0.655, 'phase': -163.7},
    5.52: {'gain': 0.509, 'phase': -194.6},
    7.95: {'gain': 0.443, 'phase': -247.7}
}

for w in ['W1_G29', 'W4_G30', 'W12_G31', 'W15_G32']:
    print(f"\n=== Champion {w} ===")
    for freq in [0.31, 0.78, 1.11, 2.48, 5.52, 7.95]:
        files = glob.glob(f"logs/candidates/{w}_{freq:.2f}Hz.json")
        if files:
            d = json.load(open(files[0], encoding="utf-8"))
            t_phase = HAPPEE_TARGETS[freq]['phase']
            diff = (d['sim_phase'] - t_phase + 180.0) % 360.0 - 180.0
            print(f"  {freq:4.2f} Hz: Sim Phase = {d['sim_phase']:>6.1f}° | Target = {t_phase:>6.1f}° | Err = {diff:>5.1f}° | Gain Err = {d['gain_err_pct']:>5.1f}%")

