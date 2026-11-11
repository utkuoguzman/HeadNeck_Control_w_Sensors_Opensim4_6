import os
import sys

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
_UTIL = os.path.join(_ROOT, "util")
if _UTIL not in sys.path:
    sys.path.insert(0, _UTIL)

os.environ["MULTISINE_CYCLES"] = "2.0"
os.environ["SAVE_TS"] = "0"
os.environ["MULTISINE_EXP"] = "1.0"
os.environ["MULTISINE_MIN_AMP_M"] = "0.0"
os.environ["NOISE_FLOOR_7927_DEG"] = "0.0"

from cmaes_worker_multisine import run_multisine_simulation

print("=== Running Quick 2-Cycle Test with p_sc = 53.284 rad/s and K_sc = 4.05, Kd = 0.525 ===")
tot, freqs, _ = run_multisine_simulation(
    3.7886, 3.7619, 0.525,
    5.1973, 16.227, 4.05, 0.0, 53.284, 14.5225, 4.712,
    0.5493, 0.0, 99.92, 1.0, 1.0,
    worker_name="test_ksc_405"
)

print(f"\nTotal Multisine Score: {tot:,.2f} pts")
for fk in ["0.324", "0.783", "1.099", "2.446", "5.583", "7.927"]:
    fd = freqs[fk]
    gdb = fd.get("gain_db_diff", 0.0)
    perr = fd.get("phase_err_deg", 0.0)
    print(f"  {fk} Hz: Gain={fd['sim_gain']:6.2f} vs {fd['target_gain']:6.2f} ({gdb:+5.2f} dB) | Phase={fd['sim_phase']:7.2f}° vs {fd['target_phase']:7.2f}° (err={perr:5.1f}°)")
