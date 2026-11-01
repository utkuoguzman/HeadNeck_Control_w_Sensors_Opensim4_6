import math

# Data
data = [
    # (freq, gt_g, gt_p, rom_g, rom_p, cb_g, cb_p)
    (0.324, 20.52,  -5.09,   34.4,  -248.4, 10.8,  -28.4),
    (0.783, 139.22, -88.73,  195.8,   29.4, 85.0,  -55.0),
    (1.099, 207.62, -99.21,  329.3,  -12.1, 210.4, -65.8),
    (2.446, 703.65, -161.58, 698.9, -103.5, 877.3, -94.5),
    (5.583, 887.19,  25.82,  972.1, -184.1, 698.2, -116.0),
    (7.927, 694.47,   5.79, 1029.1, -217.8, 429.0, -137.1)
]

def calc_phase_diff(p1, p2):
    return (p1 - p2 + 180.0) % 360.0 - 180.0

rom_gain_pct_errs = []
rom_gain_db_diffs = []
rom_phase_errs = []

cb_gain_pct_errs = []
cb_gain_db_diffs = []
cb_phase_errs = []

print(f"{'Freq':<6} | {'GT Gain':<8} {'ROM Gain':<9} {'Err%':<6} {'CB Gain':<8} {'Err%':<6} | {'GT Ph':<7} {'ROM Ph':<7} {'Err°':<6} {'CB Ph':<7} {'Err°':<6}")
print("-" * 90)

for f, gt_g, gt_p, rom_g, rom_p, cb_g, cb_p in data:
    # ROM
    rom_pct = abs(rom_g - gt_g) / gt_g * 100.0
    rom_db = abs(20.0 * math.log10(rom_g / gt_g))
    rom_p_diff = abs(calc_phase_diff(rom_p, gt_p))
    rom_gain_pct_errs.append(rom_pct)
    rom_gain_db_diffs.append(rom_db)
    rom_phase_errs.append(rom_p_diff)

    # CB
    cb_pct = abs(cb_g - gt_g) / gt_g * 100.0
    cb_db = abs(20.0 * math.log10(cb_g / gt_g))
    cb_p_diff = abs(calc_phase_diff(cb_p, gt_p))
    cb_gain_pct_errs.append(cb_pct)
    cb_gain_db_diffs.append(cb_db)
    cb_phase_errs.append(cb_p_diff)

    print(f"{f:<6.3f} | {gt_g:<8.1f} {rom_g:<9.1f} {rom_pct:<6.1f} {cb_g:<8.1f} {cb_pct:<6.1f} | {gt_p:>6.1f}° {rom_p:>6.1f}° {rom_p_diff:<6.1f} {cb_p:>6.1f}° {cb_p_diff:<6.1f}")

print("-" * 90)
print(f"Average Gain Error (MAPE): ROM = {sum(rom_gain_pct_errs)/len(rom_gain_pct_errs):.1f}%,  CB (8-Joint) = {sum(cb_gain_pct_errs)/len(cb_gain_pct_errs):.1f}%")
print(f"Average Gain Diff  (dB):   ROM = {sum(rom_gain_db_diffs)/len(rom_gain_db_diffs):.2f} dB, CB (8-Joint) = {sum(cb_gain_db_diffs)/len(cb_gain_db_diffs):.2f} dB")
print(f"Average Phase Error(MAE):  ROM = {sum(rom_phase_errs)/len(rom_phase_errs):.1f}°,  CB (8-Joint) = {sum(cb_phase_errs)/len(cb_phase_errs):.1f}°")

