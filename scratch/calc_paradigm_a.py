import numpy as np

# Original 100% ROM values from HYOID_1.2
rom_100 = {
    'pitch1': {'flex': -0.279253, 'ext': 0.418879, 'flex_deg': -16.0, 'ext_deg': 24.0},
    'pitch2': {'flex': -0.575959, 'ext': 0.837758, 'flex_deg': -33.0, 'ext_deg': 48.0},
    'roll1':  {'flex': -0.104720, 'ext': 0.104720, 'flex_deg': -6.0,  'ext_deg': 6.0},
    'roll2':  {'flex': -0.575959, 'ext': 0.575959, 'flex_deg': -33.0, 'ext_deg': 33.0},
    'yaw1':   {'flex': -0.663225, 'ext': 0.663225, 'flex_deg': -38.0, 'ext_deg': 38.0},
    'yaw2':   {'flex': -0.471239, 'ext': 0.471239, 'flex_deg': -27.0, 'ext_deg': 27.0}
}

RATIO_DEADBAND = 0.75  # Deadband ends at 75% of voluntary ROM
RATIO_HARDBOUND = 1.20 # Hard coordinate bound at 120% of voluntary ROM

print("="*80)
print(f"{'Coordinate':10s} | {'Deadband Ends (75% ROM)':30s} | {'100% ROM (Ref)':20s} | {'Hard Bound (120%)':25s}")
print("="*80)

calib_paradigm_a = {}

for name, r in rom_100.items():
    # Deadband
    db_lower = RATIO_DEADBAND * r['flex']
    db_upper = RATIO_DEADBAND * r['ext']
    db_lower_deg = RATIO_DEADBAND * r['flex_deg']
    db_upper_deg = RATIO_DEADBAND * r['ext_deg']
    
    # Transition width: 25% of average ROM span
    trans_lower = abs(r['flex'] - db_lower)
    trans_upper = abs(r['ext'] - db_upper)
    trans = (trans_lower + trans_upper) / 2.0
    trans_deg = (abs(r['flex_deg'] - db_lower_deg) + abs(r['ext_deg'] - db_upper_deg)) / 2.0
    
    # Hard bounds: 120% of voluntary ROM
    hb_lower = RATIO_HARDBOUND * r['flex']
    hb_upper = RATIO_HARDBOUND * r['ext']
    hb_lower_deg = RATIO_HARDBOUND * r['flex_deg']
    hb_upper_deg = RATIO_HARDBOUND * r['ext_deg']
    
    calib_paradigm_a[name] = {
        'db_lower': db_lower,
        'db_upper': db_upper,
        'db_lower_deg': db_lower_deg,
        'db_upper_deg': db_upper_deg,
        'trans': trans,
        'trans_deg': trans_deg,
        'hb_lower': hb_lower,
        'hb_upper': hb_upper,
        'hb_lower_deg': hb_lower_deg,
        'hb_upper_deg': hb_upper_deg,
        'stiff': 250.0 if '2' in name or 'roll' in name or 'yaw' in name else 200.0,
        'damp': 25.0 if '2' in name or 'roll' in name or 'yaw' in name else 20.0
    }
    
    print(f"{name:10s} | [{db_lower_deg:6.1f}°, {db_upper_deg:6.1f}°] ({db_lower:7.4f}, {db_upper:6.4f} rad) | [{r['flex_deg']:5.1f}°, {r['ext_deg']:5.1f}°] | [{hb_lower_deg:6.1f}°, {hb_upper_deg:6.1f}°] ({hb_lower:7.4f}, {hb_upper:6.4f} rad)")

print("="*80)

