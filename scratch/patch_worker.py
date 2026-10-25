with open("cmaes_worker_p22.py", "r", encoding="utf-8") as f:
    content = f.read()

# 1. Update delay_vcr to 0.010 (10 ms)
assert 'osim.PropertyHelper.setValueDouble(0.015, controller.updPropertyByName("delay_vcr"))' in content
content = content.replace(
    'osim.PropertyHelper.setValueDouble(0.015, controller.updPropertyByName("delay_vcr"))',
    'osim.PropertyHelper.setValueDouble(0.010, controller.updPropertyByName("delay_vcr"))'
)

# 2. Update gain calculation to directly return deg/m (amp_deg / amplitude)
old_gain_calc = """    accel_amplitude = amplitude * (2.0 * math.pi * freq)**2
    gain_deg_ms2 = amp_deg / accel_amplitude
    phase_deg = math.degrees(phase_rad)
    while phase_deg > 90.0: phase_deg -= 360.0
    while phase_deg < -270.0: phase_deg += 360.0
    
    return gain_deg_ms2, phase_deg"""

new_gain_calc = """    gain_deg_m = amp_deg / amplitude
    phase_deg = math.degrees(phase_rad)
    while phase_deg > 90.0: phase_deg -= 360.0
    while phase_deg < -270.0: phase_deg += 360.0
    
    return gain_deg_m, phase_deg"""

assert old_gain_calc in content, "old_gain_calc not found"
content = content.replace(old_gain_calc, new_gain_calc, 1)

with open("cmaes_worker_p22.py", "w", encoding="utf-8") as f:
    f.write(content)

print("cmaes_worker_p22.py successfully updated with 10ms VCR delay and deg/m gain!")

