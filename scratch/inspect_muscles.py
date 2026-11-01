import xml.etree.ElementTree as ET

tree = ET.parse('osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim')
root = tree.getroot()

muscles = root.findall('.//Millard12EqMuscleWithAfferents')
print(f"Total Millard12EqMuscleWithAfferents: {len(muscles)}")

fmax_list = []
opt_fib_list = []
tendon_slack_list = []

for m in muscles:
    name = m.attrib.get('name')
    fmax = float(m.findtext('max_isometric_force', '0'))
    opt_fib = float(m.findtext('optimal_fiber_length', '0'))
    tsl = float(m.findtext('tendon_slack_length', '0'))
    p_mult = float(m.findtext('passive_fiber_force_multiplier', '1.0') or 1.0)
    applies = m.findtext('appliesForce', 'true')
    fmax_list.append(fmax)
    opt_fib_list.append(opt_fib)
    tendon_slack_list.append(tsl)

print(f"Fmax range: min={min(fmax_list):.1f} N, max={max(fmax_list):.1f} N, mean={sum(fmax_list)/len(fmax_list):.1f} N")
print(f"OptFiber range: min={min(opt_fib_list):.4f} m, max={max(opt_fib_list):.4f} m")
print(f"TendonSlack range: min={min(tendon_slack_list):.4f} m, max={max(tendon_slack_list):.4f} m")

