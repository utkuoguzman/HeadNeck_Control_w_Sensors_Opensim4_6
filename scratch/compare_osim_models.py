import os
import xml.etree.ElementTree as ET

our_file = 'osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim'
orig_files = [
    r'D:\Akademik\PhD_Thesis\OpenSim\HYOID\HYOID_1.2_ScaledStrenght_UpdatedInertia.osim',
    r'D:\Akademik\PhD_Thesis\OpenSim\HYOID\Mortensen et al. 2018, PLOS ONE\HYOID.osim',
    r'D:\Akademik\PhD_Thesis\OpenSim\HYOID\Mortensen et al. 2018, PLOS ONE\HYOID_Scaled.osim'
]

def parse_model(file_path):
    tree = ET.parse(file_path)
    root = tree.getroot()
    
    # Bodies
    bodies = {}
    for b in root.findall('.//BodySet/objects/Body'):
        name = b.attrib.get('name')
        mass = float(b.findtext('mass', '0'))
        com = b.findtext('mass_center', '')
        inertia = b.findtext('inertia_xx', '') # or inertia
        if not inertia:
            inertia = b.findtext('inertia', '')
        bodies[name] = {'mass': mass, 'com': com, 'inertia': inertia}
        
    # Joints
    joints = {}
    for j in root.findall('.//JointSet/objects/*'):
        jname = j.attrib.get('name')
        coords = [c.attrib.get('name') for c in j.findall('.//Coordinate')]
        joints[jname] = {'type': j.tag, 'coords': coords}
        
    # Coordinates
    coords = {}
    for c in root.findall('.//Coordinate'):
        cname = c.attrib.get('name')
        coords[cname] = {
            'default_value': c.findtext('default_value'),
            'range': c.findtext('range'),
            'clamped': c.findtext('clamped'),
            'locked': c.findtext('locked')
        }
        
    # Forces
    forces = {}
    for f in root.findall('.//ForceSet/objects/*'):
        name = f.attrib.get('name')
        forces[name] = {'type': f.tag, 'appliesForce': f.findtext('appliesForce')}
        
    # Muscles
    muscles = {}
    for m in root.findall('.//ForceSet/objects/*'):
        if 'Muscle' in m.tag:
            name = m.attrib.get('name')
            muscles[name] = {
                'type': m.tag,
                'fmax': float(m.findtext('max_isometric_force', '0')),
                'l0': float(m.findtext('optimal_fiber_length', '0')),
                'lst': float(m.findtext('tendon_slack_length', '0')),
                'pa': float(m.findtext('pennation_angle_at_optimal', '0'))
            }
            
    # Constraints
    constraints = {}
    for c in root.findall('.//ConstraintSet/objects/*'):
        name = c.attrib.get('name')
        constraints[name] = {'type': c.tag}
        
    # Controllers
    controllers = {}
    for c in root.findall('.//ControllerSet/objects/*'):
        name = c.attrib.get('name')
        controllers[name] = {'type': c.tag}
        
    return {
        'bodies': bodies,
        'joints': joints,
        'coords': coords,
        'forces': forces,
        'muscles': muscles,
        'constraints': constraints,
        'controllers': controllers
    }

our_model = parse_model(our_file)

print(f"=== OUR MODEL: {our_file} ===")
print(f"Bodies: {len(our_model['bodies'])}, Joints: {len(our_model['joints'])}, Coords: {len(our_model['coords'])}, Forces: {len(our_model['forces'])}, Muscles: {len(our_model['muscles'])}, Constraints: {len(our_model['constraints'])}, Controllers: {len(our_model['controllers'])}")

for orig_file in orig_files:
    if not os.path.exists(orig_file):
        continue
    print(f"\n{'='*60}")
    print(f"=== COMPARING WITH: {os.path.basename(orig_file)} ===")
    print(f"{'='*60}")
    orig = parse_model(orig_file)
    print(f"Bodies: {len(orig['bodies'])}, Joints: {len(orig['joints'])}, Coords: {len(orig['coords'])}, Forces: {len(orig['forces'])}, Muscles: {len(orig['muscles'])}, Constraints: {len(orig['constraints'])}, Controllers: {len(orig['controllers'])}")
    
    # Body comparison (especially skull / head mass and inertia)
    print("\n--- Key Body Masses ---")
    for bname in ['skull', 'c1', 'c2', 'c3', 'c4', 'c5', 'c6', 'c7', 'hyoid']:
        our_m = our_model['bodies'].get(bname, {}).get('mass')
        orig_m = orig['bodies'].get(bname, {}).get('mass')
        our_com = our_model['bodies'].get(bname, {}).get('com')
        orig_com = orig['bodies'].get(bname, {}).get('com')
        print(f"Body '{bname}': Our mass={our_m}, Orig mass={orig_m} | Our com={our_com}, Orig com={orig_com}")
        
    # Check Skull inertia
    our_skull_in = our_model['bodies'].get('skull', {}).get('inertia')
    orig_skull_in = orig['bodies'].get('skull', {}).get('inertia')
    print(f"Skull inertia: Our={our_skull_in} vs Orig={orig_skull_in}")
    
    # Compare Muscles
    our_m_names = set(our_model['muscles'].keys())
    orig_m_names = set(orig['muscles'].keys())
    common_muscles = our_m_names.intersection(orig_m_names)
    only_in_our = our_m_names - orig_m_names
    only_in_orig = orig_m_names - our_m_names
    print(f"\n--- Muscle Counts ---")
    print(f"Common muscles: {len(common_muscles)}")
    print(f"Only in our model ({len(only_in_our)}): {sorted(list(only_in_our))}")
    print(f"Only in orig model ({len(only_in_orig)}): {sorted(list(only_in_orig))}")
    
    # Check muscle strength ratios (Fmax) for common muscles
    diff_fmax = []
    diff_l0 = []
    diff_lst = []
    for m in sorted(list(common_muscles)):
        f1 = our_model['muscles'][m]['fmax']
        f2 = orig['muscles'][m]['fmax']
        if abs(f1 - f2) > 0.01:
            diff_fmax.append((m, f1, f2, f1/f2 if f2>0 else 0))
        l1 = our_model['muscles'][m]['l0']
        l2 = orig['muscles'][m]['l0']
        if abs(l1 - l2) > 0.001:
            diff_l0.append((m, l1, l2))
        lst1 = our_model['muscles'][m]['lst']
        lst2 = orig['muscles'][m]['lst']
        if abs(lst1 - lst2) > 0.001:
            diff_lst.append((m, lst1, lst2))
            
    print(f"\nMuscles with different Fmax: {len(diff_fmax)} of {len(common_muscles)}")
    if diff_fmax:
        for m, f1, f2, ratio in diff_fmax[:10]:
            print(f"  {m}: our={f1:.1f} N, orig={f2:.1f} N (ratio={ratio:.2f})")
        if len(diff_fmax) > 10:
            print(f"  ... and {len(diff_fmax)-10} more.")
            
    print(f"\nMuscles with different l0 (fiber length): {len(diff_l0)}")
    print(f"Muscles with different lst (tendon slack length): {len(diff_lst)}")
    
    # Check Non-muscle forces
    print(f"\n--- Non-Muscle Forces ---")
    our_f_names = set(k for k in our_model['forces'].keys() if k not in our_m_names)
    orig_f_names = set(k for k in orig['forces'].keys() if k not in orig_m_names)
    print(f"Our non-muscle forces ({len(our_f_names)}): {sorted(list(our_f_names))}")
    print(f"Orig non-muscle forces ({len(orig_f_names)}): {sorted(list(orig_f_names))}")
    
    # Check Bushings
    print(f"\n--- Bushings in Orig ---")
    for f in orig_f_names:
        if 'bushing' in f.lower():
            print(f"  {f}: {orig['forces'][f]}")
            
    # Check Constraints
    print(f"\n--- Constraints ---")
    print(f"Our constraints: {len(our_model['constraints'])}")
    print(f"Orig constraints: {len(orig['constraints'])}")

