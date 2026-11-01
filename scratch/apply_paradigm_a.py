import xml.etree.ElementTree as ET

osim_path = 'osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim'
tree = ET.parse(osim_path)
root = tree.getroot()

# Paradigm A Configuration:
# 100% ROM from original HYOID_1.2
# Deadband: 75% of 100% ROM
# Hard bounds: 120% of 100% ROM
# Transition: 25% of ROM (spans from 75% to 100% ROM)
paradigm_a_config = {
    'pitch1': {
        'llim': '-0.209440',  # -12.0 deg
        'ulim': '0.314159',   # +18.0 deg
        'trans': '0.087266',  # 5.0 deg avg transition (spans 75% -> 100% ROM)
        'lstiff': '200', 'ustiff': '200', 'damp': '20',
        'crange': '-0.335103 0.502655' # -19.2 deg to +28.8 deg (120%)
    },
    'pitch2': {
        'llim': '-0.431969',  # -24.75 deg
        'ulim': '0.628319',   # +36.0 deg
        'trans': '0.174533',  # 10.0 deg avg transition (spans 75% -> 100% ROM)
        'lstiff': '250', 'ustiff': '250', 'damp': '25',
        'crange': '-0.691151 1.005310' # -39.6 deg to +57.6 deg (120%)
    },
    'roll1': {
        'llim': '-0.078540',  # -4.5 deg
        'ulim': '0.078540',   # +4.5 deg
        'trans': '0.026180',  # 1.5 deg transition (spans 75% -> 100% ROM)
        'lstiff': '250', 'ustiff': '250', 'damp': '25',
        'crange': '-0.125664 0.125664' # -7.2 deg to +7.2 deg (120%)
    },
    'roll2': {
        'llim': '-0.431969',  # -24.75 deg
        'ulim': '0.431969',   # +24.75 deg
        'trans': '0.143990',  # 8.25 deg transition (spans 75% -> 100% ROM)
        'lstiff': '250', 'ustiff': '250', 'damp': '25',
        'crange': '-0.691151 0.691151' # -39.6 deg to +39.6 deg (120%)
    },
    'yaw1': {
        'llim': '-0.497419',  # -28.5 deg
        'ulim': '0.497419',   # +28.5 deg
        'trans': '0.165806',  # 9.5 deg transition (spans 75% -> 100% ROM)
        'lstiff': '250', 'ustiff': '250', 'damp': '25',
        'crange': '-0.795870 0.795870' # -45.6 deg to +45.6 deg (120%)
    },
    'yaw2': {
        'llim': '-0.353429',  # -20.25 deg
        'ulim': '0.353429',   # +20.25 deg
        'trans': '0.117810',  # 6.75 deg transition (spans 75% -> 100% ROM)
        'lstiff': '250', 'ustiff': '250', 'damp': '25',
        'crange': '-0.565487 0.565487' # -32.4 deg to +32.4 deg (120%)
    }
}

# Update CoordinateLimitForce elements
for clf in root.findall('.//CoordinateLimitForce'):
    cname = clf.findtext('coordinate')
    if cname in paradigm_a_config:
        cfg = paradigm_a_config[cname]
        clf.find('lower_limit').text = cfg['llim']
        clf.find('upper_limit').text = cfg['ulim']
        clf.find('transition').text = cfg['trans']
        clf.find('lower_stiffness').text = cfg['lstiff']
        clf.find('upper_stiffness').text = cfg['ustiff']
        clf.find('damping').text = cfg['damp']
        print(f"Updated CoordinateLimitForce for {cname}: deadband=[{cfg['llim']}, {cfg['ulim']}] rad, trans={cfg['trans']} rad")

# Update Coordinate range elements
for coord in root.findall('.//Coordinate'):
    name = coord.attrib.get('name')
    if name in paradigm_a_config:
        coord.find('range').text = paradigm_a_config[name]['crange']
        print(f"Updated Coordinate range for {name}: {paradigm_a_config[name]['crange']}")

tree.write(osim_path, xml_declaration=True, encoding='utf-8')
print("Successfully applied Paradigm A to osim model.")

