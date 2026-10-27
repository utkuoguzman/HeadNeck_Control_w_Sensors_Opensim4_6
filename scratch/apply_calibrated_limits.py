import xml.etree.ElementTree as ET

osim_path = 'osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim'
tree = ET.parse(osim_path)
root = tree.getroot()

calib = {
    'pitch1': {
        'llim': '-0.2793', 'ulim': '0.4189', 'trans': '0.0524',
        'lstiff': '200', 'ustiff': '200', 'damp': '20',
        'crange': '-0.3840 0.5236'
    },
    'pitch2': {
        'llim': '-0.5760', 'ulim': '0.8378', 'trans': '0.0698',
        'lstiff': '250', 'ustiff': '250', 'damp': '25',
        'crange': '-0.7156 0.9774'
    },
    'roll1': {
        'llim': '-0.1047', 'ulim': '0.1047', 'trans': '0.0349',
        'lstiff': '250', 'ustiff': '250', 'damp': '25',
        'crange': '-0.1745 0.1745'
    },
    'roll2': {
        'llim': '-0.5760', 'ulim': '0.5760', 'trans': '0.0698',
        'lstiff': '250', 'ustiff': '250', 'damp': '25',
        'crange': '-0.7156 0.7156'
    },
    'yaw1': {
        'llim': '-0.6632', 'ulim': '0.6632', 'trans': '0.0698',
        'lstiff': '250', 'ustiff': '250', 'damp': '25',
        'crange': '-0.8028 0.8028'
    },
    'yaw2': {
        'llim': '-0.4712', 'ulim': '0.4712', 'trans': '0.0524',
        'lstiff': '250', 'ustiff': '250', 'damp': '25',
        'crange': '-0.5760 0.5760'
    }
}

# Update CoordinateLimitForce elements
for clf in root.findall('.//CoordinateLimitForce'):
    cname = clf.findtext('coordinate')
    if cname in calib:
        cfg = calib[cname]
        clf.find('lower_limit').text = cfg['llim']
        clf.find('upper_limit').text = cfg['ulim']
        clf.find('transition').text = cfg['trans']
        clf.find('lower_stiffness').text = cfg['lstiff']
        clf.find('upper_stiffness').text = cfg['ustiff']
        clf.find('damping').text = cfg['damp']
        print(f"Updated CoordinateLimitForce for {cname}")

# Update Coordinate range elements
for coord in root.findall('.//Coordinate'):
    name = coord.attrib.get('name')
    if name in calib:
        coord.find('range').text = calib[name]['crange']
        print(f"Updated Coordinate range for {name}: {calib[name]['crange']}")

tree.write(osim_path, xml_declaration=True, encoding='utf-8')
print("Successfully saved updated osim model.")

