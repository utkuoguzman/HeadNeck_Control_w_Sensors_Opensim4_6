import xml.etree.ElementTree as ET

tree = ET.parse('osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim')
root = tree.getroot()

joints = root.findall('.//JointSet/objects/*')
print(f"Total joints: {len(joints)}")
for j in joints:
    name = j.attrib.get('name')
    jtype = j.tag
    # check coordinates
    coords = j.findall('.//Coordinate')
    for c in coords:
        cname = c.attrib.get('name')
        # check default_value, default_speed_value, range, clamped, locked
        # check if there is damping, friction, or stiffness inside Coordinate
        damping = c.findtext('damping')
        friction = c.findtext('friction')
        print(f"Joint: {name} ({jtype}) -> Coord: {cname}, damping: {damping}, friction: {friction}")

