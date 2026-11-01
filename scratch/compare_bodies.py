import xml.etree.ElementTree as ET

tree1 = ET.parse('osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim')
tree2 = ET.parse(r'D:\Akademik\PhD_Thesis\OpenSim\HYOID\HYOID_1.2_ScaledStrenght_UpdatedInertia.osim')

def get_b_dict(tree):
    d = {}
    for b in tree.findall('.//BodySet/objects/Body'):
        name = b.attrib.get('name')
        mass = b.findtext('mass')
        com = b.findtext('mass_center')
        inertia = b.findtext('inertia')
        if not inertia:
            ixx = b.findtext('inertia_xx')
            iyy = b.findtext('inertia_yy')
            izz = b.findtext('inertia_zz')
            if ixx:
                inertia = f"{ixx} {iyy} {izz}"
        d[name] = (mass, com, inertia)
    return d

d1 = get_b_dict(tree1)
d2 = get_b_dict(tree2)

for name in d2:
    if name in d1:
        m1, c1, i1 = d1[name]
        m2, c2, i2 = d2[name]
        m_diff = abs(float(m1) - float(m2)) if m1 and m2 else None
        print(f"{name:10s} | Mass: our={m1} vs orig={m2} (diff={m_diff})")
        if c1 and c2 and c1.strip() != c2.strip():
            print(f"   COM diff: our='{c1.strip()}' vs orig='{c2.strip()}'")
        if i1 and i2 and i1.strip() != i2.strip():
            print(f"   Inertia diff: our='{i1.strip()}' vs orig='{i2.strip()}'")

