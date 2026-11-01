import xml.etree.ElementTree as ET

tree1 = ET.parse('osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim')
tree2 = ET.parse(r'D:\Akademik\PhD_Thesis\OpenSim\HYOID\HYOID_1.2_ScaledStrenght_UpdatedInertia.osim')

def get_bushings(tree):
    b_dict = {}
    for b in tree.findall('.//BushingForce'):
        name = b.attrib.get('name')
        rot_k = b.findtext('rotational_stiffness')
        trans_k = b.findtext('translational_stiffness')
        rot_d = b.findtext('rotational_damping')
        trans_d = b.findtext('translational_damping')
        b_dict[name] = (rot_k, trans_k, rot_d, trans_d)
    return b_dict

b1 = get_bushings(tree1)
b2 = get_bushings(tree2)

print(f"Bushing count: Our={len(b1)} vs Orig={len(b2)}")
for name in b2:
    if name in b1:
        rk1, tk1, rd1, td1 = b1[name]
        rk2, tk2, rd2, td2 = b2[name]
        print(f"\nBushing: {name}")
        print(f"  rot_stiff:  our='{rk1.strip()}' vs orig='{rk2.strip()}' (Match: {rk1.strip() == rk2.strip()})")
        print(f"  trans_stiff: our='{tk1.strip()}' vs orig='{tk2.strip()}' (Match: {tk1.strip() == tk2.strip()})")
        print(f"  rot_damp:   our='{rd1.strip()}' vs orig='{rd2.strip()}' (Match: {rd1.strip() == rd2.strip()})")
        print(f"  trans_damp: our='{td1.strip()}' vs orig='{td2.strip()}' (Match: {td1.strip() == td2.strip()})")

