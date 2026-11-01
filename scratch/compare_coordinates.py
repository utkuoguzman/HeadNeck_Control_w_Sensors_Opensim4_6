import xml.etree.ElementTree as ET

tree1 = ET.parse('osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim')
tree2 = ET.parse(r'D:\Akademik\PhD_Thesis\OpenSim\HYOID\HYOID_1.2_ScaledStrenght_UpdatedInertia.osim')

def get_coords(tree):
    c_dict = {}
    for c in tree.findall('.//Coordinate'):
        name = c.attrib.get('name')
        dval = c.findtext('default_value')
        dspeed = c.findtext('default_speed_value')
        crange = c.findtext('range')
        clamped = c.findtext('clamped')
        locked = c.findtext('locked')
        c_dict[name] = (dval, dspeed, crange, clamped, locked)
    return c_dict

c1 = get_coords(tree1)
c2 = get_coords(tree2)

print(f"Coordinate count: Our={len(c1)} vs Orig={len(c2)}")
for name in sorted(list(c2.keys())):
    if name in c1:
        v1, s1, r1, cl1, lk1 = c1[name]
        v2, s2, r2, cl2, lk2 = c2[name]
        diffs = []
        if (v1 or '0').strip() != (v2 or '0').strip():
            diffs.append(f"default_val: our={v1} vs orig={v2}")
        if (r1 or '').strip() != (r2 or '').strip():
            diffs.append(f"range: our={r1} vs orig={r2}")
        if (cl1 or '').strip() != (cl2 or '').strip():
            diffs.append(f"clamped: our={cl1} vs orig={cl2}")
        if (lk1 or '').strip() != (lk2 or '').strip():
            diffs.append(f"locked: our={lk1} vs orig={lk2}")
        if diffs:
            print(f"Coord {name:15s}: {', '.join(diffs)}")
        else:
            print(f"Coord {name:15s}: IDENTICAL")

