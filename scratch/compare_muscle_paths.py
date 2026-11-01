import xml.etree.ElementTree as ET

tree1 = ET.parse('osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim')
tree2 = ET.parse(r'D:\Akademik\PhD_Thesis\OpenSim\HYOID\HYOID_1.2_ScaledStrenght_UpdatedInertia.osim')

def get_muscle_paths(tree, is_v4=False):
    m_dict = {}
    for m in tree.findall('.//ForceSet/objects/*'):
        if 'Muscle' in m.tag:
            name = m.attrib.get('name')
            pts = []
            for gp in m.findall('.//GeometryPath//PathPoint'):
                loc = gp.findtext('location')
                if is_v4:
                    frame = gp.findtext('socket_parent_frame', '')
                    body = frame.split('/')[-1] if frame else ''
                else:
                    body = gp.findtext('body', '')
                pts.append((body, loc.strip() if loc else ''))
            m_dict[name] = pts
    return m_dict

p1 = get_muscle_paths(tree1, is_v4=True)
p2 = get_muscle_paths(tree2, is_v4=False)

diff_count = 0
for m in sorted(list(p2.keys())):
    if m in p1:
        pts1 = p1[m]
        pts2 = p2[m]
        if len(pts1) != len(pts2):
            print(f"Muscle {m}: path point count differs ({len(pts1)} vs {len(pts2)})")
            diff_count += 1
        else:
            for i, (pt1, pt2) in enumerate(zip(pts1, pts2)):
                if pt1[0] != pt2[0]:
                    print(f"Muscle {m} pt {i}: body differs ('{pt1[0]}' vs '{pt2[0]}')")
                    diff_count += 1
                    break
                l1 = [float(x) for x in pt1[1].split()]
                l2 = [float(x) for x in pt2[1].split()]
                max_d = max(abs(a - b) for a, b in zip(l1, l2))
                if max_d > 1e-4:
                    print(f"Muscle {m} pt {i}: location differs by {max_d:.6f} m")
                    diff_count += 1
                    break

print(f"Total muscles with different geometry paths: {diff_count} of {len(p2)}")

