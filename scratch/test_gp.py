import xml.etree.ElementTree as ET

tree = ET.parse('osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim')
m = tree.find('.//Millard12EqMuscleWithAfferents[@name="Geniohyoid"]')
for gp in m.findall('.//GeometryPath//PathPoint')[:2]:
    print('PathPoint tag:', gp.tag)
    for c in gp:
        print(' ', c.tag, c.text)

