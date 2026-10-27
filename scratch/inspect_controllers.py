import xml.etree.ElementTree as ET

tree = ET.parse('osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim')
root = tree.getroot()

ctrls = root.findall('.//ControllerSet/objects/*')
print(f'Total objects in ControllerSet: {len(ctrls)}')
for c in ctrls:
    print(f"  {c.tag}: {c.attrib.get('name')}")
    for child in c:
        if child.text and child.text.strip():
            print(f"      {child.tag}: {child.text.strip()}")

