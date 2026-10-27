import xml.etree.ElementTree as ET

tree = ET.parse('osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim')
root = tree.getroot()

print("Root tag:", root.tag)

# Check ForceSet
forces = root.findall('.//ForceSet/objects/*')
print(f"Total objects in ForceSet: {len(forces)}")
force_types = {}
for f in forces:
    force_types[f.tag] = force_types.get(f.tag, 0) + 1
for tag, count in force_types.items():
    print(f"  {tag}: {count}")

# Check non-muscle forces in detail
print("\nNon-muscle forces in ForceSet:")
for f in forces:
    if f.tag != 'Millard2012EquilibriumMuscle' and f.tag != 'Thelen2003Muscle':
        name = f.attrib.get('name')
        applies = f.findtext('appliesForce')
        is_en = f.findtext('isDisabled')
        print(f"  [{f.tag}] name='{name}', appliesForce={applies}, isDisabled={is_en}")
        # Print key parameters
        for child in f:
            if any(k in child.tag.lower() for k in ['stiffness', 'damping', 'limit', 'torque', 'force']):
                print(f"      {child.tag}: {child.text}")

# Check Actuators
actuators = root.findall('.//ActuatorSet/objects/*')
print(f"\nTotal objects in ActuatorSet: {len(actuators)}")
for a in actuators:
    print(f"  {a.tag}: {a.attrib.get('name')}")

# Check Constraints
constraints = root.findall('.//ConstraintSet/objects/*')
print(f"\nTotal objects in ConstraintSet: {len(constraints)}")
for c in constraints:
    print(f"  {c.tag}: {c.attrib.get('name')}")

# Check ContactGeometry
contacts = root.findall('.//ContactGeometrySet/objects/*')
print(f"\nTotal objects in ContactGeometrySet: {len(contacts)}")
for c in contacts:
    print(f"  {c.tag}: {c.attrib.get('name')}")

# Check Joint Limits and Coordinate Limits
print("\nCoordinates:")
coords = root.findall('.//Coordinate')
print(f"Total Coordinates: {len(coords)}")
for c in coords:
    name = c.attrib.get('name')
    clamped = c.findtext('clamped')
    range_val = c.findtext('range')
    locked = c.findtext('locked')
    print(f"  Coord: {name}, clamped={clamped}, locked={locked}, range={range_val}")

