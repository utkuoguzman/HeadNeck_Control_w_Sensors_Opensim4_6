import xml.etree.ElementTree as ET

tree = ET.parse('osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim')
root = tree.getroot()

print("--- SpringGeneralizedForce elements ---")
for s in root.findall('.//SpringGeneralizedForce'):
    name = s.attrib.get('name')
    coord = s.findtext('coordinate')
    stiff = s.findtext('stiffness')
    rest = s.findtext('rest_length')
    visc = s.findtext('viscosity')
    applies = s.findtext('appliesForce')
    disabled = s.findtext('isDisabled')
    print(f"Name: {name}, Coord: {coord}, Stiffness: {stiff}, Rest: {rest}, Viscosity: {visc}, appliesForce: {applies}, isDisabled: {disabled}")

print("\n--- BushingForce elements ---")
for b in root.findall('.//BushingForce'):
    name = b.attrib.get('name')
    applies = b.findtext('appliesForce')
    body1 = b.findtext('body_1')
    body2 = b.findtext('body_2')
    rot_stiff = b.findtext('rotational_stiffness')
    trans_stiff = b.findtext('translational_stiffness')
    rot_damp = b.findtext('rotational_damping')
    trans_damp = b.findtext('translational_damping')
    print(f"Name: {name}, Body1: {body1}, Body2: {body2}, appliesForce: {applies}")
    print(f"   rot_stiff: {rot_stiff}")
    print(f"   trans_stiff: {trans_stiff}")
    print(f"   rot_damp: {rot_damp}")

print("\n--- CoordinateCouplerConstraint elements ---")
for c in root.findall('.//CoordinateCouplerConstraint'):
    name = c.attrib.get('name')
    dep = c.findtext('dependent_coordinate_name')
    indep = c.findtext('independent_coordinate_names')
    is_en = c.findtext('isDisabled')
    func = c.find('.//PolynomialFunction')
    coeff = func.findtext('coefficients') if func is not None else None
    print(f"Name: {name}, Dep: {dep}, Indep: {indep}, isDisabled: {is_en}, PolyCoeff: {coeff}")

print("\n--- CoordinateLimitForce elements ---")
for clf in root.findall('.//CoordinateLimitForce'):
    name = clf.attrib.get('name')
    coord = clf.findtext('coordinate')
    up_stiff = clf.findtext('upper_stiffness')
    up_lim = clf.findtext('upper_limit')
    low_stiff = clf.findtext('lower_stiffness')
    low_lim = clf.findtext('lower_limit')
    damping = clf.findtext('damping')
    trans = clf.findtext('transition_region')
    applies = clf.findtext('appliesForce')
    disabled = clf.findtext('isDisabled')
    print(f"Name: {name}, Coord: {coord}, UpLim: {up_lim}, LowLim: {low_lim}, UpStiff: {up_stiff}, LowStiff: {low_stiff}, Damp: {damping}, Trans: {trans}, applies: {applies}, isDisabled: {disabled}")

