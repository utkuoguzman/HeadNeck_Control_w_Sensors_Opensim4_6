import xml.etree.ElementTree as ET

def add_limits_to_xml(model_path, output_path):
    print("Parsing %s..." % model_path)
    tree = ET.parse(model_path)
    root = tree.getroot()
    
    forces_node = root.find('.//ForceSet/objects')
    if forces_node is None:
        print("Could not find ForceSet/objects node!")
        return

    limits = {
        'pitch1': {'min': -0.5235987755982988, 'max': 0.5235987755982988}, # +/- 30 deg
        'pitch2': {'min': -0.5235987755982988, 'max': 0.5235987755982988}, # +/- 30 deg
        'roll1': {'min': -0.3490658503988659, 'max': 0.3490658503988659},  # +/- 20 deg
        'roll2': {'min': -0.3490658503988659, 'max': 0.3490658503988659},  # +/- 20 deg
        'yaw1': {'min': -0.6108652381980153, 'max': 0.6108652381980153},   # +/- 35 deg
        'yaw2': {'min': -0.6108652381980153, 'max': 0.6108652381980153}    # +/- 35 deg
    }
    
    for coord_name, bounds in limits.items():
        print("Adding physical limits to %s: %s to %s rad" % (coord_name, bounds['min'], bounds['max']))
        
        limit_force = ET.SubElement(forces_node, 'CoordinateLimitForce', {'name': coord_name + "_physical_limit"})
        
        # Upper limit
        upper_limit = ET.SubElement(limit_force, 'upper_limit')
        upper_limit.text = str(bounds['max'])
        upper_stiffness = ET.SubElement(limit_force, 'upper_stiffness')
        upper_stiffness.text = '200.0'
        
        # Lower limit
        lower_limit = ET.SubElement(limit_force, 'lower_limit')
        lower_limit.text = str(bounds['min'])
        lower_stiffness = ET.SubElement(limit_force, 'lower_stiffness')
        lower_stiffness.text = '200.0'
        
        # Damping and transition
        damping = ET.SubElement(limit_force, 'damping')
        damping.text = '20.0'
        transition = ET.SubElement(limit_force, 'transition')
        transition.text = '5.0'
        
        # coordinate
        coordinate = ET.SubElement(limit_force, 'coordinate')
        coordinate.text = coord_name
        
    print("Saving fixed model to %s..." % output_path)
    tree.write(output_path, encoding='utf-8', xml_declaration=True)
    print("Done! Open the fixed model in OpenSim GUI.")

if __name__ == '__main__':
    model_file = r'osim_files\HYOID_HeadNeckNeural_Model.osim'
    output_file = r'osim_files\HYOID_HeadNeckNeural_Model_WithLimits.osim'
    add_limits_to_xml(model_file, output_file)
