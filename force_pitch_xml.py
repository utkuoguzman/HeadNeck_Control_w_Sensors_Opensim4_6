import xml.etree.ElementTree as ET

osim_file = r"osim_files\HYOID_HeadNeckNeural_Model_WithLimits.osim"
tree = ET.parse(osim_file)
root = tree.getroot()

for ctrl in root.iter("HeadNeckNeuralController"):
    dp = ctrl.find("desired_pitch")
    if dp is not None:
        dp.text = "0.0"
    
    dr = ctrl.find("desired_roll")
    if dr is not None:
        dr.text = "0.7"  # 40 degrees roll

    dy = ctrl.find("desired_yaw")
    if dy is not None:
        dy.text = "0.0"
        
tree.write(osim_file)
print("Set desired_pitch=0.0 and desired_roll=0.7 in XML.")
