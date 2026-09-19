import xml.etree.ElementTree as ET
import os

def update_gains(model_path):
    if not os.path.exists(model_path):
        return
    print(f"Updating gains in {model_path}...")
    tree = ET.parse(model_path)
    root = tree.getroot()
    
    controllers = root.findall('.//HeadNeckNeuralController')
    for controller in controllers:
        kp_node = controller.find('Kp_task')
        if kp_node is not None:
            kp_node.text = '100.0'
            print("Changed Kp_task to 100.0")
            
        kd_node = controller.find('Kd_task')
        if kd_node is not None:
            kd_node.text = '10.0'
            print("Changed Kd_task to 10.0")
            
    tree.write(model_path, encoding='utf-8', xml_declaration=True)
    print("Done.")

if __name__ == '__main__':
    file1 = r'D:\Akademik\PhD_Thesis\OpenSim\Proprioception_Plugin_Opensim4_6_Explicit\osim_files\HYOID_HeadNeckNeural_Model.osim'
    file2 = r'D:\Akademik\PhD_Thesis\OpenSim\Proprioception_Plugin_Opensim4_6_Explicit\osim_files\HYOID_HeadNeckNeural_Model_WithLimits.osim'
    update_gains(file1)
    update_gains(file2)

