import opensim as osim
import os

osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), r'proprioception_plugin\build\Release\osimMillard12EqWithAff.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), r'HeadNeckNeuralController_plugin\build\Release\osimHeadNeckNeural.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), r'FixationController_plugin\build\Release\osimFixation.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), r'vestibular_plugin\build\Release\osimVestibular.dll'))

model_path = r'osim_files\HYOID_HeadNeckNeural_Model_Detailed.osim'
model = osim.Model(model_path)
skull = model.getBodySet().get("skull")

color_map = {
    "Right_Anterior_Cyl": [1.0, 0.0, 0.0],
    "Right_Posterior_Cyl": [0.0, 0.0, 1.0],
    "Right_Horizontal_Cyl": [0.0, 1.0, 0.0],
    "Left_Anterior_Cyl": [1.0, 0.0, 0.0],
    "Left_Posterior_Cyl": [0.0, 0.0, 1.0],
    "Left_Horizontal_Cyl": [0.0, 1.0, 0.0]
}

# Iterate through components and fix color
for comp in model.getComponentsList():
    name = comp.getName()
    if name in color_map:
        cyl = osim.Cylinder.safeDownCast(comp)
        if cyl:
            c = color_map[name]
            cyl.upd_Appearance().set_color(osim.Vec3(c[0], c[1], c[2]))
            cyl.upd_Appearance().set_opacity(1.0)
            
model.printToXML(model_path)
print("Cylinder colors successfully applied!")
