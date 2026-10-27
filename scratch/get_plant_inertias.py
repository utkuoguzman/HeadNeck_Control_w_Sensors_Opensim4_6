import os
import opensim as osim

osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'proprioception_plugin/build/Release/osimMillard12EqWithAff.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'FixationController_plugin/build/Release/osimFixation.dll'))

model = osim.Model("osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim")
state = model.initSystem()

skull = model.getBodySet().get("skull")
print(f"Skull Mass: {skull.getMass()} kg")
print(f"Skull Inertia repr: {skull.get_inertia().toString()}")

for i in range(model.getBodySet().getSize()):
    b = model.getBodySet().get(i)
    if "C" in b.getName():
        print(f"  {b.getName()}: mass={b.getMass():.4f} kg, inertia={b.get_inertia().toString()}")

