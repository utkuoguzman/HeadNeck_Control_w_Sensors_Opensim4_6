import opensim as osim
import os
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'proprioception_plugin/build/Release/osimMillard12EqWithAff.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'FixationController_plugin/build/Release/osimFixation.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'vestibular_plugin/build/Release/osimVestibular.dll'))

model = osim.Model("osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim")
skull = model.getBodySet().get("skull")

rmarker = osim.Marker()
rmarker.setName("Right_Vestibular")
rmarker.setParentFrame(skull)
rmarker.set_location(osim.Vec3(0.01, 0.035, 0.045)) 
model.addMarker(rmarker)

lmarker = osim.Marker()
lmarker.setName("Left_Vestibular")
lmarker.setParentFrame(skull)
lmarker.set_location(osim.Vec3(0.01, 0.035, -0.045))
model.addMarker(lmarker)

model.finalizeConnections()
model.printToXML("osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim")
print("Vestibular markers added successfully!")
