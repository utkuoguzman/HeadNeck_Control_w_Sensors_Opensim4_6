import opensim as osim
import math

osim.LoadOpenSimLibrary(r'proprioception_plugin\build\Release\osimMillard12EqWithAff.dll')
osim.LoadOpenSimLibrary(r'HeadNeckNeuralController_plugin\build\Release\osimHeadNeckNeural.dll')
osim.LoadOpenSimLibrary(r'FixationController_plugin\build\Release\osimFixation.dll')
osim.LoadOpenSimLibrary(r'vestibular_plugin\build\Release\osimVestibular.dll')

model = osim.Model(r'osim_files\HYOID_HeadNeckNeural_Model_Detailed.osim')
state = model.initSystem()

skull = model.getBodySet().get("skull")
R_head_to_world = skull.getTransformInGround(state).R()
q_simtk = osim.Rotation(R_head_to_world).convertRotationToQuaternion()

bq0 = q_simtk.get(0)
bq1 = q_simtk.get(1)
bq2 = q_simtk.get(2)
bq3 = q_simtk.get(3)

roll = math.atan2(2.0*(bq0*bq1 + bq2*bq3), 1.0 - 2.0*(bq1*bq1 + bq2*bq2))
yaw = math.asin(max(-1.0, min(1.0, 2.0*(bq0*bq2 - bq3*bq1))))
pitch = math.atan2(2.0*(bq0*bq3 + bq1*bq2), 1.0 - 2.0*(bq2*bq2 + bq3*bq3))

print(f"Baseline Roll:  {math.degrees(roll):.2f} degrees")
print(f"Baseline Yaw:   {math.degrees(yaw):.2f} degrees")
print(f"Baseline Pitch: {math.degrees(pitch):.2f} degrees")
