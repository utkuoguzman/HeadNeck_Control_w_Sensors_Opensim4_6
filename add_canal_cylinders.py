import opensim as osim
import os
import math

osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), r'proprioception_plugin\build\Release\osimMillard12EqWithAff.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), r'HeadNeckNeuralController_plugin\build\Release\osimHeadNeckNeural.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), r'FixationController_plugin\build\Release\osimFixation.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), r'vestibular_plugin\build\Release\osimVestibular.dll'))

model_path = r'osim_files\HYOID_HeadNeckNeural_Model_Detailed.osim'
model = osim.Model(model_path)
skull = model.getBodySet().get("skull")

# Clean up old Arrow frames if they exist
components_to_remove = []
for comp in skull.getComponentsList():
    if "_Canal_frame" in comp.getName():
        components_to_remove.append(comp.getName())

# OpenSim python API doesn't easily let you remove components by python list,
# so we will just create them with a new name if needed. Or we can just leave the old ones, they are invisible anyway.
# Actually, let's just make new frames named "_VisualFrame"

right_center = osim.Vec3(0.01, 0.035, 0.045)
left_center = osim.Vec3(0.01, 0.035, -0.045)

canals = [
    {"name": "Right_Anterior",  "center": right_center, "dir": [-0.589, -0.177,  0.788], "color": [1.0, 0.0, 0.0]},
    {"name": "Right_Posterior", "center": right_center, "dir": [-0.694, -0.270, -0.667], "color": [0.0, 0.0, 1.0]},
    {"name": "Right_Horizontal","center": right_center, "dir": [-0.323,  0.946, -0.038], "color": [0.0, 1.0, 0.0]},
    
    {"name": "Left_Anterior",   "center": left_center,  "dir": [-0.589, -0.177, -0.788], "color": [1.0, 0.0, 0.0]},
    {"name": "Left_Posterior",  "center": left_center,  "dir": [-0.694, -0.270,  0.667], "color": [0.0, 0.0, 1.0]},
    {"name": "Left_Horizontal", "center": left_center,  "dir": [-0.323,  0.946,  0.038], "color": [0.0, 1.0, 0.0]}
]

for c in canals:
    cyl = osim.Cylinder(0.002, 0.02) # Radius 2mm, Half-Length 2cm
    cyl.setName(c["name"] + "_Cyl")
    
    app = cyl.get_Appearance()
    app.set_color(osim.Vec3(c["color"][0], c["color"][1], c["color"][2]))
    app.set_opacity(1.0)
    
    # Calculate quaternion to rotate [0,1,0] to dir
    v = c["dir"]
    # u x v where u = [0,1,0]
    w = [v[2], 0.0, -v[0]]
    d = v[1]
    
    q_w = 1.0 + d
    q_x = w[0]
    q_y = w[1]
    q_z = w[2]
    
    # Normalize
    norm = math.sqrt(q_w*q_w + q_x*q_x + q_y*q_y + q_z*q_z)
    q = osim.Quaternion(q_w/norm, q_x/norm, q_y/norm, q_z/norm)
    rot = osim.Rotation(q)
    
    # Offset frame
    offset_frame = osim.PhysicalOffsetFrame()
    offset_frame.setName(c["name"] + "_VisualFrame")
    offset_frame.setParentFrame(skull)
    
    # Shift center by half-length along the dir so it sprouts FROM the center
    trans = osim.Vec3(c["center"].get(0) + v[0]*0.02, 
                      c["center"].get(1) + v[1]*0.02, 
                      c["center"].get(2) + v[2]*0.02)
                      
    offset_frame.set_translation(trans)
    
    # Set orientation via Euler angles extracted from rotation
    euler = rot.convertRotationToBodyFixedXYZ()
    offset_frame.set_orientation(euler)
    
    offset_frame.attachGeometry(cyl)
    skull.addComponent(offset_frame)

model.finalizeConnections()
model.printToXML(model_path)
print("Vestibular Cylinders successfully injected!")
