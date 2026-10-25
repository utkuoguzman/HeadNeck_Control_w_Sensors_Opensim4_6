import opensim as osim
import os

# Load required plugins
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), r'proprioception_plugin\build\Release\osimMillard12EqWithAff.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), r'HeadNeckNeuralController_plugin\build\Release\osimHeadNeckNeural.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), r'FixationController_plugin\build\Release\osimFixation.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), r'vestibular_plugin\build\Release\osimVestibular.dll'))

model_path = r'osim_files\HYOID_HeadNeckNeural_Model_Detailed.osim'
model = osim.Model(model_path)
skull = model.getBodySet().get("skull")

# Canal centers
right_center = osim.Vec3(0.01, 0.035, 0.045)
left_center = osim.Vec3(0.01, 0.035, -0.045)

# Vectors (X, Y, Z) = (Roll, Yaw, Pitch) ?? No, (Roll=X, Yaw=Y, Pitch=Z)
# Wait, the C++ code says:
# B_R(0,0) = -0.589; B_R(0,1) = -0.177; B_R(0,2) =  0.788; // RA -> X, Y, Z
canals = [
    {"name": "Right_Anterior_Canal",  "center": right_center, "dir": [-0.589, -0.177,  0.788], "color": [1.0, 0.0, 0.0]}, # Red
    {"name": "Right_Posterior_Canal", "center": right_center, "dir": [-0.694, -0.270, -0.667], "color": [0.0, 0.0, 1.0]}, # Blue
    {"name": "Right_Horizontal_Canal","center": right_center, "dir": [-0.323,  0.946, -0.038], "color": [0.0, 1.0, 0.0]}, # Green
    
    {"name": "Left_Anterior_Canal",   "center": left_center,  "dir": [-0.589, -0.177, -0.788], "color": [1.0, 0.0, 0.0]}, # Red
    {"name": "Left_Posterior_Canal",  "center": left_center,  "dir": [-0.694, -0.270,  0.667], "color": [0.0, 0.0, 1.0]}, # Blue
    {"name": "Left_Horizontal_Canal", "center": left_center,  "dir": [-0.323,  0.946,  0.038], "color": [0.0, 1.0, 0.0]}  # Green
]

for c in canals:
    arrow = osim.Arrow()
    arrow.setName(c["name"])
    
    # Set appearance
    app = arrow.get_Appearance()
    app.set_color(osim.Vec3(c["color"][0], c["color"][1], c["color"][2]))
    
    # Set properties
    # Arrow direction must be a Vec3
    direction = osim.Vec3(c["dir"][0], c["dir"][1], c["dir"][2])
    arrow.set_direction(direction)
    arrow.set_length(0.04) # 4 cm long
    
    # Set frame (wait, Arrow start_point is relative to the frame it's attached to)
    # But usually attached_geometry inherits the frame's origin. 
    # Arrow doesn't natively have 'set_start_point' exposed properly sometimes, or it doesn't move relative to the frame origin properly unless we use an OffsetFrame.
    # So let's attach the Arrow to a PhysicalOffsetFrame
    
    offset_frame = osim.PhysicalOffsetFrame()
    offset_frame.setName(c["name"] + "_frame")
    offset_frame.setParentFrame(skull)
    offset_frame.set_translation(c["center"])
    
    # Attach arrow to the offset frame
    offset_frame.attachGeometry(arrow)
    
    # Add frame to skull
    skull.addComponent(offset_frame)

model.finalizeConnections()
model.printToXML(model_path)
print("Vestibular arrows successfully injected into the .osim file!")
