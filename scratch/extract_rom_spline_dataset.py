import opensim as osim
import os
import math
import json
import numpy as np
from scipy.interpolate import CubicSpline

print("Loading OpenSim libraries...")
osim.Logger.setLevelString("Warn")
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'proprioception_plugin/build/Release/osimMillard12EqWithAff.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'FixationController_plugin/build/Release/osimFixation.dll'))

model = osim.Model("osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim")
state = model.initSystem()

cp1 = model.getCoordinateSet().get("pitch1")
cp2 = model.getCoordinateSet().get("pitch2")

skull = model.getBodySet().get("skull")
muscles = model.getMuscles()

# Filter active neck muscles (excluding eyes and hyoid)
neck_muscle_indices = []
neck_muscle_names = []
for i in range(muscles.getSize()):
    mname = muscles.get(i).getName()
    if mname.startswith("r_") or mname.startswith("l_"):
        continue # eye muscles
    neck_muscle_indices.append(i)
    neck_muscle_names.append(mname)

print(f"Sampling full ROM for {len(neck_muscle_names)} active neck muscles across pitch [-40 deg, +50 deg]...")

# 91 pitch knot points from -40 deg to +50 deg in 1 deg steps
pitch_deg_knots = np.linspace(-40.0, 50.0, 91)
pitch_rad_knots = np.radians(pitch_deg_knots)

# Coordinated cervical rhythm: 1/3 lower neck (pitch1), 2/3 upper neck (pitch2)
data_x_com = []
data_y_com = []
data_M_eff = []
data_G_grav = []
data_muscle_lengths = {mname: [] for mname in neck_muscle_names}
data_muscle_ma_p1 = {mname: [] for mname in neck_muscle_names}
data_muscle_ma_p2 = {mname: [] for mname in neck_muscle_names}

matter = model.getMatterSubsystem()

for p_deg, p_rad in zip(pitch_deg_knots, pitch_rad_knots):
    p1 = p_rad * (1.0 / 3.0)
    p2 = p_rad * (2.0 / 3.0)
    
    cp1.setValue(state, p1)
    cp2.setValue(state, p2)
    model.realizePosition(state)
    
    # Skull COM
    com = skull.findStationLocationInGround(state, skull.getMassCenter())
    data_x_com.append(float(com.get(0)))
    data_y_com.append(float(com.get(1)))
    
    # Gravitational restoring torque: tau_grav = m_head * g * x_com
    # Skull mass is 4.300 kg
    m_head = 4.300
    g = 9.80665
    tau_grav = m_head * g * com.get(0)
    data_G_grav.append(float(tau_grav))
    
    # Muscle lengths and moment arms
    for idx, mname in zip(neck_muscle_indices, neck_muscle_names):
        m = muscles.get(idx)
        l_mt = float(m.getLength(state))
        ma1 = float(m.computeMomentArm(state, cp1))
        ma2 = float(m.computeMomentArm(state, cp2))
        
        data_muscle_lengths[mname].append(l_mt)
        data_muscle_ma_p1[mname].append(ma1)
        data_muscle_ma_p2[mname].append(ma2)

print("Offline sampling completed successfully! Fitting cubic splines...")

# Build spline dictionary
spline_dataset = {
    "pitch_deg_knots": pitch_deg_knots.tolist(),
    "pitch_rad_knots": pitch_rad_knots.tolist(),
    "skull_com_x": data_x_com,
    "skull_com_y": data_y_com,
    "G_grav_Nm": data_G_grav,
    "series_bushing_K_Nm_rad": 1.724,
    "series_bushing_B_Nms_rad": 0.172,
    "muscle_names": neck_muscle_names,
    "muscle_lengths": data_muscle_lengths,
    "muscle_moment_arms_pitch1": data_muscle_ma_p1,
    "muscle_moment_arms_pitch2": data_muscle_ma_p2
}

output_path = "logs/rom_plant_spline_dataset.json"
os.makedirs("logs", exist_ok=True)
with open(output_path, "w", encoding="utf-8") as f:
    json.dump(spline_dataset, f, indent=2)

file_size_kb = os.path.getsize(output_path) / 1024.0
print(f"Exported {output_path} successfully ({file_size_kb:.1f} KB)!")

