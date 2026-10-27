import opensim as osim
import os
import math
import numpy as np

osim.Logger.setLevelString("Warn")
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'proprioception_plugin/build/Release/osimMillard12EqWithAff.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'FixationController_plugin/build/Release/osimFixation.dll'))

model = osim.Model("osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim")
state = model.initSystem()

# Let's inspect methods on model and matter
matter = model.getMatterSubsystem()

# Check coordinates
cp1 = model.getCoordinateSet().get("pitch1")
cp2 = model.getCoordinateSet().get("pitch2")

print(f"cp1 default value: {math.degrees(cp1.getValue(state)):.2f} deg")
print(f"cp2 default value: {math.degrees(cp2.getValue(state)):.2f} deg")

# Test sweep across pitch2 from -30 deg to +30 deg
pitch_vals = np.linspace(-30, 30, 7)
print("\n--- Sweeping pitch2 across ROM ---")
for p_deg in pitch_vals:
    p_rad = math.radians(p_deg)
    cp2.setValue(state, p_rad)
    model.realizeDynamics(state)
    
    # Skull COM position
    skull = model.getBodySet().get("skull")
    com = skull.findStationLocationInGround(state, skull.getMassCenter())
    
    # Sample 3 major muscles (Sternomastoid, Splenius, SemiSpinalis)
    m_scm = model.getMuscles().get("stern_mast")
    m_splen = model.getMuscles().get("splen_cap_sklc6")
    m_semi = model.getMuscles().get("semi_cap_sklc5")
    
    l_scm = m_scm.getLength(state) * 1000.0
    r_scm = (m_scm.computeMomentArm(state, cp1) + m_scm.computeMomentArm(state, cp2)) * 1000.0
    
    l_splen = m_splen.getLength(state) * 1000.0
    r_splen = (m_splen.computeMomentArm(state, cp1) + m_splen.computeMomentArm(state, cp2)) * 1000.0
    
    l_semi = m_semi.getLength(state) * 1000.0
    r_semi = (m_semi.computeMomentArm(state, cp1) + m_semi.computeMomentArm(state, cp2)) * 1000.0
    
    print(f"Pitch: {p_deg:>5.1f}° | Skull COM: x={com.get(0)*100:>5.2f}cm, y={com.get(1)*100:>5.2f}cm | SCM L={l_scm:>5.1f}mm r={r_scm:>5.1f}mm | Splen L={l_splen:>5.1f}mm r={r_splen:>5.1f}mm | Semi L={l_semi:>5.1f}mm r={r_semi:>5.1f}mm")
