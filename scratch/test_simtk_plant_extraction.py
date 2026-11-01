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

# Test coordinates
cp1 = model.getCoordinateSet().get("pitch1")
cp2 = model.getCoordinateSet().get("pitch2")

idx1 = model.getCoordinateSet().getIndex("pitch1")
idx2 = model.getCoordinateSet().getIndex("pitch2")
print(f"pitch1 set index: {idx1}, pitch2 set index: {idx2}")

# Check SimTK Mass Matrix
matter = model.getMatterSubsystem()
M = osim.Matrix()
matter.calcM(state, M)
print(f"Mass matrix size: {M.nrow()} x {M.ncol()}")

# Check Forces in Model
forces = model.getForceSet()
print(f"\n--- ForceSet ({forces.getSize()} forces) ---")
bushing_count = 0
for i in range(forces.getSize()):
    f = forces.get(i)
    cname = f.getConcreteClassName()
    if "Bushing" in cname or "CoordinateLimit" in cname:
        bushing_count += 1
        if bushing_count <= 5:
            print(f"  {f.getName():<25} | Type: {cname}")
print(f"Total Bushing/Limit Forces: {bushing_count}")

# Check COM position
skull = model.getBodySet().get("skull")
com_skull = skull.findStationLocationInGround(state, skull.getMassCenter())
print(f"Skull COM in ground: [{com_skull.get(0):.4f}, {com_skull.get(1):.4f}, {com_skull.get(2):.4f}] m")

# Check Muscle moment arms and lengths
muscles = model.getMuscles()
print(f"\nChecking first 5 muscles:")
for i in range(min(5, muscles.getSize())):
    m = muscles.get(i)
    l_mt = m.getLength(state)
    ma_p1 = m.computeMomentArm(state, cp1)
    ma_p2 = m.computeMomentArm(state, cp2)
    print(f"  {m.getName():<22} | Length: {l_mt*1000:>6.2f} mm | MA(p1): {ma_p1*1000:>6.2f} mm | MA(p2): {ma_p2*1000:>6.2f} mm")

print("\nSimTK extraction test successful!")
