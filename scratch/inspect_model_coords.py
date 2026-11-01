import opensim as osim
import os
import math

osim.Logger.setLevelString("Warn")
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'proprioception_plugin/build/Release/osimMillard12EqWithAff.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'FixationController_plugin/build/Release/osimFixation.dll'))

model = osim.Model("osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim")
state = model.initSystem()

print("--- Coordinates ---")
for i in range(model.getCoordinateSet().getSize()):
    c = model.getCoordinateSet().get(i)
    val_deg = math.degrees(c.getValue(state))
    min_deg = math.degrees(c.getRangeMin())
    max_deg = math.degrees(c.getRangeMax())
    print(f"  {c.getName():<15} | Default: {val_deg:>6.1f} deg | Range: [{min_deg:>6.1f}, {max_deg:>6.1f}] deg | Locked: {c.getLocked(state)}")

print(f"\n--- Muscles ({model.getMuscles().getSize()} total) ---")
for i in range(model.getMuscles().getSize()):
    m = model.getMuscles().get(i)
    print(f"  {i+1:>2}: {m.getName():<25} | MaxIsoForce: {m.getMaxIsometricForce():>7.1f} N | OptFiberLen: {m.getOptimalFiberLength()*1000:>5.1f} mm")

print(f"\n--- Bodies ({model.getBodySet().getSize()} total) ---")
for i in range(model.getBodySet().getSize()):
    b = model.getBodySet().get(i)
    print(f"  {b.getName():<20} | Mass: {b.getMass():>6.3f} kg")
