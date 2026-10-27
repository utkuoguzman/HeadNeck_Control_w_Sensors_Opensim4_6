import opensim as osim
import os

print("Auditing OpenSim model and forces...")
osim.Logger.setLevelString("Warn")
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'proprioception_plugin/build/Release/osimMillard12EqWithAff.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'FixationController_plugin/build/Release/osimFixation.dll'))

model = osim.Model("osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim")
state = model.initSystem()

# 1. Audit ForceSet
forces = model.getForceSet()
print(f"Total forces in ForceSet: {forces.getSize()}")
force_types = {}
for i in range(forces.getSize()):
    f = forces.get(i)
    cname = f.getConcreteClassName()
    force_types[cname] = force_types.get(cname, 0) + 1
print("Force types breakdown:", force_types)

# 2. Check for CoordinateLimitForce or Springs
for i in range(forces.getSize()):
    f = forces.get(i)
    cname = f.getConcreteClassName()
    if cname != "Millard2012EquilibriumMuscleWithAfferents":
        print(f"  Non-muscle force #{i}: Name='{f.getName()}', Type='{cname}'")

# 3. Audit Controllers
controllers = model.getControllerSet()
print(f"\nTotal controllers in ControllerSet: {controllers.getSize()}")
for i in range(controllers.getSize()):
    c = controllers.get(i)
    print(f"  Controller #{i}: Name='{c.getName()}', Type='{c.getConcreteClassName()}', Enabled={c.get_enabled()}")

# 4. Audit Coordinates (locked vs prescribed vs free)
coords = model.getCoordinateSet()
print(f"\nTotal coordinates: {coords.getSize()}")
for i in range(coords.getSize()):
    coord = coords.get(i)
    print(f"  Coord '{coord.getName()}': Locked={coord.get_locked()}, DefaultIsPrescribed={coord.getDefaultIsPrescribed()}, MotionType={coord.getMotionType()}")

