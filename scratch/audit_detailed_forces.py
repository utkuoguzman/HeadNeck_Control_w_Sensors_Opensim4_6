import opensim as osim
import os

osim.Logger.setLevelString("Warn")
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'proprioception_plugin/build/Release/osimMillard12EqWithAff.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'FixationController_plugin/build/Release/osimFixation.dll'))

model = osim.Model("osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim")
state = model.initSystem()

print("="*75)
print("1. AUDIT ACTUATORS: Skull_Impact and Skull_Moment")
print("="*75)
forces = model.getForceSet()
act_impact = forces.get("Skull_Impact")
act_moment = forces.get("Skull_Moment")
print(f"Skull_Impact: appliesForce={act_impact.appliesForce(state)}")
print(f"Skull_Moment: appliesForce={act_moment.appliesForce(state)}")

print("\n" + "="*75)
print("2. AUDIT COORDINATE LIMIT FORCES (Upper/Lower stops & stiffness)")
print("="*75)
for name in ["pitch1_physical_limit", "pitch2_physical_limit", "roll1_physical_limit", "roll2_physical_limit", "yaw1_physical_limit", "yaw2_physical_limit"]:
    clf = osim.CoordinateLimitForce.safeDownCast(forces.get(name))
    if clf:
        print(f"Limit '{name}':")
        print(f"  Coord: {clf.get_coordinate()}")
        print(f"  Upper limit: {clf.get_upper_limit():.3f} rad ({clf.get_upper_limit()*57.3:.1f} deg)")
        print(f"  Lower limit: {clf.get_lower_limit():.3f} rad ({clf.get_lower_limit()*57.3:.1f} deg)")
        print(f"  Upper stiffness: {clf.get_upper_stiffness():.1f}")
        print(f"  Lower stiffness: {clf.get_lower_stiffness():.1f}")
        print(f"  Damping:     {clf.get_damping():.2f}")

print("\n" + "="*75)
print("3. AUDIT BUSHING FORCES (Intervertebral disc/ligament stiffness)")
print("="*75)
for i in range(forces.getSize()):
    f = forces.get(i)
    if f.getConcreteClassName() == "BushingForce":
        bf = osim.BushingForce.safeDownCast(f)
        rot_k = bf.get_rotational_stiffness()
        trans_k = bf.get_translational_stiffness()
        print(f"Bushing '{bf.getName()}':")
        print(f"  Rotational stiffness:    [{rot_k.get(0):.1f}, {rot_k.get(1):.1f}, {rot_k.get(2):.1f}] Nm/rad")
        print(f"  Translational stiffness: [{trans_k.get(0):.1f}, {trans_k.get(1):.1f}, {trans_k.get(2):.1f}] N/m")

print("\n" + "="*75)
print("4. AUDIT CONTROLLER #0: Binocular_Fixation_Controller")
print("="*75)
ctrl_fix = model.getControllerSet().get("Binocular_Fixation_Controller")
actuators = ctrl_fix.getActuatorSet()
print(f"Fixation Controller controls {actuators.getSize()} actuators:")
musc_controlled = [actuators.get(i).getName() for i in range(actuators.getSize())]
print("  Actuators controlled:", musc_controlled)

print("\n" + "="*75)
print("5. AUDIT MUSCLE MINIMUM ACTIVATIONS AND BASELINE TONE")
print("="*75)
ctrl_neural = model.getControllerSet().get("VCR_CCR_Controller")
print(f"VCR_CCR_Controller controls {ctrl_neural.getActuatorSet().getSize()} actuators.")
