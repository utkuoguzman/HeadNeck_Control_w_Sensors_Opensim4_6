import opensim as osim
import os

osim.Logger.setLevelString("Warn")
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'proprioception_plugin/build/Release/osimMillard12EqWithAff.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'FixationController_plugin/build/Release/osimFixation.dll'))

model = osim.Model("osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim")
state = model.initSystem()

print("\n--- All Bushing Forces ---")
total_k_pitch = 0.0
total_b_pitch = 0.0
for i in range(model.getForceSet().getSize()):
    f = model.getForceSet().get(i)
    bf = osim.BushingForce.safeDownCast(f)
    if bf:
        k_rot = bf.get_rotational_stiffness()
        b_rot = bf.get_rotational_damping()
        # In OpenSim coordinates: X is sagittal / roll, Y is yaw, Z is pitch (transverse)
        # Let's inspect the 3 components:
        kx, ky, kz = k_rot.get(0), k_rot.get(1), k_rot.get(2)
        bx, by, bz = b_rot.get(0), b_rot.get(1), b_rot.get(2)
        print(f"  {bf.getName():<22} | K_rot: [{kx:>5.1f}, {ky:>5.1f}, {kz:>5.1f}] Nm/rad | B_rot: [{bx:>5.2f}, {by:>5.2f}, {bz:>5.2f}] Nms/rad")
        total_k_pitch += kz
        total_b_pitch += bz

print(f"\nTotal Bushing Pitch Stiffness sum: {total_k_pitch:.2f} Nm/rad")
print(f"Total Bushing Pitch Damping sum:   {total_b_pitch:.2f} Nms/rad")
