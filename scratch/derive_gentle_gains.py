import opensim as osim
import numpy as np
import math

osim.LoadOpenSimLibrary('proprioception_plugin/build/Release/osimMillard12EqWithAff.dll')
osim.LoadOpenSimLibrary('HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll')
osim.LoadOpenSimLibrary('FixationController_plugin/build/Release/osimFixation.dll')

m = osim.Model('osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim')
s = m.initSystem()

skull = m.getBodySet().get('skull')
cerv7 = m.getBodySet().get('cerv7')

m_skull = skull.getMass()
# Get CoM locations
p_skull_com_in_ground = skull.findStationLocationInGround(s, skull.getMassCenter())
p_c7_in_ground = cerv7.findStationLocationInGround(s, osim.Vec3(0,0,0))

h = p_skull_com_in_ground.get(1) - p_c7_in_ground.get(1)

# Skull inertias about CoM
I_xx_com = skull.getInertia().getMoments().get(0) # Roll
I_yy_com = skull.getInertia().getMoments().get(1) # Yaw
I_zz_com = skull.getInertia().getMoments().get(2) # Pitch

g = 9.81

# Parallel axis theorem: I_pivot = I_com + m * h^2
# For rotation about C7/T1 neck base:
I_pitch_pivot = I_zz_com + m_skull * (h**2)
I_roll_pivot  = I_xx_com + m_skull * (h**2)
I_yaw_pivot   = I_yy_com # rotation is largely about vertical axis

# Toppling torque stiffness: K_grav = m * g * h
K_grav = m_skull * g * h

print(f"Model Biomechanics Properties:")
print(f"  Head/Skull Mass: {m_skull:.3f} kg")
print(f"  CoM Height above C7/T1 base (h): {h:.3f} m ({h*100:.1f} cm)")
print(f"  Gravitational toppling stiffness (m*g*h): {K_grav:.3f} Nm/rad")
print(f"  Inertia about CoM: Pitch={I_zz_com:.4f}, Roll={I_xx_com:.4f}, Yaw={I_yy_com:.4f} kg*m^2")
print(f"  Inertia about C7/T1 Pivot: Pitch={I_pitch_pivot:.4f}, Roll={I_roll_pivot:.4f}, Yaw={I_yaw_pivot:.4f} kg*m^2\n")

# Closed-loop requirements:
# Settling time t_s = 1.0 second starting from 10 deg (0.1745 rad)
# Standard 2% settling time: t_s = 4.0 / (zeta * omega_n) -> zeta * omega_n = 4.0
# Let's evaluate for:
# Case 1: Critically damped (zeta = 1.0) -> No overshoot, smooth gentle return
# Case 2: Optimal damping (zeta = 0.707) -> Fast, slight overshoot (< 5%)

for zeta, name in [(1.0, "Critically Damped (zeta = 1.0, Zero Overshoot)"),
                   (0.707, "Optimal Damped (zeta = 0.707, 4.3% Overshoot)")]:
    omega_n = 4.0 / (zeta * 1.0) # for 1.0 sec settling time
    
    # Pitch: I_pitch * omega_n^2 = Kp_pitch - K_grav  ==> Kp_pitch = I_pitch * omega_n^2 + K_grav
    # Kd_pitch = 2 * zeta * omega_n * I_pitch = 2 * 4.0 * I_pitch = 8.0 * I_pitch
    Kp_pitch = I_pitch_pivot * (omega_n**2) + K_grav
    Kd_pitch = 2.0 * zeta * omega_n * I_pitch_pivot
    
    # Roll: similar to pitch with I_roll
    Kp_roll = I_roll_pivot * (omega_n**2) + K_grav
    Kd_roll = 2.0 * zeta * omega_n * I_roll_pivot
    
    # Yaw: No gravity toppling torque (rotation about vertical gravity vector)!
    Kp_yaw = I_yaw_pivot * (omega_n**2)
    Kd_yaw = 2.0 * zeta * omega_n * I_yaw_pivot
    
    print(f"=== {name} (omega_n = {omega_n:.2f} rad/s) ===")
    print(f"  Pitch: Kp = {Kp_pitch:.3f} Nm/rad, Kd = {Kd_pitch:.3f} Nms/rad")
    print(f"  Roll : Kp = {Kp_roll:.3f} Nm/rad, Kd = {Kd_roll:.3f} Nms/rad")
    print(f"  Yaw  : Kp = {Kp_yaw:.3f} Nm/rad, Kd = {Kd_yaw:.3f} Nms/rad")
    
    # In controller: kp = sqrt(Wp_base) -> Wp_base = kp^2
    # kd = sqrt(Wv_base + 2*kp) -> Wv_base = kd^2 - 2*kp
    Wp_pitch = Kp_pitch**2
    Wv_pitch = max(0.0, Kd_pitch**2 - 2.0 * Kp_pitch)
    
    Wp_roll = Kp_roll**2
    Wv_roll = max(0.0, Kd_roll**2 - 2.0 * Kp_roll)
    
    Wp_yaw = Kp_yaw**2
    Wv_yaw = max(0.0, Kd_yaw**2 - 2.0 * Kp_yaw)
    
    print(f"  Controller Property Equivalents (Wp_base, Wv_base):")
    print(f"    Wp_pitch = {Wp_pitch:.1f}, Wv_pitch = {Wv_pitch:.1f}")
    print(f"    Wp_roll  = {Wp_roll:.1f}, Wv_roll  = {Wv_roll:.1f} (vs CURRENT 400.0 / 40.0!)")
    print(f"    Wp_yaw   = {Wp_yaw:.1f}, Wv_yaw   = {Wv_yaw:.1f} (vs CURRENT 400.0 / 40.0!)\n")

