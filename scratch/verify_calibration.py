import xml.etree.ElementTree as ET
import numpy as np

# Load model
tree = ET.parse('osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim')
root = tree.getroot()

# New calibrated values
# name: (lower_limit, upper_limit, transition, stiffness, damping, coord_min, coord_max)
calib = {
    'pitch1': (-0.2793, 0.4189, 0.0524, 200.0, 20.0, -0.3840, 0.5236),
    'pitch2': (-0.5760, 0.8378, 0.0698, 250.0, 25.0, -0.7156, 0.9774),
    'roll1':  (-0.1047, 0.1047, 0.0349, 250.0, 25.0, -0.1745, 0.1745),
    'roll2':  (-0.5760, 0.5760, 0.0698, 250.0, 25.0, -0.7156, 0.7156),
    'yaw1':   (-0.6632, 0.6632, 0.0698, 250.0, 25.0, -0.8028, 0.8028),
    'yaw2':   (-0.4712, 0.4712, 0.0524, 250.0, 25.0, -0.5760, 0.5760)
}

print("=== VERIFYING COORDINATE LIMITS & BOUNDS CALIBRATION ===")
for cname, (llim, ulim, trans, stiff, damp, cmin, cmax) in calib.items():
    print(f"\nCoordinate: {cname}")
    print(f"  Voluntary Deadband (Zero Force): [{llim*180/np.pi:.1f} deg, {ulim*180/np.pi:.1f} deg] ({llim:.4f} to {ulim:.4f} rad)")
    print(f"  Transition Width: {trans*180/np.pi:.1f} deg ({trans:.4f} rad)")
    print(f"  Penetration Stiffness: {stiff} Nm/rad, Damping: {damp} Nms/rad")
    print(f"  Coordinate Hard Bounds (<range>): [{cmin*180/np.pi:.1f} deg, {cmax*180/np.pi:.1f} deg] ({cmin:.4f} to {cmax:.4f} rad)")
    
    # Check that hard bound is strictly outside the transition region
    assert cmin < llim - trans, f"Error on {cname}: cmin ({cmin}) is not beyond lower transition ({llim - trans})"
    assert cmax > ulim + trans, f"Error on {cname}: cmax ({cmax}) is not beyond upper transition ({ulim + trans})"
    print(f"  [OK] Hard bound is safely beyond transition region by at least {(abs(cmax - (ulim+trans))*180/np.pi):.1f} deg.")

