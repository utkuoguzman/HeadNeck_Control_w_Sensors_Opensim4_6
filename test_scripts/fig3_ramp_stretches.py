import re
import opensim as osim
import os
import sys
import numpy as np
from scipy.ndimage import gaussian_filter1d

import matplotlib.pyplot as plt

muscle_type = "Millard12EqMuscleWithAfferents"
if len(sys.argv) > 1:
    muscle_type = sys.argv[1]

osim.LoadOpenSimLibrary("osimMillard12EqWithAff")

# Replicate Figure 3 layout: 3 rows (Gamma conditions), 3 columns (Velocities)
velocities_mms = [5.0, 30.0, 70.0]
velocities_L0s = [0.11, 0.66, 1.55]
gamma_conds = [
    {"name": "no gamma", "dyn": 0.0, "stat": 0.0},
    {"name": "gamma_dyn on", "dyn": 70.0, "stat": 0.0},
    {"name": "gamma_stat on", "dyn": 0.0, "stat": 70.0}
]

opt_len = 0.04545 # so that 6mm is 0.13 L0
tendon_len = 0.001 # nearly rigid tendon
max_force = 1000.0
pennation = 0.0

L0_start_ratio = 0.95
L0_end_ratio = 1.08

start_length = L0_start_ratio * opt_len + tendon_len
end_length = L0_end_ratio * opt_len + tendon_len
stretch_dist = end_length - start_length

fig, axes = plt.subplots(3, 3, figsize=(12, 10), sharex='col', sharey='row')

for c_idx, (vel_mms, vel_L0s) in enumerate(zip(velocities_mms, velocities_L0s)):
    vel_ms = vel_L0s * opt_len
    ramp_duration = stretch_dist / vel_ms
    
    for r_idx, gamma in enumerate(gamma_conds):
        print(f"Running {vel_mms} mm/s with {gamma['name']}")
        # Build model for each condition
        model = osim.Model()
        model.setUseVisualizer(False)
        
        ground = model.getGround()
        block = osim.Body("block", 1.0, osim.Vec3(0), osim.Inertia(1,1,1,0,0,0))
        model.addBody(block)
        
        joint = osim.SliderJoint("slider", ground, osim.Vec3(0), osim.Vec3(0), block, osim.Vec3(0), osim.Vec3(0))
        coord = joint.updCoordinate()
        coord.setName("x_translation")
        model.addJoint(joint)
        
        muscle = osim.Millard2012EquilibriumMuscle("muscle", max_force, opt_len, tendon_len, pennation)
        muscle.set_ignore_tendon_compliance(True)
        muscle.addNewPathPoint("origin", ground, osim.Vec3(0, 0, 0))
        muscle.addNewPathPoint("insertion", block, osim.Vec3(0.0, 0, 0)) # distance is exactly x
        model.addForce(muscle)
        model.finalizeConnections()
        
        xml_path = "temp_fig3.osim"
        model.printToXML(xml_path)
        
        with open(xml_path, 'r') as f:
            xml_content = f.read()
        
        xml_content = xml_content.replace('Millard2012EquilibriumMuscle', muscle_type)
        
        spindle_ctrl = f"""
        <SpindleController name="spindle_controller">
            <socket_actuators>/forceset/muscle</socket_actuators>
            <ControlFunctions>
                <FunctionSet><objects><Constant><value>0.01</value></Constant></objects></FunctionSet>
            </ControlFunctions>
            <SpindleFunctionsStatic>
                <FunctionSet><objects><Constant><value>{gamma["stat"]}</value></Constant></objects></FunctionSet>
            </SpindleFunctionsStatic>
            <SpindleFunctionsDynamic>
                <FunctionSet><objects><Constant><value>{gamma["dyn"]}</value></Constant></objects></FunctionSet>
            </SpindleFunctionsDynamic>
        </SpindleController>
        """
        if "<ControllerSet name=\"controllerset\">" in xml_content:
            xml_content = re.sub(r"<objects\s*/>", f"<objects>{spindle_ctrl}</objects>", xml_content)
        else:
            ctrl_set = f'<ControllerSet name="controllerset"><objects>{spindle_ctrl}</objects></ControllerSet>'
            xml_content = xml_content.replace("</Model>", f"{ctrl_set}\n</Model>")
            
        with open(xml_path, 'w') as f:
            f.write(xml_content)
            
        model = osim.Model(xml_path)
        coord = model.updCoordinateSet().get("x_translation")
        
        # Prescribed motion
        # t=0 to 0.5: hold at start_length
        # t=0.5 to 0.5+ramp_duration: ramp to end_length
        # t=0.5+ramp_duration to ... : hold at end_length
        t_end = 1.0 + ramp_duration + 1.5 # 1.5s hold
        
        times = osim.ArrayDouble()
        values = osim.ArrayDouble()
        
        n_steps = 500
        dt = t_end / n_steps
        for i in range(n_steps + 1):
            t = i * dt
            times.append(t)
            if t <= 1.0:
                x = start_length
            elif t <= 1.0 + ramp_duration:
                x = start_length + vel_ms * (t - 1.0)
            else:
                x = end_length
            values.append(x)
            
        import numpy as np
        # Extract to numpy
        vals = np.array([values.get(i) for i in range(values.getSize())])
        # Smooth with sigma=2 to eliminate sharp corners that cause infinite acceleration
        vals_smooth = gaussian_filter1d(vals, sigma=2)
        
        spline = osim.SimmSpline()
        for i in range(times.getSize()):
            spline.addPoint(times.get(i), vals_smooth[i])
            
        pos_motion = osim.PositionMotion('pos_motion')
        pos_motion.setPositionForCoordinate(coord, spline)
        model.addModelComponent(pos_motion)
        
        muscle = model.getForceSet().get("muscle")
        muscle = osim.Muscle.safeDownCast(muscle)
        muscle.set_ignore_tendon_compliance(True)
        
        state = model.initSystem()
        model.equilibrateMuscles(state)
        manager = osim.Manager(model)
        manager.initialize(state)
        
        t_hist = []
        Ia_hist = []
        
        for i in range(1, n_steps + 1):
            t = i * dt
            manager.integrate(t)
            state = manager.getState()
            model.realizeAcceleration(state)
            
            t_hist.append(t)
            Ia = 0.0
            try:
                Ia = float(muscle.getOutput("primary_Ia").getValueAsString(state))
            except Exception as e:
                pass
            Ia_hist.append(Ia)
            
        ax = axes[r_idx, c_idx]
        # Only plot from 0.4s to end to zoom in on the stretch like the paper
        start_idx = int(0.9 / dt)
        ax.plot(t_hist[start_idx:], Ia_hist[start_idx:], 'k-', linewidth=2)
        
        if r_idx == 0:
            ax.set_title(f"{vel_mms} mm/s")
        if c_idx == 0:
            ax.set_ylabel(f"{gamma['name']}\nFiring (pps)")
        if r_idx == 2:
            ax.set_xlabel("Time (s)")
            
plt.suptitle(f"Figure 3 Ramp Stretches: {muscle_type}")
plt.tight_layout()
os.makedirs("logs", exist_ok=True)
plt.savefig(f"logs/fig3_{muscle_type}.png")
print(f"Saved logs/fig3_{muscle_type}.png")
