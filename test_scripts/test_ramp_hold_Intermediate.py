import opensim as osim
import os
import sys
import matplotlib.pyplot as plt

muscle_type = 'Millard12EqMuscleWithIntermediateAfferent'
if len(sys.argv) > 1:
    muscle_type = 'Millard12EqMuscleWithIntermediateAfferent'

print(f'Running Stretch and Hold Test for: {muscle_type}')

osim.LoadOpenSimLibrary('osimMillard12EqWithAff')

model = osim.Model()
model.setName(f'StretchHold_{muscle_type}')
model.setUseVisualizer(False)

ground = model.getGround()
mass = 1.0
block = osim.Body('block', mass, osim.Vec3(0), osim.Inertia(1,1,1,0,0,0))
model.addBody(block)

joint = osim.SliderJoint('slider', ground, osim.Vec3(0), osim.Vec3(0), block, osim.Vec3(0), osim.Vec3(0))
coord = joint.updCoordinate()
coord.setName('x_translation')
model.addJoint(joint)

max_force = 1000.0
opt_len = 0.1
tendon_len = 0.05
pennation = 0.0

muscle = osim.Millard2012EquilibriumMuscle('muscle', max_force, opt_len, tendon_len, pennation)
muscle.addNewPathPoint('origin', ground, osim.Vec3(0, 0, 0))
muscle.addNewPathPoint('insertion', block, osim.Vec3(-0.05, 0, 0))
muscle.set_ignore_tendon_compliance(True)
model.addForce(muscle)

# If it's simplified, use standard prescribed controller
if muscle_type == 'Millard12EqMuscleWithSimplifiedAfferent':
    prescribed = osim.PrescribedController()
    prescribed.addActuator(muscle)
    prescribed.prescribeControlForActuator('muscle', osim.Constant(0.1))
    model.addController(prescribed)

model.finalizeConnections()
xml_path = 'temp_model.osim'
model.printToXML(xml_path)

with open(xml_path, 'r') as f:
    xml_content = f.read()

xml_content = xml_content.replace('Millard2012EquilibriumMuscle', muscle_type)

if muscle_type != 'Millard12EqMuscleWithSimplifiedAfferent':
    spindle_ctrl = """
    <SpindleController name="spindle_controller">
        <socket_actuators>/forceset/muscle</socket_actuators>
        <ControlFunctions>
            <FunctionSet><objects><Constant><value>0.1</value></Constant></objects></FunctionSet>
        </ControlFunctions>
        <SpindleFunctionsStatic>
            <FunctionSet><objects><Constant><value>0.0</value></Constant></objects></FunctionSet>
        </SpindleFunctionsStatic>
        <SpindleFunctionsDynamic>
            <FunctionSet><objects><Constant><value>0.0</value></Constant></objects></FunctionSet>
        </SpindleFunctionsDynamic>
    </SpindleController>
    """
    if '<ControllerSet name="controllerset">' in xml_content:
        import re
        xml_content = re.sub(r'<objects\s*/>', f'<objects>{spindle_ctrl}</objects>', xml_content)
    else:
        ctrl_set = f"""
        <ControllerSet name="controllerset">
            <objects>{spindle_ctrl}</objects>
        </ControllerSet>
        """
        xml_content = xml_content.replace('</Model>', f'{ctrl_set}\n</Model>')

with open(xml_path, 'w') as f:
    f.write(xml_content)

model = osim.Model(xml_path)
coord = model.updCoordinateSet().get('x_translation')

times = osim.ArrayDouble()
values = osim.ArrayDouble()
for i in range(301):
    t = i / 100.0
    times.append(t)
    if t <= 0.5:
        x = 0.20
    elif t <= 1.5:
        x = 0.20 + 0.02 * (t - 0.5) / 1.0
    else:
        x = 0.22
    values.append(x)

spline = osim.SimmSpline()
for i in range(times.getSize()):
    spline.addPoint(times.get(i), values.get(i))

pos_motion = osim.PositionMotion('pos_motion')
pos_motion.setPositionForCoordinate(coord, spline)
model.addModelComponent(pos_motion)

state = model.initSystem()
model.equilibrateMuscles(state)
manager = osim.Manager(model)
manager.initialize(state)

t_hist = []
Ia_hist = []
L_hist = []

muscle_obj = model.getForceSet().get('muscle')
muscle_obj = osim.Muscle.safeDownCast(muscle_obj)

for i in range(1, 301):
    t = i / 100.0
    manager.integrate(t)
    state = manager.getState()
    model.realizeAcceleration(state)
    
    t_hist.append(t)
    L_hist.append(muscle_obj.getNormalizedFiberLength(state))
    
    Ia = 0.0
    try:
        Ia = float(muscle_obj.getOutput('primary_Ia').getValueAsString(state))
    except Exception as e:
        pass
        
    Ia_hist.append(Ia)

fig, ax1 = plt.subplots(figsize=(10,6))
ax2 = ax1.twinx()
ax1.plot(t_hist, L_hist, 'k--', label='Norm Fiber Length')
ax2.plot(t_hist, Ia_hist, 'r-', label='Primary (Ia) Firing')

ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Normalized Length')
ax2.set_ylabel('Firing Rate (Hz)')
plt.title(f'Stretch and Hold: {muscle_type}')
fig.legend(loc='upper left', bbox_to_anchor=(0.15, 0.85))

os.makedirs('logs', exist_ok=True)
plt.savefig(f'logs/stretch_hold_{muscle_type}.png')
print(f'Saved plot to logs/stretch_hold_{muscle_type}.png')
