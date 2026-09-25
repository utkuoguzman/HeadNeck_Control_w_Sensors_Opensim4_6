import opensim as osim
import numpy as np
osim.LoadOpenSimLibrary("osimMillard12EqWithAff")
model = osim.Model("temp_model.osim") 
coord = model.updCoordinateSet().get("x_translation")
times = osim.ArrayDouble()
values = osim.ArrayDouble()
for i in range(200):
    times.append(i * 0.01)
    values.append(0.15 + (i * 0.01) * 0.1) 
spline = osim.SimmSpline()
for i in range(times.getSize()):
    spline.addPoint(times.get(i), values.get(i))

muscle = model.getForceSet().get("muscle")
muscle = osim.Muscle.safeDownCast(muscle)
muscle.set_ignore_tendon_compliance(True)
state = model.initSystem()
manager = osim.Manager(model)
dt = 0.005
for i in range(400):
    t = i * dt
    q = spline.calcValue(osim.Vector(1, t))
    v = spline.calcDerivative([0], osim.Vector(1, t))
    coord.setLocked(state, False)
    coord.setValue(state, q)
    coord.setSpeedValue(state, v)
    coord.setLocked(state, True)
    
    manager.initialize(state)
    manager.integrate(t + dt)
    state = manager.getState()
    
    if i % 20 == 0:
        model.realizeAcceleration(state)
        Ia = 0.0
        try:
            out = muscle.getOutput("primary_Ia")
            out_d = osim.OutputDouble.safeDownCast(out)
            Ia = out_d.getValue(state)
        except Exception as e:
            pass
        print(f"t={t:.2f}, L={muscle.getLength(state):.4f}, fiber={muscle.getNormalizedFiberLength(state):.4f}, Ia={Ia:.2f}")

