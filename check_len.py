import opensim as osim
import numpy as np
osim.LoadOpenSimLibrary("osimMillard12EqWithAff")
model = osim.Model("temp_fig3.osim")
coord = model.updCoordinateSet().get("x_translation")
times = osim.ArrayDouble()
values = osim.ArrayDouble()
for i in range(100):
    times.append(i * 0.1)
    values.append(0.0441775 + (i * 0.1) * 0.05)
spline = osim.SimmSpline()
for i in range(times.getSize()):
    spline.addPoint(times.get(i), values.get(i))
    
pos_mot = osim.PositionMotion("pos_mot")
pos_mot.setPositionForCoordinate(coord, spline.clone())
model.addComponent(pos_mot)

state = model.initSystem()
manager = osim.Manager(model)
manager.initialize(state)
for i in range(20):
    t = 0.5 + i * 0.05
    manager.integrate(t)
    print(f"t={t:.2f}, q={coord.getValue(state):.4f}")

