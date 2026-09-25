import opensim as osim
import numpy as np
vel_ms = 0.005
ramp_duration = 0.006 / vel_ms
start_length = 0.0441775
end_length = 0.0501775
t_end = 0.5 + ramp_duration + 1.5
times = osim.ArrayDouble()
values = osim.ArrayDouble()
n_steps = 500
dt = t_end / n_steps
for i in range(n_steps + 1):
    t = i * dt
    times.append(t)
    if t <= 0.5:
        x = start_length
    elif t <= 0.5 + ramp_duration:
        x = start_length + vel_ms * (t - 0.5)
    else:
        x = end_length
    values.append(x)
spline = osim.SimmSpline()
for i in range(times.getSize()):
    spline.addPoint(times.get(i), values.get(i))
for t in [0.0, 0.5, 1.0, 2.0]:
    print(f"t={t}, q={spline.calcValue(osim.Vector(1, t))}")

