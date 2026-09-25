import opensim as osim
import math
import numpy as np

freq = 1.0
omega = 2 * math.pi * freq
amplitude = 1.0
target_time = 3.0

spline = osim.SimmSpline()
# Add points for t=0 to 0.5 (zero)
for t in np.arange(0.0, 0.5, 0.05):
    spline.addPoint(t, 0.0)

# Add points for the sine wave
for t in np.arange(0.5, target_time + 0.1, 0.01):
    y = amplitude * math.sin(omega * (t - 0.5))
    spline.addPoint(t, y)

print(f"t=0.0: {spline.calcValue(osim.Vector(1, 0.0))}")
print(f"t=0.4: {spline.calcValue(osim.Vector(1, 0.4))}")
print(f"t=0.5: {spline.calcValue(osim.Vector(1, 0.5))}")
print(f"t=0.6: {spline.calcValue(osim.Vector(1, 0.6))}")
print(f"t=0.75: {spline.calcValue(osim.Vector(1, 0.75))}")

