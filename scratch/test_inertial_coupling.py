import json
import math
import numpy as np

# Exact physics calculation
m_skull = 4.300 # kg
h_skull = 0.155 # m (distance from C7 to skull COM)
tau_skull_per_accel = m_skull * h_skull
print(f"Exact Skull Base Inertial Coupling: {tau_skull_per_accel:.4f} Nm / (m/s^2)")

m_neck = 1.200 # kg
h_neck = 0.075 # m
tau_neck_per_accel = m_neck * h_neck
print(f"Exact Neck Base Inertial Coupling: {tau_neck_per_accel:.4f} Nm / (m/s^2)")

