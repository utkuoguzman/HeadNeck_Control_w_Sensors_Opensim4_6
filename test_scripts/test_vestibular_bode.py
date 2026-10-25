import opensim as osim
import numpy as np
import matplotlib.pyplot as plt
import os
import scipy.signal as sig

# Load plugins
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'proprioception_plugin/build/Release/osimMillard12EqWithAff.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'FixationController_plugin/build/Release/osimFixation.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'vestibular_plugin/build/Release/osimVestibular.dll'))

model = osim.Model("osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim")
model.setUseVisualizer(False)

# Remove muscles to make simulation extremely fast
model.updForceSet().clearAndDestroy()

# Remove fixation controller which breaks without muscles
cset = model.updControllerSet()
for i in range(cset.getSize()):
    if cset.get(i).getName() == "Binocular_Fixation_Controller":
        cset.remove(i)
        break

coord_p1 = model.updCoordinateSet().get("pitch1")
coord_p2 = model.updCoordinateSet().get("pitch2")

# Create a chirp signal
dt = 0.005
t_end = 60.0
times = np.arange(0, t_end, dt)

# Frequencies for Bode plot
f0 = 0.05
f1 = 10.0

A = 50.0 * (np.pi / 180.0) # 50 deg/s max amplitude

vel_signal = A * sig.chirp(times, f0=f0, f1=f1, t1=t_end, method='logarithmic')
# position is integral of velocity
pos_signal = np.cumsum(vel_signal) * dt

# Create Spline
osim_times = osim.ArrayDouble()
osim_values_p1 = osim.ArrayDouble()
osim_values_p2 = osim.ArrayDouble()

for i in range(len(times)):
    osim_times.append(times[i])
    osim_values_p1.append(pos_signal[i] / 2.0)
    osim_values_p2.append(pos_signal[i] / 2.0)

spline_p1 = osim.SimmSpline()
spline_p2 = osim.SimmSpline()
for i in range(osim_times.getSize()):
    spline_p1.addPoint(osim_times.get(i), osim_values_p1.get(i))
    spline_p2.addPoint(osim_times.get(i), osim_values_p2.get(i))

coord_p1.setPrescribedFunction(spline_p1)
coord_p2.setPrescribedFunction(spline_p2)
coord_p1.setDefaultIsPrescribed(True)
coord_p2.setDefaultIsPrescribed(True)

state = model.initSystem()

# Let's get the controller using Component API
controller = model.updControllerSet().get(0)

# Create a TableReporter to record the output efficiently
reporter = osim.TableReporter()
reporter.setName("bode_reporter")
reporter.set_report_time_interval(dt)
reporter.updInput("inputs").connect(controller.getOutput("omega_recon_pitch"))
reporter.updInput("inputs").connect(coord_p1.getOutput("speed"))
model.addComponent(reporter)

state = model.initSystem()

integrator = osim.RungeKuttaMersonIntegrator(model.getMultibodySystem())
integrator.setMaximumStepSize(0.001)
manager = osim.Manager(model, integrator)
manager.initialize(state)

print("Running Multi-Frequency Vestibular Sweep (60 seconds) ...")
manager.integrate(60.0)

print("Simulation finished. Computing Empirical Bode Plot...")

table = reporter.getTable()
t_hist = table.getIndependentColumn()
recon_col = table.getDependentColumn("HeadNeckNeuralController|omega_recon_pitch")
# Pitch speed is half of the head speed, so multiply by 2
speed_col = table.getDependentColumn("pitch1|speed")

t_hist = np.array(t_hist)
recon_hist = np.array(recon_col) * (180.0/np.pi)
vel_hist = np.array(speed_col) * 2.0 * (180.0/np.pi)


# Use Welch's method (csd) to compute empirical transfer function
f, Pxx = sig.welch(vel_hist, fs=1.0/dt, nperseg=4096)
f, Pxy = sig.csd(vel_hist, recon_hist, fs=1.0/dt, nperseg=4096)

H_emp = Pxy / Pxx
emp_mag = np.abs(H_emp)
emp_phase = np.angle(H_emp) * 180.0 / np.pi

# Theoretical transfer function
k_reg = 2.83; T1_reg = 0.0175; T2_reg = 0.0027; Tc_reg = 5.7
k_irr = 27.09; T1_irr = 0.03; T2_irr = 0.0006; Tc_irr = 5.7

s = 2j * np.pi * f
s[0] = 1e-10

H_reg = k_reg * s * (s + 1.0/T1_reg) / ((s + 1.0/Tc_reg) * (s + 1.0/T2_reg))
H_irr = k_irr * s * (s + 1.0/T1_irr) / ((s + 1.0/Tc_irr) * (s + 1.0/T2_irr))
H_theory = (H_reg + H_irr) / 2.0

# Incorporate the 13ms delay into the theory!
delay = 0.013
H_delay = np.exp(-s * delay)
H_theory_delayed = H_theory * H_delay

theory_mag = np.abs(H_theory_delayed)
theory_phase = np.angle(H_theory_delayed) * 180.0 / np.pi

# Unwrap phase
emp_phase = np.unwrap(emp_phase * np.pi / 180.0) * 180.0 / np.pi
theory_phase = np.unwrap(theory_phase * np.pi / 180.0) * 180.0 / np.pi

# Plotting
fig, (ax_mag, ax_phase) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

mask = (f >= 0.04) & (f <= 10.0)

ax_mag.loglog(f[mask], emp_mag[mask], 'b.', label='Empirical Simulation (OpenSim C++)')
ax_mag.loglog(f[mask], theory_mag[mask], 'r-', label='Theory (Happee 2017 + 13ms delay)')
ax_mag.set_ylabel('Gain')
ax_mag.grid(True, which='both', ls='--')
ax_mag.legend()
ax_mag.set_title('Vestibular SCC Bode Plot (Pitch Axis)')

ax_phase.semilogx(f[mask], emp_phase[mask], 'b.')
ax_phase.semilogx(f[mask], theory_phase[mask], 'r-')
ax_phase.set_ylabel('Phase (deg)')
ax_phase.set_xlabel('Frequency (Hz)')
ax_phase.grid(True, which='both', ls='--')

plt.tight_layout()
os.makedirs("logs", exist_ok=True)
plt.savefig("logs/test_vestibular_bode.png")
print("Saved Bode plot to logs/test_vestibular_bode.png")
