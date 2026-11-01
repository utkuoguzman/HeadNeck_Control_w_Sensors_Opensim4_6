import opensim as osim
import numpy as np
import math, os, sys, json

osim.LoadOpenSimLibrary('proprioception_plugin/build/Release/osimMillard12EqWithAff.dll')
osim.LoadOpenSimLibrary('HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll')
osim.LoadOpenSimLibrary('FixationController_plugin/build/Release/osimFixation.dll')

with open('logs/candidates/W7_P24G1.json') as fp:
    cand = json.load(fp)

params = cand['params']
# params: [K_ton, p_ton, K_sc, z_sc, p_sc, K_phas, p_phas, kp_prop, z_ccr, p_ccr, k_dyn, k_stat]
K_ton, p_ton, K_sc, z_sc, p_sc, K_phas, p_phas, kp_prop, z_ccr, p_ccr, k_dyn, k_stat = params
freq = 0.783

model = osim.Model("osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim")

controller = None
for i in range(model.getControllerSet().getSize()):
    ctrl = model.getControllerSet().get(i)
    if ctrl.getConcreteClassName() == "FixationController":
        ctrl.set_enabled(False)
    elif ctrl.getConcreteClassName() == "HeadNeckNeuralController":
        controller = ctrl

osim.PropertyHelper.setValueDouble(K_ton, controller.updPropertyByName("K_ton"))
osim.PropertyHelper.setValueDouble(p_ton, controller.updPropertyByName("p_ton"))
osim.PropertyHelper.setValueDouble(K_sc, controller.updPropertyByName("K_sc"))
osim.PropertyHelper.setValueDouble(z_sc, controller.updPropertyByName("z_sc"))
osim.PropertyHelper.setValueDouble(p_sc, controller.updPropertyByName("p_sc"))
osim.PropertyHelper.setValueDouble(K_phas, controller.updPropertyByName("K_phas"))
osim.PropertyHelper.setValueDouble(p_phas, controller.updPropertyByName("p_phas"))
osim.PropertyHelper.setValueDouble(kp_prop, controller.updPropertyByName("k_p"))

wp_pitch = float(os.environ.get("WP_BASE_PITCH", "0.0"))
wv_pitch = float(os.environ.get("WV_BASE_PITCH", "0.0"))
osim.PropertyHelper.setValueDouble(wp_pitch, controller.updPropertyByName("Wp_base_pitch"))
osim.PropertyHelper.setValueDouble(wv_pitch, controller.updPropertyByName("Wv_base_pitch"))
for ax in ['roll', 'yaw']:
    osim.PropertyHelper.setValueDouble(400.0, controller.updPropertyByName(f"Wp_base_{ax}"))
    osim.PropertyHelper.setValueDouble(40.0, controller.updPropertyByName(f"Wv_base_{ax}"))
osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("k_vigor"))
osim.PropertyHelper.setValueDouble(0.1, controller.updPropertyByName("trajectory_tau"))
osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("K_eso"))

ki = 40.2786
osim.PropertyHelper.setValueDouble(ki, controller.updPropertyByName("Ki_task"))
osim.PropertyHelper.setValueDouble(kp_prop, controller.updPropertyByName("Kp_proprioception"))
osim.PropertyHelper.setValueDouble(z_ccr, controller.updPropertyByName("z_ccr"))
osim.PropertyHelper.setValueDouble(p_ccr, controller.updPropertyByName("p_ccr"))
osim.PropertyHelper.setValueDouble(k_dyn, controller.updPropertyByName("K_gamma_dyn"))
osim.PropertyHelper.setValueDouble(k_stat, controller.updPropertyByName("K_gamma_stat"))
osim.PropertyHelper.setValueDouble(0.010, controller.updPropertyByName("delay_vcr"))
osim.PropertyHelper.setValueDouble(0.015, controller.updPropertyByName("delay_ccr"))

zero_func = osim.Constant(0.0)
for coord_name in ["gndpitch", "gndroll", "gndyaw", "spine_ty", "spine_tz"]:
    c = model.updCoordinateSet().get(coord_name)
    c.set_locked(False)
    c.setDefaultIsPrescribed(True)
    c.setPrescribedFunction(zero_func)

c_yaw1 = model.updCoordinateSet().get("yaw1")
c_yaw1.set_locked(False)
c_yaw1.setDefaultIsPrescribed(False)

amplitude = 0.0215 * (0.835 / freq)**2
delay = 1.5
period = 1.0 / freq
ramp_duration = period
min_phase3_time = 2.0 / 1.8
n_eval = max(1, math.ceil(min_phase3_time / period))
target_time = delay + ramp_duration + (n_eval * period)

sine_function = osim.SimmSpline()
for t in np.arange(0.0, delay, 0.02):
    sine_function.addPoint(t, 0.0)
spline_dt = min(0.01, period / 40.0)
for t in np.arange(delay, target_time + 0.05, spline_dt):
    dt = t - delay
    if dt < ramp_duration:
        tau = dt / ramp_duration
        w = 10.0 * (tau**3) - 15.0 * (tau**4) + 6.0 * (tau**5)
    else:
        w = 1.0
    y = w * amplitude * math.sin(2.0 * math.pi * freq * dt)
    sine_function.addPoint(t, y)

coord_tx = model.updCoordinateSet().get("spine_tx")
coord_tx.set_locked(False)
coord_tx.setDefaultIsPrescribed(True)
coord_tx.setPrescribedFunction(sine_function)

state = model.initSystem()
c_yaw1.setIsPrescribed(state, False)
coord_tx.setSpeedValue(state, 0.0)
skull = model.getBodySet().get("skull")
R_head_to_world = skull.getTransformInGround(state).R()
q = osim.Rotation(R_head_to_world).convertRotationToQuaternion()
model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q0", q.get(0))
model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q1", q.get(1))
model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q2", q.get(2))
model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q3", q.get(3))
model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/pitch_error_integral", 0.10528)
model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/roll_error_integral", 0.0)
model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/yaw_error_integral", 0.0)
model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_pitch", ki * 0.10528)
model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_roll", 0.0)
model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_yaw", 0.0)
model.equilibrateMuscles(state)

manager = osim.Manager(model)
manager.setIntegratorMethod(5)
manager.setIntegratorAccuracy(1e-4)
manager.setIntegratorMinimumStepSize(1e-4)
manager.setIntegratorMaximumStepSize(0.01)

state.setTime(0.0)
manager.initialize(state)

step = min(0.01, (1.0/freq) / 40.0)
current_time = 0.0

coord_p1 = model.getCoordinateSet().get("pitch1")
coord_p2 = model.getCoordinateSet().get("pitch2")
coord_r1 = model.getCoordinateSet().get("roll1")
coord_r2 = model.getCoordinateSet().get("roll2")
coord_y1 = model.getCoordinateSet().get("yaw1")

max_roll = 0.0
max_yaw = 0.0
max_pitch = 0.0

# Simulate first 2.5 seconds to inspect roll and yaw excursions
while current_time < min(target_time, 2.5):
    current_time += step
    state = manager.integrate(current_time)
    p = math.degrees(coord_p1.getValue(state) + coord_p2.getValue(state))
    r = math.degrees(coord_r1.getValue(state) + coord_r2.getValue(state))
    y = math.degrees(coord_y1.getValue(state))
    max_pitch = max(max_pitch, abs(p))
    max_roll = max(max_roll, abs(r))
    max_yaw = max(max_yaw, abs(y))

print(f"Inspection at {freq} Hz:")
print(f"  Max Pitch Excursion: {max_pitch:.3f} deg")
print(f"  Max Roll  Excursion: {max_roll:.4f} deg")
print(f"  Max Yaw   Excursion: {max_yaw:.4f} deg")

