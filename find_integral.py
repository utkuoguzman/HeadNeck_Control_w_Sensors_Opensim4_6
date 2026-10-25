import sys, os, math
import opensim as osim

osim.Logger.setLevelString("Warn")
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'proprioception_plugin/build/Release/osimMillard12EqWithAff.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'vestibular_plugin/build/Release/osimVestibular.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'FixationController_plugin/build/Release/osimFixation.dll'))

model = osim.Model("osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim")
controller = None
for i in range(model.getControllerSet().getSize()):
    ctrl = model.getControllerSet().get(i)
    if ctrl.getConcreteClassName() == "FixationController":
        ctrl.set_enabled(False)
    elif ctrl.getConcreteClassName() == "HeadNeckNeuralController":
        controller = ctrl

# Task params
I_eff = 1.0
w_n = 10.1
zeta = 1.98
ki = 6.54
kp = I_eff * (w_n**2)
kd = 2.0 * zeta * I_eff * w_n

osim.PropertyHelper.setValueDouble(10.0, controller.updPropertyByName("G_ton"))
osim.PropertyHelper.setValueDouble(0.45, controller.updPropertyByName("k_p"))
osim.PropertyHelper.setValueDouble(kp, controller.updPropertyByName("Kp_task"))
osim.PropertyHelper.setValueDouble(ki, controller.updPropertyByName("Ki_task"))
osim.PropertyHelper.setValueDouble(kd, controller.updPropertyByName("Kd_task"))

zero_func = osim.Constant(0.0)
for coord_name in ["gndpitch", "gndroll", "gndyaw", "spine_tx", "spine_ty", "spine_tz"]:
    c = model.updCoordinateSet().get(coord_name)
    c.set_locked(False)
    c.setDefaultIsPrescribed(True)
    c.setPrescribedFunction(zero_func)

state = model.initSystem()
model.equilibrateMuscles(state)
manager = osim.Manager(model)
manager.setIntegratorMethod(5)
manager.setIntegratorAccuracy(1e-4)
state.setTime(0.0)

# Initialize observer
skull = model.getBodySet().get("skull")
R_head_to_world = skull.getTransformInGround(state).R()
q = osim.Rotation(R_head_to_world).convertRotationToQuaternion()
model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q0", q.get(0))
model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q1", q.get(1))
model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q2", q.get(2))
model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q3", q.get(3))

manager.initialize(state)

manager.integrate(3.0) # Let it settle completely
st = manager.getState()
integral_val = model.getStateVariableValue(st, "/controllerset/VCR_CCR_Controller/pitch_error_integral")
roll_val = model.getStateVariableValue(st, "/controllerset/VCR_CCR_Controller/roll_error_integral")
yaw_val = model.getStateVariableValue(st, "/controllerset/VCR_CCR_Controller/yaw_error_integral")

p1 = model.getCoordinateSet().get("pitch1").getValue(st)
p2 = model.getCoordinateSet().get("pitch2").getValue(st)

print(f"PITCH: {math.degrees(p1+p2):.2f} deg")
print(f"PITCH_INTEGRAL: {integral_val}")
print(f"ROLL_INTEGRAL: {roll_val}")
print(f"YAW_INTEGRAL: {yaw_val}")
