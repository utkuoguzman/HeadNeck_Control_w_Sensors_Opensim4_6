import re

with open('test_scripts/fig3_ramp_stretches.py', 'r') as f:
    content = f.read()

# Replace spindle_ctrl string
old_spindle_ctrl = """        spindle_ctrl = f\"\"\"
        <SpindleController name="spindle_controller">
            <socket_actuators>muscle</socket_actuators>
            <ControlFunctions><Constant><value>0.01</value></Constant></ControlFunctions>
            <SpindleFunctionsStatic><Constant><value>{gamma["stat"]}</value></Constant></SpindleFunctionsStatic>
            <SpindleFunctionsDynamic><Constant><value>{gamma["dyn"]}</value></Constant></SpindleFunctionsDynamic>
        </SpindleController>
        \"\"\""""

new_spindle_ctrl = """        spindle_ctrl = f\"\"\"
        <SpindleController name="spindle_controller">
            <socket_actuators>muscle</socket_actuators>
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
        \"\"\""""

content = content.replace(old_spindle_ctrl, new_spindle_ctrl)

# Replace integration loop
old_loop = """        coord.setPrescribedFunction(spline)
        coord.setDefaultIsPrescribed(True)
        
        state = model.initSystem()
        manager = osim.Manager(model)
        manager.initialize(state)
        
        t_hist = []
        Ia_hist = []
        muscle_obj = model.getForceSet().get("muscle")
        
        for i in range(1, n_steps + 1):
            t = i * dt
            state.setTime(t)
            manager.integrate(t)
            model.realizeAcceleration(state)
            
            t_hist.append(t)
            Ia = 0.0
            try:
                out = muscle_obj.getOutput("primary_Ia")
                Ia = osim.OutputDouble.safeDownCast(out).getValue(state)
            except Exception as e:
                print(f"ERROR: {e}")
            Ia_hist.append(Ia)"""

new_loop = """        muscle = model.getForceSet().get("muscle")
        muscle = osim.Muscle.safeDownCast(muscle)
        muscle.set_ignore_tendon_compliance(True)
        
        state = model.initSystem()
        manager = osim.Manager(model)
        
        t_hist = []
        Ia_hist = []
        muscle_obj = model.getForceSet().get("muscle")
        
        state.setTime(0.0)
        q_init = spline.calcValue(osim.Vector(1, 0.0))
        v_init = spline.calcDerivative([0], osim.Vector(1, 0.0))
        coord.setLocked(state, False)
        coord.setValue(state, q_init)
        coord.setSpeedValue(state, v_init)
        coord.setLocked(state, True)
        manager.initialize(state)
        
        for i in range(1, n_steps + 1):
            t = i * dt
            q = spline.calcValue(osim.Vector(1, t))
            v = spline.calcDerivative([0], osim.Vector(1, t))
            coord.setLocked(state, False)
            coord.setValue(state, q)
            coord.setSpeedValue(state, v)
            coord.setLocked(state, True)
            manager.initialize(state)
            manager.integrate(t)
            state = manager.getState()
            
            model.realizeAcceleration(state)
            
            t_hist.append(t)
            Ia = 0.0
            try:
                out = muscle_obj.getOutput("primary_Ia")
                Ia = osim.OutputDouble.safeDownCast(out).getValue(state)
            except Exception as e:
                pass
            Ia_hist.append(Ia)"""

content = content.replace(old_loop, new_loop)

with open('test_scripts/fig3_ramp_stretches.py', 'w') as f:
    f.write(content)
print('DONE')

