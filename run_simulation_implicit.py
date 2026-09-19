import os
import sys
try:
    script_dir = os.path.dirname(os.path.abspath(__file__))
except:
    script_dir = r"D:\Akademik\PhD_Thesis\OpenSim\HeadNeck_Control_w_Sensors_Opensim4_6"
os.chdir(script_dir)

import sys
import org.opensim.modeling as modeling
import time

log_file = os.path.join(script_dir, 'logs', 'simulation_log_implicit.txt')

def log(message):
    f = open(log_file, 'a')
    f.write(message + '\n')
    f.close()

f = open(log_file, 'w')
f.write("--- IMPLICIT RUN (SEMI-EXPLICIT EULER) ---\n")
f.close()

model = getCurrentModel()
state = model.initSystem()
manager = modeling.Manager(model)

manager.setIntegratorMethod(5) #Runge-Kutta-Merson (Best explicit multibody solver)

# --- THE ANTI-RINGING TUNING ---
# 1. Loosen accuracy. If it's too strict, the solver panics and shrinks dt, causing Fornberg ringing.
manager.setIntegratorAccuracy(1e-4)
# 2. Cap the max step. Prevents the solver from leaping over stiff proprioceptive curves.
manager.setIntegratorMaximumStepSize(1) 
# 3. Raise the minimum step. Prevents the Fornberg time-denominators from approaching zero.
manager.setIntegratorMinimumStepSize(1e-8) 
# 4. Give it plenty of internal steps to work with.
manager.setIntegratorInternalStepLimit(11667)

manager.initialize(state)

target_time = 1.0 

start_time = time.clock()
manager.integrate(target_time)
end_time = time.clock()
execution_time = end_time - start_time


log("Simulation finished at " + str(execution_time) + "!")

statesTable = manager.getStatesTable()
save_path = os.path.join(script_dir, 'state_files', '1sec_forward_release_implicit.sto')
modeling.STOFileAdapter.write(statesTable, save_path)
log("Results saved successfully.")
