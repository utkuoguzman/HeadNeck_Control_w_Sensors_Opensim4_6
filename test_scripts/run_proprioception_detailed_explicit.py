import os
import sys
try:
    script_dir = os.path.dirname(os.path.abspath(__file__))
except:
    script_dir = r"D:\Akademik\PhD_Thesis\OpenSim\HeadNeck_Control_w_Sensors_Opensim4_6"
os.chdir(script_dir)

import org.opensim.modeling as modeling
import time

log_file = os.path.join(script_dir, 'logs', 'log_detailed_explicit.txt')
model_file = os.path.join(script_dir, 'osim_files', 'HYOID_HeadNeckNeural_Model_Detailed.osim')
method = 3
save_path = os.path.join(script_dir, 'state_files', 'detailed_explicit.sto')

def log(message):
    f = open(log_file, 'a')
    f.write(message + '\n')
    f.close()
    print message

f = open(log_file, 'w')
f.write('--- SIMULATION START ---\n')
f.close()

log('Loading libraries...')
modeling.Model.LoadOpenSimLibrary(os.path.join(script_dir, 'proprioception_plugin/build/Release/osimMillard12EqWithAff.dll'))
modeling.Model.LoadOpenSimLibrary(os.path.join(script_dir, 'vestibular_plugin/build/Release/osimVestibular.dll'))
modeling.Model.LoadOpenSimLibrary(os.path.join(script_dir, 'HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll'))
modeling.Model.LoadOpenSimLibrary(os.path.join(script_dir, 'FixationController_plugin/build/Release/osimFixation.dll'))

log('Loading model ' + model_file)
model = modeling.Model(model_file)
state = model.initSystem()
manager = modeling.Manager(model)

manager.setIntegratorMethod(method)

if method == 3: # Explicit RK-Merson
    manager.setIntegratorAccuracy(0.00008571)
    manager.setIntegratorMaximumStepSize(1.0)
    manager.setIntegratorMinimumStepSize(0.00008571)
    manager.setIntegratorInternalStepLimit(11667)
else: # Implicit SemiExplicitEuler2
    manager.setIntegratorAccuracy(1e-4)
    manager.setIntegratorMinimumStepSize(1e-8)

manager.initialize(state)

target_time = 1.0
step = 0.25
current_t = 0.0

log('Starting integration...')
start_time = time.clock()

while current_t < target_time:
    next_t = current_t + step
    if next_t > target_time:
        next_t = target_time
    
    manager.integrate(next_t)
    current_t = next_t
    
    elapsed = time.clock() - start_time
    log('Reached time: ' + str(current_t) + 's (Wall time elapsed: ' + str(elapsed) + 's)')

end_time = time.clock()
execution_time = end_time - start_time

log('Simulation finished at ' + str(execution_time) + ' seconds!')

statesTable = manager.getStatesTable()
modeling.STOFileAdapter.write(statesTable, save_path)
log('Results saved to ' + save_path)
