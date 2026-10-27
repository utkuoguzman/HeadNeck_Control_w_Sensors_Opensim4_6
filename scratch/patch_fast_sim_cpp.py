with open('HeadNeckNeuralController_plugin/cervical_fast_sim.cpp', 'r', encoding='utf-8') as f:
    code = f.read()

old = 'controller->set_Ki_task(ki_task);'
new = '''controller->set_Ki_task(ki_task);
    controller->set_Ki_task_pitch(ki_task);
    controller->set_Ki_task_roll(0.0);
    controller->set_Ki_task_yaw(0.0);'''

assert old in code
code = code.replace(old, new, 1)

with open('HeadNeckNeuralController_plugin/cervical_fast_sim.cpp', 'w', encoding='utf-8') as f:
    f.write(code)

print("cervical_fast_sim.cpp updated successfully!")

