with open('cmaes_worker_p22.py', 'r', encoding='utf-8') as f:
    code = f.read()
old = 'osim.PropertyHelper.setValueDouble(ki, controller.updPropertyByName("Ki_task"))'
new = '''osim.PropertyHelper.setValueDouble(ki, controller.updPropertyByName("Ki_task"))
    osim.PropertyHelper.setValueDouble(ki, controller.updPropertyByName("Ki_task_pitch"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("Ki_task_roll"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("Ki_task_yaw"))'''
assert old in code, "old string not found in cmaes_worker_p22.py"
code = code.replace(old, new, 1)
with open('cmaes_worker_p22.py', 'w', encoding='utf-8') as f:
    f.write(code)
print("cmaes_worker_p22.py updated successfully!")
