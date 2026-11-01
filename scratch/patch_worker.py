path = "cmaes_worker_p22.py"
with open(path, "r", encoding="utf-8") as f:
    c = f.read()

target = 'osim.PropertyHelper.setValueDouble(0.015, controller.updPropertyByName("delay_ccr"))'
replacement = """osim.PropertyHelper.setValueDouble(0.015, controller.updPropertyByName("delay_ccr"))
    osim.PropertyHelper.setValueBool(True, controller.updPropertyByName("use_smith_predictor"))
    osim.PropertyHelper.setValueDouble(0.007, controller.updPropertyByName("delay_smith_proprio"))
    osim.PropertyHelper.setValueDouble(0.007, controller.updPropertyByName("delay_afferent_proprio"))"""

if target in c:
    c = c.replace(target, replacement)
    with open(path, "w", encoding="utf-8") as f:
        f.write(c)
    print("cmaes_worker_p22.py patched successfully!")
else:
    print("Error: target not found in cmaes_worker_p22.py!")
