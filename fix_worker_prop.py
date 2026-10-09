import os
import re

with open("cmaes_worker_tf.py", "r", encoding="utf-8") as f:
    code = f.read()

old_block = '''    osim.PropertyHelper.setValueDouble(g_ton, controller.updPropertyByName("G_ton"))
    osim.PropertyHelper.setValueDouble(g_sc, controller.updPropertyByName("G_sc"))
    osim.PropertyHelper.setValueDouble(g_phas, controller.updPropertyByName("G_phas"))'''

new_block = '''    osim.PropertyHelper.setValueDouble(K_ton, controller.updPropertyByName("K_ton"))
    osim.PropertyHelper.setValueDouble(z_ton, controller.updPropertyByName("z_ton"))
    osim.PropertyHelper.setValueDouble(p_ton, controller.updPropertyByName("p_ton"))
    osim.PropertyHelper.setValueDouble(K_sc, controller.updPropertyByName("K_sc"))
    osim.PropertyHelper.setValueDouble(z_sc, controller.updPropertyByName("z_sc"))
    osim.PropertyHelper.setValueDouble(p_sc, controller.updPropertyByName("p_sc"))
    osim.PropertyHelper.setValueDouble(K_phas, controller.updPropertyByName("K_phas"))
    osim.PropertyHelper.setValueDouble(z_phas, controller.updPropertyByName("z_phas"))
    osim.PropertyHelper.setValueDouble(p_phas, controller.updPropertyByName("p_phas"))'''

code = code.replace(old_block, new_block)

with open("cmaes_worker_tf.py", "w", encoding="utf-8") as f:
    f.write(code)

print("Fixed PropertyHelper")

