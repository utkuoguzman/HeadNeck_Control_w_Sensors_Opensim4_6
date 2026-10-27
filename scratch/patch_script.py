path = "scratch/prototype_online_fxrls.py"
with open(path, "r", encoding="utf-8") as f:
    c = f.read()

c = c.replace("/controllerset/VCR_CCR_Controller/luen_w_pitch", "/controllerset/VCR_CCR_Controller/luen_omega_pitch")

with open(path, "w", encoding="utf-8") as f:
    f.write(c)

print("Updated state variable name to luen_omega_pitch!")

