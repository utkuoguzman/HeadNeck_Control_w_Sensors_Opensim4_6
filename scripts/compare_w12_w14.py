import json

with open("logs/candidates/W12_P21G1_1.11Hz.json") as f:
    d12 = json.load(f)

with open("logs/candidates/W14_P21G1_1.11Hz.json") as f:
    d14 = json.load(f)

print("Parameter comparison:")
names = ["K_ton", "p_ton", "K_sc", "z_sc", "p_sc", "K_phas", "p_phas", "kp_prop", "z_ccr", "p_ccr", "k_dyn", "k_stat"]
for name, p12, p14 in zip(names, d12["params"], d14["params"]):
    print(f"  {name:<12}: W12 = {p12:8.2f} | W14 = {p14:8.2f}")

