import json

with open("logs/rom_plant_spline_dataset.json", "r", encoding="utf-8") as f:
    data = json.load(f)

with open("logs/cervical_72_muscle_parameters.json", "r", encoding="utf-8") as f:
    musc_params = json.load(f)

# Print a few key flexors and extensors
extensors = ["splen_cap_sklc6", "semi_cap_sklc5", "rectcap_post_maj", "trap_cl"]
flexors = ["stern_mast", "cleid_mast", "scalenus_ant", "long_cap_sklc4"]

print("EXTENSORS at equilibrium (knot 45, ~0 deg):")
for m in extensors:
    r1 = data["muscle_moment_arms_pitch1"][m][45]
    r2 = data["muscle_moment_arms_pitch2"][m][45]
    f0 = musc_params[m]["F0"]
    l0 = musc_params[m]["l0m"]
    lst = musc_params[m]["lst"]
    print(f"  {m:<18}: r1={r1:>7.4f} m, r2={r2:>7.4f} m, F0={f0:>6.1f} N, l0={l0*100:>4.1f} cm, lst={lst*100:>4.1f} cm")

print("\nFLEXORS at equilibrium (knot 45, ~0 deg):")
for m in flexors:
    r1 = data["muscle_moment_arms_pitch1"][m][45]
    r2 = data["muscle_moment_arms_pitch2"][m][45]
    f0 = musc_params[m]["F0"]
    l0 = musc_params[m]["l0m"]
    lst = musc_params[m]["lst"]
    print(f"  {m:<18}: r1={r1:>7.4f} m, r2={r2:>7.4f} m, F0={f0:>6.1f} N, l0={l0*100:>4.1f} cm, lst={lst*100:>4.1f} cm")

