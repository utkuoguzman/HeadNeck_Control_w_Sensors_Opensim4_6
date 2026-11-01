path = "scripts/run_phase2_4_campaign.py"
with open(path, "r", encoding="utf-8") as f:
    c = f.read()

target = "'popsize': 18,"
replacement = "'popsize': 14,"

if target in c:
    c = c.replace(target, replacement, 1)
    with open(path, "w", encoding="utf-8") as f:
        f.write(c)
    print("scripts/run_phase2_4_campaign.py initial popsize set to 14!")
else:
    print("Error: target not found!")

