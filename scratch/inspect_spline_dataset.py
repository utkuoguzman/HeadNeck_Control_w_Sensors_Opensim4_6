import json

with open("logs/rom_plant_spline_dataset.json", "r", encoding="utf-8") as f:
    data = json.load(f)

for k in data.keys():
    if isinstance(data[k], list):
        print(f"  {k}: list of length {len(data[k])}")
    elif isinstance(data[k], dict):
        print(f"  {k}: dict with {len(data[k])} keys")
    else:
        print(f"  {k}: {data[k]}")

