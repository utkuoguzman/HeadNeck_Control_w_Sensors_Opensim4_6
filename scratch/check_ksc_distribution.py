import json
import glob

files = sorted(glob.glob('logs/candidates/*P23G4_0.32Hz.json'))
print(f"Total candidates evaluated in Gen 4: {len(files)}")
print(f"{'Worker':<12} | {'K_sc':<8} | {'K_ton':<8} | {'p_ton':<8} | {'kp':<8} | {'K_phas':<8} | {'Score':<10}")
print("-" * 75)
for f in files:
    with open(f) as fp:
        d = json.load(fp)
    p = d['params']
    print(f"{d['worker']:<12} | {p[2]:<8.2f} | {p[0]:<8.2f} | {p[1]:<8.2f} | {p[7]:<8.2f} | {p[5]:<8.2f} | {d['score']:<10.1f}")

