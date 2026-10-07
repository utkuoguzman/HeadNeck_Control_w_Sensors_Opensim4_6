import re, glob
scores = {'F1':[], 'F2':[], 'F3':[], 'F4':[], 'F5':[], 'F6':[]}
for f in glob.glob('logs/cmaes_opt_*.log'):
    try:
        with open(f, encoding='utf-8', errors='ignore') as file:
            for line in file:
                if 'SUCCESS' in line:
                    for k in scores.keys():
                        if f'[{k}-' in line:
                            m = re.search(r'Score: ([\d\.]+)', line)
                            if m: scores[k].append(float(m.group(1)))
    except Exception as e:
        print("Error reading", f, e)
for k, v in scores.items():
    print(f'Best {k}: {min(v) if v else "N/A"}')
