import re, glob
best_params = {'F1 (0.31Hz)': (float('inf'), ""), 'F2 (0.78Hz)': (float('inf'), ""), 'F3 (1.11Hz)': (float('inf'), ""), 'F4 (1.56Hz)': (float('inf'), ""), 'F4 (2.48Hz)': (float('inf'), ""), 'F5 (2.35Hz)': (float('inf'), ""), 'F5 (5.52Hz)': (float('inf'), ""), 'F6 (7.95Hz)': (float('inf'), "")}

pattern = re.compile(r'Freq ([\d\.]+)Hz:\s*SSE=([\d\.]+)\s*\|\s*(.*)')

for f in glob.glob('logs/cmaes_opt_*.log'):
    try:
        with open(f, encoding='utf-8', errors='ignore') as file:
            for line in file:
                m = pattern.search(line)
                if m:
                    freq = float(m.group(1))
                    sse = float(m.group(2))
                    params = m.group(3).strip()
                    
                    key = None
                    if abs(freq - 0.31) < 0.05: key = 'F1 (0.31Hz)'
                    elif abs(freq - 0.78) < 0.05: key = 'F2 (0.78Hz)'
                    elif abs(freq - 1.11) < 0.05: key = 'F3 (1.11Hz)'
                    elif abs(freq - 1.56) < 0.05: key = 'F4 (1.56Hz)'
                    elif abs(freq - 2.48) < 0.05: key = 'F4 (2.48Hz)'
                    elif abs(freq - 2.35) < 0.05: key = 'F5 (2.35Hz)'
                    elif abs(freq - 5.52) < 0.05: key = 'F5 (5.52Hz)'
                    elif abs(freq - 7.95) < 0.05: key = 'F6 (7.95Hz)'
                    
                    if key and sse < best_params[key][0]:
                        best_params[key] = (sse, params)
    except Exception as e:
        pass

for k, v in best_params.items():
    if v[0] != float('inf'):
        print(f"{k} - Score: {v[0]:.2f}")
        print(f"   {v[1]}")
