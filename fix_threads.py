from pathlib import Path

p = Path('cmaes_worker.py')
content = p.read_text(encoding='utf-8')
content = content.replace('os.environ["OMP_NUM_THREADS"] = "3"', 'os.environ["OMP_NUM_THREADS"] = "1"')
p.write_text(content, encoding='utf-8')
print('Fixed thread thrashing!')
