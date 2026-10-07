from pathlib import Path

p = Path('optimize_6_independent.py')
content = p.read_text(encoding='utf-8')
content = content.replace('timeout=600', 'timeout=3600')
p.write_text(content, encoding='utf-8')
print('Timeout successfully raised to 3600 seconds (1 hour)!')
