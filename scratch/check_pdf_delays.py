import pypdf
import re
import sys

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

reader = pypdf.PdfReader('references/Harpee 2017 - Dynamic Head-Neck Stabilization and Modulation with multi-segment neuromuscular model.pdf')
text = ''.join([page.extract_text() for page in reader.pages])

print("Total characters extracted:", len(text))

matches = re.finditer(r'(delay|\btau\b|latency|\d+\s*ms\b|Hact|Hsc)', text, re.IGNORECASE)
found = 0
for match in matches:
    start = max(0, match.start() - 120)
    end = min(len(text), match.end() + 120)
    snippet = text[start:end].replace('\n', ' ')
    print(f"[{match.group()}]: ... {snippet} ...")
    print('---')
    found += 1
    if found >= 20:
        break

