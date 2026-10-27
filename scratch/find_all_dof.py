import json
import re

pattern = re.compile(r'(\d+)\s*[- ]?\s*(?:dof|degrees?\s+of\s+freedom)', re.IGNORECASE)
found_counts = {}

with open(r'C:\Users\User\.gemini\antigravity\brain\89fb8d56-9d12-4051-a256-02497ef8d6ae\.system_generated\logs\transcript.jsonl', 'r', encoding='utf-8') as f:
    for line_idx, line in enumerate(f):
        data = json.loads(line)
        content = data.get('content', '')
        if isinstance(content, str):
            for m in pattern.finditer(content):
                num = m.group(1)
                found_counts[num] = found_counts.get(num, 0) + 1

print("Found DOF counts:", found_counts)

