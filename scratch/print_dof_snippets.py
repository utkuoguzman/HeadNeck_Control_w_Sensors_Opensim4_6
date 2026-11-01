import json
import re

pattern = re.compile(r'(\d+)\s*[- ]?\s*(?:dof|degrees?\s+of\s+freedom)', re.IGNORECASE)

with open(r'C:\Users\User\.gemini\antigravity\brain\89fb8d56-9d12-4051-a256-02497ef8d6ae\.system_generated\logs\transcript.jsonl', 'r', encoding='utf-8') as f:
    for line_idx, line in enumerate(f):
        data = json.loads(line)
        content = data.get('content', '')
        if isinstance(content, str):
            for m in pattern.finditer(content):
                num = m.group(1)
                if num in ['7', '8', '2', '24']:
                    start = max(0, m.start() - 100)
                    end = min(len(content), m.end() + 100)
                    print(f"[{num}-DOF at step {data.get('step_index')}] {content[start:end].replace(chr(10), ' ')}")

