import json
import re

pattern = re.compile(r'(\d+)\s*[- ]?\s*(?:dof|degrees?\s+of\s+freedom)', re.IGNORECASE)

with open(r'C:\Users\User\.gemini\antigravity\brain\89fb8d56-9d12-4051-a256-02497ef8d6ae\.system_generated\logs\transcript.jsonl', 'r', encoding='utf-8') as f:
    for line_idx, line in enumerate(f):
        data = json.loads(line)
        content = data.get('content', '')
        if isinstance(content, str):
            matches = pattern.findall(content)
            if matches:
                nums = set(matches)
                # Check for 8 or any other numbers
                if '8' in nums:
                    print(f"Line {line_idx} (step {data.get('step_index')}, {data.get('source')}): matches={nums}")
                    # print snippet around match
                    for m in pattern.finditer(content):
                        if m.group(1) == '8':
                            start = max(0, m.start() - 60)
                            end = min(len(content), m.end() + 60)
                            print("  SNIPPET:", content[start:end].replace('\n', ' '))

