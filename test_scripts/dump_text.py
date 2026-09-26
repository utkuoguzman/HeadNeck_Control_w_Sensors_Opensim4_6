import pypdf
reader = pypdf.PdfReader(r'C:\Users\User\.gemini\antigravity\brain\89fb8d56-9d12-4051-a256-02497ef8d6ae\.user_uploaded\media_1790013953948.pdf')
text = '\n'.join([p.extract_text() for p in reader.pages])
with open('text_dump.txt', 'w', encoding='utf-8') as f:
    f.write(text)
