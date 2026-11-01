import pypdf
import re

reader = pypdf.PdfReader(r"references\Harpee 2017 - Dynamic Head-Neck Stabilization and Modulation with multi-segment neuromuscular model.pdf")
print("Num pages:", len(reader.pages))

full_text = ""
for i, page in enumerate(reader.pages):
    text = page.extract_text()
    if "Figure 5" in text or "frequency response" in text.lower() or "bode" in text.lower() or "°/m" in text or "deg/m" in text:
        print(f"\n--- PAGE {i+1} ---")
        for line in text.split("\n"):
            if any(k in line.lower() for k in ["bode", "gain", "phase", "figure 5", "angular velocity", "°/m", "deg/m", "transfer function", "co-contraction", "reflex"]):
                print("  ", line)

