import pypdf
import sys

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

reader = pypdf.PdfReader(r"references\Harpee 2017 - Dynamic Head-Neck Stabilization and Modulation with multi-segment neuromuscular model.pdf")

with open("scratch/happee_extracted_text.txt", "w", encoding="utf-8") as out:
    for i, page in enumerate(reader.pages):
        text = page.extract_text()
        out.write(f"\n\n{'='*30} PAGE {i+1} {'='*30}\n\n")
        out.write(text)

print("Saved all pages to scratch/happee_extracted_text.txt")

