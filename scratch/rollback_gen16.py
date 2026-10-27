import json
import os
import glob
import sys

# 1. Update master ledger
with open("logs/phase2_3_master_ledger.json", "r", encoding="utf-8") as f:
    ledger = json.load(f)

ledger["generations"] = [g for g in ledger["generations"] if g["generation"] <= 15]

with open("logs/phase2_3_master_ledger.json", "w", encoding="utf-8") as f:
    json.dump(ledger, f, indent=2)

print(f"Rolled back master ledger to {len(ledger['generations'])} generations.")

# 2. Re-export progress matrix
sys.path.append("scratch")
import export_progress_matrix
export_progress_matrix.generate_matrix_md()
print("Re-exported logs/bode_progress_matrix.md successfully.")

# 3. Remove gen 16 summary
if os.path.exists("logs/phase2_3_gen_16_summary.json"):
    os.remove("logs/phase2_3_gen_16_summary.json")
    print("Removed logs/phase2_3_gen_16_summary.json.")

# 4. Remove gen 16 candidate files
for f in glob.glob("logs/candidates/*P23G16*.json"):
    os.remove(f)
print("Cleaned up Gen 16 candidate files.")

