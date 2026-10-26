import json

def generate_matrix_md():
    with open("logs/phase2_3_master_ledger.json", "r", encoding="utf-8") as f:
        data = json.load(f)
        
    bench = data["benchmark"]
    freqs = list(bench["bode"].keys())
    
    header = "| Campaign Milestone | " + " | ".join([f"{f} Hz" for f in freqs]) + " | Total Score |"
    sep = "| :--- | " + " | ".join([":---:"] * len(freqs)) + " | :---: |"
    
    lines = [header, sep]
    
    # Benchmark row
    h_cells = []
    for f in freqs:
        cell = bench["bode"][f]
        h_cells.append(f"{cell['gain_db']:+.1f} dB<br>/ {cell['phase_err']:.1f}°")
    lines.append(f"| **{bench['name']}** | " + " | ".join(h_cells) + f" | **`{bench['score']:.2f}`** |")
    
    # Generations
    for g in data["generations"]:
        cand_short = g["candidate"].split("_")[0]
        g_cells = []
        for f in freqs:
            cell = g["bode"].get(f, {"gain_db": 0.0, "phase_err": 0.0})
            g_cells.append(f"{cell['gain_db']:+.1f} dB<br>/ {cell['phase_err']:.1f}°")
        lines.append(f"| **Phase 2.3 Gen {g['generation']}** (`{cand_short}`) | " + " | ".join(g_cells) + f" | **`{g['score']:.2f}`** |")
        
    md_content = "\n".join(lines) + "\n"
    with open("logs/bode_progress_matrix.md", "w", encoding="utf-8") as out:
        out.write(md_content)
    print("Exported logs/bode_progress_matrix.md successfully!")

if __name__ == "__main__":
    generate_matrix_md()

