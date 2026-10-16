import json
import os
import sys
import time
import subprocess
import concurrent.futures

# Fixed task controller parameters (Phase 1)
FIXED_W_N = 10.1
FIXED_ZETA = 1.98
FIXED_KI = 40.2786

TARGET_FREQ = 0.31
TARGET_GAIN = 0.893
TARGET_PHASE = 2.5

def load_candidates():
    cand_dir = os.path.join("logs", "candidates")
    candidates = {}
    for i in range(1, 19):
        w_name = f"W{i}"
        f_path = os.path.join(cand_dir, f"{w_name}.json")
        if os.path.exists(f_path):
            with open(f_path, "r", encoding="utf-8") as f:
                data = json.load(f)
                candidates[w_name] = data["params"]
        else:
            raise FileNotFoundError(f"Missing candidate file: {f_path}")
    return candidates

def evaluate_job(w_name, candidate):
    cmd = [
        sys.executable, "cmaes_worker_turbo_ramp.py",
        str(FIXED_W_N), str(FIXED_ZETA), str(FIXED_KI),
        *[str(v) for v in candidate],
        str(TARGET_FREQ), str(TARGET_GAIN), str(TARGET_PHASE),
        w_name
    ]
    t0 = time.time()
    res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    el = time.time() - t0
    if res.returncode == 0 and res.stdout.strip():
        score = float(res.stdout.strip().split("\n")[-1])
        return score, el
    else:
        return 999999.0, el

def main():
    sys.stdout.reconfigure(line_buffering=True)
    print("=" * 85)
    print("STAGE 2: TURBO-RAMP TOLERANCE BENCHMARK (1.32s Shortened Ramp on 0.31Hz)")
    print("Evaluating all 18 Candidates across 16 Workers")
    print("=" * 85)
    
    candidates = load_candidates()
    print(f"Loaded {len(candidates)} candidate vectors (W1 to W18).")
    
    # Load Stage 1 ground truth baseline
    with open("logs/stage1_results.json", "r", encoding="utf-8") as f:
        stage1_results = json.load(f)
    print("Loaded Stage 1 exact results for side-by-side comparison.\n")
    
    print(f"Dispatching 18 candidates on 0.31Hz to 16 workers...")
    start_time = time.time()
    stage2_031_scores = {}
    
    with concurrent.futures.ThreadPoolExecutor(max_workers=16) as executor:
        future_to_cand = {
            executor.submit(evaluate_job, w_name, cand): w_name
            for w_name, cand in candidates.items()
        }
        
        completed_count = 0
        for future in concurrent.futures.as_completed(future_to_cand):
            w_name = future_to_cand[future]
            score, el = future.result()
            stage2_031_scores[w_name] = score
            completed_count += 1
            
            s1_031 = stage1_results[w_name]["0.31"]
            delta = score - s1_031
            pct = (delta / s1_031 * 100.0) if s1_031 != 0 else 0.0
            print(f"[STAGE 2] [{w_name:3s}] 0.31Hz: S1={s1_031:5.1f} | S2={score:5.1f} | Delta={delta:+6.2f} ({pct:+6.1f}%) | Time={el:5.1f}s [{completed_count:2d}/18]", flush=True)
            
    elapsed_total = time.time() - start_time
    print(f"\nAll 18 candidates evaluated on 0.31Hz in {elapsed_total:.2f}s ({elapsed_total/60.0:.2f} min)!\n", flush=True)
    
    # Build complete 6-frequency comparison for all candidates
    # (Other 5 frequencies are mathematically identical between p15 and turbo_ramp)
    comparison_data = {
        "stage2_031_wallclock_sec": elapsed_total,
        "candidates": {}
    }
    
    print("=" * 100)
    print("TURBO-BENCHMARK COMPARISON TABLE: STAGE 1 (Original 3.23s Ramp) vs STAGE 2 (Shortened 1.32s Ramp)")
    print("=" * 100)
    print(f"{'Candidate':<10} | {'S1 (0.31Hz)':<12} | {'S2 (0.31Hz)':<12} | {'0.31Hz Delta':<14} | {'S1 Total':<10} | {'S2 Total':<10} | {'Total Delta (%)':<15}")
    print("-" * 100)
    
    # Rank candidates by S2 Total
    ranked_candidates = []
    for i in range(1, 19):
        w_name = f"W{i}"
        s1_w = stage1_results[w_name]
        s1_031 = s1_w["0.31"]
        s2_031 = stage2_031_scores[w_name]
        d_031 = s2_031 - s1_031
        
        # S1 total across all 6 freqs
        s1_tot = sum(s1_w.values())
        
        # S2 total (5 identical freqs + S2 0.31Hz)
        s2_tot = sum(score for freq_str, score in s1_w.items() if freq_str != "0.31") + s2_031
        d_tot = s2_tot - s1_tot
        pct_tot = (d_tot / s1_tot * 100.0) if s1_tot != 0 else 0.0
        
        ranked_candidates.append({
            "worker": w_name,
            "s1_031": s1_031,
            "s2_031": s2_031,
            "d_031": d_031,
            "s1_total": s1_tot,
            "s2_total": s2_tot,
            "d_total": d_tot,
            "pct_total": pct_tot
        })
        
    ranked_candidates.sort(key=lambda x: x["s2_total"])
    
    for c in ranked_candidates:
        w_name = c["worker"]
        comparison_data["candidates"][w_name] = c
        print(f"{w_name:<10} | {c['s1_031']:12.1f} | {c['s2_031']:12.1f} | {c['d_031']:+14.2f} | {c['s1_total']:10.1f} | {c['s2_total']:10.1f} | {c['pct_total']:+14.2f}%")
        
    print("-" * 100)
    
    # Winner analysis
    s1_winner = min(ranked_candidates, key=lambda x: x["s1_total"])
    s2_winner = min(ranked_candidates, key=lambda x: x["s2_total"])
    print(f"\nStage 1 Best Candidate: {s1_winner['worker']} (Score: {s1_winner['s1_total']:.2f})")
    print(f"Stage 2 Best Candidate: {s2_winner['worker']} (Score: {s2_winner['s2_total']:.2f})")
    
    with open("logs/turbo_benchmark_comparison.json", "w", encoding="utf-8") as f:
        json.dump(comparison_data, f, indent=2)
        
    print("\nComparison successfully saved to logs/turbo_benchmark_comparison.json")

if __name__ == "__main__":
    main()

