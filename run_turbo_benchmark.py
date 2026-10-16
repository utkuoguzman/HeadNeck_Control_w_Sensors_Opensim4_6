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

TARGETS = [
    (7.95, 0.443, -29.8),
    (5.52, 0.509, -29.0),
    (2.48, 0.655, -20.6),
    (1.11, 0.814, -10.9),
    (0.78, 0.840,  -8.1),
    (0.31, 0.893,   2.5)
]

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

def evaluate_job(worker_script, w_name, candidate, freq, t_gain, t_phase):
    cmd = [
        sys.executable, worker_script,
        str(FIXED_W_N), str(FIXED_ZETA), str(FIXED_KI),
        *[str(v) for v in candidate],
        str(freq), str(t_gain), str(t_phase),
        w_name
    ]
    res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if res.returncode == 0 and res.stdout.strip():
        score = float(res.stdout.strip().split("\n")[-1])
        return score
    else:
        return 999999.0

def run_stage(stage_name, worker_script, candidates, max_workers=16):
    print(f"\n{'='*70}\nSTARTING {stage_name} (Using {max_workers} Persistent Workers)\n{'='*70}")
    start_time = time.time()
    
    # 18 candidates x 6 frequencies = 108 jobs
    jobs = []
    for w_name, cand in candidates.items():
        for freq, t_gain, t_phase in TARGETS:
            jobs.append((w_name, cand, freq, t_gain, t_phase))
            
    results = {w_name: {} for w_name in candidates}
    
    with concurrent.futures.ThreadPoolExecutor(max_workers=max_workers) as executor:
        future_to_job = {}
        for (w_name, cand, freq, t_gain, t_phase) in jobs:
            f = executor.submit(evaluate_job, worker_script, w_name, cand, freq, t_gain, t_phase)
            future_to_job[f] = (w_name, freq)
            
        completed_count = 0
        for future in concurrent.futures.as_completed(future_to_job):
            w_name, freq = future_to_job[future]
            score = future.result()
            results[w_name][freq] = score
            completed_count += 1
            stage_tag = stage_name.split(":")[0]
            print(f"[{stage_tag}] [{w_name}] passed {freq}Hz (Score: {score:.1f}) [{completed_count}/108]", flush=True)
                
    elapsed_total = time.time() - start_time
    print(f"\n{stage_name} FINISHED in {elapsed_total:.2f}s ({elapsed_total/60.0:.2f} min)!\n", flush=True)
    return results, elapsed_total

def main():
    sys.stdout.reconfigure(line_buffering=True)
    print("Turbocharged Engine Validation Harness", flush=True)
    print("Loading Generation 15 Candidate Vectors...", flush=True)
    candidates = load_candidates()
    print(f"Successfully loaded {len(candidates)} candidates (W1 to W18).")
    
    # Stage 1: Exact Reproduction Benchmark (Original Ramp)
    stage1_res, stage1_time = run_stage("STAGE 1: TURBO-EXACT (Original Ramp)", "cmaes_worker_p15.py", candidates, max_workers=16)
    
    # Stage 2: Ramp Tolerance Benchmark (1.32s Ramp)
    stage2_res, stage2_time = run_stage("STAGE 2: TURBO-RAMP (1.32s Ramp on 0.31Hz)", "cmaes_worker_turbo_ramp.py", candidates, max_workers=16)
    
    # Save benchmark results
    benchmark_data = {
        "stage1_time": stage1_time,
        "stage2_time": stage2_time,
        "stage1_results": stage1_res,
        "stage2_results": stage2_res
    }
    with open("logs/turbo_benchmark_results.json", "w", encoding="utf-8") as f:
        json.dump(benchmark_data, f, indent=2)
        
    print("\n" + "="*80)
    print("TURBOCHARGED BENCHMARK SUMMARY")
    print("="*80)
    print(f"Stage 1 (Exact Reproduction) Runtime: {stage1_time/60.0:.2f} min")
    print(f"Stage 2 (Shortened Ramp)      Runtime: {stage2_time/60.0:.2f} min")
    print(f"Speedup from 1.32s Ramp:             {(stage1_time/stage2_time):.2f}x faster on Stage 2")
    
if __name__ == "__main__":
    main()

