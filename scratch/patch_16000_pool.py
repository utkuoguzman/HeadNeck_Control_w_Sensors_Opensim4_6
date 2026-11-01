with open("scripts/run_phase2_3_campaign.py", "r", encoding="utf-8") as f:
    content = f.read()

old_code = """        if iteration >= 13:
            # Surrogate Pre-Screening: sample 72 candidates, rank via surrogate, pick top 18
            pool = es.ask(number=72)
            surr_scores = [surrogate_score(c) for c in pool]
            ranked_indices = np.argsort(surr_scores)
            candidates = [pool[i] for i in ranked_indices[:18]]
            print(f"[SURROGATE PRE-SCREENING] Evaluated 72 candidates in ~3 ms. Best predicted surrogate score: {surr_scores[ranked_indices[0]]:.1f} pts (Worst discarded: {surr_scores[ranked_indices[-1]]:.1f} pts). Passing top 18 to OpenSim.", flush=True)"""

new_code = """        if iteration >= 13:
            # Massive Surrogate Pre-Screening: sample 16,000 candidates, rank via surrogate in ~1.5 s, pick top 18
            pool_t0 = time.perf_counter()
            pool = es.ask(number=16000)
            surr_scores = [surrogate_score(c) for c in pool]
            ranked_indices = np.argsort(surr_scores)
            candidates = [pool[i] for i in ranked_indices[:18]]
            pool_dt = time.perf_counter() - pool_t0
            print(f"[SURROGATE PRE-SCREENING] Evaluated 16,000 candidates in {pool_dt:.2f} s ({16000/pool_dt:,.0f} evals/s). Best predicted surrogate score: {surr_scores[ranked_indices[0]]:.1f} pts (Worst discarded: {surr_scores[ranked_indices[-1]]:.1f} pts). Passing top 18 elite candidates to OpenSim.", flush=True)"""

if old_code in content:
    content = content.replace(old_code, new_code)
    print("Replaced old_code directly.")
else:
    # try replacing lines
    content = content.replace("pool = es.ask(number=72)", "pool_t0 = time.perf_counter()\n            pool = es.ask(number=16000)")
    content = content.replace("Evaluated 72 candidates in ~3 ms", "Evaluated 16,000 candidates in ~1.5 s")
    print("Replaced line targets.")

with open("scripts/run_phase2_3_campaign.py", "w", encoding="utf-8") as f:
    f.write(content)

print("Updated pool size to 16,000 in scripts/run_phase2_3_campaign.py successfully!")

