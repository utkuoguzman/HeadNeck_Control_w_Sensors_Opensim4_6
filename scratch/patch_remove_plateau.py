with open("scripts/run_phase2_4_campaign.py", "r", encoding="utf-8") as f:
    code = f.read()

target1 = "    max_total_generations = 30"
repl1 = "    max_total_generations = int(os.environ.get('PHASE2_4_MAX_GEN', '50'))"
assert target1 in code, "target1 not found"
code = code.replace(target1, repl1)

target2 = """        # Check plateau stopping criterion
        if consecutive_non_improving >= 7:
            print(f"\\n[PLATEAU CRITERION MET] No score improvement for 7 consecutive generations. Stopping optimization loop.", flush=True)
            break"""

repl2 = """        # Check plateau stopping criterion
        max_non_improving = int(os.environ.get("MAX_NON_IMPROVING", "50"))
        if consecutive_non_improving >= max_non_improving:
            print(f"\\n[PLATEAU CRITERION MET] No score improvement for {max_non_improving} consecutive generations. Stopping optimization loop.", flush=True)
            break"""
assert target2 in code, "target2 not found"
code = code.replace(target2, repl2)

with open("scripts/run_phase2_4_campaign.py", "w", encoding="utf-8", newline="\r\n") as f:
    f.write(code)

print("SUCCESS: Relaxed plateau stopping rule to MAX_NON_IMPROVING (default 50) and increased max generations to 50!")

