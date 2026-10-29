import os

path = "scripts/run_phase2_4_campaign.py"
with open(path, "r", encoding="utf-8") as f:
    code = f.read()

target = """    iteration = completed_gens
    while not es.stop() and iteration < target_stop_generation:
        iteration += 1"""

replacement = """    iteration = completed_gens
    while not es.stop() and iteration < target_stop_generation:
        pause_disabled = os.environ.get("DISABLE_NIGHT_PAUSE", "0") == "1"
        if not pause_disabled:
            now_dt = datetime.datetime.now()
            if (now_dt.hour == 22 and now_dt.minute >= 40) or (now_dt.hour >= 23 or now_dt.hour < 7):
                print(f"\\n[PAUSE_2300] Current time {now_dt.strftime('%H:%M:%S')} reached night pause window (23:00 - 07:00). Cleanly pausing generations.", flush=True)
                break

        iteration += 1"""

assert target in code, "ERROR: target not found!"
code = code.replace(target, replacement)

target2 = 'summary_path = f"logs/phase2_4_gen_{iteration}_summary.json"'
replacement2 = '''print(f"\\n[GEN COMPLETE] Generation {iteration}: Best Candidate W{best_idx+1}_P24G{iteration} Score = {best_score:.2f} pts (Duration: {gen_duration:.1f} min)", flush=True)
        summary_path = f"logs/phase2_4_gen_{iteration}_summary.json"'''

assert target2 in code, "ERROR: target2 not found!"
code = code.replace(target2, replacement2)

with open(path, "w", encoding="utf-8", newline="\r\n") as f:
    f.write(code)

print("SUCCESS: Patched scripts/run_phase2_4_campaign.py with night pause and concise score logging!")

