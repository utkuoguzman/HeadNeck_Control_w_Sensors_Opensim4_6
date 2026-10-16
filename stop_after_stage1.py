import os
import sys
import time
import subprocess
import json
import re

LOG_FILE = r"C:\Users\User\.gemini\antigravity\brain\89fb8d56-9d12-4051-a256-02497ef8d6ae\.system_generated\tasks\task-17087.log"
TARGET_PROC = 32948

print("Monitoring Stage 1 completion to hold before Stage 2...")

while True:
    if os.path.exists(LOG_FILE):
        try:
            with open(LOG_FILE, "r", encoding="utf-8", errors="ignore") as f:
                content = f.read()
            if "[108/108]" in content or "STAGE 1: TURBO-EXACT (Original Ramp) FINISHED" in content:
                print("Stage 1 complete! Halting benchmark harness before Stage 2 starts...")
                # Kill master benchmark process
                subprocess.run(["taskkill", "/F", "/PID", str(TARGET_PROC)], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                
                # Parse scores from log
                pattern = r"\[STAGE 1\]\s+\[(W\d+)\]\s+passed\s+([\d\.]+)Hz\s+\(Score:\s+([\d\.]+)\)"
                matches = re.findall(pattern, content)
                results = {}
                for w, freq, sc in matches:
                    if w not in results: results[w] = {}
                    results[w][float(freq)] = float(sc)
                
                with open("logs/stage1_results.json", "w", encoding="utf-8") as out_f:
                    json.dump(results, out_f, indent=2)
                    
                with open("logs/stage1_completed_flag.txt", "w", encoding="utf-8") as flag_f:
                    flag_f.write(f"Completed at {time.ctime()}\nTotal jobs parsed: {len(matches)}\n")
                    
                print("Successfully stopped! Stage 1 saved. Ready for sleep.")
                break
        except Exception as e:
            print(f"Error checking log: {e}")
            
    time.sleep(2)

