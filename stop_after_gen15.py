import time
import os
import subprocess
import sys
from datetime import datetime

log_file = os.path.join("logs", "cmaes_p15_20261014_220448.log")
watcher_log = os.path.join("logs", "watcher_gen15.log")

with open(watcher_log, "w", encoding="utf-8") as f:
    f.write(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] Python Watcher started. Monitoring for Generation 15 completion...\n")

while True:
    time.sleep(10)
    if os.path.exists(log_file):
        try:
            with open(log_file, "r", encoding="utf-8", errors="ignore") as f:
                lines = f.readlines()[-60:]
            
            summary_found = any("PHASE 1.5 GENERATION 15 SUMMARY" in line for line in lines)
            if summary_found:
                with open(watcher_log, "a", encoding="utf-8") as f:
                    f.write(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] Generation 15 summary detected! Waiting 5s for disk flush...\n")
                
                time.sleep(5)
                
                # Terminate optimize_phase1_5.py using taskkill or wmic
                subprocess.run(
                    ["powershell", "-Command", "Get-CimInstance Win32_Process | Where-Object { $_.CommandLine -like '*optimize_phase1_5.py*' } | ForEach-Object { Stop-Process -Id $_.ProcessId -Force }"],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
                )
                
                with open(watcher_log, "a", encoding="utf-8") as f:
                    f.write(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] Successfully stopped optimize_phase1_5.py after Gen 15!\n")
                break
        except Exception as e:
            pass

