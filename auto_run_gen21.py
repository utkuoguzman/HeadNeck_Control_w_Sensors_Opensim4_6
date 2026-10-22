import time
import os
import subprocess
import sys
from datetime import datetime

flag_file = os.path.join("logs", "gen_20_summary.json")
print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] Auto-launcher active: waiting for Generation 20 completion ({flag_file})...")

while True:
    time.sleep(15)
    if os.path.exists(flag_file):
        time.sleep(5)  # Wait for file flush
        print(f"\n[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] Generation 20 summary detected! Launching Generation 21 automatically...")
        cmd = [sys.executable, "-u", "optimize_gen21.py"]
        res = subprocess.run(cmd)
        print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] Generation 21 execution finished (Exit code: {res.returncode})")
        
        # Trigger post-campaign repository cleanup & reorganization
        print(f"\n[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] Launching post-campaign reorganization & cleanup...")
        cmd_clean = [sys.executable, "cleanup_after_campaign.py"]
        subprocess.run(cmd_clean)
        print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] Repository organization complete!")
        break
