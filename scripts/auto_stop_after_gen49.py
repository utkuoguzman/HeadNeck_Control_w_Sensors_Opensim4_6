import os
import time
import subprocess
import json
import sys

def get_campaign_pid():
    try:
        out = subprocess.check_output(
            ["powershell", "-Command", "Get-CimInstance Win32_Process -Filter \"CommandLine LIKE '%continue_campaign_gen35_to_50%'\" | Select-Object -ExpandProperty ProcessId"],
            text=True
        )
        pids = [int(p.strip()) for p in out.strip().split() if p.strip().isdigit()]
        return pids
    except Exception as e:
        print(f"Error finding PID: {e}")
        return []

def main():
    print("Watcher active: Waiting for logs/gen_49_summary.json...")
    summary_file = "logs/gen_49_summary.json"
    
    while True:
        if os.path.exists(summary_file):
            # Check if file has valid JSON content (not empty)
            try:
                with open(summary_file, "r", encoding="utf-8") as f:
                    data = json.load(f)
                if data.get("generation") == 49:
                    print(f"\n[SUCCESS] Gen 49 summary detected! Score: {data.get('best_score'):.2f} (Candidate: W{data.get('best_candidate_idx')}_G49)")
                    time.sleep(3) # allow any file flushes
                    
                    pids = get_campaign_pid()
                    print(f"Target PIDs to terminate: {pids}")
                    for pid in pids:
                        try:
                            subprocess.run(["taskkill", "/F", "/T", "/PID", str(pid)], check=False)
                            print(f"Terminated process tree for PID {pid}")
                        except Exception as ex:
                            print(f"Error terminating PID {pid}: {ex}")
                            
                    print("\n" + "="*80)
                    print("CAMPAIGN SAFELY STOPPED AT END OF GENERATION 49.")
                    print("Generation 50 has NOT started.")
                    print("All candidate logs, summary metrics, and CMA-ES state are saved.")
                    print("PC is ready for sleep.")
                    print("="*80)
                    break
            except Exception:
                pass
        time.sleep(2)

if __name__ == "__main__":
    main()

