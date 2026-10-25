import os
import shutil
import glob

def clean_and_organize(force=False):
    if not force:
        print("[INFO] Campaign extended to Generation 50. Deferring repository cleanup until after Gen 50 completes.")
        return

    # 1. Target Directories
    dirs = ["logs", "state_files", "osim_files", "research_papers"]
    for d in dirs:
        os.makedirs(d, exist_ok=True)
        print(f"[DIR] Verified directory: {d}/")

    # 2. Remove unnecessary files
    junk_files = [
        "text_dump.txt",
        "crash_log.txt",
        "temp_test.txt"
    ]
    for jf in junk_files:
        if os.path.exists(jf):
            try:
                os.remove(jf)
                print(f"[REMOVED JUNK] {jf}")
            except Exception as e:
                print(f"[ERROR] Could not remove {jf}: {e}")

    # 3. Move research papers (*.pdf) to research_papers/
    pdf_files = glob.glob("*.pdf")
    for pdf in pdf_files:
        dest = os.path.join("research_papers", os.path.basename(pdf))
        try:
            shutil.move(pdf, dest)
            print(f"[MOVED PAPER] {pdf} -> {dest}")
        except Exception as e:
            print(f"[ERROR] Moving {pdf}: {e}")

    # 4. Move root log and log-related txt files to logs/
    log_files = glob.glob("*.log") + glob.glob("*log*.txt") + [
        "out.txt",
        "good_parameters.txt",
        "tf_initialization.txt"
    ]
    for lf in set(log_files):
        if os.path.exists(lf):
            dest = os.path.join("logs", os.path.basename(lf))
            try:
                # If target exists in logs/, overwrite or merge
                if os.path.exists(dest):
                    os.remove(dest)
                shutil.move(lf, dest)
                print(f"[MOVED LOG] {lf} -> {dest}")
            except Exception as e:
                print(f"[ERROR] Moving {lf}: {e}")

    # 5. Move root .mot and .sto state files to state_files/
    state_files = glob.glob("*.mot") + glob.glob("*.sto")
    for sf in state_files:
        dest = os.path.join("state_files", os.path.basename(sf))
        try:
            if os.path.exists(dest):
                os.remove(dest)
            shutil.move(sf, dest)
            print(f"[MOVED STATE] {sf} -> {dest}")
        except Exception as e:
            print(f"[ERROR] Moving {sf}: {e}")

    # 6. Move root .osim model files to osim_files/
    osim_files = glob.glob("*.osim")
    for of in osim_files:
        dest = os.path.join("osim_files", os.path.basename(of))
        try:
            if os.path.exists(dest):
                os.remove(dest)
            shutil.move(of, dest)
            print(f"[MOVED OSIM] {of} -> {dest}")
        except Exception as e:
            print(f"[ERROR] Moving {of}: {e}")

    print("="*80)
    print("CLEANUP & REORGANIZATION COMPLETE")
    print("="*80)

if __name__ == "__main__":
    clean_and_organize()

