import subprocess

cmd = 'powershell -Command "Get-CimInstance Win32_Process -Filter \\"Name = \'python.exe\'\\" | ForEach-Object { $_.CommandLine }"'
out = subprocess.check_output(cmd, shell=True, text=True)
for line in out.splitlines():
    if "cmaes_worker" in line:
        parts = line.split()
        worker = parts[-1]
        freq = parts[-3]
        print(f"Worker {worker} running frequency {freq} Hz")

