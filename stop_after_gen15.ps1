# Watcher to stop optimization cleanly after Generation 15 finishes
$logFile = "logs\cmaes_p15_20261014_220448.log"
$watcherLog = "logs\watcher_gen15.log"

"[" + (Get-Date -Format "yyyy-MM-dd HH:mm:ss") + "] Watcher started. Monitoring for Generation 15 completion..." | Out-File $watcherLog -Encoding utf8

while ($true) {
    Start-Sleep -Seconds 10
    if (Test-Path $logFile) {
        $content = Get-Content $logFile -Tail 50 -ErrorAction SilentlyContinue
        $found = $content | Where-Object { $_ -like "*PHASE 1.5 GENERATION 15 SUMMARY*" }
        if ($found) {
            "[" + (Get-Date -Format "yyyy-MM-dd HH:mm:ss") + "] Generation 15 summary detected! Waiting 5s for disk flush..." | Out-File $watcherLog -Append -Encoding utf8
            Start-Sleep -Seconds 5
            
            # Find optimize_phase1_5.py process
            $proc = Get-Process python -ErrorAction SilentlyContinue | Where-Object { 
                $cmd = (Get-CimInstance Win32_Process -Filter "ProcessId = $($_.Id)" -ErrorAction SilentlyContinue).CommandLine
                $cmd -like "*optimize_phase1_5.py*"
            }
            if ($proc) {
                Stop-Process -Id $proc.Id -Force -ErrorAction SilentlyContinue
                "[" + (Get-Date -Format "yyyy-MM-dd HH:mm:ss") + "] Successfully stopped optimize_phase1_5.py (PID $($proc.Id)) after Gen 15!" | Out-File $watcherLog -Append -Encoding utf8
            } else {
                "[" + (Get-Date -Format "yyyy-MM-dd HH:mm:ss") + "] optimize_phase1_5.py process already terminated." | Out-File $watcherLog -Append -Encoding utf8
            }
            break
        }
    }
}

