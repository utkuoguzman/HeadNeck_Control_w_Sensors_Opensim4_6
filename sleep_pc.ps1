# Bedtime Sleep Script
$logFile = Join-Path $PSScriptRoot "logs\bedtime_sleep.log"
$timestamp = Get-Date -Format "yyyy-MM-dd HH:mm:ss"
"[$timestamp] Bedtime sleep triggered. Putting PC to sleep..." | Out-File -FilePath $logFile -Append -Encoding utf8

# Put PC to sleep (Suspend to RAM, silent fans)
Add-Type -AssemblyName System.Windows.Forms
[System.Windows.Forms.Application]::SetSuspendState([System.Windows.Forms.PowerState]::Suspend, $false, $false)

