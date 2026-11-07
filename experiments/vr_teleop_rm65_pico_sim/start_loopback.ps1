param([ValidateSet('grip', 'trigger')][string]$Clutch = 'grip')
$ErrorActionPreference = 'Stop'
$trialDir = $PSScriptRoot
$projectDir = (Resolve-Path -LiteralPath (Join-Path $trialDir '..\..')).Path
$python = Join-Path $projectDir '.venv\Scripts\python.exe'
$simDir = Join-Path $trialDir 'vr_teleop'
$inventory = Join-Path $trialDir 'real_readonly_inventory.json'
if (-not (Test-Path -LiteralPath $python)) { throw "Missing Python: $python" }
if (-not (Test-Path -LiteralPath $inventory)) { throw "Missing read-only inventory: $inventory" }
$logDir = Join-Path $trialDir 'logs'
New-Item -ItemType Directory -Path $logDir -Force | Out-Null
$trialLog = Join-Path $logDir ('loopback_' + [Guid]::NewGuid().ToString('N') + '.jsonl')
Write-Host 'RM65 SIMULATION + LOCALHOST MOCK ONLY. No connection to the physical controller.'
Write-Host "Clutch: $Clutch; log: $trialLog"
Push-Location $simDir
try {
    & $python -u .\example\teleop_sim.py --robot rm65 --input-source xrtoolkit `
        --xrtoolkit-hand left --xrtoolkit-clutch $Clutch --bridge-loopback-clutch $Clutch `
        --view-forward-axis +x --position-scale 0.5 --bridge-loopback-log $trialLog `
        --bridge-preview-inventory $inventory
    exit $LASTEXITCODE
} finally {
    Pop-Location
}
