param([string]$OfflineInventory, [string]$RealAddress, [string]$OutputDir)
$ErrorActionPreference = 'Stop'
$trialDir = $PSScriptRoot
$projectDir = (Resolve-Path -LiteralPath (Join-Path $trialDir '..\..')).Path
$python = Join-Path $projectDir '.venv\Scripts\python.exe'
$probe = Join-Path $projectDir 'scripts\probe_real_robot.py'
$check = Join-Path $trialDir 'hardware_preflight.py'
if (-not $OutputDir) { $OutputDir = Join-Path $trialDir 'logs\readonly_preflight' }
New-Item -ItemType Directory -Path $OutputDir -Force | Out-Null
$runId = [Guid]::NewGuid().ToString('N')
$resultPath = Join-Path $OutputDir ("preflight_$runId.json")
$snapshotPath = Join-Path $OutputDir ("sim_snapshot_$runId.json")

if ($OfflineInventory) {
    $inventoryPath = (Resolve-Path -LiteralPath $OfflineInventory).Path
    Write-Host 'Checking an existing read-only inventory; no robot connection.'
} else {
    if (-not $RealAddress) {
        $config = Get-Content -LiteralPath (Join-Path $projectDir 'config\sim_real_bridge.local.json') -Raw | ConvertFrom-Json
        $RealAddress = [string]$config.real_addr
    }
    $parts = $RealAddress.Split(':')
    $parsedIp = $null
    $parsedPort = 0
    if ($parts.Count -ne 2 -or -not [Net.IPAddress]::TryParse($parts[0], [ref]$parsedIp) -or
        $parsedIp.AddressFamily -ne [Net.Sockets.AddressFamily]::InterNetwork -or
        -not [int]::TryParse($parts[1], [ref]$parsedPort) -or $parsedPort -lt 1 -or $parsedPort -gt 65535) {
        throw 'Expected controller address in IPv4:port format.'
    }
    try { $connections = @(Get-NetTCPConnection -ErrorAction Stop) }
    catch { throw "Cannot inspect controller connections; no query was started: $($_.Exception.Message)" }
    $existing = @($connections | Where-Object {
        $_.RemoteAddress -eq $parsedIp.ToString() -and $_.RemotePort -eq $parsedPort -and $_.State -eq 'Established'
    })
    if ($existing.Count) { throw 'This PC already has a controller connection; close that session before a separate read-only query.' }
    $inventoryPath = Join-Path $OutputDir ("inventory_$runId.json")
    Write-Host "Read-only controller queries: $RealAddress. Gripper status is queried; no motion, enable, gripper-action or teach commands."
    & $python $probe --real $RealAddress --out $inventoryPath
    if ($LASTEXITCODE -ne 0) { throw "Read-only inventory failed: $inventoryPath" }
}

& $python $check --inventory $inventoryPath --output $resultPath
if ($LASTEXITCODE -ne 0) { throw "Read-only model preflight failed: $inventoryPath" }
$inventory = Get-Content -LiteralPath $inventoryPath -Raw | ConvertFrom-Json
$state = $inventory.responses.get_current_arm_state.arm_state
$snapshot = [ordered]@{
    captured_utc = $inventory.created_utc
    source = "read-only inventory: $inventoryPath"
    joint_units = 'degree'
    joint_deg = @($state.joint | ForEach-Object { [double]$_ / 1000.0 })
    reported_tcp = [ordered]@{
        position_mm = @($state.pose[0..2] | ForEach-Object { [double]$_ / 1000.0 })
        rpy_rad = @($state.pose[3..5] | ForEach-Object { [double]$_ / 1000.0 })
    }
    note = 'Recorded simulation start only; not live feedback.'
}
[IO.File]::WriteAllText($snapshotPath, ($snapshot | ConvertTo-Json -Depth 6), [Text.UTF8Encoding]::new($false))
Write-Host "Read-only inventory: $inventoryPath"
Write-Host "Offline preflight: $resultPath"
Write-Host "Simulation snapshot: $snapshotPath"
Write-Host 'No physical motion was requested; this snapshot is not live feedback.'
