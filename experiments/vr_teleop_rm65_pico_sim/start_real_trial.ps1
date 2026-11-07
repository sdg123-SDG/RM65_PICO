param(
    [ValidateSet('+x', '-x')][string]$Forward = '+x',
    [ValidateSet('grip')][string]$Clutch = 'grip',
    # hand: base joint follows the hand's horizontal swing (scheme B); ik: original full IK
    [ValidateSet('hand', 'ik')][string]$BaseYaw = 'hand',
    # IK orientation weight; lower favours position over wrist orientation.
    # Default: 0.3 with -BaseYaw hand, 1.0 (original) with -BaseYaw ik.
    [ValidateRange(0.05, 2.0)][double]$RotWeight = 0,
    [string]$RealAddress
)
$ErrorActionPreference = 'Stop'
$trialDir = $PSScriptRoot
$projectDir = (Resolve-Path -LiteralPath (Join-Path $trialDir '..\..')).Path
$python = Join-Path $projectDir '.venv\Scripts\python.exe'
$simDir = Join-Path $trialDir 'vr_teleop'
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
catch { throw "Cannot inspect local controller connections: $($_.Exception.Message)" }
$busy = @($connections | Where-Object {
    $_.RemoteAddress -eq $parsedIp.ToString() -and $_.RemotePort -eq $parsedPort -and $_.State -eq 'Established'
})
if ($busy.Count) { throw 'This PC already has a connection to the controller. Close the existing session first.' }
$guards = @(Get-CimInstance Win32_Process | Where-Object {
    $_.Name -match '^python(w)?\.exe$' -and $_.CommandLine -match '(real_arm_guard\.py|real_pico_guard\.py)'
})
if ($guards.Count) { throw 'An existing robot guard was found. Inspect it before starting another session.' }
if (-not (Test-Path -LiteralPath $python)) { throw "Missing Python: $python" }

$serviceExe = Join-Path $projectDir 'software\XRoboToolkit-PC-Service\RoboticsServiceProcess.exe'
$services = @(Get-Process RoboticsServiceProcess -ErrorAction SilentlyContinue)
if ($services.Count -gt 1) { throw 'Multiple XRoboToolkit PC Service processes found.' }
if ($services.Count -eq 0) {
    if (-not (Test-Path -LiteralPath $serviceExe)) { throw "Missing PC Service: $serviceExe" }
    $startedService = Start-Process -FilePath $serviceExe -WorkingDirectory (Split-Path $serviceExe) -WindowStyle Hidden -PassThru
    Start-Sleep -Milliseconds 1200
    if ($startedService.HasExited) { throw 'XRoboToolkit PC Service exited during startup.' }
}
$forwardArgument = '--view-forward-axis=' + $Forward.ToLowerInvariant()
if (-not $PSBoundParameters.ContainsKey('RotWeight')) { $RotWeight = if ($BaseYaw -eq 'hand') { 0.3 } else { 1.0 } }
# hand mode: joint-limit clamp is built in; lower start-pose pull; keep the IK target in a
# reachable shell (>=25 cm from the base axis, <=68 cm from the shoulder). No anchor drag:
# its accumulated offset made the arm overshoot sideways and forward.
$baseYawArguments = if ($BaseYaw -eq 'hand') {
    @('--base-yaw-mode=hand-azimuth', '--ik-home-weight=0.002',
      '--workspace-min-radius-m=0.25', '--workspace-max-reach-m=0.68')
} else { @('--base-yaw-mode=ik') }
$transportLogs = Join-Path $trialDir 'logs\real\transport_logs'
$knownLogs = @{}
if (Test-Path -LiteralPath $transportLogs) {
    Get-ChildItem -LiteralPath $transportLogs -Filter 'latest_*.jsonl' -File | ForEach-Object { $knownLogs[$_.Name] = $true }
}
Write-Host "REAL RM65 bridge: $RealAddress; headset forward maps to $Forward; clutch=$Clutch; base yaw=$BaseYaw; rot weight=$RotWeight."
Write-Host 'Startup: return to the last hand-guided pose and verify it before PICO following is enabled.'
Write-Host 'Left grip: press once to start following, press again to stop.'
Write-Host 'Right trigger: press depth closes the gripper in proportion; pressed past 90% = force grasp (keeps squeezing); released = fully open.'
Push-Location $simDir
try {
    & $python -u .\example\teleop_sim.py --robot rm65 --input-source xrtoolkit `
        --xrtoolkit-hand left --xrtoolkit-clutch $Clutch $forwardArgument @baseYawArguments `
        ('--rot-weight=' + $RotWeight.ToString([Globalization.CultureInfo]::InvariantCulture)) `
        --position-scale 0.5 --bridge-real $RealAddress
    $teleopExitCode = $LASTEXITCODE
} finally {
    Pop-Location
}
if (Test-Path -LiteralPath $transportLogs) {
    Get-ChildItem -LiteralPath $transportLogs -Filter 'latest_*.jsonl' -File | Where-Object { -not $knownLogs.ContainsKey($_.Name) } | ForEach-Object {
        & $python (Join-Path $trialDir 'export_teleop_data.py') $_.FullName
        if ($LASTEXITCODE -ne 0) { Write-Warning "Teleop data export failed for $($_.FullName); original trace was retained." }
    }
}
exit $teleopExitCode
