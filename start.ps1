param(
    [ValidateSet('teleop','jog','service','diagnostics','scene')][string]$Mode='teleop',
    [string]$Config,
    [switch]$ReuseService,
    [switch]$Check
)
$ErrorActionPreference='Stop'
$root=$PSScriptRoot
function Resolve-ProjectPath([string]$Value) {
    if ([IO.Path]::IsPathRooted($Value)) { return [IO.Path]::GetFullPath($Value) }
    return [IO.Path]::GetFullPath((Join-Path $root $Value))
}
if (-not $Config) {
    $Config=Join-Path $root 'config\local.json'
    if (-not (Test-Path -LiteralPath $Config)) { $Config=Join-Path $root 'config\example.json' }
}
$cfg=Get-Content -LiteralPath $Config -Raw | ConvertFrom-Json
$serviceDir=Resolve-ProjectPath $cfg.service_dir
$env:RM65_XR_SERVICE_DIR=$serviceDir
$env:RM65_XR_SDK_DIR=Join-Path $serviceDir 'SDK\x64'
$env:RM65_SCENE=Resolve-ProjectPath $cfg.scene
$env:RM65_TCP_OFFSET=($cfg.tcp_offset_m | ForEach-Object { ([double]$_).ToString('R',[Globalization.CultureInfo]::InvariantCulture) }) -join ','
$python=Join-Path $root '.venv\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $python)) { throw 'Run install.bat first.' }
$script=switch($Mode) {
    'teleop' { 'teleop_rm65_xrt.py' }
    'jog' { 'jog_rm65_mujoco.py' }
    'diagnostics' { 'track_controller.py' }
    'scene' { 'run_mujoco_scene.py' }
}
$arguments=@()
if ($script) { $arguments=@('-u', (Join-Path $root "scripts\$script")) }
if ($Mode -eq 'teleop') {
    $arguments+=@('--frame',$cfg.teleop.frame,'--mode',$cfg.teleop.mode,'--scale',[string]$cfg.teleop.scale,
        '--speed',[string]$cfg.teleop.speed,'--yaw-deg',[string]$cfg.teleop.yaw_deg)
}
if ($Mode -eq 'jog') { $arguments+=@('--frame',$cfg.jog.frame,'--speed',[string]$cfg.jog.speed) }
if ($Check) {
    Write-Host "Mode=$Mode; root=$root; service=$serviceDir; scene=$env:RM65_SCENE"
    & $python (Join-Path $root 'scripts\doctor.py') --offline
    exit $LASTEXITCODE
}
if ($Mode -eq 'scene') {
    & $python @arguments
    exit $LASTEXITCODE
}
$serviceExe=Join-Path $serviceDir 'RoboticsServiceProcess.exe'
if (-not (Test-Path -LiteralPath $serviceExe)) { throw 'PC Service missing. Run install.bat.' }
# One SDK client/motion mode at a time. Inspect process command lines without killing unrelated processes.
$busy=@(Get-CimInstance Win32_Process | Where-Object {
    $_.Name -match '^python(w)?\.exe$' -and $_.CommandLine -match '(teleop_rm65_xrt|jog_rm65_mujoco|track_controller)\.py'
})
if ($busy.Count) { throw 'An XR controller client is already running. Close it before starting another mode.' }
$service=$null
$sim=$null
$exitCode=0
try {
    $existing=@(Get-Process RoboticsServiceProcess -ErrorAction SilentlyContinue)
    if ($existing.Count) {
        if (-not $ReuseService -or $existing.Count -ne 1 -or $existing[0].Path -ne $serviceExe) {
            throw 'PC Service is already running. Close it, or use -ReuseService for the same configured service. An adopted service will also stop on exit.'
        }
        $service=$existing[0]
    } else {
        $service=Start-Process -FilePath $serviceExe -WorkingDirectory $serviceDir -WindowStyle Hidden -PassThru
    }
    Start-Sleep -Milliseconds 1200
    if ($service.HasExited) { throw "PC Service exited with code $($service.ExitCode)." }
    Write-Host 'Select your physical LAN/Wi-Fi IPv4 on PICO; do not select loopback or VMware/VPN addresses:'
    Get-NetIPConfiguration | Where-Object { $_.NetAdapter.Status -eq 'Up' -and $_.IPv4DefaultGateway } |
        ForEach-Object { Write-Host "$($_.InterfaceAlias): $($_.IPv4Address.IPAddress -join ', ')" }
    Write-Host 'PICO: same LAN, WORKING, Head Tracking + Controller Tracking + Send.'
    Write-Host 'Missing Head: release Grip, point left controller forward, PRESS THE STICK STRAIGHT DOWN once.'
    if ($Mode -eq 'service') {
        Read-Host 'Press ENTER to stop the service'
    } else {
        # Windows PowerShell 5.1 joins ArgumentList: quote each path/argument explicitly.
        $quoted=@($arguments | ForEach-Object { '"' + ([string]$_).Replace('"','\"') + '"' })
        $sim=Start-Process -FilePath $python -ArgumentList $quoted -WorkingDirectory $root -NoNewWindow -PassThru
        Write-Host 'Close the simulation window or press Ctrl+C here to stop both processes.'
        while ($true) {
            Start-Sleep -Milliseconds 250
            $sim.Refresh()
            $service.Refresh()
            if ($sim.HasExited) { $exitCode=$sim.ExitCode; break }
            if ($service.HasExited) { $exitCode=1; break }
        }
    }
} finally {
    foreach ($proc in @($sim,$service)) {
        if ($null -ne $proc) {
            try {
                $proc.Refresh()
                if (-not $proc.HasExited) { & taskkill.exe /PID $proc.Id /T /F 2>$null | Out-Null }
            } catch {}
        }
    }
}
exit $exitCode
