param([string]$Profile, [string]$Real, [ValidateSet('+X','-X')][string]$Forward, [switch]$Check, [switch]$Inspect, [switch]$NoGripper)
$ErrorActionPreference='Stop'
$root=$PSScriptRoot
if(-not $Profile){
 $Profile=Join-Path $root 'config\sim_real_bridge.local.json'
 if(-not(Test-Path -LiteralPath $Profile)){$Profile=Join-Path $root 'config\sim_real_bridge.example.json'}
}
$python=Join-Path $root '.venv\Scripts\python.exe'
$launcher=Join-Path $root 'scripts\launch_vdo_pico_real.py'
$configPath=Join-Path $root 'config\local.json'
if(-not(Test-Path -LiteralPath $configPath)){$configPath=Join-Path $root 'config\example.json'}
$cfg=Get-Content -LiteralPath $configPath -Raw | ConvertFrom-Json
function Resolve-RootPath([string]$Path) {
 if([IO.Path]::IsPathRooted($Path)){return $Path}
 return [IO.Path]::GetFullPath((Join-Path $root $Path))
}
$serviceDir=Resolve-RootPath $cfg.service_dir
$env:RM65_XR_SERVICE_DIR=$serviceDir
$env:RM65_XR_SDK_DIR=Join-Path $serviceDir 'SDK\x64'
$vdo=if($cfg.vdo_dir){Resolve-RootPath $cfg.vdo_dir}else{Join-Path (Split-Path $root -Parent) 'vdo-rm65b'}
$arguments=@('-u',$launcher,'--vdo-dir',$vdo,'--config',[IO.Path]::GetFullPath($Profile))
if($Real){$arguments+=@('--real',$Real)}
if($Forward){$arguments+=@('--forward='+$Forward)}
if($NoGripper){$arguments+='--no-gripper'}
$robotProfile=Get-Content -LiteralPath $Profile -Raw | ConvertFrom-Json
$connectReal=if($null -ne $robotProfile.connect_real){[bool]$robotProfile.connect_real}else{$true}
if(-not $connectReal){$arguments+='--no-real'}
if($Check){
 & $python @arguments --check
 exit $LASTEXITCODE
}
$busy=@(Get-CimInstance Win32_Process | Where-Object {
 $_.Name -eq 'python.exe' -and $_.CommandLine -match '(launch_vdo_pico|launch_rm65_box_real|track_controller|teleop_rm65_xrt|jog_rm65_mujoco)'
})
if($busy.Count){throw 'Close existing robot/simulation/XR clients before starting the real entry.'}
if($connectReal){
 $guards=@(Get-CimInstance Win32_Process | Where-Object {
  $_.Name -match '^python(w)?\.exe$' -and $_.CommandLine -match '(real_arm_guard\.py|real_pico_guard\.py)'
 })
 if($guards.Count){throw ('Existing robot guard processes must be checked before restart (PID '+(($guards.ProcessId)-join ',')+'). No process was killed.')}
}
$address=if($Real){$Real}else{$robotProfile.real_addr}
$robotHost,$robotPort=$address -split ':',2
if(-not $robotPort){$robotPort=8080}
$connections=@()
if($connectReal){
 $connections=@(Get-NetTCPConnection -RemoteAddress $robotHost -RemotePort ([int]$robotPort) -State Established -ErrorAction SilentlyContinue)
 if($connections.Count){throw ('Controller connection is already in use (PID '+(($connections.OwningProcess | Select-Object -Unique)-join ',')+'). Quit the original real CLI before starting PICO; no processes were killed.')}
}
if($Inspect){
 & $python @arguments --inspect-real
 exit $LASTEXITCODE
}
$service=$null
$serviceOwned=$false
$sim=$null
$code=0
try {
 $exe=Join-Path $serviceDir 'RoboticsServiceProcess.exe'
 $existing=@(Get-Process RoboticsServiceProcess -ErrorAction SilentlyContinue)
 if($existing.Count -gt 1){throw 'Multiple PC Services found; close duplicate services first.'}
 if($existing.Count -eq 1){
  $service=$existing[0]
  Write-Host 'Using the existing PC Service. This launcher will not terminate a service it did not start.'
 }else{
  $service=Start-Process -FilePath $exe -WorkingDirectory $serviceDir -WindowStyle Hidden -PassThru
  $serviceOwned=$true
  Start-Sleep -Milliseconds 1200
 }
 if($service.HasExited){throw 'PC Service did not start'}
 if($connectReal){
  Write-Host 'PICO -> simulated model -> original real joint API. Release Grip/trigger first; no pico arm command.'
  Write-Host 'No startup home/enable/gripper command. Review docs/SIM_REAL_BRIDGE.md; physical emergency stop required.'
 }else{
  Write-Host 'PICO -> simulated model -> real-bridge preview. RM65 NETWORK DISCONNECTED.'
 }
 Get-NetIPAddress -AddressFamily IPv4 -ErrorAction SilentlyContinue | Where-Object {$_.IPAddress -notmatch '^(127\.|169\.254\.)'} | ForEach-Object {Write-Host ('PICO PC address candidate: '+$_.IPAddress+' ('+$_.InterfaceAlias+')')}
 $quoted=@($arguments | ForEach-Object {'"'+([string]$_).Replace('"','\"')+'"'})
 $sim=Start-Process -FilePath $python -ArgumentList $quoted -WorkingDirectory $root -NoNewWindow -PassThru
 while($true){
  Start-Sleep -Milliseconds 250
  $sim.Refresh()
  $service.Refresh()
  if($sim.HasExited){$code=$sim.ExitCode;break}
  if($service.HasExited){Write-Warning 'XR service stopped; release Grip and quit the real CLI.';$code=1;break}
 }
} finally {
 if($sim){
  $sim.Refresh()
  if(-not $sim.HasExited){[void]$sim.WaitForExit(7000);$sim.Refresh()}
  if(-not $sim.HasExited){
   Write-Warning 'Real process is still alive. Not force-killing it or its XR service. Use native CLI quit / physical emergency stop if necessary.'
  }
 }
 if($serviceOwned -and $service -and (-not $sim -or $sim.HasExited)){
  $service.Refresh()
  if(-not $service.HasExited){Stop-Process -Id $service.Id}
 }
}
exit $code
