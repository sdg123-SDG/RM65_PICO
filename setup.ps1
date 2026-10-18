param(
    [string]$Python,
    [switch]$SkipSoftware,
    [switch]$Offline,
    [string]$CacheDir
)
$ErrorActionPreference = 'Stop'
[Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
$root = $PSScriptRoot
if (-not $CacheDir) { $CacheDir = Join-Path $root 'software\cache' }
$CacheDir = [IO.Path]::GetFullPath($CacheDir)
New-Item -ItemType Directory -Path $CacheDir -Force | Out-Null

# Prefer a supported x64 CPython; do not depend on this machine's drive letters.
$candidates = @()
if ($Python) { $candidates += ,@($Python) }
else {
    if (Get-Command py -ErrorAction SilentlyContinue) {
        $candidates += ,@('py', '-3.11')
        $candidates += ,@('py', '-3.10')
        $candidates += ,@('py', '-3.12')
    }
    if (Get-Command python -ErrorAction SilentlyContinue) { $candidates += ,@('python') }
    foreach ($v in @('311', '310', '312')) {
        $candidate = Join-Path $env:LOCALAPPDATA "Programs\Python\Python$v\python.exe"
        if (Test-Path -LiteralPath $candidate) { $candidates += ,@($candidate) }
    }
}
$selected = $null
foreach ($candidate in $candidates) {
    $command = $candidate[0]
    $prefix = @($candidate | Select-Object -Skip 1)
    try {
        & $command @prefix -c "import sys,struct; sys.exit(0 if (3,10)<=sys.version_info[:2]<(3,13) and struct.calcsize('P')==8 else 1)" 2>$null
        if ($LASTEXITCODE -eq 0) { $selected = $candidate; break }
    } catch {}
}
if (-not $selected) {
    throw 'Install CPython 3.10-3.12 x64 from python.org, then run install.bat again (or setup.ps1 -Python C:\path\python.exe).'
}
$venvPython = Join-Path $root '.venv\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $venvPython)) {
    $command = $selected[0]
    $prefix = @($selected | Select-Object -Skip 1)
    & $command @prefix -m venv (Join-Path $root '.venv')
    if ($LASTEXITCODE -ne 0) { throw 'Creating the virtual environment failed.' }
}
$pipArgs = @('-m', 'pip', 'install', '-r', (Join-Path $root 'requirements.txt'))
if ($Offline) { $pipArgs += @('--no-index', '--find-links', (Join-Path $CacheDir 'wheels')) }
& $venvPython @pipArgs
if ($LASTEXITCODE -ne 0) { throw 'Python dependency installation failed.' }

if (-not $SkipSoftware) {
    $manifest = Get-Content -LiteralPath (Join-Path $root 'software\manifest.json') -Raw | ConvertFrom-Json
    foreach ($package in $manifest.packages) {
        $archive = Join-Path $CacheDir $package.file
        if (-not (Test-Path -LiteralPath $archive)) {
            if ($Offline) { throw "Offline package missing: $archive" }
            Write-Host "Downloading $($package.name)..."
            $partial = "$archive.download"
            Invoke-WebRequest -UseBasicParsing -Uri $package.url -OutFile $partial
            $hash = (Get-FileHash -LiteralPath $partial -Algorithm SHA256).Hash
            if ($hash -ne $package.sha256) { throw "SHA256 mismatch: $partial. File kept for inspection; nothing extracted." }
            Move-Item -LiteralPath $partial -Destination $archive
        }
        if ((Get-FileHash -LiteralPath $archive -Algorithm SHA256).Hash -ne $package.sha256) {
            throw "SHA256 mismatch: $archive. Replace this cached package from the manifest URL."
        }
        switch ($package.kind) {
            'service' {
                $destination = Join-Path $root 'software\XRoboToolkit-PC-Service'
                if (-not (Test-Path -LiteralPath (Join-Path $destination 'RoboticsServiceProcess.exe'))) {
                    $staging = Join-Path $root ("software\extract-" + [guid]::NewGuid().ToString('N'))
                    Expand-Archive -LiteralPath $archive -DestinationPath $staging
                    $exe = @(Get-ChildItem -LiteralPath $staging -Filter 'RoboticsServiceProcess.exe' -Recurse)
                    if ($exe.Count -ne 1) { throw "Unexpected service archive layout: $staging" }
                    New-Item -ItemType Directory -Path $destination -Force | Out-Null
                    Get-ChildItem -LiteralPath $exe[0].DirectoryName -Force | Copy-Item -Destination $destination -Recurse
                    # Keep extraction as a recoverable original. It is ignored by Git.
                }
                if (-not (Test-Path -LiteralPath (Join-Path $destination 'SDK\x64\PXREARobotSDK.dll'))) {
                    throw 'PC Service SDK/x64/PXREARobotSDK.dll is missing.'
                }
            }
            'adb' {
                if (-not (Test-Path -LiteralPath (Join-Path $root 'software\platform-tools\adb.exe'))) {
                    Expand-Archive -LiteralPath $archive -DestinationPath (Join-Path $root 'software')
                }
            }
            'apk' {
                $apk = Join-Path $root ("software\" + $package.file)
                if (-not (Test-Path -LiteralPath $apk)) { Copy-Item -LiteralPath $archive -Destination $apk }
            }
        }
        Write-Host "Verified: $($package.file)"
    }
}
& $venvPython (Join-Path $root 'scripts\doctor.py') --offline
if ($LASTEXITCODE -ne 0) { throw 'Scene/dependency check failed.' }
Write-Host 'Installed. Next: install the headset APK (docs/INSTALL.md), then double-click start.bat.'
