param([switch]$Offline, [string]$ParentDirectory)
$ErrorActionPreference='Stop'
if (-not $ParentDirectory) { $ParentDirectory=Split-Path $PSScriptRoot -Parent }
$manifest=Get-Content -LiteralPath (Join-Path $PSScriptRoot 'config\vdo_sources.json') -Raw | ConvertFrom-Json
foreach($name in @('vdo-rm65b','vdo-sim')) {
    $source=$manifest.$name
    $target=Join-Path $ParentDirectory $name
    if (-not(Test-Path -LiteralPath $target)) {
        if($Offline){throw "Offline source missing: $target"}
        Write-Host "Cloning $name (requires your GitLab SSH access)..."
        & git clone --depth 1 $source.url $target
        if($LASTEXITCODE -ne 0){throw "Clone failed: $name"}
        $head=(& git -C $target rev-parse HEAD)
        if($head -ne $source.commit) {
            & git -C $target fetch --depth 1 origin $source.commit
            if($LASTEXITCODE -ne 0){throw "Pinned source unavailable: $name"}
            & git -C $target checkout --detach $source.commit
            if($LASTEXITCODE -ne 0){throw "Checkout failed: $name"}
        }
    }
    $head=(& git -C $target rev-parse HEAD)
    if($LASTEXITCODE -ne 0 -or $head -ne $source.commit){
        throw "Existing $name is not the verified revision. No files reset. Check config/vdo_sources.json."
    }
    Write-Host "$name @ $head"
}
& (Join-Path $PSScriptRoot '.venv\Scripts\python.exe') (Join-Path $PSScriptRoot 'scripts\prepare_vdo.py') (Join-Path $ParentDirectory 'vdo-rm65b')
if($LASTEXITCODE -ne 0){throw 'Native PICO hook setup failed.'}
