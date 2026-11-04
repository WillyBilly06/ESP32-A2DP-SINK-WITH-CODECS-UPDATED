$ErrorActionPreference = 'Stop'
$nativeRoot = Split-Path -Parent $PSScriptRoot
$workspaceRoot = Split-Path -Parent $nativeRoot
$idfRoot = Join-Path $workspaceRoot 'esp-idf'
$buildRoot = Join-Path $nativeRoot '_build\audio_app'
$applicationRoot = Join-Path $workspaceRoot 'bt_audio_sink'
$env:GIT_NO_LAZY_FETCH = '1'
if (-not (Test-Path -LiteralPath (Join-Path $idfRoot '.git'))) {
    $env:GIT_CEILING_DIRECTORIES = $workspaceRoot
}

python (Join-Path $PSScriptRoot 'check_workspace.py')
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
. (Join-Path $idfRoot 'export.ps1')
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
$env:IDF_COMPONENT_MANAGER = '0'
$env:CCACHE_DISABLE = '1'
python (Join-Path $idfRoot 'tools\idf.py') -C $applicationRoot -B $buildRoot build
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
python (Join-Path $PSScriptRoot 'check_workspace.py')
exit $LASTEXITCODE
