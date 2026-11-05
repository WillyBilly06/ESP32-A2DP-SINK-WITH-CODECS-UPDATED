param([int]$HttpPort=8765)
$ErrorActionPreference='Stop'
$nativeRoot=Split-Path -Parent $PSScriptRoot
$workspaceRoot=Split-Path -Parent $nativeRoot
$serialFolder=Join-Path $nativeRoot '_build\serial'
$statePath=Join-Path $serialFolder 'monitor-state.json'
if(Test-Path -LiteralPath $statePath){
    try{
        $current=Get-Content -LiteralPath $statePath -Raw -Encoding utf8 | ConvertFrom-Json
        $uri=[uri]$current.url
        if($current.running -and $uri.Host -in @('127.0.0.1','localhost')){
            $live=Invoke-RestMethod -Uri ($current.url+'state') -TimeoutSec 2
            if($live.pid -eq $current.pid){Write-Output $current.url;exit 0}
        }
    }catch{}
}
$pythonPath=Join-Path $env:USERPROFILE '.espressif\python_env\idf6.1_py3.12_env\Scripts\python.exe'
if(-not(Test-Path -LiteralPath $pythonPath)){$pythonPath=(Get-Command python.exe).Source}
New-Item -ItemType Directory -Path $serialFolder -Force > $null
$serverPath=Join-Path $PSScriptRoot 'serial_monitor_server.py'
$arguments='"'+$serverPath+'" --port COM6 --usb-serial CA0ACAD67933EF1180C36117A04C7D1C --state "'+$statePath+'" --http-port '+$HttpPort+' --released'
$dashboardProcess=Start-Process -FilePath $pythonPath -ArgumentList $arguments -WorkingDirectory $workspaceRoot -WindowStyle Hidden -RedirectStandardOutput (Join-Path $serialFolder 'server-stdout.log') -RedirectStandardError (Join-Path $serialFolder 'server-stderr.log') -PassThru
for($attempt=0;$attempt -lt 50;$attempt++){
    Start-Sleep -Milliseconds 100
    try{
        $current=Get-Content -LiteralPath $statePath -Raw -Encoding utf8 | ConvertFrom-Json
        if($current.running -and $current.url -and $current.updated_unix -gt [DateTimeOffset]::UtcNow.ToUnixTimeSeconds()-3){
            $live=Invoke-RestMethod -Uri ($current.url+'state') -TimeoutSec 1
            if($live.pid -eq $current.pid){Write-Output $current.url;exit 0}
        }
    }catch{}
}
throw 'Dashboard did not become ready. Check native_bt/_build/serial/server-stderr.log.'
