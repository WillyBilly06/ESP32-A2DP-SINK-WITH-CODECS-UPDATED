param([int]$HttpPort=8765,[switch]$Restart,[switch]$Listen,[string]$PhoneHostname='')
$ErrorActionPreference='Stop'
$nativeRoot=Split-Path -Parent $PSScriptRoot
$workspaceRoot=Split-Path -Parent $nativeRoot
$serialFolder=Join-Path $nativeRoot '_build\serial'
$statePath=Join-Path $serialFolder 'monitor-state.json'
$serverPath=Join-Path $PSScriptRoot 'serial_monitor_server.py'
$pythonPath=Join-Path $env:USERPROFILE '.espressif\python_env\idf6.1_py3.12_env\Scripts\python.exe'
if(-not(Test-Path -LiteralPath $pythonPath)){$pythonPath=(Get-Command python.exe).Source}
if($PhoneHostname -and $PhoneHostname -notmatch '^[a-z0-9-]+\.[a-z0-9-]+\.ts\.net$'){throw 'Use the exact private Tailscale hostname'}
$restorePath=''
if(Test-Path -LiteralPath $statePath){
    try{
        $current=Get-Content -LiteralPath $statePath -Raw -Encoding utf8 | ConvertFrom-Json
        $uri=[uri]$current.url
        if($current.running -and $uri.Host -in @('127.0.0.1','localhost')){
            $live=Invoke-RestMethod -Uri ($current.url+'state') -TimeoutSec 2
            if($live.pid -eq $current.pid){
                if(-not $Restart){
                    if($PhoneHostname){throw 'Phone activation requires an explicit -Restart'}
                    if($Listen){& $pythonPath (Join-Path $PSScriptRoot 'serial_monitor_ctl.py') resume;if($LASTEXITCODE -ne 0){throw 'UART resume not acknowledged'}}
                    Write-Output $current.url;exit 0
                }
                $viewerProcess=Get-CimInstance Win32_Process -Filter ('ProcessId = '+$current.pid)
                if(-not $viewerProcess.CommandLine.Contains($serverPath)){throw 'Existing viewer process identity changed'}
                & $pythonPath (Join-Path $PSScriptRoot 'serial_monitor_ctl.py') release
                if($LASTEXITCODE -ne 0){throw 'Actual UART release not acknowledged'}
                $restorePath=Join-Path $serialFolder ('viewer-restart-'+[DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds()+'.json')
                $checkpointCode=@'
import json,sys,urllib.request
from pathlib import Path
base=sys.argv[2]
def read(path):
    with urllib.request.urlopen(base+path,timeout=5) as r:return r.read()
state=json.loads(read('state'));logs=[json.loads(line) for line in read('export').splitlines()]
Path(sys.argv[1]).write_text(json.dumps({'state':state,'logs':logs}),encoding='utf-8',newline='\n')
print('Preserved '+str(len(logs))+' viewer records')
'@
                & $pythonPath -c $checkpointCode $restorePath $current.url
                if($LASTEXITCODE -ne 0){throw 'History checkpoint failed; viewer left running'}
                $confirmed=Invoke-RestMethod -Uri ($current.url+'state') -TimeoutSec 2
                if($confirmed.pid -ne $current.pid -or $confirmed.connected){throw 'Viewer reader state changed before restart'}
                Stop-Process -Id $current.pid
            }
        }
    }catch{if($Restart -or $PhoneHostname -or $Listen){throw}}
}
New-Item -ItemType Directory -Path $serialFolder -Force > $null
$arguments='"'+$serverPath+'" --port COM6 --usb-serial CA0ACAD67933EF1180C36117A04C7D1C --state "'+$statePath+'" --http-port '+$HttpPort+' --released'
if($restorePath){$arguments+=' --restore-buffer "'+$restorePath+'"'}
if($PhoneHostname){$arguments+=' --phone-hostname '+$PhoneHostname}
$dashboardProcess=Start-Process -FilePath $pythonPath -ArgumentList $arguments -WorkingDirectory $workspaceRoot -WindowStyle Hidden -RedirectStandardOutput (Join-Path $serialFolder 'server-stdout.log') -RedirectStandardError (Join-Path $serialFolder 'server-stderr.log') -PassThru
for($attempt=0;$attempt -lt 50;$attempt++){
    Start-Sleep -Milliseconds 100
    try{
        $current=Get-Content -LiteralPath $statePath -Raw -Encoding utf8 | ConvertFrom-Json
        if($current.running -and $current.url -and $current.updated_unix -gt [DateTimeOffset]::UtcNow.ToUnixTimeSeconds()-3){
            $live=Invoke-RestMethod -Uri ($current.url+'state') -TimeoutSec 1
            # Python's venv launcher may keep a separate parent PID. Bind the
            # actual HTTP service to its freshly written state receipt.
            if($live.pid -eq $current.pid){
                if($Listen){& $pythonPath (Join-Path $PSScriptRoot 'serial_monitor_ctl.py') resume;if($LASTEXITCODE -ne 0){throw 'UART resume not acknowledged'}}
                Write-Output $current.url
                if($PhoneHostname){$phoneAccess=Get-Content -Raw -LiteralPath (Join-Path $serialFolder 'phone-access.json') | ConvertFrom-Json;Write-Output $phoneAccess.url}
                exit 0
            }
        }
    }catch{}
}
throw 'Dashboard did not become ready. Check native_bt/_build/serial/server-stderr.log.'
