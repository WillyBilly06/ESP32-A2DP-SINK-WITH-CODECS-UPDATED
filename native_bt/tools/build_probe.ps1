param([switch]$WithProductIo,[switch]$WithProductSecurity,[switch]$WithRfcomm,[switch]$WithLhdc,[switch]$WithOpus,[switch]$WithAac,[switch]$WithAptx,[switch]$WithLdac,[switch]$WithSmp,[switch]$WithAvrcp,[switch]$WithSbc,[switch]$WithAudioProbe,[switch]$WithAudioOutput,[switch]$WithGatt,[switch]$WithClassicSecurity,[switch]$TraceIngress,[switch]$WithoutRxFlow,[switch]$KeepOwned=$true,[string]$TestPeer='',[string]$LhdcSourceDir='')
$ErrorActionPreference = 'Stop'
if ($WithProductIo) { $WithProductSecurity=$true; $WithSmp=$true; $WithGatt=$true; $WithAudioOutput=$true; $WithAvrcp=$true; $WithClassicSecurity=$true }
if ($WithRfcomm) { $WithClassicSecurity=$true }
if ($WithAudioOutput) { $WithAudioProbe = $true }
if ($WithAudioProbe -and -not $WithClassicSecurity) { throw 'The SBC diagnostic requires -WithClassicSecurity and the exact test peer' }
if ($WithAudioProbe) { $WithSbc = $true }
if ($TraceIngress -and -not $WithClassicSecurity) {
    throw 'The ingress trace requires -WithClassicSecurity and its restricted test peer'
}
if (($TestPeer -and $TestPeer -notmatch '^[0-9a-fA-F]{2}(:[0-9a-fA-F]{2}){5}$') -or ($WithClassicSecurity -and -not $WithProductSecurity -and -not $TestPeer)) {
    throw 'The Classic pairing probe requires -TestPeer with the exact Linux adapter address'
}
$nativeRoot = Split-Path -Parent $PSScriptRoot
$workspaceRoot = Split-Path -Parent $nativeRoot
$idfRoot = Join-Path $workspaceRoot 'esp-idf'
$buildRoot = Join-Path $nativeRoot '_build\esp32'
$probeRoot = Join-Path $nativeRoot 'esp32_probe'
New-Item -ItemType Directory -Path $buildRoot -Force | Out-Null
$buildReceiptPath=Join-Path $buildRoot 'local-build-status.json'
@{schema=1;passed=$false;state='building';hardware_accessed=$false} | ConvertTo-Json | Set-Content -LiteralPath $buildReceiptPath -Encoding utf8
if ($WithAudioOutput) {
    $nativeSdkPath=Join-Path $probeRoot 'sdkconfig'
    if (Test-Path -LiteralPath $nativeSdkPath) {
        $nativeSdk=[IO.File]::ReadAllText($nativeSdkPath)
        if ($nativeSdk -match '(?m)^(# )?CONFIG_I2S_ISR_IRAM_SAFE.*$') {
            $nativeSdk=[regex]::Replace($nativeSdk,'(?m)^(# )?CONFIG_I2S_ISR_IRAM_SAFE.*$','CONFIG_I2S_ISR_IRAM_SAFE=y')
        } else { $nativeSdk += "`nCONFIG_I2S_ISR_IRAM_SAFE=y`n" }
        [IO.File]::WriteAllText($nativeSdkPath,$nativeSdk,[Text.UTF8Encoding]::new($false))
    }
}
$env:GIT_NO_LAZY_FETCH = '1'
if (-not (Test-Path -LiteralPath (Join-Path $idfRoot '.git'))) {
    # The bundled SDK is not this host repository's Git checkout.
    $env:GIT_CEILING_DIRECTORIES = $workspaceRoot
}

python (Join-Path $PSScriptRoot 'check_workspace.py')
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
. (Join-Path $idfRoot 'export.ps1')
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
$env:IDF_COMPONENT_MANAGER = '0'
$env:CCACHE_DISABLE = $null
$env:IDF_CCACHE_ENABLE = '1'
$productSecurityBuild=if ($WithProductSecurity) { 'ON' } else { 'OFF' }
$rfcommBuild=if ($WithRfcomm) { 'ON' } else { 'OFF' }
$productIoBuild=if ($WithProductIo) { 'ON' } else { 'OFF' }
$lhdcBuild = if ($WithLhdc) { 'ON' } else { 'OFF' }
$ldacBuild=if ($WithLdac) { 'ON' } else { 'OFF' }
$aacBuild=if ($WithAac) { 'ON' } else { 'OFF' }
$aptxBuild=if ($WithAptx) { 'ON' } else { 'OFF' }
$opusBuild=if ($WithOpus) { 'ON' } else { 'OFF' }
$smpBuild=if ($WithSmp) { 'ON' } else { 'OFF' }
$avrcpBuild=if ($WithAvrcp) { 'ON' } else { 'OFF' }
$sbcBuild = if ($WithSbc) { 'ON' } else { 'OFF' }
$audioBuild = if ($WithAudioProbe) { 'ON' } else { 'OFF' }
$outputBuild = if ($WithAudioOutput) { 'ON' } else { 'OFF' }
$gattBuild = if ($WithGatt) { 'ON' } else { 'OFF' }
$securityBuild = if ($WithClassicSecurity) { 'ON' } else { 'OFF' }
$ingressBuild = if ($TraceIngress) { 'ON' } else { 'OFF' }
$disableReceiveFlow = if ($WithoutRxFlow) { 'ON' } else { 'OFF' }
$keepOwnedBuild = if ($KeepOwned) { 'ON' } else { 'OFF' }
$peerBytes = '0,0,0,0,0,0'
if ($WithClassicSecurity -and $TestPeer) {
    $peerOctets=$TestPeer.Split(':')
    [array]::Reverse($peerOctets)
    $peerBytes=($peerOctets | ForEach-Object { '0x'+$_ }) -join ','
}
$idfArguments=@('--ccache','-C',$probeRoot,'-B',$buildRoot,"-DDBT_PROBE_PRODUCT_SECURITY=$productSecurityBuild", "-DDBT_PROBE_RFCOMM=$rfcommBuild", "-DDBT_PROBE_PRODUCT_IO=$productIoBuild", "-DDBT_PROBE_LHDC_BUILD=$lhdcBuild", "-DDBT_PROBE_OPUS_BUILD=$opusBuild", "-DDBT_PROBE_AAC_BUILD=$aacBuild", "-DDBT_PROBE_LDAC_BUILD=$ldacBuild", "-DDBT_PROBE_APTX_BUILD=$aptxBuild", "-DDBT_PROBE_SMP=$smpBuild", "-DDBT_PROBE_AVRCP=$avrcpBuild", "-DDBT_PROBE_SBC_BUILD=$sbcBuild", "-DDBT_PROBE_AUDIO_TEST=$audioBuild", "-DDBT_PROBE_AUDIO_OUTPUT=$outputBuild", "-DDBT_PROBE_KEEP_OWNED_ACTIVE=$keepOwnedBuild", "-DDBT_PROBE_GATT_TEST=$gattBuild", "-DDBT_PROBE_CLASSIC_SECURITY_TEST=$securityBuild", "-DDBT_PROBE_INGRESS_TRACE=$ingressBuild", "-DDBT_PROBE_DISABLE_RX_FLOW=$disableReceiveFlow", "-DDBT_PROBE_PAIRING_PEER_BYTES=$peerBytes")
if ($LhdcSourceDir) { $idfArguments += "-DLHDCV5_SOURCE_DIR=$LhdcSourceDir" }
$sourceManifestPath=Join-Path $buildRoot 'local-build-sources.json'
# Reconfigure first so generated ports, compilation flags and SDK configuration
# are final before fingerprinting the actual build inputs.
python (Join-Path $idfRoot 'tools\idf.py') @idfArguments reconfigure
$buildExitCode=$LASTEXITCODE
$sourceCheckCode=-1
if ($buildExitCode -eq 0) {
    python (Join-Path $PSScriptRoot 'source_snapshot.py') --save $sourceManifestPath
    $buildExitCode=$LASTEXITCODE
    if ($buildExitCode -eq 0) {
        python (Join-Path $idfRoot 'tools\idf.py') --ccache -C $probeRoot -B $buildRoot build
        $buildExitCode=$LASTEXITCODE
        python (Join-Path $PSScriptRoot 'source_snapshot.py') --verify $sourceManifestPath
        $sourceCheckCode=$LASTEXITCODE
    }
}
python (Join-Path $PSScriptRoot 'check_workspace.py')
$workspaceCheckCode=$LASTEXITCODE
$buildReceipt=[ordered]@{schema=1;passed=($buildExitCode -eq 0 -and $sourceCheckCode -eq 0 -and $workspaceCheckCode -eq 0);state='failed';hardware_accessed=$false;product_io=[bool]$WithProductIo;product_security=[bool]$WithProductSecurity;rfcomm=[bool]$WithRfcomm;build_exit=$buildExitCode;source_check_exit=$sourceCheckCode;workspace_check_exit=$workspaceCheckCode}
if ($buildReceipt.passed) {
    $buildReceipt.state='complete'
    $buildReceipt.source_manifest_sha256=(Get-FileHash -LiteralPath $sourceManifestPath -Algorithm SHA256).Hash.ToLowerInvariant()
    $buildReceipt.image_sha256=(Get-FileHash -LiteralPath (Join-Path $buildRoot 'native_bt_probe.bin') -Algorithm SHA256).Hash.ToLowerInvariant()
    $buildReceipt.config_sha256=(Get-FileHash -LiteralPath (Join-Path $buildRoot 'config/sdkconfig.h') -Algorithm SHA256).Hash.ToLowerInvariant()
    $buildReceipt.partition_sha256=(Get-FileHash -LiteralPath (Join-Path $buildRoot 'partition_table/partition-table.bin') -Algorithm SHA256).Hash.ToLowerInvariant()
}
$buildReceipt | ConvertTo-Json | Set-Content -LiteralPath $buildReceiptPath -Encoding utf8
if ($buildExitCode -ne 0) { exit $buildExitCode }
if ($sourceCheckCode -ne 0) { exit $sourceCheckCode }
Write-Host 'OFF-BOARD BUILD: intended destination is existing ota_1 at 0x390000 (0x280000 bytes).'
Write-Host 'Generic SDK flash hints are disabled for this project. Preserve bootloader/table/recovery; use the admission helper only after the user releases HARDWARE_HOLD.'
exit $workspaceCheckCode
