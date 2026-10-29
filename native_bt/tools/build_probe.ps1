param([switch]$WithLhdc,[switch]$WithOpus,[switch]$WithAac,[switch]$WithAptx,[switch]$WithLdac,[switch]$WithSmp,[switch]$WithAvrcp,[switch]$WithSbc,[switch]$WithAudioProbe,[switch]$WithAudioOutput,[switch]$WithGatt,[switch]$WithClassicSecurity,[switch]$TraceIngress,[switch]$WithoutRxFlow,[switch]$KeepOwned=$true,[string]$TestPeer='')
$ErrorActionPreference = 'Stop'
if ($WithAudioOutput) { $WithAudioProbe = $true }
if ($WithAudioProbe -and -not $WithClassicSecurity) { throw 'The SBC diagnostic requires -WithClassicSecurity and the exact test peer' }
if ($WithAudioProbe) { $WithSbc = $true }
if ($TraceIngress -and -not $WithClassicSecurity) {
    throw 'The ingress trace requires -WithClassicSecurity and its restricted test peer'
}
if ($WithClassicSecurity -and $TestPeer -notmatch '^[0-9a-fA-F]{2}(:[0-9a-fA-F]{2}){5}$') {
    throw 'The Classic pairing probe requires -TestPeer with the exact Linux adapter address'
}
$nativeRoot = Split-Path -Parent $PSScriptRoot
$workspaceRoot = Split-Path -Parent $nativeRoot
$idfRoot = Join-Path $workspaceRoot 'esp-idf'
$buildRoot = Join-Path $nativeRoot '_build\esp32'
$probeRoot = Join-Path $nativeRoot 'esp32_probe'
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
$env:CCACHE_DISABLE = '1'
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
if ($WithClassicSecurity) {
    $peerOctets=$TestPeer.Split(':')
    [array]::Reverse($peerOctets)
    $peerBytes=($peerOctets | ForEach-Object { '0x'+$_ }) -join ','
}
python (Join-Path $idfRoot 'tools\idf.py') -C $probeRoot -B $buildRoot "-DDBT_PROBE_LHDC_BUILD=$lhdcBuild" "-DDBT_PROBE_OPUS_BUILD=$opusBuild" "-DDBT_PROBE_AAC_BUILD=$aacBuild" "-DDBT_PROBE_LDAC_BUILD=$ldacBuild" "-DDBT_PROBE_APTX_BUILD=$aptxBuild" "-DDBT_PROBE_SMP=$smpBuild" "-DDBT_PROBE_AVRCP=$avrcpBuild" "-DDBT_PROBE_SBC_BUILD=$sbcBuild" "-DDBT_PROBE_AUDIO_TEST=$audioBuild" "-DDBT_PROBE_AUDIO_OUTPUT=$outputBuild" "-DDBT_PROBE_KEEP_OWNED_ACTIVE=$keepOwnedBuild" "-DDBT_PROBE_GATT_TEST=$gattBuild" "-DDBT_PROBE_CLASSIC_SECURITY_TEST=$securityBuild" "-DDBT_PROBE_INGRESS_TRACE=$ingressBuild" "-DDBT_PROBE_DISABLE_RX_FLOW=$disableReceiveFlow" "-DDBT_PROBE_PAIRING_PEER_BYTES=$peerBytes" build
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
python (Join-Path $PSScriptRoot 'check_workspace.py')
exit $LASTEXITCODE
